//
// HazardManager.h
//
// Tracks spawns that must be avoided ("hazards") and publishes a snapshot of their shapes and the
// navmesh polygons they affect. Every path computation (active /nav paths, path length and
// reachability queries from other plugins) reads the snapshot through NavPathFilter.
//

#pragma once

#include "common/NavModule.h"
#include "plugin/HazardGeometry.h"

#include <mq/base/Signal.h>
#include <DetourNavMesh.h>
#include <glm/glm.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class RenderGroup;
class dtNavMeshQuery;
namespace nav { class MapCircle; }
namespace eqlib { class PlayerClient; }

//----------------------------------------------------------------------------

// radius value meaning "melee range between me and the spawn + rampage margin"
constexpr float kHazardRadiusRampage = -2.f;
// radius value meaning "use the default from settings"
constexpr float kHazardRadiusDefault = -1.f;

enum class HazardMode : uint8_t { Avoid, Seek };
enum class HazardInsideAction : uint8_t { Flee, Hold };

struct HazardRule
{
	int id = 0;
	HazardMode mode = HazardMode::Avoid;
	std::string pattern;                 // case-insensitive substring of Name or DisplayedName
	int spawnType = -1;                  // eSpawnType, -1 = any
	float hard = kHazardRadiusDefault;   // never enter
	float soft = kHazardRadiusDefault;   // preferred standoff (cost penalty between hard and soft)
	float arcCenterDeg = 0.f;            // relative to the spawn's facing; 0 = front, 180 = back
	float arcWidthDeg = 360.f;           // 360 = full circle
	HazardInsideAction inside = HazardInsideAction::Flee;
	bool sticky = true;                  // keep tracking a matched spawn even if its name/type changes
	bool enabled = true;
	bool transient = false;              // added through the API without persisting

	bool IsFullCircle() const { return arcWidthDeg >= 359.9f; }
};

// One hazard in a published snapshot.
struct HazardEntry
{
	hazard::Shape shape;
	float hardRadius = 0.f;
	float softRadius = 0.f;
	float height = 0.f;          // navmesh y of the floor under the spawn
	glm::vec3 eqPos{ 0.f };
	HazardInsideAction inside = HazardInsideAction::Flee;
	int ruleId = 0;
	uint32_t spawnId = 0;
	std::string name;
};

// Bit for hazard index i in the owner masks. Hazards past 63 share the last bit and are never demoted.
inline uint64_t HazardBit(size_t index) { return 1ull << (index < 63 ? index : 63); }

struct HazardSnapshot
{
	uint64_t version = 0;
	std::vector<HazardEntry> hazards;

	// polygon -> mask of hazards whose hard radius fully contains it (blocked)
	std::unordered_map<dtPolyRef, uint64_t> hardFull;
	// polygon -> mask of hazards whose hard radius partly overlaps it (allowed, penalized, post-checked)
	std::unordered_map<dtPolyRef, uint64_t> hardPartial;
	// polygon -> path cost multiplier from soft rings and hard boundaries (>= 1)
	std::unordered_map<dtPolyRef, float> costMult;

	// Mask of hazards the position is inside that a path may leave ("demoted" to a cost): only
	// inside=flee hazards with their own bit (index < 63). inside=hold hazards stay hard, so a player
	// standing in one holds position.
	uint64_t DemotableMask(const glm::vec3& meshPos, float heightExtent) const;

	// index of the first hazard (outside skipMask) whose hard shape contains the position, or -1
	int FindContaining(const glm::vec3& meshPos, float heightExtent, uint64_t skipMask = 0) const;

	// index of the first hazard (outside skipMask) whose hard shape the segment crosses, or -1
	int FindCrossing(const glm::vec3& a, const glm::vec3& b, float heightExtent, uint64_t skipMask = 0) const;
};

//----------------------------------------------------------------------------

class HazardManager : public NavModule
{
public:
	HazardManager();
	~HazardManager() override;

	void Initialize() override;
	void Shutdown() override;
	void OnPulse() override;
	void OnBeginZone() override;
	void SetZoneId(int zoneId) override;

	void OnAddSpawn(eqlib::PlayerClient* pSpawn);
	void OnRemoveSpawn(eqlib::PlayerClient* pSpawn);

	//------------------------------------------------------------------------
	// rules

	// parse "<name> [type=..] [hard=..] [soft=..] [arc=..] [inside=..] [sticky=..]"
	bool ParseRule(std::string_view args, HazardRule& out, std::string& error) const;

	int AddRule(HazardRule rule);                  // returns the new rule id
	bool RemoveRule(int id);
	int RemoveRulesByPattern(std::string_view pattern);
	void ClearRules(bool transientOnly = false);  // transientOnly: only rules added through the API
	bool SetRuleEnabled(int id, bool enabled);

	const std::vector<HazardRule>& GetRules() const { return m_rules; }
	std::string DescribeRule(const HazardRule& rule) const;

	//------------------------------------------------------------------------
	// pathing

	// null when there are no hazards
	std::shared_ptr<const HazardSnapshot> GetSnapshot() const { return m_snapshot; }

	// changes every time the snapshot is republished
	uint64_t GetVersion() const { return m_version; }

	bool IsPlayerInsideHazard(std::string* hazardName = nullptr) const;

	// 2D distance from an EQ position to the nearest hard avoid shape: < 0 if inside, FLT_MAX if none
	float GetHazardDistance(const glm::vec3& eqPos) const;

	//------------------------------------------------------------------------
	// commands / ui

	// handles "/nav avoid ..." and "/nav hazard ..."; returns false if the line isn't a hazard command
	bool HandleCommand(const char* line);

	void DebugUI();

private:
	struct Instance
	{
		int ruleId = 0;
		uint32_t spawnId = 0;
		eqlib::PlayerClient* ptr = nullptr;
		std::string name;
		uint8_t type = 0;
		int race = 0;
		bool renamed = false;

		glm::vec3 eqPos{ 0.f };
		float floorHeight = 0.f;     // the floor under the spawn; tall/levitating spawns still block it
		float heading = 0.f;
		float hardRadius = 0.f;
		float softRadius = 0.f;

		// values the published snapshot was built from (for hysteresis)
		glm::vec3 pathedPos{ 0.f };
		float pathedHeading = 0.f;
		float pathedHard = 0.f;
		float pathedSoft = 0.f;

		// polygon classification for the pathed values, reused until this hazard changes
		enum class PolyKind : uint8_t { HardFull, HardPartial, Soft };
		struct CachedPoly { dtPolyRef ref; PolyKind kind; float mult; };
		std::vector<CachedPoly> polys;
		bool polysValid = false;
	};

	const HazardRule* FindRule(int id) const;
	bool RuleMatches(const HazardRule& rule, eqlib::PlayerClient* pSpawn) const;
	void MatchSpawn(eqlib::PlayerClient* pSpawn);
	void Rescan();
	void RefreshInstances();
	void UpdateInstanceState(Instance& inst, const HazardRule& rule, eqlib::PlayerClient* pSpawn);
	void ClassifyPolys(Instance& inst, const hazard::Shape& shape, dtNavMeshQuery* query);
	void RebuildSnapshot();
	void UpdateVisuals();
	void ClearInstances();

	void LoadRules(int zoneId);
	void SaveRules();
	std::string SerializeRule(const HazardRule& rule) const;
	bool DeserializeRule(std::string_view data, HazardRule& out) const;

	void PrintRules() const;
	void PrintStatus() const;

	using clock = std::chrono::steady_clock;

	std::vector<HazardRule> m_rules;
	int m_nextRuleId = 1;
	int m_zoneId = -1;

	std::vector<Instance> m_instances;
	bool m_dirty = true;             // snapshot needs a rebuild
	bool m_overlayDirty = true;      // in-world overlay needs a redraw
	bool m_rescanRequested = true;

	std::shared_ptr<const HazardSnapshot> m_snapshot;
	uint64_t m_version = 1;
	double m_lastBuildMs = 0.0;
	size_t m_lastPolyCount = 0;

	clock::time_point m_lastRefresh;
	clock::time_point m_lastRescan;

	std::unique_ptr<RenderGroup> m_overlay;
	std::vector<std::unique_ptr<nav::MapCircle>> m_mapCircles;
	uint64_t m_mapVersion = 0;

	mq::Signal<>::ScopedConnection m_meshChangedConn;
};

// convenience accessors used by NavigationPath
std::shared_ptr<const HazardSnapshot> GetHazardSnapshot();
uint64_t GetHazardVersion();
