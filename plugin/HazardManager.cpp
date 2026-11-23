//
// HazardManager.cpp
//

#include "pch.h"
#include "HazardManager.h"

#include "common/NavMesh.h"
#include "plugin/DebugDrawDX.h"
#include "plugin/MapAPI.h"
#include "plugin/MQ2Navigation.h"
#include "plugin/PluginSettings.h"
#include "plugin/RenderHandler.h"
#include "plugin/Utilities.h"

#include <mq/base/String.h>
#include <DetourNavMeshQuery.h>
#include <imgui.h>
#include <spdlog/spdlog.h>

#include <cfloat>
#include <sstream>

using namespace std::chrono_literals;

//============================================================================

namespace {

struct SpawnTypeName { const char* name; int type; };

const SpawnTypeName s_spawnTypeNames[] = {
	{ "any", -1 },
	{ "pc", PC },
	{ "npc", NPC },
	{ "pet", PET },
	{ "mercenary", MERCENARY },
	{ "merc", MERCENARY },
	{ "untargetable", UNTARGETABLE },
	{ "aura", AURA },
	{ "object", OBJECT },
	{ "chest", CHEST },
	{ "trap", TRAP },
	{ "trigger", TRIGGER },
	{ "timer", TIMER },
	{ "banner", BANNER },
	{ "campfire", CAMPFIRE },
	{ "corpse", CORPSE },
	{ "mount", MOUNT },
	{ "flyer", FLYER },
	{ "item", ITEM },
};

bool ParseSpawnType(std::string_view text, int& out)
{
	for (const auto& entry : s_spawnTypeNames)
	{
		if (mq::ci_equals(text, entry.name))
		{
			out = entry.type;
			return true;
		}
	}
	return false;
}

const char* SpawnTypeToString(int type)
{
	for (const auto& entry : s_spawnTypeNames)
	{
		if (entry.type == type)
			return entry.name;
	}
	return "?";
}

bool ParseRadius(std::string_view text, float& out)
{
	if (mq::ci_equals(text, "rampage"))
	{
		out = kHazardRadiusRampage;
		return true;
	}
	if (mq::ci_equals(text, "default"))
	{
		out = kHazardRadiusDefault;
		return true;
	}
	float value = GetFloatFromString(text, -1.f);
	if (value <= 0.f)
		return false;
	out = value;
	return true;
}

std::string RadiusToString(float r)
{
	if (r == kHazardRadiusRampage) return "rampage";
	if (r < 0.f) return "default";
	return fmt::format("{:g}", r);
}

bool ParseBool(std::string_view text, bool& out)
{
	if (mq::ci_equals(text, "on") || mq::ci_equals(text, "true") || text == "1") { out = true; return true; }
	if (mq::ci_equals(text, "off") || mq::ci_equals(text, "false") || text == "0") { out = false; return true; }
	return false;
}

bool ParseArc(std::string_view text, float& centerDeg, float& widthDeg)
{
	if (mq::ci_equals(text, "front")) { centerDeg = 0.f; widthDeg = 90.f; return true; }
	if (mq::ci_equals(text, "back") || mq::ci_equals(text, "behind")) { centerDeg = 180.f; widthDeg = 90.f; return true; }
	if (mq::ci_equals(text, "left")) { centerDeg = 90.f; widthDeg = 90.f; return true; }
	if (mq::ci_equals(text, "right")) { centerDeg = -90.f; widthDeg = 90.f; return true; }
	if (mq::ci_equals(text, "full") || mq::ci_equals(text, "none")) { centerDeg = 0.f; widthDeg = 360.f; return true; }

	// "<center>,<width>"
	size_t comma = text.find(',');
	if (comma == std::string_view::npos)
		return false;
	float c = GetFloatFromString(text.substr(0, comma), FLT_MAX);
	float w = GetFloatFromString(text.substr(comma + 1), -1.f);
	if (c == FLT_MAX || w <= 0.f || w > 360.f)
		return false;
	centerDeg = c;
	widthDeg = w;
	return true;
}

glm::vec2 To2D(const glm::vec3& meshPos) { return { meshPos.x, meshPos.z }; }

glm::vec3 SpawnMeshPos(PlayerClient* pSpawn)
{
	float height = nav::GetSettings().use_spawn_floor_height ? pSpawn->FloorHeight : pSpawn->Z;
	return { pSpawn->X, height, pSpawn->Y };
}

glm::vec3 PlayerMeshPos()
{
	PlayerClient* me = pControlledPlayer ? pControlledPlayer : pLocalPlayer;
	if (!me)
		return glm::vec3{ 0.f };
	return { me->X, me->FloorHeight, me->Y };
}

class PolyCollector : public dtPolyQuery
{
public:
	struct Item { const dtMeshTile* tile; const dtPoly* poly; dtPolyRef ref; };
	std::vector<Item> items;

	void process(const dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count) override
	{
		for (int i = 0; i < count; ++i)
			items.push_back({ tile, polys[i], refs[i] });
	}
};

std::string ZoneSection(int zoneId)
{
	return fmt::format("Hazards:{}", GetShortZone(zoneId));
}

} // namespace

//============================================================================

uint64_t HazardSnapshot::ContainingMask(const glm::vec3& meshPos, float heightExtent) const
{
	uint64_t mask = 0;
	glm::vec2 p = To2D(meshPos);
	for (size_t i = 0; i < hazards.size(); ++i)
	{
		const HazardEntry& h = hazards[i];
		if (std::fabs(meshPos.y - h.height) <= heightExtent && hazard::PointInShape(h.shape, p, h.hardRadius))
			mask |= HazardBit(i);
	}
	return mask;
}

int HazardSnapshot::FindContaining(const glm::vec3& meshPos, float heightExtent, uint64_t skipMask) const
{
	glm::vec2 p = To2D(meshPos);
	for (size_t i = 0; i < hazards.size(); ++i)
	{
		if (i < 63 && (skipMask & HazardBit(i)))
			continue;
		const HazardEntry& h = hazards[i];
		if (std::fabs(meshPos.y - h.height) <= heightExtent && hazard::PointInShape(h.shape, p, h.hardRadius))
			return static_cast<int>(i);
	}
	return -1;
}

int HazardSnapshot::FindCrossing(const glm::vec3& a, const glm::vec3& b, float heightExtent, uint64_t skipMask) const
{
	glm::vec2 a2 = To2D(a), b2 = To2D(b);
	for (size_t i = 0; i < hazards.size(); ++i)
	{
		if (i < 63 && (skipMask & HazardBit(i)))
			continue;
		const HazardEntry& h = hazards[i];
		if (std::fabs(a.y - h.height) > heightExtent && std::fabs(b.y - h.height) > heightExtent)
			continue;
		if (hazard::SegmentIntersectsShape(a2, b2, h.shape, h.hardRadius))
			return static_cast<int>(i);
	}
	return -1;
}

//============================================================================

std::shared_ptr<const HazardSnapshot> GetHazardSnapshot()
{
	if (!g_mq2Nav || !nav::GetSettings().hazards_enabled)
		return nullptr;
	return g_mq2Nav->Get<HazardManager>()->GetSnapshot();
}

uint64_t GetHazardVersion()
{
	if (!g_mq2Nav)
		return 0;
	// toggling hazards on/off changes the effective version too
	return g_mq2Nav->Get<HazardManager>()->GetVersion() * 2 + (nav::GetSettings().hazards_enabled ? 1 : 0);
}

//============================================================================

HazardManager::HazardManager() = default;
HazardManager::~HazardManager() = default;

void HazardManager::Initialize()
{
	if (NavMesh* mesh = g_mq2Nav->Get<NavMesh>())
	{
		m_meshChangedConn = mesh->OnNavMeshChanged.Connect([this]() { m_dirty = true; });
	}

	m_overlay = std::make_unique<RenderGroup>();
	g_renderHandler->AddRenderable(m_overlay.get());
}

void HazardManager::Shutdown()
{
	if (m_overlay)
	{
		g_renderHandler->RemoveRenderable(m_overlay.get());
		m_overlay.reset();
	}

	m_mapCircles.clear();
	m_instances.clear();
	m_snapshot.reset();
	m_meshChangedConn.Disconnect();
}

void HazardManager::OnBeginZone()
{
	ClearInstances();
}

void HazardManager::SetZoneId(int zoneId)
{
	if (zoneId == m_zoneId)
		return;

	ClearInstances();
	LoadRules(zoneId);
}

void HazardManager::ClearInstances()
{
	m_instances.clear();
	m_dirty = true;
	m_rescanRequested = true;

	// publish an empty snapshot right away so nothing paths against stale positions
	RebuildSnapshot();
	UpdateVisuals();
}

//----------------------------------------------------------------------------

void HazardManager::OnPulse()
{
	if (m_rules.empty() && m_instances.empty())
	{
		if (m_snapshot)
		{
			m_dirty = true;
			RebuildSnapshot();
			UpdateVisuals();
		}
		return;
	}

	if (!nav::ValidIngame(false) || gZoning)
		return;

	auto& settings = nav::GetSettings();
	clock::time_point now = clock::now();

	bool refreshed = false;
	if (m_rescanRequested || now - m_lastRescan >= std::chrono::milliseconds(settings.hazard_rescan_ms))
	{
		Rescan();
		m_lastRescan = now;
		m_rescanRequested = false;
		refreshed = true;
	}

	if (refreshed || now - m_lastRefresh >= std::chrono::milliseconds(settings.hazard_refresh_ms))
	{
		RefreshInstances();
		m_lastRefresh = now;

		if (m_dirty)
			RebuildSnapshot();

		UpdateVisuals();
	}
}

void HazardManager::OnAddSpawn(PlayerClient* pSpawn)
{
	if (!pSpawn || m_rules.empty())
		return;

	// Name and SpawnID are valid here, but position may not be final yet; RefreshInstances fixes that.
	MatchSpawn(pSpawn);
}

void HazardManager::OnRemoveSpawn(PlayerClient* pSpawn)
{
	auto it = std::remove_if(m_instances.begin(), m_instances.end(),
		[pSpawn](const Instance& inst) { return inst.ptr == pSpawn; });

	if (it != m_instances.end())
	{
		m_instances.erase(it, m_instances.end());
		m_dirty = true;
	}
}

//----------------------------------------------------------------------------

const HazardRule* HazardManager::FindRule(int id) const
{
	for (const HazardRule& rule : m_rules)
	{
		if (rule.id == id)
			return &rule;
	}
	return nullptr;
}

bool HazardManager::RuleMatches(const HazardRule& rule, PlayerClient* pSpawn) const
{
	if (!rule.enabled || rule.mode != HazardMode::Avoid || rule.pattern.empty())
		return false;

	if (pSpawn == pLocalPlayer || pSpawn == pControlledPlayer)
		return false;

	if (mq::ci_find_substr(pSpawn->Name, rule.pattern) == -1
		&& mq::ci_find_substr(pSpawn->DisplayedName, rule.pattern) == -1)
	{
		return false;
	}

	if (rule.spawnType != -1 && static_cast<int>(GetSpawnType(pSpawn)) != rule.spawnType)
		return false;

	return true;
}

void HazardManager::MatchSpawn(PlayerClient* pSpawn)
{
	for (const HazardRule& rule : m_rules)
	{
		if (!RuleMatches(rule, pSpawn))
			continue;

		bool exists = std::any_of(m_instances.begin(), m_instances.end(),
			[&](const Instance& inst) { return inst.ruleId == rule.id && inst.spawnId == pSpawn->SpawnID; });
		if (exists)
			continue;

		Instance inst;
		inst.ruleId = rule.id;
		inst.spawnId = pSpawn->SpawnID;
		inst.ptr = pSpawn;
		inst.name = pSpawn->Name;
		inst.type = pSpawn->Type;
		inst.race = pSpawn->GetRace();
		UpdateInstanceState(inst, rule, pSpawn);
		m_instances.push_back(std::move(inst));
		m_dirty = true;

		SPDLOG_DEBUG("Hazard: tracking {} ({}) for rule {}", pSpawn->DisplayedName, pSpawn->SpawnID, rule.id);
	}
}

void HazardManager::Rescan()
{
	if (!pSpawnManager)
		return;

	for (PlayerClient* pSpawn = pSpawnManager->FirstSpawn; pSpawn; pSpawn = pSpawn->GetNext())
	{
		MatchSpawn(pSpawn);
	}
}

void HazardManager::UpdateInstanceState(Instance& inst, const HazardRule& rule, PlayerClient* pSpawn)
{
	auto& settings = nav::GetSettings();

	inst.eqPos = GetSpawnPosition(pSpawn);
	inst.heading = pSpawn->Heading;

	auto resolve = [&](float value, float defaultValue) -> float
	{
		if (value == kHazardRadiusRampage)
		{
			PlayerClient* me = pControlledPlayer ? pControlledPlayer : pLocalPlayer;
			float melee = me ? GetMeleeRange(me, pSpawn) : 14.f;
			return melee + settings.hazard_rampage_margin;
		}
		return value > 0.f ? value : defaultValue;
	};

	float hard = resolve(rule.hard, settings.hazard_default_hard);
	float softDefault = hard + std::max(0.f, settings.hazard_default_soft - settings.hazard_default_hard);
	float soft = resolve(rule.soft, softDefault);

	inst.hardRadius = hard;
	inst.softRadius = std::max(hard, soft);
}

void HazardManager::RefreshInstances()
{
	auto& settings = nav::GetSettings();
	const float headingEpsilon = settings.hazard_heading_epsilon_deg * 512.f / 360.f;

	for (auto it = m_instances.begin(); it != m_instances.end();)
	{
		Instance& inst = *it;
		const HazardRule* rule = FindRule(inst.ruleId);
		PlayerClient* pSpawn = GetSpawnByID(inst.spawnId);

		// gone, or the id now belongs to a different spawn object: drop it and let the rescan re-match
		if (!rule || !rule->enabled || !pSpawn || pSpawn != inst.ptr)
		{
			it = m_instances.erase(it);
			m_dirty = true;
			continue;
		}

		// same object, changed identity (renamed/retyped to defeat matching)
		int race = pSpawn->GetRace();
		if (inst.name != pSpawn->Name || inst.type != pSpawn->Type || inst.race != race)
		{
			if (!rule->sticky && !RuleMatches(*rule, pSpawn))
			{
				SPDLOG_INFO("Hazard: {} ({}) changed to {} and no longer matches rule {} - dropping",
					inst.name, inst.spawnId, pSpawn->Name, rule->id);
				it = m_instances.erase(it);
				m_dirty = true;
				continue;
			}

			SPDLOG_INFO("Hazard: {} ({}) changed to {} (type {} -> {}) - still tracking",
				inst.name, inst.spawnId, pSpawn->Name, inst.type, pSpawn->Type);
			inst.name = pSpawn->Name;
			inst.type = pSpawn->Type;
			inst.race = race;
			inst.renamed = true;
		}

		UpdateInstanceState(inst, *rule, pSpawn);

		// hysteresis: only republish when the change matters
		float headingDelta = std::fabs(inst.heading - inst.pathedHeading);
		headingDelta = std::min(headingDelta, 512.f - headingDelta);
		if (glm::distance(inst.eqPos, inst.pathedPos) > settings.hazard_move_epsilon
			|| (!rule->IsFullCircle() && headingDelta > headingEpsilon)
			|| std::fabs(inst.hardRadius - inst.pathedHard) > 0.25f
			|| std::fabs(inst.softRadius - inst.pathedSoft) > 0.25f)
		{
			m_dirty = true;
		}

		++it;
	}
}

//----------------------------------------------------------------------------

void HazardManager::RebuildSnapshot()
{
	m_dirty = false;
	++m_version;

	if (m_instances.empty())
	{
		m_snapshot.reset();
		m_lastPolyCount = 0;
		return;
	}

	auto start = clock::now();
	auto& settings = nav::GetSettings();
	auto snapshot = std::make_shared<HazardSnapshot>();
	snapshot->version = m_version;

	NavMesh* mesh = g_mq2Nav->Get<NavMesh>();
	std::shared_ptr<dtNavMeshQuery> query = mesh && mesh->IsNavMeshLoaded() ? mesh->GetNavMeshQuery() : nullptr;
	dtQueryFilter queryFilter; // all polygons

	if (m_instances.size() > 63)
	{
		SPDLOG_WARN("Hazard: {} hazards tracked; hazards past 63 can't be exited when standing inside them",
			m_instances.size());
	}

	std::vector<glm::vec2> polyVerts;

	for (Instance& inst : m_instances)
	{
		const HazardRule* rule = FindRule(inst.ruleId);
		if (!rule)
			continue;

		HazardEntry entry;
		entry.shape.center = { inst.eqPos.x, inst.eqPos.y };
		entry.shape.facing = hazard::HeadingToRadians(inst.heading);
		entry.shape.arcCenter = hazard::DegToRad(rule->arcCenterDeg);
		entry.shape.arcHalfWidth = rule->IsFullCircle() ? hazard::kPi : hazard::DegToRad(rule->arcWidthDeg * 0.5f);
		entry.hardRadius = inst.hardRadius;
		entry.softRadius = inst.softRadius;
		entry.height = inst.eqPos.z;
		entry.eqPos = inst.eqPos;
		entry.ruleId = inst.ruleId;
		entry.spawnId = inst.spawnId;
		entry.name = inst.name;

		inst.pathedPos = inst.eqPos;
		inst.pathedHeading = inst.heading;
		inst.pathedHard = inst.hardRadius;
		inst.pathedSoft = inst.softRadius;

		size_t index = snapshot->hazards.size();
		uint64_t bit = HazardBit(index);
		snapshot->hazards.push_back(entry);

		if (!query)
			continue;

		const float center[3] = { entry.shape.center.x, entry.height, entry.shape.center.y };
		const float extents[3] = { entry.softRadius, settings.hazard_height_extent, entry.softRadius };

		PolyCollector collector;
		query->queryPolygons(center, extents, &queryFilter, &collector);

		for (const PolyCollector::Item& item : collector.items)
		{
			if (item.poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;

			polyVerts.clear();
			for (int v = 0; v < item.poly->vertCount; ++v)
			{
				const float* pv = &item.tile->verts[item.poly->verts[v] * 3];
				polyVerts.emplace_back(pv[0], pv[2]);
			}

			bool fullyInside = false;
			if (hazard::PolygonOverlapsShape(polyVerts, entry.shape, entry.hardRadius, &fullyInside))
			{
				if (fullyInside)
				{
					snapshot->hardFull[item.ref] |= bit;
				}
				else
				{
					snapshot->hardPartial[item.ref] |= bit;
					float& mult = snapshot->costMult[item.ref];
					mult = std::max({ mult, 1.f, settings.hazard_boundary_cost_mult });
				}
			}
			else if (entry.softRadius > entry.hardRadius
				&& hazard::PolygonOverlapsShape(polyVerts, entry.shape, entry.softRadius, nullptr))
			{
				// graded: the closer to the hard radius, the more expensive
				float dist = hazard::DistancePointPolygon(entry.shape.center, polyVerts);
				float t = std::clamp((entry.softRadius - dist) / (entry.softRadius - entry.hardRadius), 0.f, 1.f);
				float& mult = snapshot->costMult[item.ref];
				mult = std::max({ mult, 1.f, 1.f + settings.hazard_soft_cost_mult * t });
			}
		}
	}

	m_lastPolyCount = snapshot->hardFull.size() + snapshot->hardPartial.size() + snapshot->costMult.size();
	m_lastBuildMs = std::chrono::duration<double, std::milli>(clock::now() - start).count();
	m_snapshot = std::move(snapshot);

	SPDLOG_DEBUG("Hazard: snapshot v{} with {} hazards, {} polys in {:.2f}ms",
		m_version, m_snapshot->hazards.size(), m_lastPolyCount, m_lastBuildMs);
}

//----------------------------------------------------------------------------

void HazardManager::UpdateVisuals()
{
	auto& settings = nav::GetSettings();
	const float heightOffset = 0.5f;

	// in-world overlay (rebuilt with current positions, independent of snapshot hysteresis)
	if (m_overlay)
	{
		m_overlay->Reset();

		if (settings.hazard_show_overlay && !m_instances.empty())
		{
			DebugDrawDX dd(m_overlay.get());
			std::vector<glm::vec2> outline;

			dd.begin(DU_DRAW_LINES, 2.0f);
			for (const Instance& inst : m_instances)
			{
				const HazardRule* rule = FindRule(inst.ruleId);
				if (!rule)
					continue;

				hazard::Shape shape;
				shape.center = { inst.eqPos.x, inst.eqPos.y };
				shape.facing = hazard::HeadingToRadians(inst.heading);
				shape.arcCenter = hazard::DegToRad(rule->arcCenterDeg);
				shape.arcHalfWidth = rule->IsFullCircle() ? hazard::kPi : hazard::DegToRad(rule->arcWidthDeg * 0.5f);
				float y = inst.eqPos.z + heightOffset;

				auto drawOutline = [&](float r, unsigned int color)
				{
					hazard::OutlinePoints(shape, r, 48, outline);
					for (size_t i = 1; i < outline.size(); ++i)
					{
						dd.vertex(outline[i - 1].x, y, outline[i - 1].y, color);
						dd.vertex(outline[i].x, y, outline[i].y, color);
					}
				};

				drawOutline(inst.hardRadius, DXColor(255, 40, 40, 255));
				if (inst.softRadius > inst.hardRadius + 0.1f)
					drawOutline(inst.softRadius, DXColor(255, 220, 0, 200));
			}
			dd.end();
		}
	}

	// map circles (only when the snapshot changes, map lines are comparatively expensive)
	if (!settings.hazard_show_map || m_instances.empty())
	{
		m_mapCircles.clear();
		m_mapVersion = 0;
		return;
	}

	if (m_mapVersion == m_version)
		return;
	m_mapVersion = m_version;

	size_t needed = m_instances.size() * 2;
	while (m_mapCircles.size() < needed)
		m_mapCircles.push_back(std::make_unique<nav::MapCircle>());
	m_mapCircles.resize(needed);

	for (size_t i = 0; i < m_instances.size(); ++i)
	{
		const Instance& inst = m_instances[i];

		nav::MapCircle* hard = m_mapCircles[i * 2].get();
		hard->SetColor(0xffff2828);
		hard->SetLayer(settings.hazard_map_layer);
		hard->SetCircle(inst.eqPos, inst.hardRadius);

		nav::MapCircle* soft = m_mapCircles[i * 2 + 1].get();
		if (inst.softRadius > inst.hardRadius + 0.1f)
		{
			soft->SetColor(0xffffdc00);
			soft->SetLayer(settings.hazard_map_layer);
			soft->SetCircle(inst.eqPos, inst.softRadius);
		}
		else
		{
			soft->Clear();
		}
	}
}

//----------------------------------------------------------------------------

bool HazardManager::IsPlayerInsideHazard(std::string* hazardName) const
{
	auto snapshot = GetHazardSnapshot();
	if (!snapshot)
		return false;

	int index = snapshot->FindContaining(PlayerMeshPos(), nav::GetSettings().hazard_height_extent);
	if (index < 0)
		return false;

	if (hazardName)
		*hazardName = snapshot->hazards[index].name;
	return true;
}

float HazardManager::GetHazardDistance(const glm::vec3& eqPos) const
{
	auto snapshot = GetHazardSnapshot();
	if (!snapshot)
		return FLT_MAX;

	float best = FLT_MAX;
	glm::vec2 p{ eqPos.x, eqPos.y };
	for (const HazardEntry& h : snapshot->hazards)
	{
		if (std::fabs(eqPos.z - h.height) > nav::GetSettings().hazard_height_extent)
			continue;
		if (hazard::PointInShape(h.shape, p, h.hardRadius))
			return -1.f;
		best = std::min(best, hazard::DistanceToShape(h.shape, p, h.hardRadius));
	}
	return best;
}

//============================================================================
// rules

bool HazardManager::ParseRule(std::string_view args, HazardRule& out, std::string& error) const
{
	char line[MAX_STRING] = { 0 };
	strncpy_s(line, args.data(), std::min(args.size(), sizeof(line) - 1));
	char buffer[MAX_STRING] = { 0 };

	HazardRule rule;

	GetArg(buffer, line, 1);
	rule.pattern = buffer;
	if (rule.pattern.empty())
	{
		error = "missing spawn name";
		return false;
	}
	if (rule.pattern.find('|') != std::string::npos)
	{
		error = "spawn name can't contain '|'";
		return false;
	}

	for (int idx = 2;; ++idx)
	{
		GetArg(buffer, line, idx);
		if (!buffer[0])
			break;

		std::string_view arg{ buffer };
		size_t eq = arg.find('=');
		std::string_view key = arg.substr(0, eq);
		std::string_view value = eq == std::string_view::npos ? std::string_view{} : arg.substr(eq + 1);

		if (mq::ci_equals(key, "type"))
		{
			if (mq::ci_equals(value, "ground"))
			{
				error = "type=ground is not supported yet";
				return false;
			}
			if (!ParseSpawnType(value, rule.spawnType))
			{
				error = fmt::format("unknown type '{}'", value);
				return false;
			}
		}
		else if (mq::ci_equals(key, "hard") || mq::ci_equals(key, "radius"))
		{
			if (!ParseRadius(value, rule.hard)) { error = fmt::format("bad radius '{}'", value); return false; }
		}
		else if (mq::ci_equals(key, "soft"))
		{
			if (!ParseRadius(value, rule.soft)) { error = fmt::format("bad radius '{}'", value); return false; }
		}
		else if (mq::ci_equals(key, "arc"))
		{
			if (!ParseArc(value, rule.arcCenterDeg, rule.arcWidthDeg))
			{
				error = fmt::format("bad arc '{}' (front|back|left|right|<centerDeg>,<widthDeg>)", value);
				return false;
			}
		}
		else if (mq::ci_equals(key, "inside"))
		{
			if (mq::ci_equals(value, "flee")) rule.inside = HazardInsideAction::Flee;
			else if (mq::ci_equals(value, "hold")) rule.inside = HazardInsideAction::Hold;
			else { error = fmt::format("bad inside '{}' (flee|hold)", value); return false; }
		}
		else if (mq::ci_equals(key, "sticky"))
		{
			if (!ParseBool(value, rule.sticky)) { error = fmt::format("bad sticky '{}'", value); return false; }
		}
		else
		{
			error = fmt::format("unknown option '{}'", arg);
			return false;
		}
	}

	out = rule;
	return true;
}

int HazardManager::AddRule(HazardRule rule)
{
	rule.id = m_nextRuleId++;
	m_rules.push_back(rule);
	m_rescanRequested = true;

	if (!rule.transient)
		SaveRules();

	return rule.id;
}

bool HazardManager::RemoveRule(int id)
{
	auto it = std::find_if(m_rules.begin(), m_rules.end(), [id](const HazardRule& r) { return r.id == id; });
	if (it == m_rules.end())
		return false;

	bool persisted = !it->transient;
	m_rules.erase(it);
	m_instances.erase(std::remove_if(m_instances.begin(), m_instances.end(),
		[id](const Instance& inst) { return inst.ruleId == id; }), m_instances.end());
	m_dirty = true;

	if (persisted)
		SaveRules();
	return true;
}

int HazardManager::RemoveRulesByPattern(std::string_view pattern)
{
	std::vector<int> ids;
	for (const HazardRule& rule : m_rules)
	{
		if (mq::ci_equals(rule.pattern, pattern))
			ids.push_back(rule.id);
	}
	for (int id : ids)
		RemoveRule(id);
	return static_cast<int>(ids.size());
}

void HazardManager::ClearRules()
{
	m_rules.clear();
	m_instances.clear();
	m_dirty = true;
	SaveRules();
}

bool HazardManager::SetRuleEnabled(int id, bool enabled)
{
	for (HazardRule& rule : m_rules)
	{
		if (rule.id == id)
		{
			rule.enabled = enabled;
			m_rescanRequested = true;
			m_dirty = true;
			if (!rule.transient)
				SaveRules();
			return true;
		}
	}
	return false;
}

std::string HazardManager::DescribeRule(const HazardRule& rule) const
{
	std::string text = fmt::format("\"{}\"", rule.pattern);
	if (rule.spawnType != -1)
		text += fmt::format(" type={}", SpawnTypeToString(rule.spawnType));
	text += fmt::format(" hard={} soft={}", RadiusToString(rule.hard), RadiusToString(rule.soft));
	if (!rule.IsFullCircle())
		text += fmt::format(" arc={:g},{:g}", rule.arcCenterDeg, rule.arcWidthDeg);
	if (rule.inside == HazardInsideAction::Hold)
		text += " inside=hold";
	if (!rule.sticky)
		text += " sticky=off";
	return text;
}

//----------------------------------------------------------------------------
// persistence: [Hazards:<shortzone>] RuleN=mode|pattern|type|hard|soft|arcCenter|arcWidth|inside|sticky|enabled

std::string HazardManager::SerializeRule(const HazardRule& rule) const
{
	return fmt::format("{}|{}|{}|{}|{}|{:g}|{:g}|{}|{}|{}",
		rule.mode == HazardMode::Avoid ? "avoid" : "seek",
		rule.pattern,
		SpawnTypeToString(rule.spawnType),
		RadiusToString(rule.hard),
		RadiusToString(rule.soft),
		rule.arcCenterDeg,
		rule.arcWidthDeg,
		rule.inside == HazardInsideAction::Hold ? "hold" : "flee",
		rule.sticky ? 1 : 0,
		rule.enabled ? 1 : 0);
}

bool HazardManager::DeserializeRule(std::string_view data, HazardRule& out) const
{
	std::vector<std::string_view> parts = mq::split_view(data, '|');
	if (parts.size() < 10)
		return false;

	HazardRule rule;
	if (mq::ci_equals(parts[0], "avoid")) rule.mode = HazardMode::Avoid;
	else if (mq::ci_equals(parts[0], "seek")) rule.mode = HazardMode::Seek;
	else return false;

	rule.pattern = std::string(parts[1]);
	if (rule.pattern.empty() || !ParseSpawnType(parts[2], rule.spawnType))
		return false;
	if (!ParseRadius(parts[3], rule.hard) || !ParseRadius(parts[4], rule.soft))
		return false;

	rule.arcCenterDeg = GetFloatFromString(parts[5], 0.f);
	rule.arcWidthDeg = std::clamp(GetFloatFromString(parts[6], 360.f), 1.f, 360.f);
	rule.inside = mq::ci_equals(parts[7], "hold") ? HazardInsideAction::Hold : HazardInsideAction::Flee;
	rule.sticky = parts[8] != "0";
	rule.enabled = parts[9] != "0";

	out = rule;
	return true;
}

void HazardManager::LoadRules(int zoneId)
{
	m_zoneId = zoneId;
	m_rules.clear();
	m_nextRuleId = 1;
	m_rescanRequested = true;
	m_dirty = true;

	if (zoneId == -1)
		return;

	std::string section = ZoneSection(zoneId);
	char keys[MAX_STRING * 10] = { 0 };
	char value[MAX_STRING] = { 0 };

	if (!::GetPrivateProfileStringA(section.c_str(), nullptr, "", keys, sizeof(keys), INIFileName))
		return;

	for (const char* key = keys; *key; key += strlen(key) + 1)
	{
		::GetPrivateProfileStringA(section.c_str(), key, "", value, sizeof(value), INIFileName);

		HazardRule rule;
		if (DeserializeRule(value, rule))
		{
			rule.id = m_nextRuleId++;
			m_rules.push_back(rule);
		}
		else
		{
			SPDLOG_WARN("Hazard: ignoring invalid rule {}={} in [{}]", key, value, section);
		}
	}

	if (!m_rules.empty())
		SPDLOG_INFO("Loaded {} hazard rule(s) for {}", m_rules.size(), GetShortZone(zoneId));
}

void HazardManager::SaveRules()
{
	if (m_zoneId == -1)
		return;

	std::string section = ZoneSection(m_zoneId);

	// rewrite the whole section so removed rules disappear
	std::string data;
	int n = 1;
	for (const HazardRule& rule : m_rules)
	{
		if (rule.transient)
			continue;
		data += fmt::format("Rule{}={}", n++, SerializeRule(rule));
		data.push_back('\0');
	}

	if (data.empty())
	{
		::WritePrivateProfileStringA(section.c_str(), nullptr, nullptr, INIFileName);
	}
	else
	{
		data.push_back('\0');
		::WritePrivateProfileSectionA(section.c_str(), data.c_str(), INIFileName);
	}
}

//============================================================================
// commands

void HazardManager::PrintRules() const
{
	if (m_rules.empty())
	{
		WriteChatf(PLUGIN_MSG "No hazard rules for \ag%s\ax.", GetShortZone(m_zoneId));
		return;
	}

	glm::vec3 me = GetMyPosition();
	WriteChatf(PLUGIN_MSG "Hazard rules for \ag%s\ax:", GetShortZone(m_zoneId));
	for (const HazardRule& rule : m_rules)
	{
		WriteChatf(PLUGIN_MSG "  \at%d\ax: avoid %s%s%s", rule.id, DescribeRule(rule).c_str(),
			rule.enabled ? "" : " \ar[disabled]\ax", rule.transient ? " \a-w[api]\ax" : "");

		for (const Instance& inst : m_instances)
		{
			if (inst.ruleId != rule.id)
				continue;

			float dist = glm::distance(glm::vec2(me.x, me.y), glm::vec2(inst.eqPos.x, inst.eqPos.y));
			WriteChatf(PLUGIN_MSG "      \ay%s\ax (%u) dist \ag%.1f\ax hard %.1f soft %.1f%s%s",
				inst.name.c_str(), inst.spawnId, dist, inst.hardRadius, inst.softRadius,
				dist <= inst.hardRadius ? " \ar[INSIDE]\ax" : "",
				inst.renamed ? " \ao[renamed]\ax" : "");
		}
	}
}

void HazardManager::PrintStatus() const
{
	auto& settings = nav::GetSettings();
	std::string insideName;
	bool inside = IsPlayerInsideHazard(&insideName);

	WriteChatf(PLUGIN_MSG "Hazards: pathing \a%s%s\ax, %d rule(s), %d tracked, snapshot v%llu (%d polys, %.2fms)",
		settings.hazards_enabled ? "g" : "r", settings.hazards_enabled ? "on" : "off",
		static_cast<int>(m_rules.size()), static_cast<int>(m_instances.size()),
		m_version, static_cast<int>(m_lastPolyCount), m_lastBuildMs);
	if (inside)
		WriteChatf(PLUGIN_MSG "  You are \arINSIDE\ax %s", insideName.c_str());
}

static void HazardHelp()
{
	WriteChatf(PLUGIN_MSG "\aoHazard Commands:\ax");
	WriteChatf(PLUGIN_MSG "\ag/nav avoid add \"<name>\" [type=npc|untargetable|aura|object|...] [hard=<n>|rampage] [soft=<n>|rampage] [arc=front|back|left|right|<center>,<width>] [sticky=on|off]\ax");
	WriteChatf(PLUGIN_MSG "    avoid every spawn whose name contains <name>. hard = never enter, soft = keep this far if possible.");
	WriteChatf(PLUGIN_MSG "\ag/nav avoid remove <id|\"name\">\ax, \ag/nav avoid list\ax, \ag/nav avoid clear\ax, \ag/nav avoid enable|disable <id>\ax");
	WriteChatf(PLUGIN_MSG "\ag/nav hazard status|rescan\ax, \ag/nav hazard pathing on|off\ax, \ag/nav hazard show on|off\ax, \ag/nav hazard map on|off\ax");
	WriteChatf(PLUGIN_MSG "  Navigation option \aynoavoid\ax ignores hazards for one command.");
}

bool HazardManager::HandleCommand(const char* line)
{
	char mutableLine[MAX_STRING] = { 0 };
	strncpy_s(mutableLine, line, MAX_STRING - 1);
	char cmd[MAX_STRING] = { 0 };
	char sub[MAX_STRING] = { 0 };
	char arg[MAX_STRING] = { 0 };

	GetArg(cmd, mutableLine, 1);
	GetArg(sub, mutableLine, 2);
	GetArg(arg, mutableLine, 3);

	auto& settings = nav::GetSettings();

	if (!_stricmp(cmd, "avoid"))
	{
		if (!_stricmp(sub, "add"))
		{
			const char* rest = GetNextArg(mutableLine, 2);
			HazardRule rule;
			std::string error;
			if (!ParseRule(rest ? rest : "", rule, error))
			{
				SPDLOG_ERROR("Invalid hazard rule: {}", error);
				HazardHelp();
				return true;
			}

			int id = AddRule(rule);
			SPDLOG_INFO("Added hazard rule {}: avoid {}", id, DescribeRule(rule));
			return true;
		}

		if (!_stricmp(sub, "remove") || !_stricmp(sub, "delete"))
		{
			if (!arg[0])
			{
				SPDLOG_ERROR("Usage: /nav avoid remove <id|\"name\">");
				return true;
			}

			if (IsNumber(arg))
			{
				if (RemoveRule(GetIntFromString(arg, 0)))
					SPDLOG_INFO("Removed hazard rule {}", arg);
				else
					SPDLOG_ERROR("No hazard rule with id {}", arg);
			}
			else
			{
				int removed = RemoveRulesByPattern(arg);
				SPDLOG_INFO("Removed {} hazard rule(s) for \"{}\"", removed, arg);
			}
			return true;
		}

		if (!_stricmp(sub, "enable") || !_stricmp(sub, "disable"))
		{
			bool enable = !_stricmp(sub, "enable");
			if (!SetRuleEnabled(GetIntFromString(arg, 0), enable))
				SPDLOG_ERROR("No hazard rule with id {}", arg);
			return true;
		}

		if (!_stricmp(sub, "clear"))
		{
			ClearRules();
			SPDLOG_INFO("Cleared hazard rules for {}", GetShortZone(m_zoneId));
			return true;
		}

		if (!_stricmp(sub, "list") || !sub[0])
		{
			PrintRules();
			return true;
		}

		HazardHelp();
		return true;
	}

	if (!_stricmp(cmd, "hazard") || !_stricmp(cmd, "hazards"))
	{
		bool value = false;

		if (!_stricmp(sub, "rescan"))
		{
			m_rescanRequested = true;
			SPDLOG_INFO("Rescanning spawns for hazards");
			return true;
		}

		if (!_stricmp(sub, "pathing") && ParseBool(arg, value))
		{
			settings.hazards_enabled = value;
			nav::SaveSettings(false);
			SPDLOG_INFO("Hazard pathing {}", value ? "enabled" : "disabled");
			return true;
		}

		if (!_stricmp(sub, "show") && ParseBool(arg, value))
		{
			settings.hazard_show_overlay = value;
			nav::SaveSettings(false);
			UpdateVisuals();
			return true;
		}

		if (!_stricmp(sub, "map") && ParseBool(arg, value))
		{
			settings.hazard_show_map = value;
			nav::SaveSettings(false);
			m_mapVersion = 0;
			UpdateVisuals();
			return true;
		}

		if (!_stricmp(sub, "status") || !sub[0])
		{
			PrintStatus();
			return true;
		}

		HazardHelp();
		return true;
	}

	return false;
}

//============================================================================
// debug ui

void HazardManager::DebugUI()
{
	auto& settings = nav::GetSettings();
	bool changed = false;

	changed |= ImGui::Checkbox("Apply hazards to pathing", &settings.hazards_enabled);
	changed |= ImGui::Checkbox("Show overlay", &settings.hazard_show_overlay);
	ImGui::SameLine();
	if (ImGui::Checkbox("Show on map", &settings.hazard_show_map))
	{
		m_mapVersion = 0;
		changed = true;
	}

	ImGui::LabelText("Snapshot", "v%llu, %d hazards, %d polys, %.2f ms", m_version,
		m_snapshot ? static_cast<int>(m_snapshot->hazards.size()) : 0,
		static_cast<int>(m_lastPolyCount), m_lastBuildMs);

	std::string insideName;
	ImGui::LabelText("Player", "%s", IsPlayerInsideHazard(&insideName)
		? fmt::format("INSIDE {}", insideName).c_str() : "clear");

	if (pTarget && ImGui::Button("Avoid current target"))
	{
		HazardRule rule;
		rule.pattern = pTarget->DisplayedName;
		int id = AddRule(rule);
		SPDLOG_INFO("Added hazard rule {}: avoid {}", id, DescribeRule(rule));
	}

	ImGui::Separator();

	int removeId = 0;
	if (ImGui::BeginTable("##HazardRules", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit))
	{
		ImGui::TableSetupColumn("On");
		ImGui::TableSetupColumn("Id");
		ImGui::TableSetupColumn("Rule", ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableSetupColumn("Tracked");
		ImGui::TableSetupColumn("");
		ImGui::TableHeadersRow();

		for (HazardRule& rule : m_rules)
		{
			ImGui::PushID(rule.id);
			ImGui::TableNextRow();

			ImGui::TableNextColumn();
			bool enabled = rule.enabled;
			if (ImGui::Checkbox("##enabled", &enabled))
				SetRuleEnabled(rule.id, enabled);

			ImGui::TableNextColumn();
			ImGui::Text("%d", rule.id);

			ImGui::TableNextColumn();
			ImGui::TextUnformatted(DescribeRule(rule).c_str());

			ImGui::TableNextColumn();
			int count = static_cast<int>(std::count_if(m_instances.begin(), m_instances.end(),
				[&](const Instance& inst) { return inst.ruleId == rule.id; }));
			ImGui::Text("%d", count);

			ImGui::TableNextColumn();
			if (ImGui::SmallButton("Remove"))
				removeId = rule.id;

			ImGui::PopID();
		}
		ImGui::EndTable();
	}

	if (removeId)
		RemoveRule(removeId);

	if (!m_instances.empty() && ImGui::TreeNode("Tracked spawns"))
	{
		glm::vec3 me = GetMyPosition();
		for (const Instance& inst : m_instances)
		{
			float dist = glm::distance(glm::vec2(me.x, me.y), glm::vec2(inst.eqPos.x, inst.eqPos.y));
			ImGui::Text("%s (%u) rule %d  dist %.1f  hard %.1f  soft %.1f%s%s", inst.name.c_str(), inst.spawnId,
				inst.ruleId, dist, inst.hardRadius, inst.softRadius,
				dist <= inst.hardRadius ? "  INSIDE" : "", inst.renamed ? "  renamed" : "");
		}
		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Tuning"))
	{
		changed |= ImGui::DragFloat("Default hard radius", &settings.hazard_default_hard, 0.5f, 1.f, 500.f, "%.1f");
		changed |= ImGui::DragFloat("Default soft radius", &settings.hazard_default_soft, 0.5f, 1.f, 500.f, "%.1f");
		changed |= ImGui::DragFloat("Rampage margin", &settings.hazard_rampage_margin, 0.5f, 0.f, 100.f, "%.1f");
		changed |= ImGui::DragFloat("Soft cost multiplier", &settings.hazard_soft_cost_mult, 0.1f, 0.f, 50.f, "%.1f");
		changed |= ImGui::DragFloat("Boundary cost multiplier", &settings.hazard_boundary_cost_mult, 0.1f, 1.f, 50.f, "%.1f");
		changed |= ImGui::DragFloat("Inside cost multiplier", &settings.hazard_inside_cost_mult, 0.5f, 1.f, 200.f, "%.1f");
		changed |= ImGui::DragFloat("Height extent", &settings.hazard_height_extent, 0.5f, 1.f, 200.f, "%.1f");
		changed |= ImGui::DragInt("Refresh (ms)", &settings.hazard_refresh_ms, 10.f, 50, 5000);
		changed |= ImGui::DragInt("Rescan (ms)", &settings.hazard_rescan_ms, 10.f, 100, 10000);
		changed |= ImGui::DragFloat("Move epsilon", &settings.hazard_move_epsilon, 0.1f, 0.f, 20.f, "%.1f");
		changed |= ImGui::DragInt("Post-check iterations", &settings.hazard_postcheck_iterations, 0.2f, 0, 16);
		if (changed)
			m_dirty = true;
		ImGui::TreePop();
	}

	if (changed)
		nav::SaveSettings(false);
}
