//
// NavigationPath.h
//

#pragma once

#include "common/Utilities.h"
#include "plugin/MQ2Navigation.h"
#include "plugin/Renderable.h"
#include "plugin/RenderList.h"

#include <DetourNavMeshQuery.h>
#include <DetourPathCorridor.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <mq/base/Signal.h>
#include <d3d9.h>

#include <imgui.h>
#include <memory>
#include <string>
#include <unordered_set>

#define DEBUG_NAVIGATION_LINES 1

//----------------------------------------------------------------------------

class NavMesh;
class NavigationLine;
struct DestinationInfo;
struct ID3D11Buffer;

struct StraightPath
{
	StraightPath() = default;
	StraightPath(int size) { Reset(size); }

	std::unique_ptr<glm::vec3[]> verts;
	std::unique_ptr<uint8_t[]> flags;
	std::unique_ptr<dtPolyRef[]> polys;
	int length = 0;
	int cursor = 0;

	void Reset(int size);
	int GetAllocatedSize() { return alloc; }

	using Node = std::tuple<glm::vec3, uint8_t, dtPolyRef>;

	inline Node GetNode(int pos) const
	{
		if (pos >= length)
			return { {}, 0, 0 };

		return { verts[pos], flags[pos], polys[pos] };
	}

private:
	int alloc = 0;
};

struct HazardSnapshot;

// Query filter that can restrict a path to navmesh polygons within a 2D radius of an origin
// (e.g. keep a pull path inside the camp radius), and applies hazards: polygons fully inside a hard
// hazard radius are rejected, polygons near hazards cost more. Requires DT_VIRTUAL_QUERYFILTER so
// Detour calls the overrides.
class NavPathFilter : public dtQueryFilter
{
private:
	int m_searchRadius = 0;            // 0 = no radius restriction
	glm::vec3 m_searchOrigin{ 0.f };   // navmesh coordinates

	// hazards (null = none)
	std::shared_ptr<const HazardSnapshot> m_hazards;
	uint64_t m_demotedMask = 0;        // hazards the path starts inside: passable, but expensive
	float m_insideCostMult = 1.f;
	dtPolyRef m_exemptStart = 0;
	dtPolyRef m_exemptEnd = 0;
	std::unordered_set<dtPolyRef> m_extraBlocked; // added by the post-check for this path

	// ledge drop links (Drop / LevDrop off-mesh connections)
	bool m_dropLinksEnabled = true;
	bool m_dropsRequireLevitate = false;
	bool m_levitating = false;
	float m_maxSafeDrop = 20.f;

	bool IsPolyWithinRadius(const dtMeshTile* tile, const dtPoly* poly) const;

public:
	// may this ledge-drop link be taken under the current drop rules?
	bool IsDropAllowed(const dtMeshTile* tile, const dtPoly* poly) const;

	NavPathFilter() = default;
	~NavPathFilter() override = default;

	bool passFilter(const dtPolyRef ref, const dtMeshTile* tile, const dtPoly* poly) const override;
	float getCost(const float* pa, const float* pb,
		const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
		const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
		const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const override;

	void SetSearchRadius(int radius) { m_searchRadius = radius; }
	void SetSearchOrigin(const glm::vec3& pos) { m_searchOrigin = pos; }

	void SetHazards(std::shared_ptr<const HazardSnapshot> hazards, uint64_t demotedMask = 0, float insideCostMult = 1.f);
	void SetExemptPolys(dtPolyRef start, dtPolyRef end) { m_exemptStart = start; m_exemptEnd = end; }
	void AddExtraBlocked(dtPolyRef ref) { m_extraBlocked.insert(ref); }
	void ClearPathLocalState();

	void SetDropRules(bool enabled, bool levitating, float maxSafeDrop, bool requireLevitate)
	{
		m_dropLinksEnabled = enabled;
		m_levitating = levitating;
		m_maxSafeDrop = maxSafeDrop;
		m_dropsRequireLevitate = requireLevitate;
	}
};

class NavigationPath
{
	friend class NavigationLine;

public:
	NavigationPath(const std::shared_ptr<DestinationInfo>& dest);
	~NavigationPath();

	void SetDestination(const std::shared_ptr<DestinationInfo>& info);
	std::shared_ptr<DestinationInfo> GetDestinationInfo() const { return m_destinationInfo; }

	// try to find a path to the current destination. Returns true if a path has been found.
	bool FindPath();

	// trigger a recalculation of the path towards the destination. Returns true if the path
	// has been changed.
	void UpdatePath(bool force = false, bool incremental = false);

	void SetShowNavigationPaths(bool renderPaths);

	//----------------------------------------------------------------------------

	// get the destination point this path is navigating to.
	glm::vec3 GetDestination() const;

	// get the full length of the path as traversed
	float GetPathTraversalDistance() const;

	// Get the number of nodes in the path and the index of the current node
	// along that path.
	int GetPathSize() const { return m_currentPath->length; }
	int GetPathIndex() const { return m_currentPath->cursor; }

	// Check if we are at the end if our path
	inline bool IsAtEnd() const
	{
		return m_currentPath->cursor >= m_currentPath->length
			|| m_currentPath->length <= 0;
	}

	inline glm::vec3 GetNextPosition() const
	{
		if (m_currentPath->cursor < m_currentPath->length)
			return GetPosition(m_currentPath->cursor);

		return glm::vec3{};
	}

	// get the coordinates of a point in a raw float3 form
	inline const float* GetRawPosition(int index) const
	{
		assert(index < m_currentPath->length);
		return glm::value_ptr(m_currentPath->verts[index]);
	}

	// get the coordinates in silly eq coordinates
	inline glm::vec3 GetPosition(int index) const
	{
		assert(index < m_currentPath->length);
		return m_currentPath->verts[index];
	}

	inline auto GetNode(int index) const
	{
		return m_currentPath->GetNode(index);
	}

	void Increment();

	// render path is a list of node indexes into the current
	// path that should be used to render current path. index of
	// -1 represents the current location and not a node.
	const std::vector<int>& GetRenderPath() const { return m_renderPath; }

	bool CanSeeDestination() const;

	dtNavMesh* GetNavMesh() const { return m_navMesh.get(); }
	dtNavMeshQuery* GetNavMeshQuery() const { return m_query.get(); }

	bool IsFailed() const { return m_failed; }

	// the last path computation failed because of a hazard (destination inside one, or no route around)
	bool IsHazardBlocked() const { return m_hazardBlocked; }
	const std::string& GetHazardBlockReason() const { return m_hazardBlockReason; }

	mq::Signal<> PathUpdated;
	mq::Signal<> RenderPathUpdated;

private:
	void SetNavMesh(const std::shared_ptr<dtNavMesh>& navMesh,
		bool updatePath = true);

	std::unique_ptr<StraightPath> RecomputePath(
		const glm::vec3& startPos,
		const glm::vec3& endPos,
		bool force,
		bool incremental);
	void UpdatePathProperties();

	// does the current path still contain a ledge drop the current rules forbid?
	bool PathHasForbiddenDrop() const;

	// Insert a waypoint after `segment` that bends the path around hazard `hazardIndex` without
	// crossing any hard radius. Returns false if no walkable detour point was found.
	bool TryHazardDetour(StraightPath& path, int segment, const HazardSnapshot& hazards,
		int hazardIndex, uint64_t skipMask, const glm::vec3& extents);

private:
	std::shared_ptr<DestinationInfo> m_destinationInfo;

	std::unique_ptr<RenderGroup> m_debugDrawGrp;
	glm::vec3 m_destination;
	glm::vec3 m_lastPos;

	bool m_failed = false;
	bool m_hazardBlocked = false;
	std::string m_hazardBlockReason;
	uint64_t m_hazardVersion = 0;
	uint64_t m_dropRulesKey = 0;         // drop link rules the current path was computed with

	// the plugin owns the mesh
	std::shared_ptr<dtNavMesh> m_navMesh;

	// we own the query
	deleting_unique_ptr<dtNavMeshQuery> m_query;

	std::unique_ptr<StraightPath> m_currentPath;

	bool m_renderPaths;
	std::shared_ptr<NavigationLine> m_line;

	// path for debug rendering
	std::vector<int> m_renderPath;
	bool m_followingLink = false;

	NavPathFilter m_filter;
	glm::vec3 m_extents = { 5, 10, 5 }; // note: X, Z, Y

	mq::Signal<>::ScopedConnection m_navMeshConn;
};

//----------------------------------------------------------------------------

class NavigationLine : public Renderable
{
public:
	NavigationLine();
	virtual ~NavigationLine();

	void SetNavigationPath(NavigationPath* path);

	void SetThickness(float thickness);
	float GetThickness() const { return m_thickness; }
	void SetVisible(bool visible);
	void SetCurrentPos(const glm::vec3& currentPos);

	void Update();

	virtual void Render() override;
	virtual bool CreateDeviceObjects() override;
	virtual void InvalidateDeviceObjects() override;

	void ReleasePath();
	void GenerateBuffers();

	struct LineStyle
	{
		ImColor borderColor{ 0, 0, 0, 0 };
		ImColor hiddenColor{ 52, 152, 219, 0 };
		ImColor visibleColor{ 241, 196, 15, 0 };
		ImColor linkColor{ 152, 52, 219, 0 };
		float opacity = .80f;
		float hiddenOpacity = 0.60f;
		float borderWidth = 0.2f;
		float lineWidth = 0.9f;
	};

private:
	NavigationPath* m_path = nullptr;

	// The vertex structure we'll be using for line drawing. Each line is defined as two vertices,
	// and the vertex shader will create a quad from these two vertices. However, since the vertex
	// shader can only process one vertex at a time, we need to store information in each vertex
	// about the other vertex (the other end of the line).
	struct TVertex
	{
		glm::vec3    pos;
		glm::vec3    otherPos;
		glm::vec3    adjPos; // position of previous or next line segment, depending on the vertex
		float        thickness;
		float        adjHint;
		float        type;
	};

#if HAS_DIRECTX_9
	IDirect3DVertexBuffer9* m_vertexBuffer = nullptr;
#elif HAS_DIRECTX_11
	ID3D11Buffer* m_vertexBuffer = nullptr;
#endif

	struct RenderCommand
	{
		UINT StartVertex;
		UINT PrimitiveCount;
	};
	std::vector<RenderCommand> m_commands;
	int m_lastSize = 0;

	bool m_loaded = false;
	bool m_needsUpdate = false;
	float m_thickness = 0.25f;
	bool m_visible = false;
	glm::vec3 m_startPos;

	mq::Signal<>::ScopedConnection m_pathUpdated;
};

extern NavigationLine::LineStyle gNavigationLineStyle;

