//
// NavigationPath.cpp
//

#include "pch.h"
#include "NavigationPath.h"

#include "common/Logging.h"
#include "common/NavMesh.h"
#include "common/NavMeshData.h"
#include "common/Utilities.h"
#include "plugin/DebugDrawDX.h"
#include "plugin/HazardManager.h"
#include "plugin/MQ2Navigation.h"
#include "plugin/PluginSettings.h"
#include "plugin/NavMeshLoader.h"
#include "plugin/RenderHandler.h"

#include <DetourNavMesh.h>
#include <DetourCommon.h>
#include <DetourNode.h>
#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>


//----------------------------------------------------------------------------
// constants

const int MAX_STRAIGHT_PATH_LENGTH = 16384;

const float NODE_POOL_GROWTH_FACTOR = 1.5f;
const int NODE_POOL_MAX_SIZE = 1024 * 1024; // the zone is insanely large if this gets hit

NavigationLine::LineStyle gNavigationLineStyle;

//----------------------------------------------------------------------------

void StraightPath::Reset(int size_)
{
	cursor = 0;
	length = 0;

	if (alloc == size_)
		return;

	alloc = size_;

	if (alloc == 0)
	{
		verts.reset();
		flags.reset();
		polys.reset();
	}
	else
	{
		verts = std::make_unique<glm::vec3[]>(alloc);
		flags = std::make_unique<uint8_t[]>(alloc);
		polys = std::make_unique<dtPolyRef[]>(alloc);
	}
}

//----------------------------------------------------------------------------

NavigationPath::NavigationPath(const std::shared_ptr<DestinationInfo>& dest)
	: m_currentPath(std::make_unique<StraightPath>())
{
	auto* mesh = g_mq2Nav->Get<NavMesh>();
	m_navMeshConn = mesh->OnNavMeshChanged.Connect(
		[this, mesh]() { SetNavMesh(mesh->GetNavMesh()); });

	m_renderPaths = nav::GetSettings().show_nav_path;

	SetNavMesh(mesh->GetNavMesh(), false);
	SetDestination(dest);
}

NavigationPath::~NavigationPath()
{
	SetShowNavigationPaths(false);
}

//----------------------------------------------------------------------------

void NavigationPath::SetShowNavigationPaths(bool renderPaths)
{
	if (m_line)
	{
		m_line->SetVisible(renderPaths);
	}

	if (m_renderPaths == renderPaths && (m_renderPaths == (m_line != nullptr)))
		return;

	m_renderPaths = renderPaths;

	if (m_renderPaths)
	{
		m_line = g_mq2Nav->GetGameLine();
		m_line->SetVisible(m_renderPaths);
		m_line->SetCurrentPos(m_lastPos);
	}
	else
	{
		if (m_debugDrawGrp)
			g_renderHandler->RemoveRenderable(m_debugDrawGrp.get());

		if (m_line)
		{
			m_line->SetVisible(false);
			m_line.reset();
		}
		m_debugDrawGrp.reset();
	}
}

void NavigationPath::SetDestination(const std::shared_ptr<DestinationInfo>& info)
{
	m_destinationInfo = info;
}

bool NavigationPath::FindPath()
{
	if (!m_navMesh)
		return false;

	if (!m_destinationInfo || !m_destinationInfo->valid)
		return false;

	// Reset the path in case of failure
	m_currentPath->Reset(0);

	UpdatePath(true);

	return m_currentPath->length > 0;
}

void NavigationPath::Increment()
{
	++m_currentPath->cursor;

	UpdatePathProperties();
}

bool NavigationPath::CanSeeDestination() const
{
	if (!m_query)
	{
		return false;
	}

	// if less than two nodes, we're basically already on the destination.
	if (m_currentPath->length < 2)
	{
		return true;
	}

	// Get current and last poly/verts
	auto curr = m_currentPath->GetNode(0);
	auto last = m_currentPath->GetNode(m_currentPath->length - 1);

	dtRaycastHit hit;
	std::memset(&hit, 0, sizeof(hit));

	dtStatus result = m_query->raycast(
		std::get<dtPolyRef>(curr),
		glm::value_ptr(std::get<glm::vec3>(curr)),
		glm::value_ptr(std::get<glm::vec3>(last)),
		&m_filter,
		0,
		&hit);

	// line of sight if no hit
	return dtStatusSucceed(result) && hit.t == FLT_MAX;
}

void NavigationPath::SetNavMesh(const std::shared_ptr<dtNavMesh>& navMesh,
	bool updatePath)
{
	m_navMesh = navMesh;

	m_query.reset();

	m_filter = NavPathFilter{};
	m_filter.setIncludeFlags(+PolyFlags::All);
	m_filter.setExcludeFlags(+PolyFlags::Disabled);
	if (auto* mesh = g_mq2Nav->Get<NavMesh>())
	{
		mesh->FillFilterAreaCosts(m_filter);
	}

	if (updatePath && m_navMesh)
	{
		UpdatePath();
	}
}

void NavigationPath::UpdatePath(bool force, bool incremental)
{
	if (m_navMesh == nullptr || m_destinationInfo == nullptr)
		return;

	// don't perform incremental update if updates are disabled
	if (incremental && !nav::GetSettings().poll_navigation_path)
		return;

	// don't perform incremental update if we're following an off-mesh link
	if (incremental && m_followingLink)
		return;

	if (m_query == nullptr)
	{
		m_query = deleting_unique_ptr<dtNavMeshQuery>(dtAllocNavMeshQuery(),
			[](dtNavMeshQuery* ptr) { dtFreeNavMeshQuery(ptr); });

		m_query->init(m_navMesh.get(), NAVMESH_QUERY_MAX_NODES);
	}

	PSPAWNINFO me = GetCharInfo()->pSpawn;
	if (me == nullptr)
		return;

	// current position in mesh coordinates
	glm::vec3 thisPos{ me->X, me->FloorHeight, me->Y };

	// a change in hazards (added, moved, removed) also requires a new path
	if (thisPos == m_lastPos && m_destination == m_destinationInfo->eqDestinationPos && !force
		&& (!m_destinationInfo->options.avoidHazards || m_hazardVersion == GetHazardVersion()))
	{
		return;
	}

	m_destination = m_destinationInfo->eqDestinationPos;
	m_lastPos = thisPos;

	// convert destination to mesh coordinates
	glm::vec3 dest = m_destination;
	std::swap(dest.y, dest.z);

	std::unique_ptr<StraightPath> newPath = RecomputePath(thisPos, dest, force, incremental);
	bool changed = false;

	if (newPath)
	{
		m_currentPath = std::move(newPath);
		changed = true;
	}
	else if (!incremental || m_hazardBlocked)
	{
		// not an incremental update (or a hazard now blocks the way), and no new path, so make sure
		// there is no current path. Keeping the old path would walk straight into the hazard.
		m_currentPath->Reset(0);
		changed = true;
	}

	UpdatePathProperties();

	if (changed)
	{
		PathUpdated();
	}

	// Update render path
	int renderSize = 1 + m_currentPath->length - m_currentPath->cursor;
	std::vector<int> renderPath;
	renderPath.resize(renderSize);

	renderPath[0] = -1;
	if (renderSize > 1)
	{
		for (int i = 0, node = m_currentPath->cursor; node != m_currentPath->length; ++node, ++i)
			renderPath[i + 1] = node;
	}
	m_renderPath = renderPath;

	if (m_renderPaths && !m_debugDrawGrp && gpD3D9Device)
	{
		m_debugDrawGrp = std::make_unique<RenderGroup>();
		g_renderHandler->AddRenderable(m_debugDrawGrp.get());
	}

	// Give the path debug an update
	if (m_debugDrawGrp)
	{
		m_debugDrawGrp->Reset();

		auto& settings = nav::GetSettings();

		if (m_debugDrawGrp && settings.debug_render_pathing)
		{
			DebugDrawDX dd(m_debugDrawGrp.get());

			// draw current position
			duDebugDrawCross(&dd, me->X, me->FloorHeight, me->Y, 0.5, DXColor(51, 255, 255), 1);

			// Draw the waypoints. Green is next point
			for (int i = 0; i < m_currentPath->length; ++i)
			{
				int color = DXColor(255, 255, 255);
				if (i < m_currentPath->cursor)
					color = DXColor(0, 102, 204);
				if (i == m_currentPath->cursor)
					color = DXColor(0, 255, 0);
				if (i == m_currentPath->length - 1)
					color = DXColor(255, 0, 0);

				const glm::vec3& pos = m_currentPath->verts[i];

				duDebugDrawCross(&dd, pos.x, pos.y, pos.z, 0.5, color, 1);
			}
		}
	}

	// update the pathing line
	if (m_line)
	{
		// Update render line initial position
		m_line->SetCurrentPos(m_lastPos);
		m_line->Update();
	}
}

std::unique_ptr<StraightPath> NavigationPath::RecomputePath(
	const glm::vec3& startPos, const glm::vec3& endPos, bool force, bool incremental)
{
	auto& settings = nav::GetSettings();

	glm::vec3 extents = m_extents;
	if (settings.use_find_polygon_extents)
	{
		extents = settings.find_polygon_extents;
	}

	dtPolyRef startRef;
	glm::vec3 spos;

	m_failed = false;
	m_hazardBlocked = false;
	m_hazardBlockReason.clear();

	// TODO: Cache the last known valid starting position to detect when moving off the mesh
	// Apply (or clear, when searchRadius is 0) the radius restriction for this destination
	if (m_destinationInfo)
	{
		m_filter.SetSearchOrigin(m_destinationInfo->options.searchOrigin);
		m_filter.SetSearchRadius(m_destinationInfo->options.searchRadius);
	}

	// Hazards. The start lookup ignores them so the player can always be located, even while standing
	// in a hazard; hazards the player is inside are "demoted" (passable but expensive) so the path can
	// leave them, while every other hazard stays a hard constraint.
	std::shared_ptr<const HazardSnapshot> hazards;
	if (!m_destinationInfo || m_destinationInfo->options.avoidHazards)
		hazards = GetHazardSnapshot();
	m_hazardVersion = GetHazardVersion();

	const float heightExtent = settings.hazard_height_extent;
	uint64_t demotedMask = hazards ? hazards->ContainingMask(startPos, heightExtent) : 0;

	m_filter.ClearPathLocalState();
	m_filter.SetHazards(nullptr);

	m_query->findNearestPoly(
		glm::value_ptr(startPos),
		glm::value_ptr(extents),
		&m_filter, &startRef, glm::value_ptr(spos));

	if (!startRef)
	{
		if (!incremental)
		{
			SPDLOG_ERROR("Could not locate starting point on navmesh: {:.2f}", startPos.zxy());
		}

		m_failed = true;
		return {};
	}

	// from here on, hazards apply
	m_filter.SetHazards(hazards, demotedMask, settings.hazard_inside_cost_mult);

	auto failHazard = [&](std::string reason) -> std::unique_ptr<StraightPath>
	{
		if (!incremental)
			SPDLOG_ERROR("{}", reason);
		else
			SPDLOG_DEBUG("{}", reason);

		m_hazardBlockReason = std::move(reason);
		m_hazardBlocked = true;
		m_failed = true;
		return {};
	};

	// a destination inside a hazard (other than one we're already standing in) can't be reached
	if (hazards)
	{
		int inside = hazards->FindContaining(endPos, heightExtent, demotedMask);
		if (inside >= 0)
		{
			return failHazard(fmt::format("Destination is inside hazard '{}'", hazards->hazards[inside].name));
		}
	}

	glm::vec3 epos;
	dtPolyRef endRef;

	m_query->findNearestPoly(
		glm::value_ptr(endPos),
		glm::value_ptr(extents),
		&m_filter, &endRef, glm::value_ptr(epos));

	if (!endRef)
	{
		if (!incremental)
		{
			SPDLOG_ERROR("Could not locate destination on navmesh: {:.2f}", endPos.zxy());
		}

		m_failed = true;
		return {};
	}

	m_filter.SetExemptPolys(startRef, endRef);

	int polysSize = MAX_STRAIGHT_PATH_LENGTH;
	std::unique_ptr<dtPolyRef[]> polys = std::make_unique<dtPolyRef[]>(polysSize);
	const int maxHazardRetries = hazards ? settings.hazard_postcheck_iterations : 0;

	for (int hazardAttempt = 0; ; ++hazardAttempt)
	{
		int numPolys = 0;
		int iters = 0;
		dtStatus status = 0;

		while (iters < 100)
		{
			status = m_query->findPath(
				startRef, endRef,
				glm::value_ptr(spos),
				glm::value_ptr(epos), &m_filter, polys.get(), &numPolys, polysSize);

			bool retry = false;

			if (dtStatusDetail(status, DT_OUT_OF_NODES))
			{
				// need to expand the node pool size
				uint32_t maxNodes = (uint32_t)m_query->getNodePool()->getMaxNodes();
				uint32_t newMaxNodes = std::min<uint32_t>(maxNodes * NODE_POOL_GROWTH_FACTOR, std::min<uint32_t>(DT_NULL_IDX, 1 << DT_NODE_PARENT_BITS) - 1);
				if (maxNodes != newMaxNodes && newMaxNodes < NODE_POOL_MAX_SIZE)
				{
					SPDLOG_DEBUG("Growing node pool: {}", newMaxNodes);

					m_query->init(m_query->getAttachedNavMesh(), newMaxNodes);
					retry = true;
				}
				else
				{
					SPDLOG_WARN("Couldn't increase size of node pool. existing: {}, attempted: {}", maxNodes, newMaxNodes);
				}
			}

			if (dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
			{
				// need to expand polys buffer
				int newPolysSize = polysSize *= NODE_POOL_GROWTH_FACTOR;
				if (newPolysSize < NODE_POOL_MAX_SIZE)
				{
					SPDLOG_DEBUG("Growing polys buffer: {}", newPolysSize);

					polysSize = newPolysSize;
					polys = std::make_unique<dtPolyRef[]>(polysSize);
					retry = true;
				}
				else
				{
					SPDLOG_WARN("Couldn't increase size of polys buffer. size: {}", newPolysSize);
				}
			}

			if (!retry)
			{
				break;
			}

			iters++;
		}

		if (dtStatusFailed(status))
		{
			SPDLOG_DEBUG("findPath from {} to {} failed.", startPos, endPos);

			if (hazards)
				return failHazard("No path to the destination that stays outside hazards");

			m_failed = true;
			return {};
		}

		if (dtStatusDetail(status, DT_OUT_OF_NODES)
			|| dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
		{
			SPDLOG_DEBUG("findPath from {} to {} failed: incomplete result ({:#x})",
				startPos, endPos, (status & DT_STATUS_DETAIL_MASK));

			if (!incremental)
			{
				SPDLOG_ERROR("Could not reach destination (too far away): {:.2f}", endPos.zxy());
			}

			m_failed = true;
			return {};
		}

		if ((numPolys > 0 && (polys[numPolys - 1] != endRef))
			|| dtStatusDetail(status, DT_PARTIAL_RESULT))
		{
			// Partial path, did not find path to target
			SPDLOG_DEBUG("findPath from {:.2f} to {:.2f} returned a partial result.", startPos, endPos);

			// with hazards active, a partial result usually means they wall off the destination
			if (hazards)
				return failHazard("No path to the destination that stays outside hazards");

			if (!incremental)
			{
				SPDLOG_ERROR("Could not find path to destination: {:.2f}", endPos.zxy());
			}

			m_failed = true;
			return {};
		}

		auto path = std::make_unique<StraightPath>(MAX_STRAIGHT_PATH_LENGTH);

		if (numPolys > 0)
		{
			m_query->findStraightPath(
				glm::value_ptr(spos),
				glm::value_ptr(epos), polys.get(), numPolys,
				glm::value_ptr(path->verts[0]),
				path->flags.get(),
				path->polys.get(),
				&path->length,
				MAX_STRAIGHT_PATH_LENGTH,
				DT_STRAIGHTPATH_AREA_CROSSINGS);

			// The 0th index is the starting point. Begin by trying to reach the
			// 2nd point...
			if (path->length > 1)
				path->cursor = 1;
		}

		if (!hazards)
			return std::move(path);

		// Post-check: the corridor only avoids polygons *fully* inside hazards, and string-pulling hugs
		// corners, so a straightened segment can still cut through a hard radius. Find the first such
		// segment, block the corridor polygons that overlap that hazard, and search again.
		int crossed = -1;
		for (int i = 0; i + 1 < path->length && crossed < 0; ++i)
		{
			const glm::vec3& a = path->verts[i];
			const glm::vec3& b = path->verts[i + 1];

			// can't reroute inside an off-mesh link, only check where it lands
			if (path->flags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
				crossed = hazards->FindContaining(b, heightExtent, demotedMask);
			else
				crossed = hazards->FindCrossing(a, b, heightExtent, demotedMask);
		}

		if (crossed < 0)
			return std::move(path);

		const HazardEntry& hit = hazards->hazards[crossed];
		bool blockedAny = false;
		for (int i = 0; i < numPolys; ++i)
		{
			dtPolyRef ref = polys[i];
			if (ref == startRef || ref == endRef)
				continue;

			auto it = hazards->hardPartial.find(ref);
			if (it != hazards->hardPartial.end() && (it->second & HazardBit(crossed)))
			{
				m_filter.AddExtraBlocked(ref);
				blockedAny = true;
			}
		}

		if (!blockedAny)
		{
			// the crossing is inside the polygon we start or end in; there's nothing left to block
			return failHazard(fmt::format("Path would cross hazard '{}' inside the start/end navmesh polygon", hit.name));
		}

		if (hazardAttempt >= maxHazardRetries)
		{
			return failHazard(fmt::format("No path around hazard '{}' (gave up after {} reroutes)",
				hit.name, hazardAttempt));
		}

		SPDLOG_DEBUG("Path crosses hazard '{}', rerouting (attempt {})", hit.name, hazardAttempt + 1);
	}
}

void NavigationPath::UpdatePathProperties()
{
	if (!m_currentPath)
	{
		m_followingLink = false;
		return;
	}

	// Check the current path node. Cursor points at the current dest,
	// so the previous point will be our current node.
	auto node = m_currentPath->GetNode(std::max(0, m_currentPath->cursor - 1));

	// we following a link if the current node is an off-mesh connection
	m_followingLink = (std::get<uint8_t>(node) == DT_STRAIGHTPATH_OFFMESH_CONNECTION);
}

float NavigationPath::GetPathTraversalDistance() const
{
	float result = 0.f;

	if (!m_destinationInfo || !m_destinationInfo->valid)
		return -1.f;

	for (int i = 0; i < m_currentPath->length - 1; ++i)
	{
		const float* first = GetRawPosition(i);
		const float* second = GetRawPosition(i + 1);

		// accumulate the distance
		result += dtVdist(first, second);
	}

	return result;
}

glm::vec3 NavigationPath::GetDestination() const
{
	return m_destinationInfo ? m_destinationInfo->eqDestinationPos : glm::vec3{};
}

//----------------------------------------------------------------------------

NavigationLine::NavigationLine()
{
}

NavigationLine::~NavigationLine()
{
	InvalidateDeviceObjects();
}

void NavigationLine::SetNavigationPath(NavigationPath* path)
{
	m_path = path;

	if (m_visible && m_path)
	{
		GenerateBuffers();
	}
	else
	{
		ReleasePath();
	}
}

bool NavigationLine::CreateDeviceObjects()
{
	// The shaders belong to the render backend, the vertex buffer is created on demand.
	if (m_visible)
		m_loaded = true;
	return true;
}

void NavigationLine::InvalidateDeviceObjects()
{
	ReleasePath();
	m_loaded = false;
}

void NavigationLine::ReleasePath()
{
	if (m_vertexBuffer)
	{
		m_vertexBuffer->Release();
		m_vertexBuffer = nullptr;
	}
}

void NavigationLine::Render()
{
	RenderBackend* renderer = GetRenderBackend();
	if (!m_loaded || !renderer)
		return;
	if (m_needsUpdate && m_path)
		GenerateBuffers();
	if (!m_vertexBuffer)
		return;

	const LineStyle& style = gNavigationLineStyle;
	auto color = [](const ImColor& c, float opacity) { return glm::vec4{ c.Value.x, c.Value.y, c.Value.z, opacity }; };
	float innerWidth = style.lineWidth - (2 * style.borderWidth);

	renderer->BindVolumeLines(m_vertexBuffer, sizeof(TVertex));

	// Same passes as the VolumeLine technique in VolumeLines.fx
	for (int pass = 0; pass < 4; ++pass)
	{
		switch (pass)
		{
		case 0: // hidden
			if (style.hiddenOpacity == 0)
				continue;
			renderer->SetVolumeLinePass(pass, color(style.hiddenColor, style.hiddenOpacity), color(style.hiddenColor, style.hiddenOpacity), innerWidth);
			break;

		case 1: // visible
			renderer->SetVolumeLinePass(pass, color(style.visibleColor, style.opacity), color(style.linkColor, style.opacity), innerWidth);
			break;

		case 2: // border - hidden
			if (style.hiddenOpacity == 0)
				continue;
			renderer->SetVolumeLinePass(pass, color(style.borderColor, style.hiddenOpacity), color(style.borderColor, style.hiddenOpacity), style.lineWidth);
			break;

		case 3: // border - visible
			renderer->SetVolumeLinePass(pass, color(style.borderColor, style.opacity), color(style.borderColor, style.opacity), style.lineWidth);
			break;
		}

		for (auto& cmd : m_commands)
		{
			renderer->DrawVolumeLine(cmd.StartVertex, cmd.PrimitiveCount);
		}
	}

	renderer->EndVolumeLines();
}

void NavigationLine::GenerateBuffers()
{
	const std::vector<int>& renderPath = m_path->GetRenderPath();
	int size = (int)renderPath.size();

	if (size <= 0)
		return;

	int bufferLength = (size - 1) * 4;
	TVertex* vertexDest = nullptr;

	RenderBackend* renderer = GetRenderBackend();
	if (!renderer)
		return;

#if HAS_DIRECTX_9
	if (!m_vertexBuffer || size > m_lastSize)
	{
		if (m_vertexBuffer) {
			m_vertexBuffer->Release();
		}

		HRESULT hr = renderer->GetDevice()->CreateVertexBuffer(bufferLength * sizeof(TVertex),
			D3DUSAGE_DYNAMIC | D3DUSAGE_WRITEONLY, 0, D3DPOOL_DEFAULT,
			&m_vertexBuffer, nullptr);

		if (FAILED(hr))
		{
			m_vertexBuffer = nullptr;
			return;
		}
	}

	m_lastSize = size;

	if (m_vertexBuffer->Lock(0, bufferLength * sizeof(TVertex), (void**)&vertexDest, D3DLOCK_DISCARD) < 0)
		return;
#else
	if (!m_vertexBuffer || size > m_lastSize)
	{
		ReleasePath();

		D3D11_BUFFER_DESC desc = {};
		desc.ByteWidth = bufferLength * sizeof(TVertex);
		desc.Usage = D3D11_USAGE_DYNAMIC;
		desc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

		if (FAILED(renderer->GetDevice()->CreateBuffer(&desc, nullptr, &m_vertexBuffer)))
			return;
	}

	m_lastSize = size;

	D3D11_MAPPED_SUBRESOURCE mapped;
	if (FAILED(renderer->GetContext()->Map(m_vertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)))
		return;
	vertexDest = static_cast<TVertex*>(mapped.pData);
#endif

	glm::vec3* pt = &m_path->m_currentPath->verts[0];
	uint8_t* flags = &m_path->m_currentPath->flags[0];

	int index = 0;
	m_commands.resize(size - 1);

	for (int node = 0; node < size - 1; ++node, ++pt)
	{
		int i = renderPath[node];
		float type = 0.f;

		// this point
		glm::vec3 v0;
		if (i == -1) // use starting pos for first point
		{
			v0.x = m_startPos.z;
			v0.y = m_startPos.x;
			v0.z = m_startPos.y + 1;

			if (m_path->m_followingLink)
				type = 1.0f;
		}
		else
		{
			v0.x = pt[0].z;
			v0.y = pt[0].x;
			v0.z = pt[0].y + 1;

			if (flags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
				type = 1.0f;
		}

		// next point
		glm::vec3 v1;
		v1.x = pt[1].z;
		v1.y = pt[1].x;
		v1.z = pt[1].y + 1;

		// following point
		glm::vec3 nextPos;
		int nextIdx = node < size - 2 ? 2 : 1;
		nextPos.x = pt[nextIdx].z;
		nextPos.y = pt[nextIdx].x;
		nextPos.z = pt[nextIdx].y + 1;

		// previous point
		glm::vec3 prevPos;
		int prevIdx = node > 0 ? -1 : 0;
		prevPos.x = pt[prevIdx].z;
		prevPos.y = pt[prevIdx].x;
		prevPos.z = pt[prevIdx].y + 1;

		vertexDest[index + 0].pos = v0;
		vertexDest[index + 0].otherPos = v1;
		vertexDest[index + 0].thickness = -m_thickness;
		vertexDest[index + 0].adjPos = prevPos;
		vertexDest[index + 0].adjHint = prevIdx;
		vertexDest[index + 0].type = type;

		vertexDest[index + 1].pos = v1;
		vertexDest[index + 1].otherPos = v0;
		vertexDest[index + 1].thickness = m_thickness;
		vertexDest[index + 1].adjPos = nextPos;
		vertexDest[index + 1].adjHint = nextIdx - 1;
		vertexDest[index + 1].type = type;

		vertexDest[index + 2].pos = v0;
		vertexDest[index + 2].otherPos = v1;
		vertexDest[index + 2].thickness = m_thickness;
		vertexDest[index + 2].adjPos = prevPos;
		vertexDest[index + 2].adjHint = prevIdx;
		vertexDest[index + 2].type = type;

		vertexDest[index + 3].pos = v1;
		vertexDest[index + 3].otherPos = v0;
		vertexDest[index + 3].thickness = -m_thickness;
		vertexDest[index + 3].adjPos = nextPos;
		vertexDest[index + 3].adjHint = nextIdx - 1;
		vertexDest[index + 3].type = type;

		m_commands[node].StartVertex = index;
		m_commands[node].PrimitiveCount = 2; // 2 triangles in a strip
		index += 4;
	}

#if HAS_DIRECTX_9
	m_vertexBuffer->Unlock();
#else
	renderer->GetContext()->Unmap(m_vertexBuffer, 0);
#endif
	m_needsUpdate = false;
}

void NavigationLine::SetThickness(float thickness)
{
	if (thickness != m_thickness)
	{
		m_thickness = thickness;
		Update();
	}
}

void NavigationLine::SetVisible(bool visible)
{
	if (m_visible != visible)
	{
		m_visible = visible;

		if (m_visible)
		{
			m_needsUpdate = true;
			CreateDeviceObjects();
		}
		else
		{
			ReleasePath();
		}
	}
}

void NavigationLine::SetCurrentPos(const glm::vec3& currentPos)
{
	if (m_startPos != currentPos)
	{
		m_startPos = currentPos;
		m_needsUpdate = true;
	}
}

void NavigationLine::Update()
{
	if (nav::GetSettings().show_nav_path)
	{
		m_needsUpdate = true;
	}
}

bool NavPathFilter::passFilter(const dtPolyRef ref, const dtMeshTile* tile, const dtPoly* poly) const
{
	if (m_searchRadius > 0 && !IsPolyWithinRadius(tile, poly))
		return false;

	if (m_hazards && ref != m_exemptStart && ref != m_exemptEnd)
	{
		if (!m_extraBlocked.empty() && m_extraBlocked.count(ref))
			return false;

		auto it = m_hazards->hardFull.find(ref);
		if (it != m_hazards->hardFull.end() && (it->second & ~m_demotedMask) != 0)
			return false;
	}

	return dtQueryFilter::passFilter(ref, tile, poly);
}

float NavPathFilter::getCost(const float* pa, const float* pb,
	const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
	const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
	const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const
{
	float cost = dtQueryFilter::getCost(pa, pb, prevRef, prevTile, prevPoly,
		curRef, curTile, curPoly, nextRef, nextTile, nextPoly);

	if (m_hazards)
	{
		// all multipliers are >= 1, which keeps the A* heuristic admissible
		float mult = 1.f;

		auto costIt = m_hazards->costMult.find(curRef);
		if (costIt != m_hazards->costMult.end())
			mult = costIt->second;

		if (m_demotedMask)
		{
			auto fullIt = m_hazards->hardFull.find(curRef);
			if (fullIt != m_hazards->hardFull.end() && (fullIt->second & m_demotedMask) != 0)
				mult = std::max(mult, m_insideCostMult);
		}

		cost *= mult;
	}

	return cost;
}

void NavPathFilter::SetHazards(std::shared_ptr<const HazardSnapshot> hazards, uint64_t demotedMask, float insideCostMult)
{
	m_hazards = std::move(hazards);
	m_demotedMask = demotedMask;
	m_insideCostMult = std::max(1.f, insideCostMult);
}

void NavPathFilter::ClearPathLocalState()
{
	m_exemptStart = 0;
	m_exemptEnd = 0;
	m_extraBlocked.clear();
}

bool NavPathFilter::IsPolyWithinRadius(const dtMeshTile* tile, const dtPoly* poly) const
{
	// 2D test on the mesh x/z plane (EQ x/y), like a radius drawn on the map - height is ignored.
	// A polygon passes if any part of it is within the radius. Testing only its centre would reject a
	// large polygon that contains the origin, and then even the start position can't be found.
	const float radiusSqr = static_cast<float>(m_searchRadius) * static_cast<float>(m_searchRadius);
	const float* origin = glm::value_ptr(m_searchOrigin);
	const int nverts = poly->vertCount;

	float verts[DT_VERTS_PER_POLYGON * 3];
	for (int i = 0; i < nverts; ++i)
		dtVcopy(&verts[i * 3], &tile->verts[poly->verts[i] * 3]);

	// Off-mesh connections are a line between two points: both ends must be inside
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION || nverts < 3)
	{
		for (int i = 0; i < nverts; ++i)
		{
			if (dtVdist2DSqr(origin, &verts[i * 3]) > radiusSqr)
				return false;
		}
		return true;
	}

	float edgeDistSqr[DT_VERTS_PER_POLYGON];
	float edgeT[DT_VERTS_PER_POLYGON];
	if (dtDistancePtPolyEdgesSqr(origin, verts, nverts, edgeDistSqr, edgeT))
		return true; // origin is inside the polygon

	for (int i = 0; i < nverts; ++i)
	{
		if (edgeDistSqr[i] <= radiusSqr)
			return true;
	}
	return false;
}

//----------------------------------------------------------------------------
