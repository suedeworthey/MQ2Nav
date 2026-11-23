//
// HazardMover.cpp
//

#include "pch.h"
#include "HazardMover.h"

#include "common/NavMesh.h"
#include "plugin/HazardManager.h"
#include "plugin/KeybindHandler.h"
#include "plugin/MQ2Navigation.h"
#include "plugin/NavigationPath.h"
#include "plugin/PluginSettings.h"
#include "plugin/Utilities.h"

#include <mq/base/String.h>
#include <DetourNavMeshQuery.h>
#include <imgui.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>

using namespace std::chrono_literals;

namespace {

constexpr const char* kHazardTag = "hazard";

glm::vec3 PlayerMeshPosition()
{
	PlayerClient* me = GetCharInfo() ? GetCharInfo()->pSpawn : nullptr;
	return me ? glm::vec3{ me->X, me->FloorHeight, me->Y } : glm::vec3{ 0.f };
}

void RunCommandList(const std::string& commands)
{
	for (std::string_view part : mq::split_view(commands, ';'))
	{
		std::string cmd(part);
		mq::trim(cmd);
		if (!cmd.empty())
			EzCommand(cmd.c_str());
	}
}

bool InputString(const char* label, std::string& value)
{
	char buffer[512] = { 0 };
	strncpy_s(buffer, value.c_str(), _TRUNCATE);
	if (ImGui::InputText(label, buffer, sizeof(buffer)))
	{
		value = buffer;
		return true;
	}
	return false;
}

std::string ReplaceClass(std::string text, const std::string& classCode)
{
	const std::string token = "{class}";
	for (size_t pos = text.find(token); pos != std::string::npos; pos = text.find(token, pos + classCode.size()))
		text.replace(pos, token.size(), classCode);
	return text;
}

} // namespace

//============================================================================

HazardMover::HazardMover(HazardManager* manager)
	: m_manager(manager)
	, m_rng(std::random_device{}())
{
}

HazardMover::~HazardMover() = default;

void HazardMover::Initialize()
{
	m_endedConn = g_mq2Nav->NavigationEnded.Connect(
		[this](const std::shared_ptr<DestinationInfo>& info, NavEndReason reason) { OnNavigationEnded(info, reason); });

	if (auto* keybinds = g_mq2Nav->Get<KeybindHandler>())
		m_keyConn = keybinds->OnMovementKeyPressed.Connect([this]() { OnMovementKey(); });
}

void HazardMover::Shutdown()
{
	if (m_moveActive)
		RunEndCommands();

	m_endedConn.Disconnect();
	m_keyConn.Disconnect();
	m_moveDest.reset();
	m_resume.reset();
	m_moveActive = false;
	m_state = State::Idle;
}

void HazardMover::OnBeginZone()
{
	// the move (if any) was already ended by the plugin's zone reset; drop everything else
	m_resume.reset();
	m_resumePending = false;
	m_state = State::Idle;
	m_lastCandidates.clear();
}

void HazardMover::SetEnabled(bool enabled)
{
	m_enabled = enabled;
	if (!enabled && m_state == State::Pending)
		m_state = State::Idle;
}

const char* HazardMover::GetStateName() const
{
	switch (m_state)
	{
	case State::Idle: return "idle";
	case State::Pending: return "reacting";
	case State::Moving: return "moving out";
	}
	return "?";
}

float HazardMover::Random01()
{
	return std::uniform_real_distribution<float>(0.f, 1.f)(m_rng);
}

//----------------------------------------------------------------------------

int HazardMover::FindHazardToEscape(const HazardSnapshot& snapshot, const glm::vec3& meshPos, bool* hardBreach) const
{
	auto& settings = nav::GetSettings();
	glm::vec2 p{ meshPos.x, meshPos.z };
	int softHit = -1;

	for (size_t i = 0; i < snapshot.hazards.size(); ++i)
	{
		const HazardEntry& h = snapshot.hazards[i];

		// inside=hold means "stay put if caught in it"
		if (h.inside != HazardInsideAction::Flee)
			continue;
		if (std::fabs(meshPos.y - h.height) > settings.hazard_height_extent)
			continue;

		if (hazard::PointInShape(h.shape, p, h.hardRadius))
		{
			*hardBreach = true;
			return static_cast<int>(i);
		}

		if (softHit < 0 && settings.hazard_evade_soft && h.softRadius > h.hardRadius
			&& hazard::PointInShape(h.shape, p, h.softRadius))
		{
			softHit = static_cast<int>(i);
		}
	}

	*hardBreach = false;
	return softHit;
}

void HazardMover::OnPulse()
{
	if (m_resumePending)
	{
		m_resumePending = false;
		TryResume();
		return;
	}

	if (!m_enabled || !nav::ValidIngame(true) || gZoning)
		return;

	NavMesh* mesh = g_mq2Nav->Get<NavMesh>();
	if (!mesh || !mesh->IsNavMeshLoaded())
		return;

	auto snapshot = GetHazardSnapshot();
	clock::time_point now = clock::now();
	auto& settings = nav::GetSettings();
	glm::vec3 meshPos = PlayerMeshPosition();

	switch (m_state)
	{
	case State::Idle:
	{
		if (!snapshot || now < m_suppressedUntil)
			return;

		bool breach = false;
		int index = FindHazardToEscape(*snapshot, meshPos, &breach);
		if (index < 0)
		{
			m_lastFailedSpawnId = 0;
			return;
		}

		// a hard breach always gets a reaction; soft-only evasion respects the retarget throttle
		if (!breach && now - m_lastMoveStart < std::chrono::milliseconds(settings.hazard_retarget_min_ms))
			return;

		int minMs = settings.hazard_reaction_min_ms;
		int maxMs = std::max(minMs, settings.hazard_reaction_max_ms);
		int delay = std::uniform_int_distribution<int>(minMs, maxMs)(m_rng);

		m_deadline = now + std::chrono::milliseconds(delay);
		m_pendingSpawnId = snapshot->hazards[index].spawnId;
		m_state = State::Pending;
		SPDLOG_DEBUG("Hazard autonomy: inside '{}', reacting in {}ms", snapshot->hazards[index].name, delay);
		return;
	}

	case State::Pending:
	{
		if (!snapshot)
		{
			m_state = State::Idle;
			return;
		}

		bool breach = false;
		int index = FindHazardToEscape(*snapshot, meshPos, &breach);
		if (index < 0)
		{
			m_state = State::Idle; // it moved away (or we did)
			return;
		}

		if (now < m_deadline)
			return;

		m_state = State::Idle;
		PlanAndStart(*snapshot, index);
		return;
	}

	case State::Moving:
	{
		if (!m_moveActive)
		{
			m_state = State::Idle;
			return;
		}

		// the escape point became unsafe (the hazard moved onto it, or a new one appeared): pick another
		if (snapshot && now - m_lastMoveStart > 500ms)
		{
			glm::vec3 targetMesh{ m_moveTarget.x, m_moveTarget.z, m_moveTarget.y };
			if (snapshot->FindContaining(targetMesh, settings.hazard_height_extent) >= 0)
			{
				bool breach = false;
				int index = FindHazardToEscape(*snapshot, meshPos, &breach);
				if (index >= 0)
				{
					SPDLOG_DEBUG("Hazard autonomy: escape point is no longer safe, replanning");
					PlanAndStart(*snapshot, index);
				}
			}
		}
		return;
	}
	}
}

//----------------------------------------------------------------------------

bool HazardMover::PlanAndStart(const HazardSnapshot& snapshot, int hazardIndex)
{
	auto& settings = nav::GetSettings();
	NavMesh* mesh = g_mq2Nav->Get<NavMesh>();
	std::shared_ptr<dtNavMeshQuery> query = mesh ? mesh->GetNavMeshQuery() : nullptr;
	if (!query)
		return false;

	const HazardEntry& hazard = snapshot.hazards[hazardIndex];
	const glm::vec3 meshPos = PlayerMeshPosition();
	const float heightExtent = settings.hazard_height_extent;

	// Standoff: just outside the hard radius plus a random part of the soft band, so the spot differs
	// every time (and isn't always the same exact distance).
	const float margin = std::max(2.f, hazard.hardRadius * 0.05f);
	const float band = std::max(0.f, hazard.softRadius - hazard.hardRadius);
	const int count = settings.hazard_candidate_count;
	const float angleJitter = Random01() * hazard::kTwoPi;

	dtQueryFilter filter; // projection only
	const float extents[3] = { 2.f, heightExtent, 2.f };

	m_lastCandidates.clear();
	Candidate best;
	bool found = false;

	for (int i = 0; i < count; ++i)
	{
		float angle = angleJitter + hazard::kTwoPi * i / count;
		float radius = hazard.hardRadius + margin + band * Random01();
		glm::vec2 p2 = hazard.shape.center + hazard::DirectionFromAngle(angle) * radius;

		// project onto the mesh near our own floor
		float probe[3] = { p2.x, meshPos.y, p2.y };
		float nearest[3];
		dtPolyRef ref = 0;
		query->findNearestPoly(probe, extents, &filter, &ref, nearest);
		if (!ref || glm::distance(glm::vec2{ nearest[0], nearest[2] }, p2) > 2.f)
			continue;

		glm::vec3 candidateMesh{ nearest[0], nearest[1], nearest[2] };
		if (std::fabs(candidateMesh.y - meshPos.y) > heightExtent)
			continue; // different floor
		if (snapshot.FindContaining(candidateMesh, heightExtent) >= 0)
			continue; // inside this or another hazard

		glm::vec3 eqPos{ candidateMesh.x, candidateMesh.z, candidateMesh.y };
		float length = g_mq2Nav->GetNavigationPathLength(
			fmt::format("locxyz {:.2f} {:.2f} {:.2f}", eqPos.x, eqPos.y, eqPos.z));
		if (length < 0.f)
			continue;

		// prefer spots away from other hazards
		float otherDistance = 50.f;
		glm::vec2 c2{ candidateMesh.x, candidateMesh.z };
		for (size_t j = 0; j < snapshot.hazards.size(); ++j)
		{
			if (static_cast<int>(j) == hazardIndex)
				continue;
			const HazardEntry& other = snapshot.hazards[j];
			if (std::fabs(candidateMesh.y - other.height) > heightExtent)
				continue;
			otherDistance = std::min(otherDistance, hazard::DistanceToShape(other.shape, c2, other.hardRadius));
		}

		Candidate c;
		c.eqPos = eqPos;
		c.pathLength = length;
		c.score = length
			- settings.hazard_other_hazard_weight * otherDistance
			+ settings.hazard_candidate_random_weight * length * Random01();
		m_lastCandidates.push_back(c);

		if (!found || c.score < best.score)
		{
			best = c;
			found = true;
		}
	}

	if (!found)
	{
		if (m_lastFailedSpawnId != hazard.spawnId)
		{
			SPDLOG_WARN("Hazard autonomy: no safe spot found to get out of '{}', holding position", hazard.name);
			m_lastFailedSpawnId = hazard.spawnId;
		}
		m_lastMoveStart = clock::now(); // throttle retries
		return false;
	}

	m_lastFailedSpawnId = 0;
	return BeginMove(best.eqPos, hazard.name);
}

bool HazardMover::BeginMove(const glm::vec3& eqTarget, const std::string& hazardName)
{
	// remember the command that was running, unless it's already one of ours
	if (g_mq2Nav->IsActive())
	{
		auto active = g_mq2Nav->GetActivePath();
		auto info = active ? active->GetDestinationInfo() : nullptr;
		if (info && info->tag != kHazardTag)
		{
			m_resume = info;
			m_resumePaused = g_mq2Nav->IsPaused();
			m_resumeSpawnId = info->pSpawn ? info->pSpawn->SpawnID : 0;
		}
	}

	auto dest = g_mq2Nav->ParseDestination(
		fmt::format("locxyz {:.2f} {:.2f} {:.2f} | tag={}", eqTarget.x, eqTarget.y, eqTarget.z, kHazardTag),
		spdlog::level::info);
	if (!dest || !dest->valid)
		return false;

	dest->command = fmt::format("hazard: moving out of '{}'", hazardName);

	// set before starting so the NavigationEnded for whatever we replace isn't mistaken for ours
	m_moveDest = dest;
	m_moveTarget = eqTarget;
	m_moveHazardName = hazardName;
	m_lastMoveStart = clock::now();

	g_mq2Nav->BeginNavigation(dest);

	auto active = g_mq2Nav->GetActivePath();
	bool started = g_mq2Nav->IsActive() && active && active->GetDestinationInfo() == dest;
	if (!started)
	{
		m_moveDest.reset();
		if (m_moveActive)
			FinishMove(NavEndReason::Failed);
		else if (m_resume)
			m_resumePending = true; // we replaced the user's command; put it back

		SPDLOG_WARN("Hazard autonomy: couldn't start moving out of '{}'", hazardName);
		m_state = State::Idle;
		return false;
	}

	m_state = State::Moving;
	if (!m_moveActive)
	{
		m_moveActive = true;
		RunStartCommands();
		SPDLOG_INFO("Hazard: moving out of '{}'", hazardName);
	}
	g_mq2Nav->DispatchNavEvent(nav::NavObserverEvent::HazardMoveStarted);
	return true;
}

void HazardMover::OnNavigationEnded(const std::shared_ptr<DestinationInfo>& info, NavEndReason reason)
{
	if (!m_moveDest || info != m_moveDest)
		return;

	FinishMove(reason);
}

void HazardMover::FinishMove(NavEndReason reason)
{
	m_moveDest.reset();
	m_state = State::Idle;
	m_lastMoveStart = clock::now();

	if (!m_moveActive)
		return;

	m_moveActive = false;
	RunEndCommands();
	g_mq2Nav->DispatchNavEvent(nav::NavObserverEvent::HazardMoveFinished);

	switch (reason)
	{
	case NavEndReason::Reached:
		// resume what we interrupted (next pulse; starting a command inside NavigationEnded isn't allowed)
		if (m_resume)
			m_resumePending = true;
		break;

	case NavEndReason::Canceled:
		// the player stopped us: back off for a while and forget the old command
		m_suppressedUntil = clock::now() + std::chrono::milliseconds(nav::GetSettings().hazard_stop_suppress_ms);
		m_resume.reset();
		break;

	case NavEndReason::Failed:
		// keep the old command; it resumes after a successful move
		break;

	case NavEndReason::Replaced:
		// someone issued a new command: theirs wins
		m_resume.reset();
		break;
	}
}

void HazardMover::TryResume()
{
	auto resume = std::move(m_resume);
	m_resume.reset();

	if (!resume || g_mq2Nav->IsActive())
		return; // nothing to resume, or something else is running now

	// raw pointers in the saved destination may be stale by now
	if (resume->type == nav::DestinationType::Spawn)
	{
		PlayerClient* pSpawn = m_resumeSpawnId ? GetSpawnByID(m_resumeSpawnId) : nullptr;
		if (!pSpawn)
		{
			SPDLOG_INFO("Hazard: not resuming navigation, the target is gone");
			return;
		}
		resume->pSpawn = pSpawn;
	}
	else if (resume->type == nav::DestinationType::GroundItem && !resume->groundItem)
	{
		return;
	}

	resume->options.paused = m_resumePaused;
	SPDLOG_INFO("Hazard: resuming navigation");
	g_mq2Nav->BeginNavigation(resume);
}

void HazardMover::OnMovementKey()
{
	// the player is steering: don't fight them
	auto delay = std::chrono::milliseconds(nav::GetSettings().hazard_manual_input_suppress_ms);
	m_suppressedUntil = std::max(m_suppressedUntil, clock::now() + delay);

	if (m_state == State::Pending)
		m_state = State::Idle;
}

//----------------------------------------------------------------------------

void HazardMover::RunStartCommands()
{
	auto& settings = nav::GetSettings();

	m_pausedMacro = false;
	m_pausedCwtn = false;

	if (settings.hazard_pause_macro && gMacroBlock && !gMacroBlock->Paused)
	{
		EzCommand("/mqpause on");
		m_pausedMacro = true;
	}

	if (settings.hazard_pause_cwtn && pLocalPlayer && pEverQuest)
	{
		const char* code = pEverQuest->GetClassThreeLetterCode(static_cast<EQClass>(pLocalPlayer->GetClass()));
		std::string classCode = code ? code : "";
		std::transform(classCode.begin(), classCode.end(), classCode.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

		if (!classCode.empty() && IsPluginLoaded("MQ2" + classCode))
		{
			m_cwtnClass = classCode;
			RunCommandList(ReplaceClass(settings.hazard_cwtn_pause_command, classCode));
			m_pausedCwtn = true;
		}
	}

	RunCommandList(settings.hazard_move_start_commands);
}

void HazardMover::RunEndCommands()
{
	auto& settings = nav::GetSettings();

	RunCommandList(settings.hazard_move_end_commands);

	if (m_pausedCwtn)
	{
		RunCommandList(ReplaceClass(settings.hazard_cwtn_resume_command, m_cwtnClass));
		m_pausedCwtn = false;
	}

	if (m_pausedMacro)
	{
		EzCommand("/mqpause off");
		m_pausedMacro = false;
	}
}

//----------------------------------------------------------------------------

void HazardMover::DebugUI()
{
	auto& settings = nav::GetSettings();
	bool changed = false;

	bool enabled = m_enabled;
	if (ImGui::Checkbox("Autonomy: move out of hazards on my own (this zone)", &enabled))
		m_manager->SetAutonomy(enabled);

	ImGui::LabelText("Mover", "%s%s", GetStateName(), m_moveActive ? fmt::format(" ({})", m_moveHazardName).c_str() : "");

	if (ImGui::TreeNode("Autonomy tuning"))
	{
		changed |= ImGui::DragIntRange2("Reaction (ms)", &settings.hazard_reaction_min_ms, &settings.hazard_reaction_max_ms, 10.f, 0, 5000);
		changed |= ImGui::DragInt("Min time between moves (ms)", &settings.hazard_retarget_min_ms, 10.f, 0, 10000);
		changed |= ImGui::Checkbox("Also move when only inside the soft radius", &settings.hazard_evade_soft);
		changed |= ImGui::DragInt("Escape points to consider", &settings.hazard_candidate_count, 0.2f, 4, 64);
		changed |= ImGui::DragFloat("Randomness", &settings.hazard_candidate_random_weight, 0.01f, 0.f, 2.f, "%.2f");
		changed |= ImGui::DragFloat("Prefer distance from other hazards", &settings.hazard_other_hazard_weight, 0.05f, 0.f, 5.f, "%.2f");
		changed |= ImGui::DragInt("Back off after /nav stop (ms)", &settings.hazard_stop_suppress_ms, 10.f, 0, 30000);
		changed |= ImGui::DragInt("Back off after manual movement (ms)", &settings.hazard_manual_input_suppress_ms, 10.f, 0, 30000);
		ImGui::Separator();
		changed |= ImGui::Checkbox("Pause the running macro while moving", &settings.hazard_pause_macro);
		changed |= ImGui::Checkbox("Pause the CWTN class plugin while moving", &settings.hazard_pause_cwtn);
		changed |= InputString("CWTN pause command", settings.hazard_cwtn_pause_command);
		changed |= InputString("CWTN resume command", settings.hazard_cwtn_resume_command);
		changed |= InputString("Extra start commands", settings.hazard_move_start_commands);
		changed |= InputString("Extra end commands", settings.hazard_move_end_commands);
		ImGui::TextDisabled("Commands are ';'-separated. {class} = your class code (war, enc, ...).");
		ImGui::TreePop();
	}

	if (!m_lastCandidates.empty() && ImGui::TreeNode("Last escape candidates"))
	{
		for (const Candidate& c : m_lastCandidates)
		{
			ImGui::Text("(%.1f, %.1f, %.1f)  path %.1f  score %.1f", c.eqPos.y, c.eqPos.x, c.eqPos.z, c.pathLength, c.score);
		}
		ImGui::TreePop();
	}

	if (changed)
	{
		settings.hazard_reaction_max_ms = std::max(settings.hazard_reaction_min_ms, settings.hazard_reaction_max_ms);
		nav::SaveSettings(false);
	}
}
