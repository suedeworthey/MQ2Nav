//
// HazardMover.h
//
// Hazard autonomy (/nav hazard on): moves the character out of hazards on its own, with a human-like
// reaction delay and a varied escape point, pausing other things that steer the character meanwhile,
// and resumes the navigation command that was running before.
//

#pragma once

#include <mq/base/Signal.h>
#include <glm/glm.hpp>

#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <vector>

class HazardManager;
struct HazardSnapshot;
struct DestinationInfo;
enum class NavEndReason;

class HazardMover
{
public:
	explicit HazardMover(HazardManager* manager);
	~HazardMover();

	void Initialize();
	void Shutdown();
	void OnPulse();
	void OnBeginZone();

	void SetEnabled(bool enabled);
	bool IsEnabled() const { return m_enabled; }

	// an autonomous move is in progress (HazardMoveStarted .. HazardMoveFinished)
	bool IsMoveActive() const { return m_moveActive; }

	const char* GetStateName() const;
	void DebugUI();

private:
	enum class State { Idle, Pending, Moving };

	struct Candidate
	{
		glm::vec3 eqPos{ 0.f };
		float pathLength = 0.f;
		float score = 0.f;
	};

	using clock = std::chrono::steady_clock;

	void OnNavigationEnded(const std::shared_ptr<DestinationInfo>& info, NavEndReason reason);
	void OnMovementKey();

	// index of the hazard the player should get out of, or -1
	int FindHazardToEscape(const HazardSnapshot& snapshot, const glm::vec3& meshPos, bool* hardBreach) const;

	bool PlanAndStart(const HazardSnapshot& snapshot, int hazardIndex);
	bool BeginMove(const glm::vec3& eqTarget, const std::string& hazardName);
	void FinishMove(NavEndReason reason);
	void TryResume();

	void RunStartCommands();
	void RunEndCommands();

	float Random01();

	HazardManager* m_manager;
	bool m_enabled = false;
	State m_state = State::Idle;

	clock::time_point m_deadline;          // Pending: when to move
	clock::time_point m_suppressedUntil;   // don't act before this
	clock::time_point m_lastMoveStart;     // retarget throttle
	uint32_t m_pendingSpawnId = 0;

	// the autonomous move in progress
	std::shared_ptr<DestinationInfo> m_moveDest;
	bool m_moveActive = false;
	std::string m_moveHazardName;
	glm::vec3 m_moveTarget{ 0.f };

	// the command that was running before, to resume afterwards
	std::shared_ptr<DestinationInfo> m_resume;
	uint32_t m_resumeSpawnId = 0;
	bool m_resumePaused = false;
	bool m_resumePending = false;

	// what RunStartCommands paused, so RunEndCommands only resumes those
	bool m_pausedMacro = false;
	bool m_pausedCwtn = false;
	std::string m_cwtnClass;

	std::vector<Candidate> m_lastCandidates;
	uint32_t m_lastFailedSpawnId = 0;

	std::mt19937 m_rng;

	mq::Signal<const std::shared_ptr<DestinationInfo>&, NavEndReason>::ScopedConnection m_endedConn;
	mq::Signal<>::ScopedConnection m_keyConn;
};
