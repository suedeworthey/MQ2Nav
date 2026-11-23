//
// PluginAPI.h
//

#pragma once

#include <mq/Plugin.h>
#include <glm/vec3.hpp>

// Exports functions that can be used by other plugins to interface with MQ2Nav

#ifdef NAV_API_EXPORTS
#define NAV_API extern "C" __declspec(dllexport)
#define NAV_OBJECT __declspec(dllexport)
#else
#define NAV_API extern "C" __declspec(dllimport)
#define NAV_OBJECT __declspec(dllimport)
#endif

//----------------------------------------------------------------------------

namespace nav {

//----------------------------------------------------------------------------

// NavAPI interface version. 1 = original methods, 2 = adds the hazard methods (AddHazardRule ..
// IsHazardMoveActive). Calling a method the loaded MQ2Nav doesn't implement would crash, so check
// NavSupportsAPIVersion(2) (or the GetNavAPIVersion export) before using newer methods.
constexpr int NAV_API_VERSION = 2;

// For other plugins: true if the loaded MQ2Nav implements at least this NavAPI version.
inline bool NavSupportsAPIVersion(int version)
{
	using fGetNavAPIVersion = int(*)();
	auto fn = reinterpret_cast<fGetNavAPIVersion>(GetPluginProc("MQ2Nav", "GetNavAPIVersion"));
	return fn ? fn() >= version : version <= 1;
}

// defines a type of destination that is being navigated to.
enum class DestinationType
{
	None,
	Location,
	Door,
	GroundItem,
	Spawn,
	Waypoint,
};

// defines the facing mode for the nav command.
enum class FacingType
{
	Forward,
	Backward,
};

// defines the options that can be used to create a navigation command.
struct NavigationOptions
{
	float distance = 0.f;          // distance to target to stop at
	bool lineOfSight = true;       // does target need to be in los
	bool paused = false;           // pathing is paused
	bool track = true;             // if spawn is to be tracked
	FacingType facing = FacingType::Forward; //  Forward = normal, Backward = move along path facing backward.
	int searchRadius = 0;          // if > 0, the path may only use navmesh polygons within this 2D radius of searchOrigin
	glm::vec3 searchOrigin{ 0.f }; // centre of searchRadius, in navmesh coordinates (EQ X, height, EQ Y)

	// set a new default log level while the path is running. info is the default.
	spdlog::level::level_enum logLevel = spdlog::level::info;
};

// Represents the state of a nav command. This state is only valid during a
// callback from MQ2Nav. Do not store or otherwise make copies of this data
// as the information it points to may go out of scope.

// This state is read-only. Modifying it will have no impact on the navigation
// command that it represents.

struct NavCommandState
{
	// the original command string.
	std::string_view command;

	// the position of the destination.
	glm::vec3 destination;

	// indicates if the command is paused
	bool paused;

	// the destination type
	DestinationType type;

	// options used to create the command.
	NavigationOptions options;

	// The tag value used in the nav command if one was provided
	std::string_view tag;
};

//----------------------------------------------------------------------------

// Observer event - Indicates the type of event that occurred in the observer.
enum class NavObserverEvent
{
	NavStarted,                  // when navigation is started
	NavPauseChanged,             // when navigation is paused or unpaused by the user
	NavCanceled,                 // when navigation is canceled, either by starting a new command or by /nav stop
	NavDestinationReached,       // when navigation reaches its destination.
	NavFailed,                   // navigation command failed to start, or was stopped because a hazard blocks the path

	// Hazard autonomy (/nav hazard on). Nav is moving the character out of a hazard on its own; the
	// command state's tag is "hazard". Plugins that steer the character (stick, follow, pulling,
	// casting with a cast bar) should hold off until HazardMoveFinished. Finished is sent in every
	// case: reached, canceled, failed or replaced by another command.
	HazardMoveStarted,
	HazardMoveFinished,
};

// Observer function - Registering an observer function will allow your plugin to
// get notified of changes in current navigation state.

using fObserverCallback = void(*)(nav::NavObserverEvent eventType, const nav::NavCommandState& commandState, void* userData);

//----------------------------------------------------------------------------

// The main interface to Nav. Acquire it by calling GetNavAPIFromPlugin(). Be sure
// to stop using it if you unload the plugin.
class NavAPI : public mq::PluginInterface
{
public:
	//----------------------------------------------------------------------------
	// Original Nav api exposed through the NavAPI interface

	// Check if MQ2Nav is initialized
	virtual bool IsInitialized() = 0;

	// Check if a mesh is loaded
	virtual bool IsNavMeshLoaded() = 0;

	// Check if a navigation command is running.
	virtual bool IsNavPathActive() = 0;

	// check if a navigation command is paused.
	virtual bool IsNavPathPaused() = 0;

	// Check if a path is possible to the specified destination
	virtual bool IsDestinationReachable(std::string_view destination) = 0;

	// Calculate the length of the path to the specified destination
	virtual float GetPathLength(std::string_view destination) = 0;

	// Execute a nav command
	virtual bool ExecuteNavCommand(std::string_view command) = 0;

	//----------------------------------------------------------------------------

	// Retrieves current command state. Do not copy this value or store it for later.
	// If no command is active, returns nullptr.
	virtual const NavCommandState* GetNavCommandState() = 0;

	// Register an observer to get notifications of nav command state changes.
	// Returns the observer id. Use this to unregister before unloading your plugin.
	virtual int RegisterNavObserver(fObserverCallback callback, void* userData) = 0;

	// Unregisters an observer that was registered with RegisterNavObserver
	virtual void UnregisterNavObserver(int observerId) = 0;

	//----------------------------------------------------------------------------
	// Hazards (spawns to avoid). These virtuals are appended after the original ones so the vtable
	// layout of the methods above is unchanged; add new methods below this line only. (Data structs such
	// as NavigationOptions/NavCommandState are a separate matter: any change to them requires rebuilding
	// plugins that read them.)

	// Add an avoid rule. spec uses the same arguments as "/nav avoid add", for example
	//   "\"a swirling void\" type=untargetable hard=15 soft=25"
	// persist = false keeps the rule only until the zone changes or the plugin unloads.
	// Returns the rule id, or -1 if the spec is invalid.
	virtual int AddHazardRule(std::string_view spec, bool persist) = 0;

	// Remove a rule by id. Returns false if no such rule exists.
	virtual bool RemoveHazardRule(int ruleId) = 0;

	// Remove every rule added through AddHazardRule with persist = false. Persisted rules (including the
	// user's /nav avoid rules) are kept; remove those individually with RemoveHazardRule.
	virtual void ClearHazardRules() = 0;

	virtual int GetHazardRuleCount() = 0;

	// Write a description of the rule at index (0..GetHazardRuleCount()-1). Returns its id, or -1.
	virtual int GetHazardRuleSpec(int index, char* buffer, size_t bufferSize) = 0;

	// True if the player stands inside the hard radius of any hazard.
	virtual bool IsPlayerInsideHazard() = 0;

	// 2D distance from an EQ position (x, y, z) to the nearest hard hazard radius.
	// Returns < 0 if the position is inside one, FLT_MAX if there are no hazards nearby.
	virtual float GetHazardDistance(const glm::vec3& eqPos) = 0;

	// Hazard autonomy: when enabled (per zone), nav moves the character out of hazards on its own.
	virtual void SetHazardAutonomy(bool enabled) = 0;
	virtual bool IsHazardAutonomyEnabled() = 0;

	// True from HazardMoveStarted until HazardMoveFinished. Poll this as a fallback for the events.
	virtual bool IsHazardMoveActive() = 0;
};

} // namespace nav
