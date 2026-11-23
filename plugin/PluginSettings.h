//
// PluginSettings.h
//

#pragma once

#include <glm/vec3.hpp>

namespace nav {

struct SettingsData
{
	// auto break navigation if keyboard input is received
	bool autobreak = false;

	// auto pause navigation if keyboard input is received
	bool autopause = false;

	// auto reload navmesh if file changes
	bool autoreload = true;

	// render targeted door objects
	bool render_doortarget = true;

	// show the MQ2Nav Tools debug ui
	bool show_ui = true;

	// show the current navigation path
	bool show_nav_path = true;

	// attempt to get unstuck
	bool attempt_unstuck = false;

	// use floor height for spawn z axis
	bool use_spawn_floor_height = true;

	// use custom extents in findNearestPoly
	bool use_find_polygon_extents = false;
	glm::vec3 find_polygon_extents{ 2, 4, 2 };

	// render pathing 3d debugging
	bool debug_render_pathing = false;

	// mapline settings
	bool map_line_enabled = true;
	uint32_t map_line_color = 0xff00ff00;
	int map_line_layer = 3;

	// nav path settings
	bool poll_navigation_path = true;

	// open doors while navigation
	bool open_doors = true;

	// ignore scripted doors
	bool ignore_scripted_doors = true;

	//------------------------------------------------------------------------
	// ledge drops (one-way "Drop" / "Drop (levitate)" links placed in the mesh)

	// use drop links at all
	bool use_drop_links = true;

	// highest fall (navmesh units) taken without levitation on "Drop" links
	float max_safe_drop_height = 20.f;

	// only take any drop link while levitating
	bool drop_links_require_levitate = false;

	//------------------------------------------------------------------------
	// hazards (spawn avoidance)

	// apply hazard rules to path calculations (per-command opt-out: noavoid)
	bool hazards_enabled = true;

	// radii used when a rule doesn't specify them
	float hazard_default_hard = 15.f;
	float hazard_default_soft = 25.f;

	// radius=rampage resolves to melee range between you and the spawn plus this margin
	float hazard_rampage_margin = 5.f;

	// path cost multipliers (all >= 1). soft: graded penalty inside the soft ring.
	// boundary: polygons that touch a hard radius. inside: the hazard you're standing in while leaving it.
	float hazard_soft_cost_mult = 6.f;
	float hazard_boundary_cost_mult = 3.f;
	float hazard_inside_cost_mult = 20.f;

	// vertical half-extent: polygons/paths further than this above or below a hazard are not affected
	float hazard_height_extent = 15.f;

	// update intervals (ms): positions of tracked hazards, and full spawn rescans for new matches
	int hazard_refresh_ms = 200;
	int hazard_rescan_ms = 1000;

	// hysteresis: rebuild the hazard snapshot (and repath) only when a hazard moved/turned more than this
	float hazard_move_epsilon = 1.0f;
	float hazard_heading_epsilon_deg = 5.f;

	// how many times to reroute when the straightened path still cuts through a hard radius
	int hazard_postcheck_iterations = 4;

	// visuals
	bool hazard_show_overlay = true;
	bool hazard_show_map = true;
	int hazard_map_layer = 3;
};
SettingsData& GetSettings();

// Load settings from the .ini file
void LoadSettings(bool showMessage = false);

// Save settings to the .ini file
void SaveSettings(bool showMessage = false);

// Parse a command to change settings
bool ParseIniCommand(const char* command);

// Read a setting string into the buffer
bool ReadIniSetting(const char* keyName, char* pBuffer, size_t length);

} // namespace nav
