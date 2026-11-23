//
// PluginSettings.cpp
//

#include "pch.h"
#include "PluginSettings.h"

#include "plugin/MQ2Navigation.h"
#include "plugin/NavigationPath.h"

#include <mq/base/Config.h>
#include <spdlog/spdlog.h>

namespace nav {

//----------------------------------------------------------------------------

SettingsData g_settings;
static const std::string SettingsSection = "Settings";

SettingsData& GetSettings()
{
	return g_settings;
}

static inline bool LoadBoolSetting(const std::string& name, bool defaultValue)
{
	return mq::GetPrivateProfileBool(SettingsSection, name, defaultValue, INIFileName);
}

static inline glm::vec3 LoadVec3Setting(const std::string& name, const glm::vec3& defaultValue)
{
	glm::vec3 value;

	value.x = mq::GetPrivateProfileFloat(SettingsSection, fmt::format("{}X", name), defaultValue.x, INIFileName);
	value.y = mq::GetPrivateProfileFloat(SettingsSection, fmt::format("{}Y", name), defaultValue.y, INIFileName);
	value.z = mq::GetPrivateProfileFloat(SettingsSection, fmt::format("{}Z", name), defaultValue.z, INIFileName);

	return value;
}

template <typename T>
static inline T LoadNumberSetting(const std::string& name, T defaultValue)
{
	return mq::GetPrivateProfileValue<T>(SettingsSection, name, defaultValue, INIFileName);
}

static inline std::string LoadStringSetting(const std::string& name, const std::string& defaultValue)
{
	// "<none>" stores an intentionally empty value (an empty INI value reads back as the default)
	std::string value = mq::GetPrivateProfileString(SettingsSection, name, defaultValue, INIFileName);
	return value == "<none>" ? std::string() : value;
}

static inline void SaveStringSetting(const std::string& name, const std::string& value)
{
	mq::WritePrivateProfileString(SettingsSection, name, value.empty() ? "<none>" : value, INIFileName);
}

static inline void SaveBoolSetting(const std::string& name, bool value)
{
	mq::WritePrivateProfileBool(SettingsSection, name, value, INIFileName);
}

template <typename T>
static inline void SaveNumberSetting(const std::string& name, T value)
{
	mq::WritePrivateProfileString(SettingsSection, name, std::to_string(value).c_str(), INIFileName);
}

static inline void SaveVec3Setting(const std::string& name, const glm::vec3& value)
{
	mq::WritePrivateProfileFloat(SettingsSection, fmt::format("{}X", name), value.x, INIFileName);
	mq::WritePrivateProfileFloat(SettingsSection, fmt::format("{}Y", name), value.y, INIFileName);
	mq::WritePrivateProfileFloat(SettingsSection, fmt::format("{}Z", name), value.z, INIFileName);
}

void LoadSettings(bool showMessage/* = true*/)
{
	if (showMessage)
	{
		SPDLOG_INFO("Loading settings...");
	}

	SettingsData defaults, &settings = g_settings;

	settings.autobreak = LoadBoolSetting("AutoBreak", defaults.autobreak);
	settings.autopause = LoadBoolSetting("AutoPause", defaults.autopause);
	settings.autoreload = LoadBoolSetting("AutoReload", defaults.autoreload);
	settings.render_doortarget = LoadBoolSetting("RenderDoorTarget", defaults.render_doortarget);
	settings.show_ui = LoadBoolSetting("ShowUI", defaults.show_ui);
	settings.show_nav_path = LoadBoolSetting("ShowNavPath", defaults.show_nav_path);
	settings.attempt_unstuck = LoadBoolSetting("AttemptUnstuck", defaults.attempt_unstuck);
	settings.use_spawn_floor_height = LoadBoolSetting("UseSpawnFloorHeight", defaults.use_spawn_floor_height);

	settings.use_find_polygon_extents = LoadBoolSetting("UseFindPolygonExtents", defaults.use_find_polygon_extents);
	settings.find_polygon_extents = LoadVec3Setting("FindPolygonExtents", defaults.find_polygon_extents);

	settings.map_line_enabled = LoadBoolSetting("MapLineEnabled", defaults.map_line_enabled);
	settings.map_line_color = LoadNumberSetting("MapLineColor", defaults.map_line_color);
	settings.map_line_layer = LoadNumberSetting("MapLineLayer", defaults.map_line_layer);

	settings.open_doors = LoadBoolSetting("OpenDoors", defaults.open_doors);
	settings.ignore_scripted_doors = LoadBoolSetting("IgnoreScriptedDoors", defaults.ignore_scripted_doors);

	// debug settings
	settings.debug_render_pathing = LoadBoolSetting("DebugRenderPathing", defaults.debug_render_pathing);

	// drop links
	settings.use_drop_links = LoadBoolSetting("UseDropLinks", defaults.use_drop_links);
	settings.max_safe_drop_height = LoadNumberSetting("MaxSafeDropHeight", defaults.max_safe_drop_height);
	settings.drop_links_require_levitate = LoadBoolSetting("DropLinksRequireLevitate", defaults.drop_links_require_levitate);

	// hazard settings
	settings.hazards_enabled = LoadBoolSetting("HazardsEnabled", defaults.hazards_enabled);
	settings.hazard_default_hard = LoadNumberSetting("HazardDefaultHard", defaults.hazard_default_hard);
	settings.hazard_default_soft = LoadNumberSetting("HazardDefaultSoft", defaults.hazard_default_soft);
	settings.hazard_rampage_margin = LoadNumberSetting("HazardRampageMargin", defaults.hazard_rampage_margin);
	settings.hazard_soft_cost_mult = std::max(1.f, LoadNumberSetting("HazardSoftCostMult", defaults.hazard_soft_cost_mult));
	settings.hazard_boundary_cost_mult = std::max(1.f, LoadNumberSetting("HazardBoundaryCostMult", defaults.hazard_boundary_cost_mult));
	settings.hazard_inside_cost_mult = std::max(1.f, LoadNumberSetting("HazardInsideCostMult", defaults.hazard_inside_cost_mult));
	settings.hazard_height_extent = LoadNumberSetting("HazardHeightExtent", defaults.hazard_height_extent);
	settings.hazard_refresh_ms = std::max(50, LoadNumberSetting("HazardRefreshMs", defaults.hazard_refresh_ms));
	settings.hazard_rescan_ms = std::max(100, LoadNumberSetting("HazardRescanMs", defaults.hazard_rescan_ms));
	settings.hazard_move_epsilon = LoadNumberSetting("HazardMoveEpsilon", defaults.hazard_move_epsilon);
	settings.hazard_heading_epsilon_deg = LoadNumberSetting("HazardHeadingEpsilonDeg", defaults.hazard_heading_epsilon_deg);
	settings.hazard_postcheck_iterations = std::clamp(LoadNumberSetting("HazardPostCheckIterations", defaults.hazard_postcheck_iterations), 0, 16);
	settings.hazard_show_overlay = LoadBoolSetting("HazardShowOverlay", defaults.hazard_show_overlay);
	settings.hazard_show_map = LoadBoolSetting("HazardShowMap", defaults.hazard_show_map);
	settings.hazard_map_layer = LoadNumberSetting("HazardMapLayer", defaults.hazard_map_layer);

	// hazard autonomy
	settings.hazard_reaction_min_ms = std::max(0, LoadNumberSetting("HazardReactionMinMs", defaults.hazard_reaction_min_ms));
	settings.hazard_reaction_max_ms = std::max(settings.hazard_reaction_min_ms, LoadNumberSetting("HazardReactionMaxMs", defaults.hazard_reaction_max_ms));
	settings.hazard_retarget_min_ms = std::max(0, LoadNumberSetting("HazardRetargetMinMs", defaults.hazard_retarget_min_ms));
	settings.hazard_evade_soft = LoadBoolSetting("HazardEvadeSoft", defaults.hazard_evade_soft);
	settings.hazard_candidate_count = std::clamp(LoadNumberSetting("HazardCandidateCount", defaults.hazard_candidate_count), 4, 64);
	settings.hazard_candidate_random_weight = std::max(0.f, LoadNumberSetting("HazardCandidateRandomWeight", defaults.hazard_candidate_random_weight));
	settings.hazard_other_hazard_weight = std::max(0.f, LoadNumberSetting("HazardOtherHazardWeight", defaults.hazard_other_hazard_weight));
	settings.hazard_stop_suppress_ms = std::max(0, LoadNumberSetting("HazardStopSuppressMs", defaults.hazard_stop_suppress_ms));
	settings.hazard_manual_input_suppress_ms = std::max(0, LoadNumberSetting("HazardManualInputSuppressMs", defaults.hazard_manual_input_suppress_ms));
	settings.hazard_pause_macro = LoadBoolSetting("HazardPauseMacro", defaults.hazard_pause_macro);
	settings.hazard_pause_cwtn = LoadBoolSetting("HazardPauseCWTN", defaults.hazard_pause_cwtn);
	settings.hazard_cwtn_pause_command = LoadStringSetting("HazardCWTNPauseCommand", defaults.hazard_cwtn_pause_command);
	settings.hazard_cwtn_resume_command = LoadStringSetting("HazardCWTNResumeCommand", defaults.hazard_cwtn_resume_command);
	settings.hazard_move_start_commands = LoadStringSetting("HazardMoveStartCommands", defaults.hazard_move_start_commands);
	settings.hazard_move_end_commands = LoadStringSetting("HazardMoveEndCommands", defaults.hazard_move_end_commands);

	// navigation line settings
	NavigationLine::LineStyle defaultLineStyle;
	gNavigationLineStyle.borderColor = LoadNumberSetting<uint32_t>("VisualNavPathBorderColor", defaultLineStyle.borderColor);
	gNavigationLineStyle.hiddenColor = LoadNumberSetting<uint32_t>("VisualNavPathHiddenColor", defaultLineStyle.hiddenColor);
	gNavigationLineStyle.visibleColor = LoadNumberSetting<uint32_t>("VisualNavPathVisibleColor", defaultLineStyle.visibleColor);
	gNavigationLineStyle.linkColor = LoadNumberSetting<uint32_t>("VisualNavPathLinkColor", defaultLineStyle.linkColor);
	gNavigationLineStyle.opacity = LoadNumberSetting<float>("VisualNavPathVisibleOpacity", defaultLineStyle.opacity);
	gNavigationLineStyle.hiddenOpacity = LoadNumberSetting<float>("VisualNavPathHiddenOpacity", defaultLineStyle.hiddenOpacity);
	gNavigationLineStyle.borderWidth = LoadNumberSetting<float>("VisualNavPathBorderWidth", defaultLineStyle.borderWidth);
	gNavigationLineStyle.lineWidth = LoadNumberSetting<float>("VisualNavPathLineWidth", defaultLineStyle.lineWidth);
}

void SaveSettings(bool showMessage/* = true*/)
{
	if (showMessage)
	{
		SPDLOG_INFO("Saving settings...");
	}

	// default settings
	SaveBoolSetting("AutoBreak", g_settings.autobreak);
	SaveBoolSetting("AutoPause", g_settings.autopause);
	SaveBoolSetting("AutoReload", g_settings.autoreload);
	SaveBoolSetting("ShowUI", g_settings.show_ui);
	SaveBoolSetting("ShowNavPath", g_settings.show_nav_path);
	SaveBoolSetting("AttemptUnstuck", g_settings.attempt_unstuck);
	SaveBoolSetting("UseSpawnFloorHeight", g_settings.use_spawn_floor_height);

	SaveBoolSetting("UseFindPolygonExtents", g_settings.use_find_polygon_extents);
	SaveVec3Setting("FindPolygonExtents", g_settings.find_polygon_extents);

	SaveBoolSetting("OpenDoors", g_settings.open_doors);
	SaveBoolSetting("IgnoreScriptedDoors", g_settings.ignore_scripted_doors);

	SaveBoolSetting("MapLineEnabled", g_settings.map_line_enabled);
	SaveNumberSetting("MapLineColor", g_settings.map_line_color);
	SaveNumberSetting("MapLineLayer", g_settings.map_line_layer);

	// in game nave path
	SaveNumberSetting<uint32_t>("VisualNavPathBorderColor", gNavigationLineStyle.borderColor);
	SaveNumberSetting<uint32_t>("VisualNavPathHiddenColor", gNavigationLineStyle.hiddenColor);
	SaveNumberSetting<uint32_t>("VisualNavPathVisibleColor", gNavigationLineStyle.visibleColor);
	SaveNumberSetting<uint32_t>("VisualNavPathLinkColor", gNavigationLineStyle.linkColor);
	SaveNumberSetting<float>("VisualNavPathVisibleOpacity", gNavigationLineStyle.opacity);
	SaveNumberSetting<float>("VisualNavPathHiddenOpacity", gNavigationLineStyle.hiddenOpacity);
	SaveNumberSetting<float>("VisualNavPathBorderWidth", gNavigationLineStyle.borderWidth);
	SaveNumberSetting<float>("VisualNavPathLineWidth", gNavigationLineStyle.lineWidth);

	// debug settings
	SaveBoolSetting("DebugRenderPathing", g_settings.debug_render_pathing);

	// drop links
	SaveBoolSetting("UseDropLinks", g_settings.use_drop_links);
	SaveNumberSetting("MaxSafeDropHeight", g_settings.max_safe_drop_height);
	SaveBoolSetting("DropLinksRequireLevitate", g_settings.drop_links_require_levitate);

	// hazard settings
	SaveBoolSetting("HazardsEnabled", g_settings.hazards_enabled);
	SaveNumberSetting("HazardDefaultHard", g_settings.hazard_default_hard);
	SaveNumberSetting("HazardDefaultSoft", g_settings.hazard_default_soft);
	SaveNumberSetting("HazardRampageMargin", g_settings.hazard_rampage_margin);
	SaveNumberSetting("HazardSoftCostMult", g_settings.hazard_soft_cost_mult);
	SaveNumberSetting("HazardBoundaryCostMult", g_settings.hazard_boundary_cost_mult);
	SaveNumberSetting("HazardInsideCostMult", g_settings.hazard_inside_cost_mult);
	SaveNumberSetting("HazardHeightExtent", g_settings.hazard_height_extent);
	SaveNumberSetting("HazardRefreshMs", g_settings.hazard_refresh_ms);
	SaveNumberSetting("HazardRescanMs", g_settings.hazard_rescan_ms);
	SaveNumberSetting("HazardMoveEpsilon", g_settings.hazard_move_epsilon);
	SaveNumberSetting("HazardHeadingEpsilonDeg", g_settings.hazard_heading_epsilon_deg);
	SaveNumberSetting("HazardPostCheckIterations", g_settings.hazard_postcheck_iterations);
	SaveBoolSetting("HazardShowOverlay", g_settings.hazard_show_overlay);
	SaveBoolSetting("HazardShowMap", g_settings.hazard_show_map);
	SaveNumberSetting("HazardMapLayer", g_settings.hazard_map_layer);

	// hazard autonomy
	SaveNumberSetting("HazardReactionMinMs", g_settings.hazard_reaction_min_ms);
	SaveNumberSetting("HazardReactionMaxMs", g_settings.hazard_reaction_max_ms);
	SaveNumberSetting("HazardRetargetMinMs", g_settings.hazard_retarget_min_ms);
	SaveBoolSetting("HazardEvadeSoft", g_settings.hazard_evade_soft);
	SaveNumberSetting("HazardCandidateCount", g_settings.hazard_candidate_count);
	SaveNumberSetting("HazardCandidateRandomWeight", g_settings.hazard_candidate_random_weight);
	SaveNumberSetting("HazardOtherHazardWeight", g_settings.hazard_other_hazard_weight);
	SaveNumberSetting("HazardStopSuppressMs", g_settings.hazard_stop_suppress_ms);
	SaveNumberSetting("HazardManualInputSuppressMs", g_settings.hazard_manual_input_suppress_ms);
	SaveBoolSetting("HazardPauseMacro", g_settings.hazard_pause_macro);
	SaveBoolSetting("HazardPauseCWTN", g_settings.hazard_pause_cwtn);
	SaveStringSetting("HazardCWTNPauseCommand", g_settings.hazard_cwtn_pause_command);
	SaveStringSetting("HazardCWTNResumeCommand", g_settings.hazard_cwtn_resume_command);
	SaveStringSetting("HazardMoveStartCommands", g_settings.hazard_move_start_commands);
	SaveStringSetting("HazardMoveEndCommands", g_settings.hazard_move_end_commands);
}

bool ParseIniCommand(const char* command)
{
	char szKeyName[MAX_STRING];
	char szValue[MAX_STRING];

	GetArg(szKeyName, command, 1);
	GetArg(szValue, command, 2);

	if (szKeyName[0] != 0 && szValue[0] != 0)
	{
		WritePrivateProfileStringA("Settings", szKeyName, szValue, INIFileName);

		// cycle settings to clear any wierd inputs
		LoadSettings(false);
		SaveSettings(false);

		return true;
	}

	return false;
}

bool ReadIniSetting(const char* keyName, char* pBuffer, size_t length)
{
	return GetPrivateProfileString("Settings", keyName, "", pBuffer, length, INIFileName) != 0;
}

//----------------------------------------------------------------------------

} // namespace nav
