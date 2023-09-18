project(CFS_BC42_CTRL C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)

include_directories(${bc42__intf_MISSION_DIR}/fsw/src)
include_directories(${bc42__intf_MISSION_DIR}/fsw/platform_inc)
include_directories(${bc42_lib_MISSION_DIR}/fsw/app_inc)
include_directories(${app_c_fw_MISSION_DIR}/fsw/app_inc)
include_directories(${app_c_fw_MISSION_DIR}/fsw/platform_inc)
include_directories(${app_c_fw_MISSION_DIR}/fsw/mission_inc)

aux_source_directory(fsw/src APP_SRC_FILES)

# Create the app module
add_cfe_app(bc42_ctrl ${APP_SRC_FILES})
target_link_libraries (bc42_ctrl m)


