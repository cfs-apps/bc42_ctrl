/*
**  Copyright 2022 bitValence, Inc.
**  All Rights Reserved.
**
**  This program is free software; you can modify and/or redistribute it
**  under the terms of the GNU Affero General Public License
**  as published by the Free Software Foundation; version 3 with
**  attribution addendums as found in the LICENSE.txt
**
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU Affero General Public License for more details.
**
**  Purpose:
**    Define platform configurations for the 42 controller application
**
**  Notes:
**    1. Compile-time configurations that are applied to each platform
**       deployment of this app.
**    2. These definitions should minimal and only contain parameters that
**       need to be configurable at compile time that allows users to tuned
**       the app for a particular platform. Use app_c_demo_mission_cfg.h
**       for compile-time parameters that need to be tuned at the mission
**       level across all platform deployments. Use app_cfg.h for compile-time
**       parameters that don't need to be configured when an app is deployed
**       but are useful to be parameterized for maintaining the app and use
**       the JSON initialization file for parameters that can be defined at
**       runtime.
**
*/
#ifndef _bc42_ctrl_platform_cfg_
#define _bc42_ctrl_platform_cfg_

/*
** Includes
*/

#include "bc42_ctrl_mission_cfg.h"


/******************************************************************************
** Platform Deployment Configurations
*/

#define BC42_CTRL_PLATFORM_REV   0
#define BC42_CTRL_INI_FILENAME   "/cf/bc42_ctrl_ini.json"


#endif /* _bc42_ctrl_platform_cfg_ */

