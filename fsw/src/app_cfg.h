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
**    Define application configurations for the 42 Interface application
**
**  Notes:
**    1. This is part of prototype effort to port a 42 simulator FSW controller
**       component into a cFS-based application 
**    2. These macros can only be built with the application and can't
**       have a platform scope because the same app_cfg.h file name is used for
**       all applications following the object-based application design.
*/

#ifndef _app_cfg_
#define _app_cfg_

/*
** Includes
*/

#include "app_c_fw.h"
#include "bc42_ctrl_platform_cfg.h"
#include "bc42_ctrl_eds_typedefs.h"


/******************************************************************************
** Versions
**
** 1.0 - Initial Basecamp release created from OpenSatKit and upgraded to the
**       latest 42 version
*/

#define  BC42_CTRL_MAJOR_VER   1
#define  BC42_CTRL_MINOR_VER   0


/******************************************************************************
** JSON init file definitions/declarations.
** - See app_c_demo::app_cfg.h for how to define configuration macros 
*/

#define CFG_APP_CFE_NAME        APP_CFE_NAME
#define CFG_APP_PERF_ID         APP_PERF_ID
#define CFG_APP_CMD_PIPE_NAME   APP_CMD_PIPE_NAME
#define CFG_APP_CMD_PIPE_DEPTH  APP_CMD_PIPE_DEPTH
#define CFG_APP_DEBUG_FILE      APP_DEBUG_FILE

#define CFG_BC42_CTRL_CMD_TOPICID              BC42_CTRL_CMD_TOPICID
#define CFG_BC42_CTRL_STATUS_TLM_TOPICID       BC42_CTRL_STATUS_TLM_TOPICID
#define CFG_BC42_INTF_SENSOR_DATA_MSG_TOPICID  BC42_INTF_SENSOR_DATA_MSG_TOPICID
#define CFG_BC42_INTF_ACTUATOR_CMD_MSG_TOPICID BC42_INTF_ACTUATOR_CMD_MSG_TOPICID
#define CFG_BC_SCH_1_HZ_TOPICID                BC_SCH_1_HZ_TOPICID
#define CFG_BC42_INTF_SENSOR_DATA_MSG_TIMEOUT  BC42_INTF_SENSOR_DATA_MSG_TIMEOUT /* Pend timeout (ms) for sensor data read */

#define CFG_CTRL42_TAKE_SCI_INIT_CYC  CTRL42_TAKE_SCI_INIT_CYC   /* Number of control cycles before start computing take science flag */ 
#define CFG_CTRL42_TAKE_SCI_TRANS_CYC CTRL42_TAKE_SCI_TRANS_CYC  /* Number of control cycles for new value to be considered steady state for a transition */

#define CFG_CRTL42_DEBUG_FILE      CRTL42_DEBUG_FILE
#define CFG_CTRL42_TBL_LOAD_FILE   CTRL42_TBL_LOAD_FILE
#define CFG_CTRL42_TBL_DUMP_FILE   CTRL42_TBL_DUMP_FILE

#define APP_CONFIG(XX) \
   XX(APP_CFE_NAME,char*) \
   XX(APP_PERF_ID,uint32) \
   XX(APP_CMD_PIPE_NAME,char*) \
   XX(APP_CMD_PIPE_DEPTH,uint32) \
   XX(APP_DEBUG_FILE,char*) \
   XX(BC42_CTRL_CMD_TOPICID,uint32) \
   XX(BC42_CTRL_STATUS_TLM_TOPICID,uint32) \
   XX(BC42_INTF_SENSOR_DATA_MSG_TOPICID,uint32) \
   XX(BC42_INTF_ACTUATOR_CMD_MSG_TOPICID,uint32) \
   XX(BC_SCH_1_HZ_TOPICID,uint32) \
   XX(BC42_INTF_SENSOR_DATA_MSG_TIMEOUT,uint32) \
   XX(CTRL42_TAKE_SCI_INIT_CNT,uint32) \
   XX(CTRL42_TAKE_SCI_TRANS_CNT,uint32) \
   XX(CRTL42_DEBUG_FILE,char*) \
   XX(CTRL42_TBL_LOAD_FILE,char*) \
   XX(CTRL42_TBL_DUMP_FILE,char*) \
    

DECLARE_ENUM(Config,APP_CONFIG)


/******************************************************************************
** Event Macros
**
** Define the base event message IDs used by each object/component used by the
** application. There are no automated checks to ensure an ID range is not
** exceeded so it is the developer's responsibility to verify the ranges. 
*/

#define BC42_CTRL_BASE_EID   (APP_C_FW_APP_BASE_EID +  0)
#define CTRL42_BASE_EID      (APP_C_FW_APP_BASE_EID + 20)
#define CTRL42_TBL_BASE_EID  (APP_C_FW_APP_BASE_EID + 40)


/*
** One event ID is used for all initialization debug messages. Uncomment one of
** the F42_INIT_EVS_TYPE definitions. Set it to INFORMATION if you want to
** see the events during initialization. This is opposite to what you'd expect 
** because INFORMATION messages are enabled by default when an app is loaded.
*/

#define BC42_CTRL_INIT_DEBUG_EID 999
#define BC42_CTRL_INIT_EVS_TYPE CFE_EVS_DEBUG
//#define BC42_CTRL_INIT_EVS_TYPE CFE_EVS_INFORMATION


/******************************************************************************
** CTRL42 Configurations
*/


#endif /* _app_cfg_ */
