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
**    Define the 42 Attitude Control (AC) application
**
**  Notes:
**    1. This is part of prototype effort to port a 42 simulator FSW controller
**       component into a cFS-based application. See bc42_lib.h for more
**       information.
**
*/
#ifndef _bc42_ctrl_app_
#define _bc42_ctrl_app_

/*
** Includes
*/

#include "app_cfg.h"
#include "ctrl42.h"

/***********************/
/** Macro Definitions **/
/***********************/

/*
** Events Message IDs
*/

#define BC42_CTRL_INIT_APP_EID          (BC42_CTRL_BASE_EID + 0)
#define BC42_CTRL_EXIT_EID              (BC42_CTRL_BASE_EID + 1)
#define BC42_CTRL_NOOP_EID              (BC42_CTRL_BASE_EID + 2)
#define BC42_CTRL_PROCESS_CMD_PIPE_EID  (BC42_CTRL_BASE_EID + 3)


/**********************/
/** Type Definitions **/
/**********************/

/******************************************************************************
** Command Packets
** - See bc42_ctrl.xml EDS definitions
*/

/******************************************************************************
** Telemetry Packets
** - See bc42_ctrl.xml EDS definitions
*/


/******************************************************************************
** BC42_CTRL_APP Class
*/

typedef struct
{

   /*
   ** App Framework
   */

   INITBL_Class_t    IniTbl; 
   CFE_SB_PipeId_t   CmdPipe;
   CMDMGR_Class_t    CmdMgr;
   TBLMGR_Class_t    TblMgr;

   /*
   ** App State
   */

   uint32          PerfId;
   CFE_SB_MsgId_t  CmdMid;
   CFE_SB_MsgId_t  SendStatusTlmMid;
   CFE_SB_MsgId_t  StatusTlmMid;
   CFE_SB_MsgId_t  SensorDataMsgMid;
   
   /*
   ** Telemetry Packets
   */
   
   BC42_CTRL_StatusTlm_t  StatusTlm;
   
   /*
   ** App Objects
   */

   CTRL42_Class_t  Ctrl42;


} BC42_CTRL_APP_Class_t;


/*******************/
/** Exported Data **/
/*******************/

extern BC42_CTRL_APP_Class_t Bc42Ctrl;


/************************/
/** Exported Functions **/
/************************/

/******************************************************************************
** Function: BC42_CTRL_AppMain
**
*/
void BC42_CTRL_AppMain(void);


/******************************************************************************
** Function: BC42_CTRL_NoOpCmd
**
*/
bool BC42_CTRL_NoOpCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr);


/******************************************************************************
** Function: BC42_CTRL_ResetAppCmd
**
*/
bool BC42_CTRL_ResetAppCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr);


#endif /* _bc42_ctrl_app_ */
