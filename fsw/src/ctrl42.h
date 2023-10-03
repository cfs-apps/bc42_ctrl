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
**    Provide an adapter to 42's flight software controller algorithm
**
**  Notes:
**   1. This is part of prototype effort to port a 42 simulator FSW controller
**      component into a cFS-based application
**   2. This object serves as a wrapper/adapter for the 42 FSW module. The cFS
**      application should use this adapter for all interactions to/from the 
**      42 interface. 
**   3. In a more complex design individual objects should be created for
**      sensors and actuators.
*/
#ifndef _ctrl42_
#define _ctrl42_

/*
** Includes
*/

#include "bc42_intf_eds_typedefs.h"

#include "app_cfg.h"
#include "bc42_lib.h"
#include "ctrl42_tbl.h"

/***********************/
/** Macro Definitions **/
/***********************/

/*
** Event Message IDs
*/


#define CTRL42_SET_BOOL_OVR_EID         (CTRL42_BASE_EID + 0)
#define CTRL42_SET_CTRL_MODE_EID        (CTRL42_BASE_EID + 1)
#define CTRL42_ENA_DEBUG_CMD_EID        (CTRL42_BASE_EID + 2)
#define CTRL42_DIS_DEBUG_CMD_EID        (CTRL42_BASE_EID + 3)
#define CTRL42_WHL_TARGET_MOM_CMD_EID   (CTRL42_BASE_EID + 4)
#define CTRL42_INIT_CONTROLLER_EID      (CTRL42_BASE_EID + 5)
#define CTRL42_DEBUG_CONTROLLER_EID     (CTRL42_BASE_EID + 6)
#define CTRL42_ACCEPT_NEW_TBL_EID       (CTRL42_BASE_EID + 7)


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
** BC42_CTRL Class
*/

typedef struct
{


   /*
   ** Telemetry
   */
   
   BC42_CTRL_ControllerTlm_t     ControllerTlm;
   BC42_CTRL_ControlGainsTlm_t   ControlGainsTlm;
   BC42_INTF_ActuatorCmdMsg_t    ActuatorCmdMsg;

   /*
   ** Contained Objects
   */
   
   BC42_Class_t      *Bc42;
   CTRL42_TBL_Class_t Tbl;

   /*
   ** CTRL42 Data 
   */
   
   uint32  CtrlExeCnt;

   int16   TakeSciInitCyc;
   int16   TakeSciInitCycCtr;
   int16   TakeSciTransCyc;
   int16   TakeSciTransCycCtr;

   BC42_CTRL_Bool42State_Enum_t  BoolOverride[BC42_CTRL_Bool42State_COUNT];
   uint16  CtrlMode;
   
   float   Hcmd[BC42_NWHL]; /* TODO - 42 controller command interface */
   
   bool      DebugEnabled;
   osal_id_t DebugFileHandle;
   char      DebugFilename[OS_MAX_PATH_LEN];
   
} CTRL42_Class_t;


/************************/
/** Exported Functions **/
/************************/

/******************************************************************************
** Function: CTRL42_Constructor
**
** Initialize a CTRL42 object.
**
** Notes:
**   1. This must be called prior to any other function.
**
*/
void CTRL42_Constructor(CTRL42_Class_t *Ctrl42Obj, const INITBL_Class_t *IniTbl,
                        TBLMGR_Class_t *TblMgr);


/******************************************************************************
** Function: CTRL42_DisableDebugLogCmd
**
** TODO - Add file command parameter
*/
bool CTRL42_DisableDebugLogCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr);


/******************************************************************************
** Function: CTRL42_EnableDebugLogCmd
**
** TODO - Add file command parameter
*/
bool CTRL42_EnableDebugLogCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr);


/******************************************************************************
** Function:  CTRL42_ResetStatus
**
*/
void CTRL42_ResetStatus(void);


/******************************************************************************
** Function: CTRL42_Run42Fsw
**
** Run the 42 simulator's FSW algorithms
**
*/
void CTRL42_Run42Fsw(BC42_INTF_SensorDataMsg_t *SensorDataMsg);


/******************************************************************************
** Function: CTRL42_SendCtrlGainsCmd
**
** Send the control gains telemetry packet containing the gains from the
** control table.
*/
bool CTRL42_SendCtrlGainsTlmCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr);

   
/******************************************************************************
** Function: CTRL42_SetBoolOvrStateCmd
**
** Set/Clear the command specified 
**
** Notes:
**   1. This is useful for fault testing.
*/
bool CTRL42_SetBoolOvrStateCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr);


/******************************************************************************
** Function: CTRL42_SetCtrlModeCmd
**
** Currently controller doesn't have modes so this command only lets the
** user force a controller initialization.
*/
bool CTRL42_SetCtrlModeCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr);


/******************************************************************************
** Function: CTRL42_SetWheelTargetMomCmd
**
** Set the wheel target momentum.
*/
bool CTRL42_SetWheelTargetMomCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr);


#endif /* _ctrl42_ */
