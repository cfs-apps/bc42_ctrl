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
**    Implement the 42 Attitude Control (AC) application
**
**  Notes:
**    1. This is part of prototype effort to port a 42 simulator FSW controller
**       component into a cFS-based application 
**
*/

/*
** Includes
*/

#include <string.h>
#include "bc42_ctrl_app.h"
#include "bc42_ctrl_eds_cc.h"


/***********************/
/** Macro Definitions **/
/***********************/

/* Convenience macros */
#define  INITBL_OBJ   (&(Bc42Ctrl.IniTbl))
#define  CMDMGR_OBJ   (&(Bc42Ctrl.CmdMgr))
#define  TBLMGR_OBJ   (&(Bc42Ctrl.TblMgr))
#define  CTRL42_OBJ   (&(Bc42Ctrl.Ctrl42))


/*******************************/
/** Local Function Prototypes **/
/*******************************/

static int32 InitApp(void);
static int32 ProcessCmdPipe(void);
static void SendHousekeepingPkt(void);


/**********************/
/** File Global Data **/
/**********************/

/* 
** Must match DECLARE ENUM() declaration in app_cfg.h
** Defines "static INILIB_CfgEnum IniCfgEnum"
*/
DEFINE_ENUM(Config,APP_CONFIG)  

static CFE_EVS_BinFilter_t  EventFilters[] =
{  
   /* Event ID                           Mask */
   {CTRL42_DEBUG_CONTROLLER_EID,  CFE_EVS_FIRST_64_STOP}, //CFE_EVS_NO_FILTER
};

/*****************/
/** Global Data **/
/*****************/

BC42_CTRL_APP_Class_t   Bc42Ctrl;


/******************************************************************************
** Function: BC42_CTRL_AppMain
**
*/
void BC42_CTRL_AppMain(void)
{

   uint32 RunStatus = CFE_ES_RunStatus_APP_ERROR;

   CFE_EVS_Register(EventFilters, sizeof(EventFilters)/sizeof(CFE_EVS_BinFilter_t),
                    CFE_EVS_EventFilter_BINARY);

   if (InitApp() == CFE_SUCCESS)      /* Performs initial CFE_ES_PerfLogEntry() call */
   {
      RunStatus = CFE_ES_RunStatus_APP_RUN; 
   }

   /*
   ** Main process loop
   */
   while (CFE_ES_RunLoop(&RunStatus)) 
   {

      RunStatus = ProcessCmdPipe();
      
   } /* End CFE_ES_RunLoop */


   /* Write to system log in case events not working */

   CFE_ES_WriteToSysLog("Bascamp 42 Controller App terminating, run status = 0x%08X\n", RunStatus);   /* Use SysLog, events may not be working */

   CFE_EVS_SendEvent(BC42_CTRL_EXIT_EID, CFE_EVS_EventType_CRITICAL, "Bascamp 42 Controller App terminating, run status = 0x%08X", RunStatus);

   CFE_ES_ExitApp(RunStatus);  /* Let cFE kill the task (and any child tasks) */

} /* End of BC42_CTRL_AppMain() */


/******************************************************************************
** Function: BC42_CTRL_NoOpCmd
**
*/

bool BC42_CTRL_NoOpCmd(void* ObjDataPtr, const CFE_MSG_Message_t *MsgPtr)
{

   CFE_EVS_SendEvent (BC42_CTRL_NOOP_EID, CFE_EVS_EventType_INFORMATION,
                      "No operation command received for BC42_CTRL App version %d.%d.%d",
                      BC42_CTRL_MAJOR_VER, BC42_CTRL_MINOR_VER, BC42_CTRL_PLATFORM_REV);

   return true;


} /* End BC42_CTRL_NoOpCmd() */


/******************************************************************************
** Function: BC42_CTRL_ResetAppCmd
**
*/

bool BC42_CTRL_ResetAppCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr)
{

   CMDMGR_ResetStatus(CMDMGR_OBJ);
   TBLMGR_ResetStatus(TBLMGR_OBJ);

   CTRL42_ResetStatus();
	  
   return true;

} /* End BC42_CTRL_ResetAppCmd() */


/******************************************************************************
** Function: InitApp
**
*/
static int32 InitApp(void)
{

   int32 RetStatus = APP_C_FW_CFS_ERROR;

   CFE_PSP_MemSet((void*)&Bc42Ctrl, 0, sizeof(BC42_CTRL_APP_Class_t));
 
   /*
   ** Read JSON INI Table & class variable defaults defined in JSON  
   */ 
 
   if (INITBL_Constructor(INITBL_OBJ, BC42_CTRL_INI_FILENAME, &IniCfgEnum))
   {
      
      Bc42Ctrl.PerfId = INITBL_GetIntConfig(INITBL_OBJ, APP_PERF_ID);  

      Bc42Ctrl.CmdMid            = CFE_SB_ValueToMsgId(INITBL_GetIntConfig(INITBL_OBJ, CFG_BC42_CTRL_CMD_TOPICID));
      Bc42Ctrl.StatusTlmMid      = CFE_SB_ValueToMsgId(INITBL_GetIntConfig(INITBL_OBJ, CFG_BC42_CTRL_STATUS_TLM_TOPICID));
      Bc42Ctrl.ExecuteMid        = CFE_SB_ValueToMsgId(INITBL_GetIntConfig(INITBL_OBJ, CFG_BC42_CTRL_EXECUTE_TOPICID));
      Bc42Ctrl.SensorDataMsgMid  = CFE_SB_ValueToMsgId(INITBL_GetIntConfig(INITBL_OBJ, CFG_BC42_INTF_SENSOR_DATA_MSG_TOPICID));

      /* Must constructor table manager prior to any app objects that contain tables */
      TBLMGR_Constructor(TBLMGR_OBJ, INITBL_GetStrConfig(INITBL_OBJ, CFG_APP_CFE_NAME));

      /*
      ** Initialize objects 
      */
      CTRL42_Constructor(CTRL42_OBJ, INITBL_OBJ, TBLMGR_OBJ);
 
      /*
      ** Initialize app level interfaces
      */
      CFE_SB_CreatePipe(&Bc42Ctrl.CmdPipe, INITBL_GetIntConfig(INITBL_OBJ, CFG_APP_CMD_PIPE_DEPTH), INITBL_GetStrConfig(INITBL_OBJ, CFG_APP_CMD_PIPE_NAME));
      CFE_SB_Subscribe(Bc42Ctrl.CmdMid, Bc42Ctrl.CmdPipe);
      CFE_SB_Subscribe(Bc42Ctrl.ExecuteMid, Bc42Ctrl.CmdPipe);
      CFE_SB_Subscribe(Bc42Ctrl.SensorDataMsgMid, Bc42Ctrl.CmdPipe);
   
      CMDMGR_Constructor(CMDMGR_OBJ);
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BC42_CTRL_NOOP_CC,           NULL, BC42_CTRL_NoOpCmd,     0);
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BC42_CTRL_RESET_CC,          NULL, BC42_CTRL_ResetAppCmd, 0);
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BC42_CTRL_LOAD_TBL_CC, TBLMGR_OBJ, TBLMGR_LoadTblCmd,     sizeof(BC42_CTRL_LoadTbl_CmdPayload_t));
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BC42_CTRL_DUMP_TBL_CC, TBLMGR_OBJ, TBLMGR_DumpTblCmd,     sizeof(BC42_CTRL_DumpTbl_CmdPayload_t));
      
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BC42_CTRL_SEND_CTRL_GAINS_TLM_CC,  CTRL42_OBJ, CTRL42_SendCtrlGainsTlmCmd,  0);
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BC42_CTRL_SET_CTRL_MODE_CC,        CTRL42_OBJ, CTRL42_SetCtrlModeCmd,       sizeof(BC42_CTRL_SetCtrlMode_CmdPayload_t));
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BC42_CTRL_SET_BOOL_OVR_STATE_CC,   CTRL42_OBJ, CTRL42_SetBoolOvrStateCmd,   sizeof(BC42_CTRL_SetBoolOvrState_CmdPayload_t));
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BC42_CTRL_SET_WHEEL_TARGET_MOM_CC, CTRL42_OBJ, CTRL42_SetWheelTargetMomCmd, sizeof(BC42_CTRL_SetWheelTargetMom_CmdPayload_t));
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BC42_CTRL_ENABLE_DEBUG_LOG_CC,     CTRL42_OBJ, CTRL42_EnableDebugLogCmd,    0);
      CMDMGR_RegisterFunc(CMDMGR_OBJ, BC42_CTRL_DISABLE_DEBUG_LOG_CC,    CTRL42_OBJ, CTRL42_DisableDebugLogCmd,   0);

      CFE_MSG_Init(CFE_MSG_PTR(Bc42Ctrl.StatusTlm.TelemetryHeader), CFE_SB_ValueToMsgId(INITBL_GetIntConfig(INITBL_OBJ, CFG_BC42_CTRL_STATUS_TLM_TOPICID)), sizeof(BC42_CTRL_StatusTlm_t));
      
      /*
      ** Application startup event message
      */
      CFE_EVS_SendEvent(BC42_CTRL_INIT_APP_EID, CFE_EVS_EventType_INFORMATION,
                        "BC42_CTRL App Initialized. Version %d.%d.%d",
                        BC42_CTRL_MAJOR_VER, BC42_CTRL_MINOR_VER, BC42_CTRL_PLATFORM_REV);

      RetStatus = CFE_SUCCESS;
      
   } /* End if INITBL Constructed */ 
   
   return  RetStatus;

} /* End of InitApp() */


/******************************************************************************
** Function: ProcessCmdPipe
**
** Notes:
**   1. TODO: Decide on execution design and relationshop with BC42_INTF.
**      - Use scheduler to drive execution and sensor data messages are flushed
**        and run controller once on the last sensor data packet. Status packet
**        sent each scheduler execution. 
**      - Execute controller for each sensor data packet. Status packet sent for
**        controller execution. It can be filtered down stream. 
*/
static int32 ProcessCmdPipe(void)
{
   
   int32   RetStatus = CFE_ES_RunStatus_APP_RUN;
   int32   SbStatus;
   int32   MsgStatus;
   int32   SensorMsgCnt = 0;
   
   CFE_SB_Buffer_t  *SbBufPtr;
   CFE_SB_MsgId_t   MsgId = CFE_SB_INVALID_MSG_ID;
   

   CFE_ES_PerfLogExit(Bc42Ctrl.PerfId);
   SbStatus = CFE_SB_ReceiveBuffer(&SbBufPtr, Bc42Ctrl.CmdPipe, CFE_SB_PEND_FOREVER);
   CFE_ES_PerfLogEntry(Bc42Ctrl.PerfId);
   
   do
   {
    
      if (SbStatus == CFE_SUCCESS)
      {
         MsgStatus = CFE_MSG_GetMsgId(&SbBufPtr->Msg, &MsgId);

         if (MsgStatus == CFE_SUCCESS)
         {
            if (CFE_SB_MsgId_Equal(MsgId, Bc42Ctrl.CmdMid))
            {
               CMDMGR_DispatchFunc(CMDMGR_OBJ, &SbBufPtr->Msg);
            }
            else if (CFE_SB_MsgId_Equal(MsgId, Bc42Ctrl.ExecuteMid))
            {               
               SendHousekeepingPkt();
            }
            else if (CFE_SB_MsgId_Equal(MsgId, Bc42Ctrl.SensorDataMsgMid))
            {
               SensorMsgCnt++;
               CTRL42_Run42Fsw((BC42_INTF_SensorDataMsg_t *)&SbBufPtr->Msg);
            }
            else
            {            
               CFE_EVS_SendEvent(BC42_CTRL_PROCESS_CMD_PIPE_EID, CFE_EVS_EventType_ERROR,
                                 "Received invalid command packet, MID = 0x%04X(%d)", 
                                 CFE_SB_MsgIdToValue(MsgId), CFE_SB_MsgIdToValue(MsgId));
            }
         } /* End valid message ID */
      } /* End if SB received a packet */
      
      CFE_ES_PerfLogExit(Bc42Ctrl.PerfId);
      SbStatus = CFE_SB_ReceiveBuffer(&SbBufPtr, Bc42Ctrl.CmdPipe, CFE_SB_POLL);
      CFE_ES_PerfLogEntry(Bc42Ctrl.PerfId);      

   } while (SbStatus == CFE_SUCCESS);

   if (SbStatus != CFE_SB_NO_MESSAGE)
   {
      RetStatus = CFE_ES_RunStatus_APP_ERROR;
   }

   if (SensorMsgCnt > 1)
   {
      CFE_EVS_SendEvent(BC42_CTRL_PROCESS_CMD_PIPE_EID, CFE_EVS_EventType_INFORMATION,
                        "Processed %d sensor data messages in one execution cycle",SensorMsgCnt);      
   }

   return RetStatus;
   
} /* End ProcessCmdPipe() */

/******************************************************************************
** Function: SendHousekeepingPkt
**
*/
static void SendHousekeepingPkt(void)
{
  
   /* Good design practice in case app expands to more than one table */
   const TBLMGR_Tbl_t *LastTbl = TBLMGR_GetLastTblStatus(TBLMGR_OBJ);

   BC42_CTRL_StatusTlm_Payload_t *Payload = &Bc42Ctrl.StatusTlm.Payload;
 	  
   /*
   ** F42 Application Data
   */

   Payload->ValidCmdCnt   = Bc42Ctrl.CmdMgr.ValidCmdCnt;
   Payload->InvalidCmdCnt = Bc42Ctrl.CmdMgr.InvalidCmdCnt;

   Payload->LastTblAction       = LastTbl->LastAction;
   Payload->LastTblActionStatus = LastTbl->LastActionStatus;

   /*.
   ** CTRL42 Data
   */

   Payload->ControlExecutionCnt = Bc42Ctrl.Ctrl42.CtrlExeCnt;
   Payload->ControlMode         = Bc42Ctrl.Ctrl42.CtrlMode;
   Payload->OverrideSunValid    = Bc42Ctrl.Ctrl42.BoolOverride[BC42_CTRL_Bool42Id_Sun_VALID];

   CFE_SB_TimeStampMsg(CFE_MSG_PTR(Bc42Ctrl.StatusTlm.TelemetryHeader));
   CFE_SB_TransmitMsg(CFE_MSG_PTR(Bc42Ctrl.StatusTlm.TelemetryHeader), true);

} /* End SendHousekeepingPkt() */
