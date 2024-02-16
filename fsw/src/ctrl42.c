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
**    Implement the adapter to 42's flight software controller algorithm
**
**  Notes:
**    1. This is part of prototype effort to port a 42 simulator FSW controller
**       component into a cFS-based application.
**    2. The 42 AcType data structure is used unchanged and serves as the I/O
**       between this adapter and the 42 code.
**    3. This object serves as a wrapper/adapter for the 42 FSW module. The cFS
**       FSW controller application should use this adapter for all interactions
**       to/from the 42 interface. I42's interface object IF42 defines the packets
**       that are exchanged between the two apps. If 
**       i42 is changed/replaced to interface with hardware then only this adapter
**       object will need to change.
**    4. Since this is an educational app many events are defined as informational. A
**       flight app should minimize "event clutter" and define some of these as debug.
**    5. TODO - Fix partial & full table load/commit process. Right now a quick & dirty
**       copy to local structure and then update 42's FSW structure. Get rid of
**       local adapter table copy and add a commit function that is called from
**       the control table load function. Think through initial values from a
**       table or use #defines
*/

/*
** Includes
*/

#include "ctrl42.h"

/***********************/
/** Macro Definitions **/
/***********************/

/* Ac struct access macros */
#define AC42          &(Ctrl42->Bc42->AcVar)  
#define AC42_(field)  (Ctrl42->Bc42->AcVar.field)

/*
** Global File Data
*/

static CTRL42_Class_t *Ctrl42 = NULL;


/*******************************/
/** Local Function Prototypes **/
/*******************************/

static void AcceptNewTbl(void);
static void SendAcStructTlm(void);
static void SensorDataMsgToAcStruct(const BC42_INTF_SensorDataMsg_t *SensorDataMsg);
static void SetTakeSci(void);
static const char *BoolOverrideStr(BC42_CTRL_Bool42State_Enum_t State);


/******************************************************************************
** Function: CTRL42_Constructor
**
** Initialize a CTRL42 object.
**
** Notes:
**   1. This must be called prior to any other function.
**   2. Assumes IF42 has constructed osk_42_lib's AC42 shared data structure 
**
*/
void CTRL42_Constructor(CTRL42_Class_t *Ctrl42Obj, const INITBL_Class_t *IniTbl,
                        TBLMGR_Class_t *TblMgr)
{

   int i;
   
   Ctrl42 = Ctrl42Obj;

   /* If a class state variable can't default to zero then must be set after this */
   CFE_PSP_MemSet((void*)Ctrl42, 0, sizeof(CTRL42_Class_t));
   
   /* Down counters */
   Ctrl42->TakeSciInitCyc  = INITBL_GetIntConfig(IniTbl, CFG_CTRL42_TAKE_SCI_INIT_CYC);
   Ctrl42->TakeSciTransCyc = INITBL_GetIntConfig(IniTbl, CFG_CTRL42_TAKE_SCI_TRANS_CYC);
   
   for (i=BC42_CTRL_Bool42State_Enum_t_MIN; i<=BC42_CTRL_Bool42State_Enum_t_MAX; i++)
   {
      Ctrl42->BoolOverride[i] = BC42_CTRL_Bool42State_USE_SIM;
   }
   
   strncpy (Ctrl42->DebugFilename, INITBL_GetStrConfig(IniTbl, CFG_CTRL42_DEBUG_FILE),OS_MAX_PATH_LEN);
   Ctrl42->DebugFilename[OS_MAX_PATH_LEN-1] = '\0';
   
   CTRL42_TBL_Constructor(&(Ctrl42->Tbl), AcceptNewTbl);
   TBLMGR_RegisterTblWithDef(TblMgr, CTRL42_TBL_NAME, CTRL42_TBL_LoadCmd, CTRL42_TBL_DumpCmd,  
                             INITBL_GetStrConfig(IniTbl, CFG_CTRL42_TBL_LOAD_FILE));

   CFE_MSG_Init(CFE_MSG_PTR(Ctrl42->ActuatorCmdMsg.TelemetryHeader), 
                CFE_SB_ValueToMsgId(INITBL_GetIntConfig(IniTbl, CFG_BC42_INTF_ACTUATOR_CMD_MSG_TOPICID)),
                sizeof(BC42_INTF_ActuatorCmdMsg_t));

   CFE_MSG_Init(CFE_MSG_PTR(Ctrl42->ControllerTlm.TelemetryHeader), 
                CFE_SB_ValueToMsgId(INITBL_GetIntConfig(IniTbl, CFG_BC42_CTRL_CONTROLLER_TLM_TOPICID)),
                sizeof(BC42_CTRL_ControllerTlm_t));
                
   CFE_MSG_Init(CFE_MSG_PTR(Ctrl42->ControlGainsTlm.TelemetryHeader), 
                CFE_SB_ValueToMsgId(INITBL_GetIntConfig(IniTbl, CFG_BC42_CTRL_CONTROL_GAINS_TLM_TOPICID)),
                sizeof(BC42_CTRL_ControlGainsTlm_t));
                                
                             
} /* End CTRL42_Constructor() */


/******************************************************************************
** Function: CTRL42_DisableDebugLogCmd
**
*/
bool CTRL42_DisableDebugLogCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr)
{

   bool  RetStatus = true;

   if (Ctrl42->DebugEnabled)
   {
      Ctrl42->DebugEnabled = false;
      OS_close(Ctrl42->DebugFileHandle);
      CFE_EVS_SendEvent(CTRL42_DIS_DEBUG_CMD_EID, CFE_EVS_EventType_INFORMATION,
                        "Debug log file %s closed", Ctrl42->DebugFilename);
   }
   else
   {
      CFE_EVS_SendEvent(CTRL42_DIS_DEBUG_CMD_EID, CFE_EVS_EventType_INFORMATION,
                        "Disable debug command had no effect, debug log not currently enabled.");
   }
              
   return RetStatus;

} /* End CTRL42_DisableDebugLogCmd() */


/******************************************************************************
** Function: CTRL42_EnableDebugLogCmd
**
** TODO - Add file command parameter
*/
bool CTRL42_EnableDebugLogCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr)
{

   bool          RetStatus = true;
   int32         SysStatus;
   os_err_name_t OsErrStr;

   if (Ctrl42->DebugEnabled == true)
   {
      CFE_EVS_SendEvent(CTRL42_ENA_DEBUG_CMD_EID, CFE_EVS_EventType_ERROR, 
                        "Enable debug command had no effect, debug log already enabled");
   }
   else
   {
      
      SysStatus = OS_OpenCreate(&Ctrl42->DebugFileHandle, Ctrl42->DebugFilename, 
                                OS_FILE_FLAG_CREATE, OS_READ_WRITE);
      
      if (SysStatus == OS_SUCCESS)
      {   
         Ctrl42->DebugEnabled = true;
         CFE_EVS_SendEvent(CTRL42_ENA_DEBUG_CMD_EID, CFE_EVS_EventType_INFORMATION,
                           "Created debug log file %s",Ctrl42->DebugFilename);
         RetStatus = true;
      }
      else 
      {
         OS_GetErrorName(SysStatus, &OsErrStr);
         CFE_EVS_SendEvent(CTRL42_ENA_DEBUG_CMD_EID, CFE_EVS_EventType_ERROR, 
         "Error creating debug log file %s. Status = %s", Ctrl42->DebugFilename, OsErrStr);
      
      } /* End if error creating file */
   } /* End if debug not enabled */

   return RetStatus;

} /* End CTRL42_EnableDebugLogCmd() */


/******************************************************************************
** Function:  CTRL42_ResetStatus
**
*/
void CTRL42_ResetStatus(void)
{
  
   CTRL42_TBL_ResetStatus();
  
   Ctrl42->CtrlExeCnt = 0;

} /* End CTRL42_ResetStatus() */


/******************************************************************************
** Function: CTRL42_Run42Fsw
**
** Run the 42 simulator's FSW control law. 42's Ac structure is used for
** all sensor/actuator data I/O. 
**
*/
void CTRL42_Run42Fsw(BC42_INTF_SensorDataMsg_t *SensorDataMsg)
{

   CFE_EVS_SendEvent(CTRL42_DEBUG_CONTROLLER_EID, CFE_EVS_EventType_DEBUG,
                     "**** CTRL42_Run42Fsw(%d) ****", (int)Ctrl42->CtrlExeCnt);
    
   Ctrl42->Bc42 = BC42_TakePtr();
   AC42_(EchoEnabled) = false;
   
   SensorDataMsgToAcStruct(SensorDataMsg);

   if (SensorDataMsg->Payload.InitCycle == true)
   {
      Ctrl42->TakeSciInitCycCtr = Ctrl42->TakeSciInitCyc;
      CFE_EVS_SendEvent(CTRL42_INIT_CONTROLLER_EID, CFE_EVS_EventType_INFORMATION, "Initialized contoller");
      InitAC(AC42);
   }
   
   AcFsw(AC42);
   ++Ctrl42->CtrlExeCnt;

   SendAcStructTlm();
   SetTakeSci();

   BC42_GivePtr(Ctrl42->Bc42);

} /* End CTRL42_Run42Fsw() */


/******************************************************************************
** Function: CTRL42_SendCtrlGainsTlmCmd
**
** Send the control gains telemetry packet containing the gains from the
** control table.
*/
bool CTRL42_SendCtrlGainsTlmCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr)
{

   int32  CfeStatus;
   
   BC42_CTRL_ControlGainsTlm_Payload_t *TlmPayload = &Ctrl42->ControlGainsTlm.Payload;

   TlmPayload->Kp[0] = Ctrl42->Tbl.Data.Kp[0];
   TlmPayload->Kp[1] = Ctrl42->Tbl.Data.Kp[1];
   TlmPayload->Kp[2] = Ctrl42->Tbl.Data.Kp[2];
   
   TlmPayload->Kr[0] = Ctrl42->Tbl.Data.Kr[0];
   TlmPayload->Kr[1] = Ctrl42->Tbl.Data.Kr[1];
   TlmPayload->Kr[2] = Ctrl42->Tbl.Data.Kr[2];

   TlmPayload->Kunl  = Ctrl42->Tbl.Data.Kunl;
   
   Ctrl42->Bc42 = BC42_TakePtr();
   
   TlmPayload->Kp[0] = AC42_(CfsCtrl.Kp[0]);
   TlmPayload->Kp[1] = AC42_(CfsCtrl.Kp[1]);
   TlmPayload->Kp[2] = AC42_(CfsCtrl.Kp[2]);
   
   TlmPayload->Kr[0] = AC42_(CfsCtrl.Kr[0]);
   TlmPayload->Kr[1] = AC42_(CfsCtrl.Kr[1]);
   TlmPayload->Kr[2] = AC42_(CfsCtrl.Kr[2]);

   TlmPayload->Kunl  = AC42_(CfsCtrl.Kunl);
   
   BC42_GivePtr(Ctrl42->Bc42);

   CFE_SB_TimeStampMsg(CFE_MSG_PTR(Ctrl42->ControlGainsTlm.TelemetryHeader));
   CfeStatus = CFE_SB_TransmitMsg(CFE_MSG_PTR(Ctrl42->ControlGainsTlm.TelemetryHeader), true);
   
   return (CfeStatus == CFE_SUCCESS);

} /* End CTRL42_SendCtrlGainsTlmCmd() */


/******************************************************************************
** Function: CTRL42_SetBoolOvrStateCmd
**
** Set/Clear the command specified boolean overrride
**
** Notes:
**   1. This is useful for fault testing.
*/
bool CTRL42_SetBoolOvrStateCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr)
{

   const BC42_CTRL_SetBoolOvrState_CmdPayload_t *CmdPayload = CMDMGR_PAYLOAD_PTR(MsgPtr, BC42_CTRL_SetBoolOvrState_t);
   bool  RetStatus = false;
   int   i;
   
   for (i=BC42_CTRL_Bool42Id_Enum_t_MIN; i<BC42_CTRL_Bool42State_Enum_t_MAX; i++)
   {
      Ctrl42->BoolOverride[i] = BC42_CTRL_Bool42State_USE_SIM;
   }

   if (CmdPayload->Id <= BC42_CTRL_Bool42Id_Enum_t_MAX)
   {
     
      if (CmdPayload->State >= BC42_CTRL_Bool42State_Enum_t_MIN && CmdPayload->State <= BC42_CTRL_Bool42State_Enum_t_MAX)
      {
         Ctrl42->BoolOverride[CmdPayload->Id] = CmdPayload->State;
         RetStatus = true;
         CFE_EVS_SendEvent(CTRL42_SET_BOOL_OVR_EID, CFE_EVS_EventType_INFORMATION,
                           "Set override identifier %d to state %s", CmdPayload->Id, BoolOverrideStr(CmdPayload->State));
         CFE_EVS_SendEvent(CTRL42_SET_BOOL_OVR_EID, CFE_EVS_EventType_INFORMATION,
                           "Currently this command does not effect the 42 controller ported to this app"); 
	  }
	  else {
         CFE_EVS_SendEvent(CTRL42_SET_BOOL_OVR_EID, CFE_EVS_EventType_ERROR,
                           "Invalid commanded override state %d. Must be between %d and %d inclusively",\
                           CmdPayload->State,BC42_CTRL_Bool42State_Enum_t_MIN,BC42_CTRL_Bool42State_Enum_t_MAX);
	  } /* End if valid state */
	  
   } /* End if valid ID */
   
   else
   {
      CFE_EVS_SendEvent(CTRL42_SET_BOOL_OVR_EID, CFE_EVS_EventType_ERROR,
                        "Invalid commanded override identifier %d is greater than max ID %d",
                        CmdPayload->Id, BC42_CTRL_Bool42Id_Enum_t_MAX); 
   }

   return RetStatus;

} /* End CTRL42_SetBoolOvrStateCmd() */


/******************************************************************************
** Function: CTRL42_SetCtrlModeCmd
**
** Currently controller doesn't have modes so this command only lets the
** user force a controller initialization.
*/
bool CTRL42_SetCtrlModeCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr)
{

   const BC42_CTRL_SetCtrlMode_CmdPayload_t *CmdPayload = CMDMGR_PAYLOAD_PTR(MsgPtr, BC42_CTRL_SetCtrlMode_t);
   bool  RetStatus = true;

   CFE_EVS_SendEvent(CTRL42_SET_CTRL_MODE_EID, CFE_EVS_EventType_INFORMATION,
                    "Received set controler mode %d", CmdPayload->NewMode);

   return RetStatus;

} /* End CTRL42_SetCtrlModeCmd() */


/******************************************************************************
** Function: CTRL42_SetWheelTargetMomCmd
**
** Set the target wheel momentum. 
**
*/
bool CTRL42_SetWheelTargetMomCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr)
{

   const BC42_CTRL_SetWheelTargetMom_CmdPayload_t *CmdPayload = CMDMGR_PAYLOAD_PTR(MsgPtr, BC42_CTRL_SetWheelTargetMom_t);
   bool   RetStatus = false;
   uint8  ValidWheels = 0, i;   

   for (i=0; i<BC42_NWHL; i++)
   {
   
      if ( CmdPayload->Wheel[i] >= Ctrl42->Tbl.Data.HcmdLim.Lower &&
           CmdPayload->Wheel[i] <= Ctrl42->Tbl.Data.HcmdLim.Upper )
      {
         Ctrl42->Hcmd[i] = CmdPayload->Wheel[i];
         ValidWheels++;
      }
      else
      {   
         CFE_EVS_SendEvent(CTRL42_WHL_TARGET_MOM_CMD_EID, CFE_EVS_EventType_ERROR,
                           "Commanded target wheel %d momentum %0.6e exceeds (lower,upper) limits (%0.6e,%0.6e)",
                           i, CmdPayload->Wheel[i], Ctrl42->Tbl.Data.HcmdLim.Lower, Ctrl42->Tbl.Data.HcmdLim.Upper);
      }
      
   } /* End wheel loop */ 
      
   if (ValidWheels == BC42_NWHL)
   {
      
      CFE_EVS_SendEvent(CTRL42_WHL_TARGET_MOM_CMD_EID, CFE_EVS_EventType_INFORMATION,
                        "Target wheel momentum set to %0.6e, %0.6e, %0.6e, %0.6e",
                        Ctrl42->Hcmd[0],Ctrl42->Hcmd[1],Ctrl42->Hcmd[2],Ctrl42->Hcmd[3]);
      
      CFE_EVS_SendEvent(CTRL42_WHL_TARGET_MOM_CMD_EID, CFE_EVS_EventType_INFORMATION,
                        "This command has no effect with the current 42 standalone controller"); 

      RetStatus = true;
      
   } /* End if valid command */
   
   return RetStatus;

} /* End CTRL42_SetWheelTargetMomCmd() */


/******************************************************************************
** Function: AcceptNewTbl
**
** Notes:
**   1. This is an example table load callback function to illustrate how table
**      load paramaters can be validated as part of the table load process.
*/
static void AcceptNewTbl(void)
{

   CFE_EVS_SendEvent (CTRL42_ACCEPT_NEW_TBL_EID, CFE_EVS_EventType_INFORMATION, 
                      "New CTRL42 table loaded");

} /* End AcceptNewTbl() */


/******************************************************************************
** Function: BoolOverrideStr
**
** Return an override string for event messages. See bc42_ctrl.xml for
** enumeration definitions.
**
** TODO: Decide whether to define strings in EDS.
**
*/
static const char *BoolOverrideStr(BC42_CTRL_Bool42State_Enum_t State)
{

   const static char *OverrideStr[BC42_CTRL_Bool42State_COUNT+1] = 
   {
      "USE 42 SIM", "TRUE", "FALSE", "UNDEFINED"
   };
   
   if (State >= BC42_CTRL_Bool42State_Enum_t_MIN && State <= BC42_CTRL_Bool42State_Enum_t_MAX)

      return OverrideStr[State];
 
   else

      return OverrideStr[BC42_CTRL_Bool42State_COUNT];
 
} /* End BoolOverrideStr() */


/******************************************************************************
** Function:  SensorDataMsgToAcStruct
**
** Notes:
**   1. Assumes caller is performing BC42_TakePtr() and BC42_GivePtr() calls.
**
*/
static void SensorDataMsgToAcStruct(const BC42_INTF_SensorDataMsg_t *SensorDataMsg) 
{
   
   int i;
   
   AC42_(Time)         = SensorDataMsg->Payload.GpsTime;
   AC42_(GPS[0].Valid) = SensorDataMsg->Payload.GpsValid;
   AC42_(StValid)      = SensorDataMsg->Payload.StValid;  
   AC42_(SunValid)     = SensorDataMsg->Payload.SunValid;  /* CSS/FSS */

   for (i=0; i < 3; i++) {

      AC42_(PosN[i])  = SensorDataMsg->Payload.PosN[i];  /* GPS */
      AC42_(VelN[i])  = SensorDataMsg->Payload.VelN[i];
   
      AC42_(qbn[i])   = SensorDataMsg->Payload.qbn[i];   /* ST */

      AC42_(wbn[i])   = SensorDataMsg->Payload.wbn[i];   /* Gyro */
   
      AC42_(svb[i])   = SensorDataMsg->Payload.svb[i];   /* CSS/FSS */

      AC42_(bvb[i])   = SensorDataMsg->Payload.bvb[i];   /* MTB s*/

      AC42_(Whl[i].H) = SensorDataMsg->Payload.WhlH[i];  /* Wheels */

   }
   
   AC42_(qbn[3])   = SensorDataMsg->Payload.qbn[3];   /* ST */
   AC42_(Whl[3].H) = SensorDataMsg->Payload.WhlH[4];  /* Wheels */

} /* End SensorDataMsgToAcStruct() */


/******************************************************************************
** Function: SendAcStructTlm
**
** Notes:
**   1. Assumes caller is performng BC42_TakePtr() and BC42_GivePtr() calls.
**
*/
static void SendAcStructTlm(void)
{

   int i;
   BC42_CTRL_ControllerTlm_Payload_t  *ControllerTlmPayload = &Ctrl42->ControllerTlm.Payload;
   BC42_INTF_ActuatorCmdMsg_Payload_t *ActuatorCmdPayload = &Ctrl42->ActuatorCmdMsg.Payload;
   
   ControllerTlmPayload->GpsValid = AC42_(GPS[0].Valid);
   ControllerTlmPayload->StValid  = AC42_(StValid);  
   ControllerTlmPayload->SunValid = AC42_(SunValid); 

   ControllerTlmPayload->SaGcmd = (float)AC42_(G->Cmd.Ang[0]);
   ActuatorCmdPayload->SaGcmd   = AC42_(G->Cmd.Ang[0]);

   for (i=0; i < 3; i++) {

      ControllerTlmPayload->wbn[i]   = (float)AC42_(wbn[i]);
      ControllerTlmPayload->wln[i]   = (float)AC42_(wln[i]);
      ControllerTlmPayload->qbr[i]   = (float)AC42_(qbr[i]);
      ControllerTlmPayload->therr[i] = (float)AC42_(CfsCtrl.therr[i]);
      ControllerTlmPayload->werr[i]  = (float)AC42_(CfsCtrl.werr[i]);
      ControllerTlmPayload->Hvb[i]   = (float)AC42_(Hvb[i]);
      ControllerTlmPayload->svb[i]   = (float)AC42_(svb[i]);

      ControllerTlmPayload->Tcmd[i] = (float)AC42_(Tcmd[i]); /* Wheel */
      ActuatorCmdPayload->Tcmd[i]   = AC42_(Tcmd[i]);
     
      ControllerTlmPayload->Mcmd[i] = (float)AC42_(Mcmd[i]); /* MTB   */
      ActuatorCmdPayload->Mcmd[i]   = AC42_(Mcmd[i]);

   }

   ControllerTlmPayload->qbr[3] = (float)AC42_(qbr[3]);

   CFE_EVS_SendEvent(CTRL42_DEBUG_CONTROLLER_EID, CFE_EVS_EventType_DEBUG, "**** SendActuatorPkt()\n");
      
   CFE_SB_TimeStampMsg(CFE_MSG_PTR(Ctrl42->ActuatorCmdMsg.TelemetryHeader));
   CFE_SB_TransmitMsg(CFE_MSG_PTR(Ctrl42->ActuatorCmdMsg.TelemetryHeader), true);

   CFE_SB_TimeStampMsg(CFE_MSG_PTR(Ctrl42->ControllerTlm.TelemetryHeader));
   CFE_SB_TransmitMsg(CFE_MSG_PTR(Ctrl42->ControllerTlm.TelemetryHeader), true);
   
} /* End SendAcStructTlm() */

                    
/******************************************************************************
** Function: SetTakeSci
**
** Notes:
**   1. Checks whether control errors within science instrument accuracy needs
**
*/
static void SetTakeSci(void)
{

   bool TakeSci;
   BC42_CTRL_ControllerTlm_Payload_t *ControllerTlm = &Ctrl42->ControllerTlm.Payload;

   if (Ctrl42->TakeSciInitCycCtr <= 0)
   {
   
      TakeSci = ((fabs(ControllerTlm->therr[0]) < Ctrl42->Tbl.Data.SciThetaLim[0]) &&
                 (fabs(ControllerTlm->therr[1]) < Ctrl42->Tbl.Data.SciThetaLim[1]) &&
                 (fabs(ControllerTlm->therr[2]) < Ctrl42->Tbl.Data.SciThetaLim[2]));

      if (TakeSci == ControllerTlm->TakeSci)
      {
         Ctrl42->TakeSciInitCycCtr = Ctrl42->TakeSciTransCyc;
      }
      else
      {
          if (Ctrl42->TakeSciInitCycCtr <=0)
          {
             ControllerTlm->TakeSci  = TakeSci;
             Ctrl42->TakeSciInitCycCtr = Ctrl42->TakeSciTransCyc;
          } 
          else
          {
             --Ctrl42->TakeSciInitCycCtr;
          }
      }

   } /* End if init cycle */
   else
   {
      --Ctrl42->TakeSciInitCycCtr;
      ControllerTlm->TakeSci = false;
   }

} /* End SetTakeSci() */
