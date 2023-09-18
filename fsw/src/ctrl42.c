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

static void  AcStructToTlm(void);
static void  SensorDataMsgToAcStruct(const BC42_INTF_SensorDataMsg_t *SensorDataMsg);
static void  SetTakeSci(void);
static void  TblToAcStruct(void);
static char  *BoolOverrideStr(BC42_CTRL_Bool42State_Enum_t State);


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
void CTRL42_Constructor(CTRL42_Class_t *Ctrl42Obj, const INITBL_Class_t *IniTbl)
{

   int i;
   
   Ctrl42 = Ctrl42Obj;

   /* If a class state variable can't default to zero then must be set after this */
   CFE_PSP_MemSet((void*)Ctrl42, 0, sizeof(CTRL42_Class_t));
   
   /* Down counters */
   Ctrl42->TakeSciInitCyc  = INITBL_GetIntConfig(INITBL_OBJ, CFG_CTRL42_TAKE_SCI_INIT_CYC);
   Ctrl42->TakeSciTransCyc = INITBL_GetIntConfig(INITBL_OBJ, CFG_CTRL42_TAKE_SCI_TRANS_CYC);
   
   for (i=BC42_CTRL_Bool42State_Enum_t_MIN; i<=BC42_CTRL_Bool42State_Enum_t_MAX; i++)
   {
      Ctrl42->BoolOverride[i] = BC42_CTRL_Bool42State_USE_SIM;
   }
   
   strncpy (Ctrl42->Filename, INITBL_GetStrConfig(INITBL_OBJ, CFG_CRTL42_DEBUG_FILE),OS_MAX_PATH_LEN);
   Ctrl42->Filename[OS_MAX_PATH_LEN-1] = '\0';
   
   CTRL42_TBL_Constructor(&(Ctrl42->CtrlTbl), AcceptNewTbl, INITBL_GetStrConfig(IniTbl, CFG_APP_CFE_NAME));

   CFE_MSG_Init(CFE_MSG_PTR(Ctrl42->ActuatorCmdMsg.TelemetryHeader), 
                CFE_SB_ValueToMsgId(INITBL_GetIntConfig(INITBL_OBJ, CFG_BC42_CTRL_STATUS_TLM_TOPICID)),
                sizeof(BC42_INTF_ActuatorCmdMsg_t));

   CFE_MSG_Init(CFE_MSG_PTR(Ctrl42->ControllerTlm.TelemetryHeader), 
                CFE_SB_ValueToMsgId(INITBL_GetIntConfig(INITBL_OBJ, CFG_BC42_CTRL_CONTROLLER_TLM_TOPICID)),
                sizeof(BC42_CTRL_ControllerTlm_t));
                
   CFE_MSG_Init(CFE_MSG_PTR(Ctrl42->ControlGainsTlm.TelemetryHeader), 
                CFE_SB_ValueToMsgId(INITBL_GetIntConfig(INITBL_OBJ, CFG_BC42_CTRL_CONTROL_GAINS_TLM_TOPICID)),
                sizeof(BC42_CTRL_ControlGainsTlm_t));
                
} /* End CTRL42_Constructor() */

o
/******************************************************************************
** Function: CTRL42_DisableDebugLogCmd
**
*/
bool CTRL42_DisableDebugLogCmd(void *ObjDataPtr, const CFE_MSG_Message_t *MsgPtr)
{

   bool  RetStatus = true;

   if (Ctrl42->DebugEnabled)
   {
      Ctrl42->Ctrl42->DebugEnabled = false;
      OS_close(Ctrl42->DebugFileHandle);
      CFE_EVS_SendEvent(CTRL42_DEBUG_CMD_EID, CFE_EVS_EventType_INFORMATION,
                        "Debug log file %s closed", Ctrl42->Filename);
   }
   else
   {
      CFE_EVS_SendEvent(CTRL42_DEBUG_CMD_EID, CFE_EVS_EventType_INFORMATION,
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
      CFE_EVS_SendEvent(CTRL42_DEBUG_CMD_ERR_EID, CFE_EVS_EventType_ERROR, 
                        "Enable debug command had no effect, debug log already enabled");
   }
   else
   {
      
      SysStatus = OS_OpenCreate(&Ctrl42->DebugFileHandle, Ctrl42->DebugFilename, 
                                OS_FILE_FLAG_CREATE, OS_READ_WRITE);
      
      if (SysStatus == OS_SUCCESS)
      {   
         Ctrl42->DebugEnabled = true;
         CFE_EVS_SendEvent(CTRL42_DEBUG_CMD_EID, CFE_EVS_EventType_INFORMATION,
                           "Created debug log file %s",Ctrl42->DebugFilename);
         RetStatus = true;
      }
      else 
      {
         OS_GetErrorName(SysStatus, &OsErrStr);
         CFE_EVS_SendEvent(CTRL42_DEBUG_CMD_ERR_EID, CFE_EVS_EventType_ERROR, 
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

   CFE_EVS_SendEvent(CTRL42_DEBUG_EID, CFE_EVS_EventType_DEBUG,
                     "**** CTRL42_Run42Fsw(%d) ****", (int)Ctrl42->CtrlExeCnt);
    
   Ctrl42->Bc42 = BC42_TakePtr();
   AC42_(EchoEnabled) = false;
   
   SensorPktToAcStruct(SensorDataMsg);

   if (SensorDataMsg->Payload.InitCycle == true)
   {
      Ctrl42->TakeSciInitCycCtr = Ctrl42->TakeSciInitCyc;
      CFE_EVS_SendEvent(CTRL42_INIT_CONTROLLER_EID, CFE_EVS_EventType_INFORMATION, "Initialized contoller");
      InitAC(AC42);
   }
   
   AcFsw(AC42);
   ++Ctrl42->CtrlExeCnt;

   AcStructToTlm();
   SetTakeSci();

   AC42_ReleasePtr(Ctrl42->Bc42);

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

   TlmPayload->Kp[0] = Ctrl42->CtrlTbl.Data.Kp.X;
   TlmPayload->Kp[1] = Ctrl42->CtrlTbl.Data.Kp.Y;
   TlmPayload->Kp[2] = Ctrl42->CtrlTbl.Data.Kp.Z;
   
   TlmPayload->Kr[0] = Ctrl42->CtrlTbl.Data.Kr.X;
   TlmPayload->Kr[1] = Ctrl42->CtrlTbl.Data.Kr.Y;
   TlmPayload->Kr[2] = Ctrl42->CtrlTbl.Data.Kr.Z;

   TlmPayload->Kunl  = Ctrl42->CtrlTbl.Data.Kunl;
   
   Ctrl42->Ac42 = BC42_TakePtr();
   
   TlmPayload->Kp[0] = AC42_(CfsCtrl.Kp[0]);
   TlmPayload->Kp[1] = AC42_(CfsCtrl.Kp[1]);
   TlmPayload->Kp[2] = AC42_(CfsCtrl.Kp[2]);
   
   TlmPayload->Kr[0] = AC42_(CfsCtrl.Kr[0]);
   TlmPayload->Kr[1] = AC42_(CfsCtrl.Kr[1]);
   TlmPayload->Kr[2] = AC42_(CfsCtrl.Kr[2]);

   TlmPayload->Kunl  = AC42_(CfsCtrl.Kunl);
   
   BC42_GivePtr(Ctrl42->Ac42);

   CFE_SB_TimeStampMsg(CFE_MSG_PTR(Ctrl42->ControlGainsTlm.TelemetryHeader));
   CFE_SB_TransmitMsg(CFE_MSG_PTR(Ctrl42->ControlGainsTlm.TelemetryHeader), true);
   
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

   const BC42_CTRL_SetBoolOvrStateCmd_Payload_t *CmdPayload = CMDMGR_PAYLOAD_PTR(MsgPtr, BC42_CTRL_SetBoolOvrStateCmd_t);
   bool  RetStatus = false;
   
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
                           "Set override identifier %d to state %s", CmdPayload->Id, OverrideStr(CmdPayload->State));
         CFE_EVS_SendEvent(CTRL42_SET_BOOL_OVR_EID, CFE_EVS_EventType_INFORMATION,
                           "**** Currently this command does not effect the 42 controller ported to this app ****"); 
	  }
	  else {
         CFE_EVS_SendEvent(CTRL42_SET_BOOL_OVR_EID, CFE_EVS_EventType_ERROR,
                           "Invalid commanded override state %d. Must be between %d and %d inclusively",\
                           SetOvrCmd->State,BC42_CTRL_Bool42State_Enum_t_MIN,BC42_CTRL_Bool42State_Enum_t_MAX);
	  } /* End if valid state */
	  
   } /* End if valid ID */
   
   else
   {
      CFE_EVS_SendEvent(CTRL42_SET_BOOL_OVR_EID, CFE_EVS_EventType_ERROR,
                        "Invalid commanded override identifier %d is greater than max ID %d",
                        SetOvrCmd->Id, BC42_CTRL_Bool42Id_Enum_t_MAX); 
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

   const BC42_CTRL_SetCtrlModeCmd_Payload_t *CmdPayload = CMDMGR_PAYLOAD_PTR(MsgPtr, BC42_CTRL_SetCtrlModeCmd_t);
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

   const BC42_CTRL_SetWheelTargetMomCmd_Payload_t *CmdPayload = CMDMGR_PAYLOAD_PTR(MsgPtr, BC42_CTRL_SetWheelTargetMomCmd_t);
   bool   RetStatus = false;
   uint8  ValidWheels = 0, i;   

   for (i=0; i<AC42_NWHL; i++)
   {
   
      if ( SetTargetWhlMomCmd->Whl[i] >= Ctrl42->CtrlTbl.Data.HcmdLim.Lower &&
           SetTargetWhlMomCmd->Whl[i] <= Ctrl42->CtrlTbl.Data.HcmdLim.Upper ) {
      
         Ctrl42->Hcmd[i] = SetTargetWhlMomCmd->Whl[i];
         ValidWheels++;
      
      }
      else {
         
         CFE_EVS_SendEvent(CTRL42_INVLD_TARGET_WHL_MOM_EID, CFE_EVS_EventType_ERROR,
                           "Commanded target wheel %d momentum %0.6e exceeds (lower,upper) limits (%0.6e,%0.6e)",
                           i, SetTargetWhlMomCmd->Whl[i], Ctrl42->CtrlTbl.Data.HcmdLim.Lower, Ctrl42->CtrlTbl.Data.HcmdLim.Upper);
      }
      
   } /* End wheel loop */ 
      
   if (ValidWheels == AC42_NWHL) {
      
      CFE_EVS_SendEvent(CTRL42_SET_TARGET_WHL_MOM_EID, CFE_EVS_EventType_INFORMATION,
                        "Target wheel momentum set to %0.6e, %0.6e, %0.6e, %0.6e",
                        Ctrl42->Hcmd[0],Ctrl42->Hcmd[1],Ctrl42->Hcmd[2],Ctrl42->Hcmd[3]);
      
      CFE_EVS_SendEvent(CTRL42_SET_TARGET_WHL_MOM_EID, CFE_EVS_EventType_INFORMATION,
                        "**** OSK v2.3: This command has no effect with the current 42 standalone controller ****"); 

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
static char *BoolOverrideStr(BC42_CTRL_Bool42State_Enum_t State)
{

   static char* OverrideStr[BC42_CTRL_Bool42State_COUNT+1] = 
   {
      "USE 42 SIM", "TRUE", "FALSE", "UNDEFINED"
   };
   
   if (State >= BC42_CTRL_Bool42State_Enum_t_MIN && State <= BC42_CTRL_Bool42State_Enum_t_MAX)

      return Override[State];
 
   else

      return Override[BC42_CTRL_Bool42State_COUNT];
 
} /* End BoolOverrideStr() */


/******************************************************************************
** Function:  SensorDataMsgToAcStruct
**
** Notes:
**   1. Assumes caller is performing AC42_GetPtr() and AC42_ReleasePtr() calls.
**
*/
static void SensorDataMsgToAcStruct(const BC42_INTF_SensorDataMsg_t *SensorDataMsg) 
{
   
   int i;
   BC42_CTRL_ControlGainsTlm_Payload_t *TlmPayload = &Ctrl42->ControlGainsTlm.Payload;
   AC42_(Time)         = SensorDataPkt->Time;
   AC42_(GPS[0].Valid) = SensorDataPkt->GpsValid;
   AC42_(StValid)      = SensorDataPkt->StValid;  
   AC42_(SunValid)     = SensorDataPkt->SunValid;  /* CSS/FSS */

   for (i=0; i < 3; i++) {

      AC42_(PosN[i])  = SensorDataPkt->PosN[i];  /* GPS */
      AC42_(VelN[i])  = SensorDataPkt->VelN[i];
   
      AC42_(qbn[i])   = SensorDataPkt->qbn[i];   /* ST */

      AC42_(wbn[i])   = SensorDataPkt->wbn[i];   /* Gyro */
   
      AC42_(svb[i])   = SensorDataPkt->svb[i];   /* CSS/FSS */

      AC42_(bvb[i])   = SensorDataPkt->bvb[i];   /* MTB s*/

      AC42_(Whl[i].H) = SensorDataPkt->WhlH[i];  /* Wheels */

   }
   
   AC42_(qbn[3])   = SensorDataPkt->qbn[3];   /* ST */
   AC42_(Whl[3].H) = SensorDataPkt->WhlH[4];  /* Wheels */

} /* End SensorDataMsgToAcStruct() */


/******************************************************************************
** Function: AcStructToTlm
**
** Notes:
**   1. Assumes caller is performng AC42_GetPtr() and AC42_ReleasePtr() calls.
**
*/
static void AcStructToTlm(void)
{

   int i;
   BC42_CTRL_ControlGainsTlm_Payload_t *TlmPayload = &Ctrl42->ControlGainsTlm.Payload;
   
   Ctrl42->CtrlPkt.GpsValid = AC42_(GPS[0].Valid);
   Ctrl42->CtrlPkt.StValid  = AC42_(StValid);  
   Ctrl42->CtrlPkt.SunValid = AC42_(SunValid); 

   Ctrl42->CtrlPkt.SaGcmd     = (float)AC42_(G->Cmd.Ang[0]);
   Ctrl42->ActuatorPkt.SaGcmd = AC42_(G->Cmd.Ang[0]);

   for (i=0; i < 3; i++) {

      Ctrl42->CtrlPkt.wbn[i]   = (float)AC42_(wbn[i]);
      Ctrl42->CtrlPkt.wln[i]   = (float)AC42_(wln[i]);
      Ctrl42->CtrlPkt.qbr[i]   = (float)AC42_(qbr[i]);
      Ctrl42->CtrlPkt.therr[i] = (float)AC42_(CfsCtrl.therr[i]);
      Ctrl42->CtrlPkt.werr[i]  = (float)AC42_(CfsCtrl.werr[i]);
      Ctrl42->CtrlPkt.Hvb[i]   = (float)AC42_(Hvb[i]);
      Ctrl42->CtrlPkt.svb[i]   = (float)AC42_(svb[i]);

      Ctrl42->CtrlPkt.Tcmd[i]     = (float)AC42_(Tcmd[i]); /* Wheel */
      Ctrl42->ActuatorPkt.Tcmd[i] = AC42_(Tcmd[i]);
     
      Ctrl42->CtrlPkt.Mcmd[i]     = (float)AC42_(Mcmd[i]); /* MTB   */
      Ctrl42->ActuatorPkt.Mcmd[i] = AC42_(Mcmd[i]);

   }

   Ctrl42->CtrlPkt.qbr[3] = (float)AC42_(qbr[3]);

   CFE_EVS_SendEvent(CTRL42_DEBUG_EID, CFE_EVS_EventType_DEBUG, "**** SendActuatorPkt()\n");
   
   CFE_SB_TimeStampMsg((CFE_SB_Msg_t *) &Ctrl42->ActuatorPkt);
   CFE_SB_SendMsg((CFE_SB_Msg_t *) &Ctrl42->ActuatorPkt);

   CFE_SB_TimeStampMsg((CFE_SB_Msg_t *) &Ctrl42->CtrlPkt);
   CFE_SB_SendMsg((CFE_SB_Msg_t *) &Ctrl42->CtrlPkt);

} /* End AcStructToTlm() */

                    
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

   if (Ctrl42->TakeSciInitCnt <= 0)
   {
   
      TakeSci = ((fabs(Ctrl42->CtrlPkt.therr[0]) < Ctrl42->CtrlTbl.Data.SciThetaErrLim.X) &&
                 (fabs(Ctrl42->CtrlPkt.therr[1]) < Ctrl42->CtrlTbl.Data.SciThetaErrLim.Y) &&
                 (fabs(Ctrl42->CtrlPkt.therr[2]) < Ctrl42->CtrlTbl.Data.SciThetaErrLim.Z));

      if (TakeSci == Ctrl42->CtrlPkt.TakeSci)
      {
         Ctrl42->TakeSciTransCnt = CTRL42_TAKE_SCI_TRANS_CNT;
      }
      else
      {
          if (Ctrl42->TakeSciTransCnt <=0)
          {
             Ctrl42->CtrlPkt.TakeSci = TakeSci;
             Ctrl42->TakeSciTransCnt = CTRL42_TAKE_SCI_TRANS_CNT;
          } 
          else
          {
             --Ctrl42->TakeSciTransCnt;
          }
      }

   } /* End if init cycle */
   else
   {
      --Ctrl42->TakeSciInitCnt;
      Ctrl42->CtrlPkt.TakeSci = false;
   }

} /* End SetTakeSci()


/******************************************************************************
** Function: TblToAcStruct
**
*/
static void TblToAcStruct(void)
{

   Ctrl42->Ac42 = AC42_GetPtr();
 
   AC42_(CfsCtrl.Kp[0]) = Ctrl42->CtrlTbl.Data.Kp.X;
   AC42_(CfsCtrl.Kp[1]) = Ctrl42->CtrlTbl.Data.Kp.Y;
   AC42_(CfsCtrl.Kp[2]) = Ctrl42->CtrlTbl.Data.Kp.Z;

   AC42_(CfsCtrl.Kr[0]) = Ctrl42->CtrlTbl.Data.Kr.X;
   AC42_(CfsCtrl.Kr[1]) = Ctrl42->CtrlTbl.Data.Kr.Y;
   AC42_(CfsCtrl.Kr[2]) = Ctrl42->CtrlTbl.Data.Kr.Z;

   AC42_(CfsCtrl.Kunl)  = Ctrl42->CtrlTbl.Data.Kunl;

   AC42_ReleasePtr(Ctrl42->Ac42);

} /* End TblToAcStruct() */