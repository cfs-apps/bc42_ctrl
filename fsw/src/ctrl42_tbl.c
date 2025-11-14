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
**    Implement the Controller parameter table
**
**  Notes:
**    1. The static "TblData" serves as a table load buffer. Table dump data is
**       read directly from table owner's table storage.
**
*/

/*
** Include Files:
*/

#include <string.h>
#include "ctrl42_tbl.h"


/***********************/
/** Macro Definitions **/
/***********************/


/**********************/
/** Type Definitions **/
/**********************/


/************************************/
/** Local File Function Prototypes **/
/************************************/

static bool LoadJsonData(size_t JsonFileLen);


/**********************/
/** Global File Data **/
/**********************/

static CTRL42_TBL_Class_t *Ctrl42Tbl = NULL;

static CTRL42_TBL_Data_t TblData; /* Working buffer for loads */

static CJSON_Obj_t JsonTblObjs[] = 
{

   /* Table Data Address        Table Data Length   Updated, Data Type,  Float,  core-json query string,  length of query string(exclude '\0') */
   
   { &TblData.Kp[0],            sizeof(float),      false,   JSONNumber, true,   { "kp.x",                (sizeof("kp.x")-1)}             },
   { &TblData.Kp[1],            sizeof(float),      false,   JSONNumber, true,   { "kp.y",                (sizeof("kp.y")-1)}             },
   { &TblData.Kp[2],            sizeof(float),      false,   JSONNumber, true,   { "kp.z",                (sizeof("kp.z")-1)}             },
   { &TblData.Kr[0],            sizeof(float),      false,   JSONNumber, true,   { "kr.x",                (sizeof("kr.x")-1)}             },
   { &TblData.Kr[1],            sizeof(float),      false,   JSONNumber, true,   { "kr.y",                (sizeof("kr.y")-1)}             },
   { &TblData.Kr[2],            sizeof(float),      false,   JSONNumber, true,   { "kr.z",                (sizeof("kr.z")-1)}             },
   { &TblData.Kunl,             sizeof(float),      false,   JSONNumber, true,   { "kunl.k",              (sizeof("kunl.k")-1)}             },
   { &TblData.SciThetaLim[0],   sizeof(float),      false,   JSONNumber, true,   { "sci-theta-lim.x",     (sizeof("sci-theta-lim.x")-1)}  },
   { &TblData.SciThetaLim[1],   sizeof(float),      false,   JSONNumber, true,   { "sci-theta-lim.y",     (sizeof("sci-theta-lim.y")-1)}  },
   { &TblData.SciThetaLim[2],   sizeof(float),      false,   JSONNumber, true,   { "sci-theta-lim.z",     (sizeof("sci-theta-lim.z")-1)}  },
   { &TblData.HcmdLim.Lower,    sizeof(float),      false,   JSONNumber, true,   { "hcmd-lim.lower",      (sizeof("hcmd-lim.lower")-1)}   },
   { &TblData.HcmdLim.Upper,    sizeof(float),      false,   JSONNumber, true,   { "hcmd-lim.upper",      (sizeof("hcmd-lim.upper")-1)}   }
};


/******************************************************************************
** Function: CTRL42_TBL_Constructor
**
** Notes:
**    1. This must be called prior to any other functions
**
*/
void CTRL42_TBL_Constructor(CTRL42_TBL_Class_t *Ctrl42TblPtr, 
                            CTRL42_TBL_LoadFunc_t LoadFunc)
{

   Ctrl42Tbl = Ctrl42TblPtr;

   CFE_PSP_MemSet(Ctrl42Tbl, 0, sizeof(CTRL42_TBL_Class_t));
 
   Ctrl42Tbl->LoadFunc = LoadFunc;
   Ctrl42Tbl->JsonObjCnt = (sizeof(JsonTblObjs)/sizeof(CJSON_Obj_t));
         
} /* End CTRL42_TBL_Constructor() */


/******************************************************************************
** Function: CTRL42_TBL_DumpCmd
**
** Notes:
**  1. Function signature must match TBLMGR_DumpTblFuncPtr_t.
**  2. File is formatted so it can be used as a load file.
*/
bool CTRL42_TBL_DumpCmd(osal_id_t FileHandle)
{

   char DumpRecord[256];

   sprintf(DumpRecord,"   \"kp\": {\n      \"x\": %4.8e,\n      \"y\": %4.8e,\n      \"z\": %4.8e\n   },\n",
           Ctrl42Tbl->Data.Kp[0], Ctrl42Tbl->Data.Kp[1], Ctrl42Tbl->Data.Kp[2]);
   OS_write(FileHandle, DumpRecord, strlen(DumpRecord));

   sprintf(DumpRecord,"   \"kr\": {\n      \"x\": %4.8e,\n      \"y\": %4.8e,\n      \"z\": %4.8e\n   },\n",
           Ctrl42Tbl->Data.Kr[0], Ctrl42Tbl->Data.Kr[1], Ctrl42Tbl->Data.Kr[2]);
   OS_write(FileHandle, DumpRecord, strlen(DumpRecord));

   sprintf(DumpRecord,"   \"kunl\": {\n      \"k\": %4.8e   },\n",
           Ctrl42Tbl->Data.Kunl);
   OS_write(FileHandle, DumpRecord, strlen(DumpRecord));

   sprintf(DumpRecord,"   \"hcmd-lim\": {\n      \"lower\": %4.8e,\n      \"upper\": %4.8e\n   },\n",
           Ctrl42Tbl->Data.HcmdLim.Lower, Ctrl42Tbl->Data.HcmdLim.Upper);
   OS_write(FileHandle, DumpRecord, strlen(DumpRecord));

   sprintf(DumpRecord,"   \"sci-theta-lim\": {\n      \"x\": %4.8e,\n      \"y\": %4.8e,\n      \"z\": %4.8e\n   }\n",
           Ctrl42Tbl->Data.SciThetaLim[0], Ctrl42Tbl->Data.SciThetaLim[1], Ctrl42Tbl->Data.SciThetaLim[2]);
   OS_write(FileHandle, DumpRecord, strlen(DumpRecord));

   return true;
   
} /* End of CTRL42_TBL_DumpCmd() */


/******************************************************************************
** Function: CTRL42_TBL_LoadCmd
**
** Notes:
**  1. Function signature must match TBLMGR_LoadTblFuncPtr_t.
*/
bool CTRL42_TBL_LoadCmd(APP_C_FW_TblLoadOptions_Enum_t LoadType, const char *Filename)
{

   bool  RetStatus = false;

   if (CJSON_ProcessFile(Filename, Ctrl42Tbl->JsonBuf, CTRL42_TBL_JSON_FILE_MAX_CHAR, LoadJsonData))
   {
      Ctrl42Tbl->Loaded = true;
      RetStatus = true;   
   }

   return RetStatus;
   
} /* End CTRL42_TBL_LoadCmd() */


/******************************************************************************
** Function: CTRL42_TBL_ResetStatus
**
*/
void CTRL42_TBL_ResetStatus(void)
{

   Ctrl42Tbl->LastLoadCnt = 0;
 
} /* End CTRL42_TBL_ResetStatus() */


/******************************************************************************
** Function: LoadJsonData
**
** Notes:
**  1. See file prologue for full/partial table load scenarios
*/
static bool LoadJsonData(size_t JsonFileLen)
{

   bool      RetStatus = false;
   size_t    ObjLoadCnt;


   Ctrl42Tbl->JsonFileLen = JsonFileLen;

   /* 
   ** 1. Copy table owner data into local table buffer
   ** 2. Process JSON file which updates local table buffer with JSON supplied values
   ** 3. If valid, copy local buffer over owner's data 
   */
   
   memcpy(&TblData, &Ctrl42Tbl->Data, sizeof(CTRL42_TBL_Data_t));
   
   ObjLoadCnt = CJSON_LoadObjArray(JsonTblObjs, Ctrl42Tbl->JsonObjCnt, Ctrl42Tbl->JsonBuf, Ctrl42Tbl->JsonFileLen);

   /* Partial table only accepted after a full table load has been performed */
   if (!Ctrl42Tbl->Loaded && (ObjLoadCnt != Ctrl42Tbl->JsonObjCnt))
   {

      CFE_EVS_SendEvent(CTRL42_TBL_LOAD_EID, CFE_EVS_EventType_ERROR, 
                        "Table has never been loaded and new table only contains %d of %d data objects",
                        (unsigned int)ObjLoadCnt, (unsigned int)Ctrl42Tbl->JsonObjCnt);
   
   }
   else
   {
      // If no validation function assume table is valid
      RetStatus = true;
      if (Ctrl42Tbl->LoadFunc != NULL)
      {
         RetStatus = (Ctrl42Tbl->LoadFunc)(&TblData);
      }
      
      if (RetStatus)
      {
         memcpy(&Ctrl42Tbl->Data,&TblData, sizeof(CTRL42_TBL_Data_t));
         Ctrl42Tbl->LastLoadCnt = ObjLoadCnt;
         CFE_EVS_SendEvent(CTRL42_TBL_LOAD_EID, CFE_EVS_EventType_INFORMATION, 
                           "Successfully loaded %d JSON objects", 
                           (unsigned int)ObjLoadCnt);
      }
   }
   
   return RetStatus;
   
} /* End LoadJsonData() */

