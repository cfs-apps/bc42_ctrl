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
**    Manage the Controller parameter table
**
**  Notes:
**    1. Use the Singleton design pattern. A pointer to the table object
**       is passed to the constructor and saved for all other operations.
**       This is a table-specific file so it doesn't need to be re-entrant.
**
*/
#ifndef _ctrl42_tbl_
#define _ctrl42_tbl_

/*
** Includes
*/

#include "app_cfg.h"

/***********************/
/** Macro Definitions **/
/***********************/

/*
** Event Message IDs
*/

#define CTRL42_TBL_DUMP_EID  (CTRL42_TBL_BASE_EID + 0)
#define CTRL42_TBL_LOAD_EID  (CTRL42_TBL_BASE_EID + 1)


/**********************/
/** Type Definitions **/
/**********************/

/*
** Table load callback function
*/
typedef void (*CTRL42_TBL_LoadFunc_t)(void);


/******************************************************************************
** Table - Local table copy used for table loads
** 
*/

typedef struct
{

   float  Lower;
   float  Upper;
   
} CTRL42_TBL_Lim_t;

typedef struct
{
   float  Kp[3];
   float  Kr[3];
   float  Kunl;
   float  SciThetaLim[3];
   CTRL42_TBL_Lim_t HcmdLim;
   
} CTRL42_TBL_Data_t;


/******************************************************************************
** Class
*/

typedef struct
{

   /*
   ** Table Data
   */
   
   CTRL42_TBL_Data_t     Data;
   CTRL42_TBL_LoadFunc_t LoadFunc; 
   
   /*
   ** Standard CJSON table data
   */
   
   bool         Loaded;   /* Has entire table been loaded? */
   uint16       LastLoadCnt;
   
   size_t       JsonObjCnt;
   char         JsonBuf[CTRL42_TBL_JSON_FILE_MAX_CHAR];   
   size_t       JsonFileLen;
   
} CTRL42_TBL_Class_t;


/************************/
/** Exported Functions **/
/************************/


/******************************************************************************
** Function: CTRL42_TBL_Constructor
**
** Initialize the Histogram table object.
**
** Notes:
**   1. The table values are not populated. This is done when the table is 
**      registered with the table manager.
**
*/
void CTRL42_TBL_Constructor(CTRL42_TBL_Class_t *TblObj, 
                            CTRL42_TBL_LoadFunc_t LoadFunc);


/******************************************************************************
** Function: CTRL42_TBL_DumpCmd
**
** Command to write the table data from memory to a JSON file.
**
** Notes:
**  1. Function signature must match TBLMGR_DumpTblFuncPtr_t.
**
*/
bool CTRL42_TBL_DumpCmd(osal_id_t FileHandle);


/******************************************************************************
** Function: CTRL42_TBL_LoadCmd
**
** Command to copy the table data from a JSON file to memory.
**
** Notes:
**  1. Function signature must match TBLMGR_LoadTblFuncPtr_t.
**  2. Can assume valid table file name because this is a callback from 
**     the app framework table manager.
**
*/
bool CTRL42_TBL_LoadCmd(APP_C_FW_TblLoadOptions_Enum_t LoadType, const char *Filename);


/******************************************************************************
** Function: CTRL42_TBL_ResetStatus
**
** Reset counters and status flags to a known reset state.  The behavior of
** the table manager should not be impacted. The intent is to clear counters
** and flags to a known default state for telemetry.
**
*/
void CTRL42_TBL_ResetStatus(void);


#endif /* _ctrl42_tbl_ */

