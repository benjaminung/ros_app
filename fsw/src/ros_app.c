/*******************************************************************************
**
**      GSC-18128-1, "Core Flight Executive Version 6.7"
**
**      Copyright (c) 2006-2019 United States Government as represented by
**      the Administrator of the National Aeronautics and Space Administration.
**      All Rights Reserved.
**
**      Licensed under the Apache License, Version 2.0 (the "License");
**      you may not use this file except in compliance with the License.
**      You may obtain a copy of the License at
**
**        http://www.apache.org/licenses/LICENSE-2.0
**
**      Unless required by applicable law or agreed to in writing, software
**      distributed under the License is distributed on an "AS IS" BASIS,
**      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**      See the License for the specific language governing permissions and
**      limitations under the License.
**
** File: ros_app.c
**
** Purpose:
**   This file contains the source code for the Sample App.
**
*******************************************************************************/

/*
** Include Files:
*/
#include "ros_app_events.h"
#include "ros_app_version.h"
#include "ros_app.h"

/* The ros_lib module provides the ROS_LIB_Function() prototype */
#include <string.h>
#include "ros_lib.h"
#include "vec_app_msg.h"
#include "vec_app_msgids.h"

/*
** global data
*/
ROS_APP_Data_t ROS_APP_Data;
RosNode *rosNode;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/* ROS_APP_Main() -- Application entry point and main process loop         */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void ROS_APP_Main(void)
{
    uint32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(ROS_APP_PERF_ID);


    char c[1] = "\0";
    char * a = &c[0];
    rosNode = ROS_LIB_InitRos(0,&a);
    RosMessageType rosMsgType = ROS_LIB_Vector3;
    char rosTopicToCfs[] = "to_cfs_vectormsg";
    char rosTopicToRos[] = "to_ros_vectormsg";
    ROS_LIB_InitPublisher(rosNode, rosMsgType, rosTopicToRos);
    ROS_LIB_InitSubscriber(rosNode, rosMsgType, rosTopicToCfs);

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = ROS_APP_Init();
    if (status != CFE_SUCCESS)
    {
        ROS_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    /*
    ** ROS Runloop
    */
    while (CFE_ES_RunLoop(&ROS_APP_Data.RunStatus) == true)
    {
        /*
        ** Performance Log Exit Stamp
        */
        CFE_ES_PerfLogExit(ROS_APP_PERF_ID);

        /* Pend on receipt of command packet */
        // CFE_ES_WriteToSysLog("ROS_APP: Polling command pipe for messages.\n");
        status = CFE_SB_RcvMsg(&ROS_APP_Data.MsgPtr, ROS_APP_Data.CommandPipe, CFE_SB_POLL);
        // CFE_ES_WriteToSysLog("ROS_APP: Finished polling command pipe for messages.\n");
        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(ROS_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            ROS_APP_ProcessCommandPacket(ROS_AppData.MsgPtr);
        }
        else if (status == CFE_SB_NO_MESSAGE)
        {
            //do nothing
        }
        else
        {
            CFE_EVS_SendEvent(ROS_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "ROS APP: SB Pipe Read Error, App Will Exit. Error Code is - %d",status);

            ROS_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }

        ROS_LIB_SpinOnce();
        ROS_APP_SendRosVectorToSB();
    }
    ROS_LIB_DeleteRosNode(rosNode);
    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(ROS_APP_PERF_ID);

    CFE_ES_ExitApp(ROS_APP_Data.RunStatus);
    
} /* End of ROS_APP_Main() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* ROS_APP_Init() --  initialization                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 ROS_APP_Init(void)
{
    int32 status;

    ROS_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Initialize app command execution counters
    */
    ROS_APP_Data.CmdCounter = 0;
    ROS_APP_Data.ErrCounter = 0;

    /*
    ** Initialize app configuration data
    */
    ROS_APP_Data.PipeDepth = ROS_APP_PIPE_DEPTH;

    strncpy(ROS_APP_Data.PipeName, "ROS_APP_CMD_PIPE");

    /*
    ** Initialize event filter table...
    */
    ROS_APP_Data.EventFilters[0].EventID = ROS_APP_STARTUP_INF_EID;
    ROS_APP_Data.EventFilters[0].Mask    = 0x0000;
    ROS_APP_Data.EventFilters[1].EventID = ROS_APP_COMMAND_ERR_EID;
    ROS_APP_Data.EventFilters[1].Mask    = 0x0000;
    ROS_APP_Data.EventFilters[2].EventID = ROS_APP_COMMANDNOP_INF_EID;
    ROS_APP_Data.EventFilters[2].Mask    = 0x0000;
    ROS_APP_Data.EventFilters[3].EventID = ROS_APP_COMMANDRST_INF_EID;
    ROS_APP_Data.EventFilters[3].Mask    = 0x0000;
    ROS_APP_Data.EventFilters[4].EventID = ROS_APP_INVALID_MSGID_ERR_EID;
    ROS_APP_Data.EventFilters[4].Mask    = 0x0000;
    ROS_APP_Data.EventFilters[5].EventID = ROS_APP_LEN_ERR_EID;
    ROS_APP_Data.EventFilters[5].Mask    = 0x0000;
    ROS_APP_Data.EventFilters[6].EventID = ROS_APP_PIPE_ERR_EID;
    ROS_APP_Data.EventFilters[6].Mask    = 0x0000;

    /*
    ** Register the events
    */
    status = CFE_EVS_Register(ROS_APP_Data.EventFilters, ROS_APP_EVENT_COUNTS, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Sample App: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Create Software Bus message pipe.
    */
    status = CFE_SB_CreatePipe(&ROS_APP_Data.CommandPipe, ROS_APP_Data.PipeDepth, ROS_APP_Data.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("ROS App: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Initialize message header for message sent to VEC_APP
    */
    CFE_MSG_Init(&ROS_APP_Data.Vector3.TlmHeader.Msg, VEC_APP_ROSVEC3_MID, sizeof(ROS_APP_Data.Vector3));

    /*
    ** Subscribe to vector packets coming from VEC_APP
    */
    status = CFE_SB_Subscribe(VEC_APP_VECTOR3_MID, ROS_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("ROS App: Error Subscribing to Vector request, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    CFE_EVS_SendEvent(ROS_APP_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "ROS App Initialized.%s",
                      ROS_APP_VERSION_STRING);

    return (CFE_SUCCESS);

} /* End of ROS_APP_Init() */

void ROS_APP_SendRosVectorToSB()
{
    while(ROS_LIB_GetVectorListSize() > 0)
    {
        cfsVector3 rosVec = ROS_LIB_GetNextVector();
        ROS_APP_Data.Vector3.x = rosVec.x;
        ROS_APP_Data.Vector3.y = rosVec.y;
        ROS_APP_Data.Vector3.z = rosVec.z;
        CFE_SB_TimeStampMsg(&ROS_APP_Data.Vector3.TlmHeader.Msg);
        CFE_SB_TransmitMsg(&ROS_APP_Data.Vector3.TlmHeader.Msg, true);
        
        CFE_ES_WriteToSysLog("ROS_APP: Sent Vector3 packet to software bus\n");
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  ROS_APP_ProcessCommandPacket                                    */
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the ROS    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void ROS_APP_ProcessCommandPacket(CFE_SB_MsgPtr_t Msg)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    MsgId = CFE_SB_GetMsgId(Msg);

    switch (MsgId)
    {
        case VEC_APP_VECTOR3_MID:
            ROS_APP_ProcessVecMsg(Msg);
            break;

        default:
            CFE_EVS_SendEvent(ROS_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "ROS: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }

    return;

} /* End ROS_APP_ProcessCommandPacket */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* ROS_APP_ProcessVecMsg() -- ROS ground commands                */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void ROS_APP_ProcessVecMsg(CFE_SB_MsgPtr_t Msg)
{
    if (ROS_APP_VerifyCmdLength(Msg, sizeof(VEC_APP_Vector3_t)))
    {
        VEC_APP_Vector3_t *vecMsg = (VEC_APP_Vector3_t *)(&Msg);
        double x = vecMsg->x;
        double y = vecMsg->y;
        double z = vecMsg->z;
        CFE_ES_WriteToSysLog("ROS_APP: Received Vec Msg from software bus: %.0f, %.0f, %.0f\n", x, y, z);

        ROS_LIB_SendVectorToRos(rosNode,x,y,z);
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* ROS_APP_Noop -- ROS NOOP commands                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 ROS_APP_Noop(const ROS_APP_NoopCmd_t *Msg)
{

    ROS_APP_Data.CmdCounter++;

    CFE_EVS_SendEvent(ROS_APP_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "ROS: NOOP command %s",
                      ROS_APP_VERSION);

    return CFE_SUCCESS;

} /* End of ROS_APP_Noop */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  ROS_APP_ResetCounters                                               */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 ROS_APP_ResetCounters(const ROS_APP_ResetCounters_t *Msg)
{

    ROS_APP_Data.CmdCounter = 0;
    ROS_APP_Data.ErrCounter = 0;

    CFE_EVS_SendEvent(ROS_APP_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION, "ROS: RESET command");

    return CFE_SUCCESS;

} /* End of ROS_APP_ResetCounters() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* ROS_APP_VerifyCmdLength() -- Verify command packet length                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool ROS_APP_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    uint16            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;

    ActualLength = CFE_SB_GetTotalMsgLength(Msg);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        MsgId   = CFE_SB_GetMsgId(Msg);
        uint16         CommandCode = CFE_SB_GetCmdCode(Msg);

        CFE_EVS_SendEvent(ROS_APP_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), CommandCode, ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        ROS_APP_Data.ErrCounter++;
    }

    return (result);

} /* End of ROS_APP_VerifyCmdLength() */