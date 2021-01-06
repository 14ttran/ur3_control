/*
@file robot_controller_api.h Class declaration of the robot controller api, uploads a program to the robot controller and then sends requests for actions and data, units in mm and deg

@dependencies "../src/ur3_application_config.xml"; "../src/ur3_application.txt"

@language C++14

@author 14ttran
*/

#pragma once

#include <unordered_map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <errno.h>
#include <stdexcept>
#include <limits>
#include <deque>
#include <mutex>
#include <chrono>
#include "network_device.h"
#include "motion_planning.h"
#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_sinks.h"

struct SRobotData
{
    std::vector<float> lstfVectorData = {};
    std::vector<float> lstfVectorData2 = {};
    CAngleAxisPose clstfAngleAxisPose = {};
    float fValue = {};
    int nValue1 = {};
    int nValue2 = {};
    std::vector<std::string> lststrMoveTypes = {};
    std::vector<std::vector<float>> lstlstfPoses = {};
    std::string strValue = {};
    std::vector<CAngleAxisPose> lstclstfPoses = {};
    std::vector<std::string> lststrValues;
    std::vector<int> lstnValues;

    std::vector<int> lstnApproachLocations;
    std::vector<std::vector<float>> lstlstfApproachVectors;
    std::deque<int> lstnWithdrawalLocations;
    std::deque<std::vector<float>> lstlstfWithdrawalVectors;

    std::deque<std::string> lststrReadPoseSystem;
    std::deque<int> lstnReadPoseLocation;
    std::deque<std::vector<float>> lstlstfReadPoseOffset;
    std::vector<int> lstnReadPoseType;

    bool bControlledMoveFlag {0};
    std::string strPathType;
    CAngleAxisPose clstfTiltAngle {0,0,0,0,0,0};
    std::string strPath;

    bool FillRobotPoses(std::unordered_map<std::string, CAngleAxisPose> lststrclstfPoses);
};


class CRobotController : private CNetworkDevice
{
private:
    std::shared_ptr<spdlog::logger> m_lstLogger;
    std::mutex m_cLock;
    std::string m_strRobotId;
    static int nRobotIndex;
    std::unordered_map<std::string, float> m_lstfDefaultValues;
    std::unordered_map<std::string, std::string> m_lststrCommSymbols;
    std::stringstream m_strmNetwork {};
    CAngleAxisPose m_clstfCurrentTcp;
    std::unordered_map<std::string, CAngleAxisPose> m_lstlstfTcps;
    std::unordered_map<std::string, int> m_lststrnDigitalOutputs;

    template<typename T>
    std::string GetVectorData(std::vector<T> lstTVector);

    bool RetrieveParameters(const char* strXmlPath);
    std::string FormatUr3Application(std::string strAppPath, const char* strXmlPath);
    bool Set(std::string strCommand, std::vector<float> lstfData);
    int CheckIfMoving();


public:
    CRobotController()=default;
    CRobotController(const std::vector<spdlog::sink_ptr> sinks);
    ~CRobotController();
    
    bool Disconnect();
    std::vector<float> GetTcpPose();
    CAngleAxisPose GetTcp();
    std::vector<float> GetJointAngles(const std::vector<float>& lstfToolPose={-1,-1,-1,-1,-1,-1});
    bool SetGravity(std::vector<float> lstfDirectionUnitVector);
    bool DSetGravity(SRobotData sRobotDataPackage);
    bool SetTcp(CAngleAxisPose clstfTcp);
    bool SetTcp(std::string strKey);
    bool DSetTcp(SRobotData sRobotDataPackage);
    bool SetRobotFromStage(CAngleAxisPose clstfRobotFromStage);
    bool DSetRobotFromStage(SRobotData sRobotDataPackage);
    bool SetToolSpeed(float fToolSpeed_mmdot);
    bool DSetToolSpeed(SRobotData sRobotDataPackage);
    bool SetToolAccel(float fToolAccel_mmdotdot);
    bool DSetToolAccel(SRobotData sRobotDataPackage);
    bool SetJointSpeed(float fJointSpeed_degdot);
    bool DSetJointSpeed(SRobotData sRobotDataPackage);
    bool SetJointAccel(float fJointAccel_degdotdot);
    bool DSetJointAccel(SRobotData sRobotDataPackage);
    bool SetPayloadMass(float fMass_kg);
    bool DSetPayloadMass(SRobotData sRobotDataPackage);
    bool SetDigitalOutput(int nOutput, int nState);
    bool SetDigitalOutput(std::string strKey, int nState);
    bool DSetDigitalOutput(SRobotData sRobotDataPackage);
    bool Move(int nNumMoves, std::vector<std::string> lststrMoveTypes, std::vector<std::vector<float>> lstlstfPose, std::vector<float> lstfSpeeds={0.0, 0.0, 0.0, 0.0, 0.0}, std::vector<float>lstfAccels={0.0, 0.0, 0.0, 0.0, 0.0}, float fBlendRadius_mm=0.0);
    bool DMove(SRobotData sRobotDataPackage);
    bool ControlledMove(std::vector<std::vector<float>> lstlstfCoordinates);
    bool DControlledMove(SRobotData sRobotDataPackage);
    bool SynchronizedMove();
    bool StopMoving();

    int GetDefaultValue(std::string strKey, float& fBuffer);
};

};   // end robot
};   // end motion
};   // end denali