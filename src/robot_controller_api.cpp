/*
@file robot_controller_api.cpp Class definition of the robot controller api, uploads a program to the robot controller and then sends requests for actions and data, units in mm and deg

@dependencies "../src/ur3_application_config.xml"; "../src/ur3_application.txt"

@language C++14

@author 14ttran

Thomas Tran Robot controller api for UR3, uploads a program to the robot controller and then sends requests for actions and data, units in mm and deg
Copyright (C) 2021 Thomas Tran

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>
*/

#include "robot_controller_api.h"

using namespace std;

bool SRobotData::FillRobotPoses(unordered_map<string, CAngleAxisPose> lststrclstfPoses)
{
    /**
     * @brief Populates all of the approach pose placeholders for the robot data, should only be run after runtime input has been populated
     * @return true if all approaches calculated, otherwise false
     */
    try
    {
        if (bControlledMoveFlag == 1)
        {
            if (strPathType == "Path")
                lstclstfPoses = Generate6DofPath(LoadPath(strPath), lststrclstfPoses["Robot"]);
            else if (strPathType == "Clip")
                lstclstfPoses = Generate6DofTab(LoadPath(strPath), lststrclstfPoses["Robot"], clstfTiltAngle);
            
            cout << "==========PATH POSES==========" << endl;
            for (auto pose : lstclstfPoses)
            {
                cout << "(" << pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "," << pose[4] << "," << pose[5] << ")" << endl;
            }
        }
        while (!lstnReadPoseLocation.empty())
        {
            int nReadPoseLocation = lstnReadPoseLocation.back();
            lstnReadPoseLocation.pop_back();
            string strReadPoseSystem = lststrReadPoseSystem.back();
            lststrReadPoseSystem.pop_back();
            int nType = lstnReadPoseType.back();
            lstnReadPoseType.pop_back();
            CAngleAxisPose clstfReadPoseOffset {};
            CAngleAxisPose clstfTest {0,0,0,0,0,0};
            if (lstclstfPoses.at(nReadPoseLocation) != clstfTest)
                clstfReadPoseOffset = lstclstfPoses.at(nReadPoseLocation);
            else
                clstfReadPoseOffset = lstlstfReadPoseOffset.back();
            lstlstfReadPoseOffset.pop_back();
            CAngleAxisPose clstfReadPose;
            clstfReadPose = lststrclstfPoses[strReadPoseSystem];
            clstfReadPose[0] += clstfReadPoseOffset[0];
            clstfReadPoseOffset[0] = 0;
            clstfReadPose[1] += clstfReadPoseOffset[1];
            clstfReadPoseOffset[1] = 0;
            clstfReadPose[2] += clstfReadPoseOffset[2];
            clstfReadPoseOffset[2] = 0;
            if (nType == 0)
                clstfReadPose = clstfReadPose.PostMultiply(clstfReadPoseOffset);
            else if (nType == 1)
            {
                clstfReadPose[3] = clstfReadPoseOffset[3];
                clstfReadPose[4] = clstfReadPoseOffset[4];
                clstfReadPose[5] = clstfReadPoseOffset[5];
            }
            lstclstfPoses.at(nReadPoseLocation) = clstfReadPose;
            cout << "Read pose is: " << "(" << clstfReadPose[0] << "," << clstfReadPose[1] << "," << clstfReadPose[2] << "," << clstfReadPose[3] << "," << clstfReadPose[4] << "," << clstfReadPose[5] << ")" << endl;
            cout << "Placed at: " << nReadPoseLocation << endl;
            
        }
        cout << "Num of approaches: " << lstnApproachLocations.size() << endl;
        
        while (!lstnApproachLocations.empty())
        {
            int nApproachLocation = lstnApproachLocations.back();
            lstnApproachLocations.pop_back();
            vector<float> lstfApproach = lstlstfApproachVectors.back();
            lstlstfApproachVectors.pop_back(); 
            while (lstfVectorData.size() < lststrMoveTypes.size())
            {
                //push 0.0 and the CRobotController class manages filling in with default kinematics
                lstfVectorData.push_back(0.0);
                lstfVectorData2.push_back(0.0);
            }

            cout << "Getting approach at location: " << nApproachLocation << endl;
           

            CAngleAxisPose clstfTempPose;
            clstfTempPose = lstclstfPoses.at(nApproachLocation+1);
            lstclstfPoses.at(nApproachLocation) = clstfTempPose.GetApproachPose(lstfApproach);
        }
        while (!lstnWithdrawalLocations.empty())
        {
            int nWithdrawalLocation = lstnWithdrawalLocations.front();
            lstnWithdrawalLocations.pop_front();
            vector<float> lstfWithdrawal = lstlstfWithdrawalVectors.front();
            lstlstfWithdrawalVectors.pop_front();
            while (lstfVectorData.size() < lststrMoveTypes.size())
            {
                //push 0.0 and the CRobotController class manages filling in with default kinematics
                lstfVectorData.push_back(0.0);
                lstfVectorData2.push_back(0.0);
            }

            cout << "Getting withdrawal at location" << nWithdrawalLocation << endl;
            

            CAngleAxisPose clstfTempPose;
            if (nWithdrawalLocation == 0)
                clstfTempPose = lststrclstfPoses["Robot"];
            else 
                clstfTempPose = lstclstfPoses.at(nWithdrawalLocation-1);
            lstclstfPoses.at(nWithdrawalLocation) = clstfTempPose.GetApproachPose(lstfWithdrawal);
        }
    }
    catch(const std::exception& e)
    {
        cout << "Error is: " << e.what() << endl;
        return false;
    }
    return true;
    
}

int CRobotController::nRobotIndex {0};

CRobotController::CRobotController(const vector<spdlog::sink_ptr> sinks)
: CNetworkDevice {sinks}, m_strRobotId {"ur3-" + to_string(nRobotIndex)}
{
    /**
     * @brief Creates a software controller for a robot and connects to it over a TCP/IP socket
     * @param sinks Vector for pointers to logger objects
     */
    string strLogName = "CRobotController_" + m_strRobotId;
    shared_ptr<spdlog::logger> lstLogger = make_shared<spdlog::logger>(strLogName, begin(sinks), end(sinks));
    spdlog::register_logger(lstLogger);
    m_lstLogger = spdlog::get(strLogName);
    m_lstLogger->set_level(spdlog::level::trace);

    RetrieveParameters("../config/ur3_application_config.xml");

    string strRobotApplication = FormatUr3Application("../core/robot/ur3_application.txt", "../config/ur3_application_config.xml");
    string strLocalIp {};
    string strRobotIp {};
    int nLocalPort {};
    int nRobotPort {};
    m_strmNetwork >> nLocalPort >> strLocalIp >> nRobotPort >> strRobotIp;
    Connect(strRobotIp, nRobotPort, "TCP");
    Send(strRobotApplication.c_str());
    Close();
    Accept(strLocalIp, nLocalPort);
    
    string strResponse {Receive()};
    if (strResponse.find("Success") != string::npos)
    {
        m_lstLogger->debug("{} application started successfully", m_strRobotId);
    }
    else
    {
        m_lstLogger->critical("{} application not started, check program", m_strRobotId);
        // throw 0;
    }

    SetGravity({m_lstfDefaultValues["fGravityX"], m_lstfDefaultValues["fGravityY"], m_lstfDefaultValues["fGravityZ"]});
    SetToolSpeed(m_lstfDefaultValues["fToolSpeed"]);
    SetToolAccel(m_lstfDefaultValues["fToolAccel"]);
    SetJointSpeed(m_lstfDefaultValues["fJointSpeed"]);
    SetJointAccel(m_lstfDefaultValues["fJointAccel"]);
    SetPayloadMass(m_lstfDefaultValues["fPayloadMass"]);
    CAngleAxisPose clstNoTcp {0.0,0.0,0.0,0.0,0.0,0.0};
    SetTcp(clstNoTcp);

    nRobotIndex++;
}

string CRobotController::FormatUr3Application(string strAppPath, const char* strXmlPath)
{
    /**
     * @brief Autofills a UR3 application file with constants taken from an XML configuration file
     * @param strAppPath Path to UR3 application text file
     * @param strXmlPath Path to XML configuration file associated with UR3 application file
     * @return String containing entire formatted application
     */
    // application file setup
    ifstream strmFileApplication {strAppPath, ios::in};
    stringstream strmApplication {};
    strmApplication << strmFileApplication.rdbuf();
    string strApplication {strmApplication.str()};

    // arg file setup
    rapidxml::file<> fileArguments(strXmlPath);
    rapidxml::xml_document<> fileArgData;
    fileArgData.parse<0>(fileArguments.data());

    // xml parsing
    rapidxml::xml_node<>* cRootNode = fileArgData.first_node();
    rapidxml::xml_node<>* cCommandsNode = cRootNode->first_node("commands");
    rapidxml::xml_node<>* cAppNode = cRootNode->first_node("application");

    for (rapidxml::xml_node<>* cArgNode = cCommandsNode->first_node(); cArgNode; cArgNode = cArgNode->next_sibling())
    {
        string strArgName {cArgNode->first_attribute("name")->value()};
        int nIndex {0};
        while (true)
        {
            nIndex = strApplication.find(strArgName, nIndex);
            if (nIndex == string::npos)
            {
                m_lststrCommSymbols.insert(make_pair(strArgName, cArgNode->value()));
                break;
            }
            strApplication.replace(nIndex, strArgName.length(), cArgNode->value());
            nIndex++;
        }
    }
    for (rapidxml::xml_node<>* cArgNode = cAppNode->first_node(); cArgNode; cArgNode = cArgNode->next_sibling())
    {
        string strArgName {cArgNode->first_attribute("name")->value()};
        int nIndex {0};
        while (true)
        {
            nIndex = strApplication.find(strArgName, nIndex);
            if (nIndex == string::npos)
                break;
            strApplication.replace(nIndex, strArgName.length(), cArgNode->value());
            nIndex++;
        }
    }

    return strApplication;
}

bool CRobotController::RetrieveParameters(const char* strXmlPath)
{
    /**
     * @brief Retrieves configuration constants from an XML config file associated with the specific robot
     * @param strXmlPath Path to XML config file associated with robot
     * @return true if successful retrievel, otherwise false
     */
    stringstream strmDoubleConverter {};
    float fArgValue {};
    int nArgValue {};

    rapidxml::file<> fileArguments(strXmlPath);
    rapidxml::xml_document<> fileArgData;
    fileArgData.parse<0>(fileArguments.data());

    rapidxml::xml_node<>* cRootNode = fileArgData.first_node();
    rapidxml::xml_node<>* cDefaultsNode = cRootNode->first_node("defaults");
    rapidxml::xml_node<>* cNetworkNode = cRootNode->first_node("application");
    rapidxml::xml_node<>* cTcpNode = cRootNode->first_node("robot_tcps");
    rapidxml::xml_node<>* cDigitalOutputNode = cRootNode->first_node("digital_outputs");
    for (rapidxml::xml_node<>* cArgNode = cDefaultsNode->first_node(); cArgNode; cArgNode = cArgNode->next_sibling())
    {
        string strArgName {cArgNode->first_attribute("name")->value()};
        strmDoubleConverter << cArgNode->value() << " ";
        if (strmDoubleConverter >> fArgValue)
            ;
        else
        {
            m_lstLogger->error("Could not convert robot application default to double: {}", strArgName);
            return false;
        }
        m_lstfDefaultValues.insert(make_pair(strArgName, fArgValue));
    }
    for (rapidxml::xml_node<>* cArgNode = cTcpNode->first_node(); cArgNode; cArgNode = cArgNode->next_sibling())
    {
        string strArgName {cArgNode->first_attribute("name")->value()};
        string strTcp = cArgNode->value();
        vector<float> lstfTcp {};
        char* cstrToken = strtok(const_cast<char*>(strTcp.c_str()), ",");
        while (cstrToken != nullptr)
        {
            strmDoubleConverter << cstrToken << " ";
            if (strmDoubleConverter >> fArgValue)
                ;
            else
            {
                m_lstLogger->error("Could not convert robot tcp to float: {}", strmDoubleConverter.str());
                return false;
            }
            lstfTcp.push_back(fArgValue);
            cstrToken = strtok(nullptr, ",");
        }
        CAngleAxisPose clstfTcp {};
        clstfTcp = lstfTcp;
        m_lstlstfTcps.insert(make_pair(strArgName, clstfTcp));
        m_lstLogger->debug("{} tcp name is {}, values are {}", m_strRobotId, strArgName, GetVectorData<float>(clstfTcp.Vector()));
    }
    for (rapidxml::xml_node<>* cArgNode = cDigitalOutputNode->first_node(); cArgNode; cArgNode = cArgNode->next_sibling())
    {
        string strArgName {cArgNode->first_attribute("name")->value()};
        string strCell {cArgNode->value()};
        strmDoubleConverter << strCell << " ";
        if (strmDoubleConverter >> nArgValue)
            m_lstLogger->debug("{}, arg {}, value {}",m_strRobotId, strArgName, nArgValue);
        else
        {
            m_lstLogger->error("Could not convert robot digital output to int: {}", strmDoubleConverter.str());
            return false;
        }
        m_lststrnDigitalOutputs.insert(make_pair(strArgName, nArgValue));
    }
    strmDoubleConverter << cNetworkNode->first_node("sample_rate")->value() << " ";
    strmDoubleConverter >> fArgValue;
    m_lstfDefaultValues.insert(make_pair(cNetworkNode->first_node("sample_rate")->first_attribute("name")->value(), fArgValue));
    m_strmNetwork << cNetworkNode->first_node("local_port")->value() << " ";
    m_strmNetwork << cNetworkNode->first_node("local_ip")->value() << " ";
    m_strmNetwork << cNetworkNode->first_node("robot_port")->value() << " ";
    m_strmNetwork << cNetworkNode->first_node("robot_ip")->value() << " ";
    
    return true;
}

bool CRobotController::Set(string strCommand, vector<float> lstfData)
{
    /**
     * @brief Private generic method for setting values on the robot host controller
     * @param strCommand The command, should only be taken from class member m_lststrCommSymbols
     * @param lstfData Packed data, structure should match what's in associated robot application file
     * @return true if successful in setting, otherwise false
     */
    stringstream strmConverter {};
    strmConverter << m_lststrCommSymbols["{strMessageHeader}"] << m_lststrCommSymbols["{strSetCommand}"] << m_lststrCommSymbols["{strMessageFooter}"];
    strmConverter << m_lststrCommSymbols["{strMessageHeader}"] << strCommand << m_lststrCommSymbols["{strMessageFooter}"];

    strmConverter << GetVectorData<float>(lstfData);
    // m_lstLogger->info("{}", strmConverter.str());
    string strMessage {strmConverter.str()};

    m_cLock.lock();
    if (Send(strMessage.c_str()) == false)
        return false;

    string strRobotResponse {Receive()};
    m_cLock.unlock();
    if (strRobotResponse.find(m_lststrCommSymbols["{strDoneFlag}"]) != string::npos)
    {
        m_lstLogger->debug("Success: set on {}, response is {}", m_strRobotId, strRobotResponse);
        return true;
    }
    else if (strRobotResponse.find(m_lststrCommSymbols["{strDoneFlag}"]) == string::npos)
    {
        m_lstLogger->error("Error: not set on {}, response is {}", m_strRobotId, strRobotResponse);
        return false;
    }
}

int CRobotController::CheckIfMoving()
{
    /**
     * @brief Asks the robot host if the robot is currently moving
     * @return true if it is, otherwise false
     */
    stringstream strmConverter {};
    strmConverter << m_lststrCommSymbols["{strMessageHeader}"] << m_lststrCommSymbols["{strCheckIfMoving}"] << m_lststrCommSymbols["{strMessageFooter}"];

    string strMessage {strmConverter.str()};

    m_cLock.lock();
    if (Send(strMessage.c_str()) == false)
        return 0;

    string strRobotResponse {Receive()};
    m_cLock.unlock();
    if (strRobotResponse.find(m_lststrCommSymbols["{strMovingFlag}"]) != string::npos)
    {
        m_lstLogger->debug("{} is moving, response is {}", m_strRobotId, strRobotResponse);
        return 1;
    }
    else if (strRobotResponse.find(m_lststrCommSymbols["{strDoneFlag}"]) != string::npos)
    {
        m_lstLogger->debug("{} is not moving, response is {}", m_strRobotId, strRobotResponse);
        return 0;
    }
    else if (strRobotResponse.find(m_lststrCommSymbols["{strBadFlag}"]) != string::npos)
    {
        m_lstLogger->error("{} was given a bad move, response is {}", m_strRobotId, strRobotResponse);
        return -1;
    }
    else
    {
        m_lstLogger->error("Error: {} bad response, response is {}", m_strRobotId, strRobotResponse);
        return -1;
    }

}

template <typename T>
string CRobotController::GetVectorData(vector<T> lstTVector)
{
    /**
     * @brief Template method for retrieving data from a vector as a string
     * @param lstTVector Vector with data
     * @return String of vector comma-separated data and closed by parentheses
     */
    stringstream strmConverter {};
    strmConverter << "(";
    copy(lstTVector.begin(), lstTVector.end()-1, ostream_iterator<T>(strmConverter, ","));
    strmConverter << lstTVector.back() << ")";
    return strmConverter.str();
}


vector<float> CRobotController::GetTcpPose()
{
    /**
     * @brief Queries the robot host controller for the current pose of the tcp
     * @return Vector with 6DOF axis-angle pose of tcp
     */
    stringstream strmConverter {};
    strmConverter << m_lststrCommSymbols["{strMessageHeader}"] << m_lststrCommSymbols["{strGetCommand}"] << m_lststrCommSymbols["{strMessageFooter}"] << m_lststrCommSymbols["{strMessageHeader}"] << m_lststrCommSymbols["{strGetTcp}"] << m_lststrCommSymbols["{strMessageFooter}"];
    string strMessage {strmConverter.str()};

    vector<float> lstfCurrentPose {};

    m_cLock.lock();
    if (Send(strMessage.c_str()) == true)
    {
        m_lstLogger->debug("Sent get tcp command to {}, message is {}", m_strRobotId, strMessage);
        strmConverter.str(string());
    }
    else
    {
        m_lstLogger->error("Did not send get tcp command to {}, message is {}", m_strRobotId, strMessage);
        m_cLock.unlock();
        return lstfCurrentPose;
    }

    string strRobotResponse {Receive()};
    m_cLock.unlock();
    if (strRobotResponse.find("p") != string::npos)
    {
        int nEndPosition = strRobotResponse.find("]");
        strRobotResponse = strRobotResponse.substr(2, nEndPosition - 2);
        char* cstrToken = strtok(const_cast<char*>(strRobotResponse.c_str()), ",");
        while (cstrToken != nullptr)
        {
            strmConverter << " " << cstrToken;
            cstrToken = strtok(nullptr, ",");
        }
        int i {0};
        float fTempValue {};
        while (i < 6)
        {
            strmConverter >> fTempValue;
            lstfCurrentPose.push_back(fTempValue);
            i++;
        }
        m_lstLogger->debug("Success: {} tcp pose is {}", m_strRobotId, GetVectorData<float>(lstfCurrentPose));
        return lstfCurrentPose;
    }
    else
    {
        m_lstLogger->error("Error: did not get tcp pose from {}, response is {}", m_strRobotId, strRobotResponse);
        return lstfCurrentPose;
    }
    
}

CAngleAxisPose CRobotController::GetTcp()
{
    /**
     * @brief Gets the currently set tcp information
     * @return CAngleAxisPose object of the current tcp
     */
    return m_clstfCurrentTcp;
}

vector<float> CRobotController::GetJointAngles(const vector<float>& lstfToolPose)
{
    /**
     * @brief Requests the joint angles for a given tool pose or the current tool pose
     * @param lstfToolPose The tcp pose to get joint angles for, default is {-1,-1,-1,-1,-1,-1} which indicates a request for the current pose
     * @return 6 joint angles {base, shoulder, elbow, wrist1, wrist2, wrist3}
     */
    stringstream strmConverter {};
    strmConverter << m_lststrCommSymbols["{strMessageHeader}"] << m_lststrCommSymbols["{strGetCommand}"] << m_lststrCommSymbols["{strMessageFooter}"] << m_lststrCommSymbols["{strMessageHeader}"] << m_lststrCommSymbols["{strGetJointAngles}"] << m_lststrCommSymbols["{strMessageFooter}"];

    strmConverter << GetVectorData<float>(lstfToolPose);

    vector<float> lstfJointAngles {};

    string strMessage {strmConverter.str()};

    m_cLock.lock();
    if (Send(strMessage.c_str()) == true)
    {
        m_lstLogger->debug("Sent get joint angles command to {}, message is {}", m_strRobotId, strMessage);
        strmConverter.str(string());
    }
    else
    {
        m_lstLogger->error("Did not send get joint angles command to {}, message is {}", m_strRobotId, strMessage);
        m_cLock.unlock();
        return {0,0,0,0,0,0};
    }

    string strRobotResponse {Receive()};
    m_cLock.unlock();
    if (strRobotResponse.find("[") != string::npos)
    {
        int nEndPosition = strRobotResponse.find("]");
        strRobotResponse = strRobotResponse.substr(1, nEndPosition - 2);
        char* cstrToken = strtok(const_cast<char*>(strRobotResponse.c_str()), ",");
        while (cstrToken != nullptr)
        {
            strmConverter << " " << cstrToken;
            cstrToken = strtok(nullptr, ",");
        }
        int i {0};
        float fTempValue {};
        while (i < 6)
        {
            strmConverter >> fTempValue;
            lstfJointAngles.push_back(fTempValue);
            i++;
        }
        m_lstLogger->debug("Success: {} joint angles are {}", m_strRobotId, GetVectorData<float>(lstfJointAngles));
        return lstfJointAngles;
    }
    else
    {
        m_lstLogger->error("Error: did not get joint angles from {}, response is {}", m_strRobotId, strRobotResponse);
        return {0,0,0,0,0,0};
    }
}

bool CRobotController::SetGravity(vector<float> lstfDirectionUnitVector)
{
    /**
     * @brief Sets the gravity on the robot host controller
     * @param lstfDirectionUnitVector 3DOF unit vector in form [x, y, z] with values equalling direction of gravity
     * @return true if setting successful, otherwise false
     */

    if (lstfDirectionUnitVector.size() != 3)
        m_lstLogger->error("Error: Vector size mismatch on {}, vector length is {}", m_strRobotId, lstfDirectionUnitVector.size());
        return false;

    m_lstLogger->info("Setting gravity on {}, value is {}", m_strRobotId, GetVectorData<float>(lstfDirectionUnitVector));

    return Set(m_lststrCommSymbols["{strSetGravity}"], lstfDirectionUnitVector);
}

bool CRobotController::DSetGravity(SRobotData sRobotDataPackage)
{
    /**
     * @brief Sets the gravity on the robot host controller
     * @param sRobotDataPackage Package containing all robot parameters, uses vector<float>
     * @return true if setting successful, otherwise false
     */

    return SetGravity(sRobotDataPackage.lstfVectorData);
}

bool CRobotController::SetTcp(CAngleAxisPose clstfTcp)
{
    /**
     * @brief Sets the tcp on the robot host controller
     * @param clstfTcp Angle-axis object with 6DOF pose of tcp
     * @return true if setting successful, otherwise false
     */
    m_lstLogger->debug("{} angle axis tcp is {}", m_strRobotId, GetVectorData<float>(clstfTcp.Vector()));
    m_clstfCurrentTcp = clstfTcp;
    vector<float> lstfValues {clstfTcp.Vector()};
    m_lstLogger->info("Setting tcp on {}, value is {}", m_strRobotId, GetVectorData<float>(lstfValues));

    return Set(m_lststrCommSymbols["{strSetTcp}"], lstfValues);
}

bool CRobotController::SetTcp(string strKey)
{
    /**
     * @brief Sets the tcp on the robot host controller
     * @param strKey Key for list of tcps
     * @return true if setting successful, otherwise false
     */
    try
    {
        CAngleAxisPose clstfTcp {m_lstlstfTcps[strKey]};
        m_lstLogger->debug("{} setting tcp, key is {}, values are {}", m_strRobotId, strKey, GetVectorData<float>(clstfTcp.Vector()));
        return SetTcp(clstfTcp);
    }
    catch (const exception& e)
    {
        m_lstLogger->error("Error: {} wrong key, error is {}", m_strRobotId, e.what());
        return false;
    }
}

bool CRobotController::DSetTcp(SRobotData sRobotDataPackage)
{
    /**
     * @brief Sets the tcp on the robot host controller
     * @param sRobotDataPackage Package containing all robot parameters, uses strValue
     * @return true if setting successful, otherwise false
     */

    return SetTcp(sRobotDataPackage.strValue);
}

bool CRobotController::SetRobotFromStage(CAngleAxisPose clstfRobotFromStage)
{
    /**
     * @brief Sets the robot position from the stage origin on the robot host controller
     * @param clstfRobotFromStage Angle-axis object with 6DOF pose of robot base from stage
     * @return true if setting successful, otherwise false
     */
    vector<float> lstfValues {clstfRobotFromStage.Vector()};
    m_lstLogger->info("Setting {} location from stage, value is {}", m_strRobotId, GetVectorData<float>(lstfValues));

    return Set(m_lststrCommSymbols["{strSetRobotFromStage}"], lstfValues);
}

bool CRobotController::DSetRobotFromStage(SRobotData sRobotDataPackage)
{
    /**
     * @brief Sets the robot position from the stage origin on the robot host controller
     * @param sRobotDataPackage Package containing all robot parameters, uses CAngleAxisPose clstAngleAxisPose
     * @return true if setting successful, otherwise false
     */

    return SetRobotFromStage(sRobotDataPackage.clstfAngleAxisPose);
}

bool CRobotController::SetToolSpeed(float fToolSpeed_mmdot)
{
    /**
     * @brief Sets the linear tcp speed on the robot host controller
     * @param fToolSpeed_mmdot Linear tool speed in mm/s, range is [0,3000]
     * @return true if setting successful, otherwise false
     */
    m_lstfDefaultValues["fToolSpeed"] = fToolSpeed_mmdot;
    vector<float> lstfValues {fToolSpeed_mmdot};
    m_lstLogger->info("Setting tool speed on {}, value is {}", m_strRobotId, GetVectorData<float>(lstfValues));

    return Set(m_lststrCommSymbols["{strSetToolSpeed}"], lstfValues);
}

bool CRobotController::DSetToolSpeed(SRobotData sRobotDataPackage)
{
    /**
     * @brief Sets the linear tcp speed on the robot host controller
     * @param sRobotDataPackage Package containing all robot parameters, uses float fValue
     * @return true if setting successful, otherwise false
     */

    return SetToolSpeed(sRobotDataPackage.fValue);
}

bool CRobotController::SetToolAccel(float fToolAccel_mmdotdot)
{
    /**
     * @brief Sets the linear tcp accleration on the robot host controller
     * @param fToolAccel_mmdotdot Linear tool acceleration in mm/s^2, range is [0,10000]
     * @return true if setting successful, otherwise false
     */
    m_lstfDefaultValues["fToolAccel"] = fToolAccel_mmdotdot;
    vector<float> lstfValues {fToolAccel_mmdotdot};
    m_lstLogger->info("Setting tool accel on {}, value is {}", m_strRobotId, GetVectorData<float>(lstfValues));

    return Set(m_lststrCommSymbols["{strSetToolAccel}"], lstfValues);
}

bool CRobotController::DSetToolAccel(SRobotData sRobotDataPackage)
{
    /**
     * @brief Sets the linear tcp accleration on the robot host controller
     * @param sRobotDataPackage Package containing all robot parameters, uses float fValue
     * @return true if setting successful, otherwise false
     */

    return SetToolAccel(sRobotDataPackage.fValue);
}

bool CRobotController::SetJointSpeed(float fJointSpeed_degdot)
{
    /**
     * @brief Sets the joint speed on the robot host controller
     * @param fJointSpeed_degdot Joint speed in deg/s, range is [0,360]
     * @return true if successful, otherwise false
     */
    m_lstfDefaultValues["fJointSpeed"] = fJointSpeed_degdot;
    vector<float> lstfValues {fJointSpeed_degdot};
    m_lstLogger->info("Setting joint speed on {}, value is {}", m_strRobotId, GetVectorData<float>(lstfValues));

    return Set(m_lststrCommSymbols["{strSetJointSpeed}"], lstfValues);
}

bool CRobotController::DSetJointSpeed(SRobotData sRobotDataPackage)
{
    /**
     * @brief Sets the joint speed on the robot host controller
     * @param sRobotDataPackage Package containing all robot parameters, uses float fValue
     * @return true if successful, otherwise false
     */

    return SetJointSpeed(sRobotDataPackage.fValue);
}

bool CRobotController::SetJointAccel(float fJointAccel_degdotdot)
{
    /**
     * @brief Sets the joint acceleration on the robot host controller
     * @param fJointAccel_degdotdot Joint acceleration in deg/s^2, range is [0,4285]
     * @return true if successful, otherwise false
     */
    m_lstfDefaultValues["fJointAccel"] = fJointAccel_degdotdot;
    vector<float> lstfValues {fJointAccel_degdotdot};
    m_lstLogger->info("Setting joint accel on {}, value is {}", m_strRobotId, GetVectorData<float>(lstfValues));

    return Set(m_lststrCommSymbols["{strSetJointAccel}"], lstfValues);
}

bool CRobotController::DSetJointAccel(SRobotData sRobotDataPackage)
{
    /**
     * @brief Sets the joint acceleration on the robot host controller
     * @param sRobotDataPackage Package containing all robot parameters, uses float fValue
     * @return true if successful, otherwise false
     */

    return SetJointAccel(sRobotDataPackage.fValue);
}

bool CRobotController::SetPayloadMass(float fMass_kg)
{
    /**
     * @brief Sets the payload mass on the robot host controller
     * @param fMass_kg Mass of payload in kg, range is [0,3]
     * @return true if successful, otherwise false
     */
    vector<float> lstfValues {fMass_kg};
    m_lstLogger->info("Setting payload mass on {}, value is {}", m_strRobotId, GetVectorData<float>(lstfValues));

    return Set(m_lststrCommSymbols["{strSetPayloadMass}"], lstfValues);
}

bool CRobotController::DSetPayloadMass(SRobotData sRobotDataPackage)
{
    /**
     * @brief Sets the payload mass on the robot host controller
     * @param sRobotDataPackage Package containing all robot parameters, uses float fValue
     * @return true if successful, otherwise false
     */

    return SetPayloadMass(sRobotDataPackage.fValue);
}

bool CRobotController::SetDigitalOutput(int nOutput, int nState)
{
    /**
     * @brief Sets a digital output to the desired state on the robot host controller
     * @param nOutput Output number, range is [0,7]
     * @param nState State of output, 0 or 1
     * @return true if successful, otherwise false
     */
    vector<float> lstfValues {(float) nOutput, (float) nState};
    m_lstLogger->info("Setting digital output {} on {}, value is {}", nOutput, m_strRobotId, nState);

    return Set(m_lststrCommSymbols["{strSetDigitalOutput}"], lstfValues);
}

bool CRobotController::SetDigitalOutput(string strKey, int nState)
{
    /**
     * @brief Sets a digital output to the desired state on the robot host controller
     * @param nOutput Output number, range is [0,7]
     * @param nState State of output, 0 or 1
     * @return true if successful, otherwise false
     */
    return SetDigitalOutput(m_lststrnDigitalOutputs[strKey], nState);
}

bool CRobotController::DSetDigitalOutput(SRobotData sRobotDataPackage)
{
    /**
     * @brief Sets a digital output to the desired state on the robot host controller
     * @param sRobotDataPackage Package containing all robot parameters, uses int lststrValues and lstnValues
     * @return true if successful, otherwise false
     */
    for (int i = 0; i < sRobotDataPackage.lststrValues.size(); i++)
    {
        bool bFlag = SetDigitalOutput(sRobotDataPackage.lststrValues.at(i), sRobotDataPackage.lstnValues.at(i));
        if (bFlag == false)
        {
            m_lstLogger->error("Error: {} failed to set {} to {}", m_strRobotId, sRobotDataPackage.lststrValues.at(i), sRobotDataPackage.lstnValues.at(i));
            return false;
        }
    }
    this_thread::sleep_for(chrono::milliseconds((int)(m_lstfDefaultValues["fDOWaitTime"])));
    return true;
}

bool CRobotController::Move(int nNumMoves, vector<string> lststrMoveTypes, vector<vector<float>> lstlstfPose, vector<float> lstfSpeeds, vector<float> lstfAccels, float fBlendRadius_mm)
{
    /**
     * @brief Moves the robot to the specified tcp poses or joint positions and blends them, 5 moves max, blocking
     * @param nNumMoves Number of moves, max is 5
     * @param lststrMoveTypes List of string indicators determining linear or joint moves, ex ["l", "j"]
     * @param lstlstfPose List of 6DOF poses matching lststrMoveTypes
     * @param fBlendRadius_mm Radius determining amount of blend between moves, default is 0 for no blend
     * @param lstfSpeeds List of speeds that each move should be conducted at, default is all 0.0 which will be set to the default setting
     * @param lstfAccels List of accels that each move should be conducted at, default is all 0.0 which will be set to the default setting
     * @return true when moves are completed, false if moves cannot be started or completed
     */
    if (nNumMoves > 5)
    {
        m_lstLogger->info("{}: Number of moves is {}(>5), cancelling move", m_strRobotId, nNumMoves);
        return false;
    }
    // cout << "Num moves: " << nNumMoves << endl;
    int nSpeedSize = lstfSpeeds.size();
    int nAccelSize = lstfAccels.size();
    // cout << "Speed size: " << nSpeedSize << endl;
    // cout << "Accel size: " << nAccelSize << endl;
    while (lstfSpeeds.size() < 5)
    {
        lstfSpeeds.push_back(0.0);
    }
        
    while (lstfAccels.size() < 5)
    {
        lstfAccels.push_back(0.0);
    }

    string strMoveTypes {};
    for (int i = 0; i < 5; i++)
    {
        if (i < lststrMoveTypes.size())
            strMoveTypes += lststrMoveTypes[i];
        else 
            strMoveTypes += " ";
    }
    // cout << "Moves: " << strMoveTypes << endl;
    for (int i = 0; i < 5; i++)
    {
        if (lstfSpeeds[i] == 0.0)
        {
            if (strMoveTypes.at(i) == 'l')
                lstfSpeeds[i] = m_lstfDefaultValues["fToolSpeed"];
            else if (strMoveTypes.at(i) == 'j')
                lstfSpeeds[i] = m_lstfDefaultValues["fJointSpeed"];
        }
        if (lstfAccels[i] == 0.0)
        {
            if (strMoveTypes.at(i) == 'l')
                lstfAccels[i] = m_lstfDefaultValues["fToolAccel"];
            else if (strMoveTypes.at(i) == 'j')
                lstfAccels[i] = m_lstfDefaultValues["fJointAccel"];
        }
    }

    m_lstLogger->debug("{} speed {}",m_strRobotId, GetVectorData<float>(lstfSpeeds));
    m_lstLogger->debug("{} accel {}",m_strRobotId, GetVectorData<float>(lstfAccels));

    vector<float> lstfData {};
    for (vector<float> lstPose : lstlstfPose)
    {
        for (float fDatum : lstPose)
        {
            lstfData.push_back(fDatum);
        }
    }

    // cout << "Data: " << GetVectorData<float>(lstfData) << endl;

    vector<float> lstfMoveParameters {(float) nNumMoves, fBlendRadius_mm};
    stringstream strmConverter {};
    strmConverter << m_lststrCommSymbols["{strMessageHeader}"] << m_lststrCommSymbols["{strMoveCommand}"] << m_lststrCommSymbols["{strMessageFooter}"] << GetVectorData<float>(lstfMoveParameters) << strMoveTypes << GetVectorData<float>(lstfData) << GetVectorData<float>(lstfSpeeds) << GetVectorData<float>(lstfAccels);
    string strMessage {strmConverter.str()};

    m_lstLogger->info("Performing move on {}, values are {}", m_strRobotId, GetVectorData<float>(lstfData));
    m_cLock.lock();
    if (Send(strMessage.c_str()) == false)
    {
        m_lstLogger->error("Error: did not send to {}, message is {}", m_strRobotId, strMessage);
        m_lstLogger->info("Aborting move on {}", m_strRobotId);
        m_cLock.unlock();
        return false;
    }

    string strResponse {Receive()};
    m_cLock.unlock();
    if (strResponse.find(m_lststrCommSymbols["{strDoneFlag}"]) != string::npos)
        m_lstLogger->debug("Success: {} move started, response is {}", m_strRobotId, strResponse);
    else
    {
        m_lstLogger->error("Error: {} bad response, response is {}", m_strRobotId, strResponse);
        return false;
    }

    while (true)
    {
        int bIsMoving {CheckIfMoving()};
        if (bIsMoving == 1)
            m_lstLogger->debug("{} move not done yet", m_strRobotId);
        else if (bIsMoving == 0)
        {
            m_lstLogger->debug("{} move completed", m_strRobotId);
            break;
        }
        else if (bIsMoving == -1)
        {
            m_lstLogger->error("{} robot error", m_strRobotId);
            break;
        }
    }
    return true;
}

bool CRobotController::DMove(SRobotData sRobotDataPackage)
{
    /**
     * @brief Moves the robot to the specified tcp poses or joint positions and blends them, 5 moves max, blocking
     * @param sRobotDataPackage Package containing all robot parameters, uses int nValue1 for nNumMoves, lststrMoveTypes for lststrMoveTypes, lstlstfPoses for lstlstfPose, fValue for fBlendRadius
     * @return true when moves are completed, false if moves cannot be started or completed
     */

    return Move(sRobotDataPackage.nValue1, sRobotDataPackage.lststrMoveTypes, sRobotDataPackage.lstlstfPoses, sRobotDataPackage.lstfVectorData, sRobotDataPackage.lstfVectorData2, sRobotDataPackage.fValue);
}

bool CRobotController::ControlledMove(vector<vector<float>> lstlstfCoordinates)
{
    /**
     * @brief Moves the robot to follow a specific path and trajectory based on the input path parameter and gain/control/lookahead time parameters set by the XML config file, blocking
     * @param lstlstfCoordinates List of 6DOF poses that form a path for the robot to follow
     * @return true when move completed, false if move could not be started or completed
     */
    string strMessage {m_lststrCommSymbols["{strMessageHeader}"] + m_lststrCommSymbols["{strControlledMove}"] + m_lststrCommSymbols["{strMessageFooter}"]};

    m_lstLogger->info("Performing controlled move on {}", m_strRobotId);
    m_cLock.lock();
    if (Send(strMessage.c_str()) == false)
    {
        m_lstLogger->error("Error: did not send to {}, message is {}", m_strRobotId, strMessage);
        m_lstLogger->info("Aborting controlled move on {}", m_strRobotId);
        m_cLock.unlock();
        return false;
    }
    string strResponse {Receive()};
    m_cLock.unlock();
    if (strResponse.find("Ready") != string::npos)
        m_lstLogger->debug("Success: {} ready for data transfer", m_strRobotId);
    else
    {
        m_lstLogger->error("Error: {} not ready for data transfer", m_strRobotId);
        m_lstLogger->info("Aborting controlled move on {}", m_strRobotId);
        return false;
    }

    for (vector<float> lstfCoord : lstlstfCoordinates)
    {
        string strPose {GetVectorData<float>(lstfCoord)};
        cout << "Sending pose: " << GetVectorData<float>(lstfCoord) << endl;
        m_cLock.lock();
        if (Send(strPose.c_str()) == false)
        {
            m_lstLogger->error("Error: did not send to {}, message is {}", m_strRobotId, strPose);
            m_lstLogger->info("Aborting controlled move on {}", m_strRobotId);
            m_cLock.unlock();
            break;
        }
        m_cLock.unlock();
        this_thread::sleep_for(chrono::milliseconds((int)(m_lstfDefaultValues["{nSampleRate_ms}"])));
    }

    vector<float> lstfEnding {-1,-1,-1,-1,-1,-1};
    string strEnding {GetVectorData<float>(lstfEnding)};
    cout << "Sending stop: " << GetVectorData<float>(lstfEnding) << endl;
    m_cLock.lock();
    if (Send(strEnding.c_str()) == false)
    {
        m_lstLogger->error("Error: did not send stop to {}, message is {}", m_strRobotId, strEnding);
        m_lstLogger->info("Aborting controlled move on {}", m_strRobotId);
        m_cLock.unlock();
        return false;
    }

    strResponse = Receive();
    m_cLock.unlock();
    if (strResponse.find(m_lststrCommSymbols["{strDoneFlag}"]) != string::npos)
        m_lstLogger->debug("Success: {} completed move", m_strRobotId);
    else
    {
        m_lstLogger->error("Error: {} could not complete move, response is {}", m_strRobotId, strResponse);
        return false;
    }
    return true;
    
}

bool CRobotController::DControlledMove(SRobotData sRobotDataPackage)
{
    /**
     * @brief Moves the robot to follow a specific path and trajectory based on the input path parameter and gain/control/lookahead time parameters set by the XML config file, blocking
     * @param sRobotDataPackage Package containing all robot parameters, uses lstlstfPoses
     * @return true when move completed, false if move could not be started or completed
     */

    return ControlledMove(sRobotDataPackage.lstlstfPoses);
}

bool CRobotController::SynchronizedMove()
{
    /**
     * @brief Puts the robot host controller into a state to follow data sent from the stage over a Modbus connection, blocking
     * @return true when move following completed, false if move could not be started or completed
     */
    string strMessage {m_lststrCommSymbols["{strMessageHeader}"] + m_lststrCommSymbols["{strSyncMove}"] + m_lststrCommSymbols["{strMessageFooter}"]};

    m_lstLogger->info("Performing synchronized move on {}", m_strRobotId);
    m_cLock.lock();
    if (Send(strMessage.c_str()) == false)
    {
        m_lstLogger->error("Error: did not send to {}, message is {}", m_strRobotId, strMessage);
        m_lstLogger->info("Aborting synchronized move on {}", m_strRobotId);
        m_cLock.unlock();
        return false;
    }

    while (true)
    {
        m_cLock.lock();
        string strResponse {Receive()};
        m_cLock.unlock();
        if (strResponse.find(m_lststrCommSymbols["{strDoneFlag}"]) != string::npos)
        {
            m_lstLogger->debug("Success: {} completed synchronized move", m_strRobotId);
            break;
        }
        else if (strResponse.empty() == true)
        {
            continue;
        }
        else
        {
            m_lstLogger->error("Error: {} did not complete synchronized move, response is {}", m_strRobotId, strResponse);
            return false;
        }
    }
    return true;
}

bool CRobotController::StopMoving()
{
    /**
     * @brief Sends a command to robot host to stop moving
     * @return true if succesful, otherwise false
     */
    stringstream strmConverter {};
    strmConverter << m_lststrCommSymbols["{strMessageHeader}"] << m_lststrCommSymbols["{strStopCommand}"] << m_lststrCommSymbols["{strMessageFooter}"];

    string strMessage {strmConverter.str()};

    m_cLock.lock();
    if (Send(strMessage.c_str()) == false)
        return false;

    string strRobotResponse {Receive()};
    m_cLock.unlock();
    if (strRobotResponse.find(m_lststrCommSymbols["{strDoneFlag}"]) != string::npos)
    {
        m_lstLogger->debug("Success: {} stopped, response is {}", m_strRobotId, strRobotResponse);
        return true;
    }
    else
    {
        m_lstLogger->error("Error: {} bad response, response is {}", m_strRobotId, strRobotResponse);
        return false;
    }
}

bool CRobotController::Disconnect()
{
    /**
     * @brief SEnds a command to the robot host controller to quit it's program and closes the socket connection to the robot host controller
     * @return true if successful, otherwise false
     */
    stringstream strmConverter {};
    strmConverter << m_lststrCommSymbols["{strMessageHeader}"] << m_lststrCommSymbols["{strDisconnectCommand}"] << m_lststrCommSymbols["{strMessageFooter}"];
    string strMessage {strmConverter.str()};

    if (Send(strMessage.c_str()) == true)
    {
        m_lstLogger->debug("Sent disconnect command to {}, message is {}", m_strRobotId, strMessage);
    }
    else
    {
        m_lstLogger->error("Did not send disconnect command to {}, message is {}", m_strRobotId, strMessage);
        return false;
    }
    Close();
    return true;
}


int CRobotController::GetDefaultValue(string strKey, float& fBuffer)
{
    /**
     * @brief Gets the default value for given key and places into the input param
     * @return 0 if retreival succesful, otherwise -1 if error occurs
     */
    try
    {
        fBuffer = m_lstfDefaultValues[strKey];
        return 0;
    }
    catch (const exception& e)
    {
        m_lstLogger->error("Error: {} error is: {}", m_strRobotId, e.what());
        return -1;
    }
}

CRobotController::~CRobotController()
{
    /**
     * @brief Destroys the object and runs the Disconnect() method
     */
    Disconnect();
}
