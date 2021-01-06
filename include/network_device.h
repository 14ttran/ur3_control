/*
@file network_device.h Class definition of network devices for using socket utilities

@language C++14

@author 14ttran
*/

#pragma once

#include <sys/socket.h>
#include <errno.h>
#include <string>
#include <vector>
#include <arpa/inet.h>
#include <mutex>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_sinks.h"

class CNetworkDevice
{
private:
    int m_nSocketFd;
    int m_nBindedSocketFd;
    bool m_bBinded {false};
    std::string m_strSocketType;
    struct sockaddr_in m_sHost;
    char* m_lstCStrBuffer;
    int m_nBufSize {1024};
    std::shared_ptr<spdlog::logger> m_lstLogger;
    static int m_nNetworkDeviceIndex;
    std::mutex m_cLock;

protected:
    std::string m_strIpAddress;
    int m_nPort;
    bool Connect(std::string strIpAddress, int nPort, std::string strSocketType, int nBufSize=1024);
    bool Accept();
    bool Accept(std::string strIpAddress, int nPort);
    bool Accept(std::string strIpAddress, int nPort, std::string strSocketType);
    bool Send(const char* lstCStrData, size_t nDataLength=0, bool bUnlockedMutex=true);
    char* Receive(bool bNonBlock=false, bool bLockedMutex=true);
    bool Close();

public:
    CNetworkDevice()=default;
    CNetworkDevice(const std::vector<spdlog::sink_ptr> sinks);
    ~CNetworkDevice();
};