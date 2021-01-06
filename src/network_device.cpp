/*
@file network_device.cpp Class definition of network devices for using socket utilities

@language C++14

@author 14trann

Thomas Tran Network devices for using socket utilities
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

#include "network_device.h"

using namespace std;

int CNetworkDevice::m_nNetworkDeviceIndex {0};

CNetworkDevice::CNetworkDevice(const vector<spdlog::sink_ptr> sinks)
{
    /**
     * @brief Used to handle connections and sending/receiving data over a socket
     * @param sinks Vector for logger pointers
     */
    // create the logger for this class
    string strLogName = "CNetworkDevice_" + to_string(m_nNetworkDeviceIndex);
    shared_ptr<spdlog::logger> lstLogger = make_shared<spdlog::logger>(strLogName, begin(sinks), end(sinks));
    spdlog::register_logger(lstLogger);
    m_lstLogger = spdlog::get(strLogName);
    m_lstLogger->set_level(spdlog::level::trace);
    
    m_lstCStrBuffer = new char[m_nBufSize] {};

    m_nNetworkDeviceIndex++;
}

bool CNetworkDevice::Connect(string strIpAddress, int nPort, string strSocketType, int nBufSize)
{
    /**
     * @brief Connect to the socket associated with the object
     * @param strIpAddress IPv4 address to connect to, ex "192.168.1.xxx"
     * @param nPort Port number to connect to , ex 30230
     * @param strSocketType String to describe the socket protocol, "TCP" or "UDP" only
     * @param nBufSize Size of the receive buffer must be power of 2, default is 1024
     * @return true if connection successful, otherwise false
     */
    // setup the socket
    m_strIpAddress = strIpAddress;
    m_nPort = nPort;
    m_strSocketType = strSocketType;
    m_nBufSize = nBufSize;
    
    if (m_strSocketType == "TCP")
        m_nSocketFd = socket(AF_INET, SOCK_STREAM, 0);

    else if (m_strSocketType == "UDP")
        m_nSocketFd = socket(AF_INET, SOCK_DGRAM, 0);

    // fcntl(m_nSocketFd, F_SETFL, O_NONBLOCK);
    m_sHost.sin_addr.s_addr = inet_addr(m_strIpAddress.c_str());
    m_sHost.sin_family = AF_INET;
    m_sHost.sin_port = htons(m_nPort);

    if (connect(m_nSocketFd, (struct sockaddr*) &m_sHost, sizeof(m_sHost)) == 0)
    {
        // log the socket connection
        m_lstLogger->debug("Connected to socket: {0}, {1}", inet_ntoa(m_sHost.sin_addr), ntohs(m_sHost.sin_port));
        return true;
    }
    else
    {
        // do some error logging
        m_lstLogger->error("Did not connect to socket, error is {}", errno);
        return false;
    }
}

bool CNetworkDevice::Accept()
{
    /**
     * @brief Binds to the socket associated with the object and accepts incoming connection requests
     * @return true if successful, otherwise false
     */

    return CNetworkDevice::Accept(m_strIpAddress, m_nPort);
}

bool CNetworkDevice::Accept(string strIpAddress, int nPort)
{
    /**
     * @brief Binds to the socket associated with the object and accepts incoming connection requests
     * @param strIpAddress IPv4 address to connect to, ex "192.168.1.xxx"
     * @param nPort Port number to connect to , ex 30230
     * @return true if successful, otherwise false
     */
    if (m_strSocketType == "TCP")
        m_nSocketFd = socket(AF_INET, SOCK_STREAM, 0);

    else if (m_strSocketType == "UDP")
        m_nSocketFd = socket(AF_INET, SOCK_DGRAM, 0);

    m_sHost.sin_addr.s_addr = inet_addr(strIpAddress.c_str());
    m_sHost.sin_family = AF_INET;
    m_sHost.sin_port = htons(nPort);
    
    if (bind(m_nSocketFd, (struct sockaddr*) &m_sHost, sizeof(m_sHost)) == 0)
        // do some logging
        m_lstLogger->debug("Bound socket: {0}, {1}", inet_ntoa(m_sHost.sin_addr), ntohs(m_sHost.sin_port));
    else
    {
        // do some error logging
        m_lstLogger->error("Did not bind socket, error is {}", errno);
        return false;
    }

    if (listen(m_nSocketFd, 1) == 0) 
        // do some logging
        m_lstLogger->debug("Listening for socket");
    else
    {
        // do some error logging
        m_lstLogger->error("Could not listen for socket, error is {}", errno);
        return false;
    }

    int nSocketLength {sizeof(m_sHost)};
    m_nSocketFd = accept(m_nSocketFd, (struct sockaddr*) &m_sHost, (socklen_t*) &nSocketLength);
    if (m_nSocketFd == -1)
    {
        m_lstLogger->error("Could not accept socket, error is {}", errno);
        return false;
    }
    else
        m_lstLogger->debug("Accepted socket connection: {0}, {1}", inet_ntoa(m_sHost.sin_addr), ntohs(m_sHost.sin_port));

    return true;
    
}

bool CNetworkDevice::Accept(string strIpAddress, int nPort, string strSocketType)
{
    /**
     * @brief Binds to the socket associated with the object and accepts incoming connection requests
     * @param strIpAddress IPv4 address to connect to, ex "192.168.1.xxx"
     * @param nPort Port number to connect to , ex 30230
     * @return true if successful, otherwise false
     */
    m_strSocketType = strSocketType;
    if (m_strSocketType == "TCP")
        m_nSocketFd = socket(AF_INET, SOCK_STREAM, 0);

    else if (m_strSocketType == "UDP")
        m_nSocketFd = socket(AF_INET, SOCK_DGRAM, 0);

    int reuse = 1;
    if (setsockopt(m_nSocketFd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0)
        perror("setsockopt(SO_REUSEADDR) failed");

    if (setsockopt(m_nSocketFd, SOL_SOCKET, SO_REUSEPORT, (const char*)&reuse, sizeof(reuse)) < 0) 
        perror("setsockopt(SO_REUSEPORT) failed");

    m_sHost.sin_addr.s_addr = inet_addr(strIpAddress.c_str());
    m_sHost.sin_family = AF_INET;
    m_sHost.sin_port = htons(nPort);
    
    if (bind(m_nSocketFd, (struct sockaddr*) &m_sHost, sizeof(m_sHost)) == 0)
        // do some logging
        m_lstLogger->debug("Bound socket: {0}, {1}", inet_ntoa(m_sHost.sin_addr), ntohs(m_sHost.sin_port));
    else
    {
        // do some error logging
        m_lstLogger->error("Did not bind socket, error is {}", errno);
        return false;
    }

    if (listen(m_nSocketFd, 1) == 0) 
        // do some logging
        m_lstLogger->debug("Listening for socket");
    else
    {
        // do some error logging
        m_lstLogger->error("Could not listen for socket, error is {}", errno);
        return false;
    }

    int nSocketLength {sizeof(m_sHost)};
    m_nBindedSocketFd = accept(m_nSocketFd, (struct sockaddr*) &m_sHost, (socklen_t*) &nSocketLength);
    m_bBinded = true;
    if (setsockopt(m_nBindedSocketFd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0)
        perror("setsockopt(SO_REUSEADDR) failed");

    if (setsockopt(m_nBindedSocketFd, SOL_SOCKET, SO_REUSEPORT, (const char*)&reuse, sizeof(reuse)) < 0) 
        perror("setsockopt(SO_REUSEPORT) failed");
    if (m_nBindedSocketFd == -1)
    {
        m_lstLogger->error("Could not accept socket, error is {}", errno);
        return false;
    }
    else
        m_lstLogger->debug("Accepted socket connection: {0}, {1}", inet_ntoa(m_sHost.sin_addr), ntohs(m_sHost.sin_port));

    return true;
    
}

bool CNetworkDevice::Send(const char* lstCStrData, size_t nDataLength, bool bUnlockedMutex)
{
    /**
     * @brief Send data to the socket, blocking until all data sent
     * @param lstCStrData C-string or byte array to send
     * @param nDataLength Length of lstCStrData, only used for custom non-\0 terminated data
     * @param bUnlockedMutex flag to indicate if the mutex should be unlocked on return
     * @return true if successful, otherwise false
     */
    int nSocket = m_nSocketFd;
    if (m_bBinded)
        nSocket = m_nBindedSocketFd;

    if (nDataLength == 0)
        nDataLength = strlen(lstCStrData);
    
    m_cLock.lock();
    int nBytesSent = send(nSocket, lstCStrData, nDataLength, 0);
    if (bUnlockedMutex == true)
        m_cLock.unlock();
    if (nBytesSent == nDataLength)
    {
        //do some data logging
        m_lstLogger->debug("All data sent to: {0}, {1}", inet_ntoa(m_sHost.sin_addr), ntohs(m_sHost.sin_port));
        return true;
    }
    else if (nBytesSent == -1)
    {
        //do some error logging
        m_lstLogger->error("Data not sent, error is {}", errno);
        return false;
    }
    else
    {
        m_lstLogger->warn("Not all data sent to: {0}, {1}", inet_ntoa(m_sHost.sin_addr), ntohs(m_sHost.sin_port));
        return true;
    }
    
}

char* CNetworkDevice::Receive(bool bNonBlock, bool bLockedMutex)
{
    /**
     * @brief Receive data from the socket, blocking until something is read
     * @param bNonBlock flag to set nonblocking receive
     * @param bLockedMutex flag to determined if the mutex should be locked on function call
     * @return m_lstCStrBuffer containing c-string data terminated by \0, contains previous data if no new data read
     */
    int nSocket = m_nSocketFd;
    if (m_bBinded)
        nSocket = m_nBindedSocketFd;

    int flags {0};
    if (bNonBlock == true)
        flags |= MSG_DONTWAIT;

    if (bLockedMutex == true)
        m_cLock.lock();
    int nBytesReceived = recv(nSocket, m_lstCStrBuffer, m_nBufSize, flags);
    m_cLock.unlock();
    if (nBytesReceived == -1)
    {
        if (errno == EAGAIN or errno == EWOULDBLOCK)
        {
            m_lstLogger->warn("No message in buffer, setting return to '-', error is {}", errno);
            m_lstCStrBuffer[0] = '-';
        }
        else
        {
            m_lstLogger->error("Could not receive from socket, error is {}", errno);
        }
    }
    else
    {
        // do some logging
        m_lstLogger->debug("Received data from: {0}, {1}", inet_ntoa(m_sHost.sin_addr), ntohs(m_sHost.sin_port));
        if (nBytesReceived <= m_nBufSize-1)
        {
            m_lstCStrBuffer[nBytesReceived] = '\0';
        }
        else
        {
            // log that the data was too long and truncate
            m_lstLogger->debug("Data exceeded buffer size, truncated");
            m_lstCStrBuffer[nBytesReceived-1] = '\0';
        }
    }
    return m_lstCStrBuffer;
    
}

bool CNetworkDevice::Close()
{
    /**
     * @brief Closes the socket associated with the object
     * @return true if successful, otherwise false
     */
    if (close(m_nSocketFd) == 0)
        m_lstLogger->debug("Closed socket at {0}, {1}", inet_ntoa(m_sHost.sin_addr), ntohs(m_sHost.sin_port));
    else
    {
        m_lstLogger->error("Could not close socket, error is {}", errno);
        return false;
    }
    if (m_bBinded == true)
    {
        if (close(m_nBindedSocketFd) == 0)
            m_lstLogger->debug("Closed socket at {0}, {1}", inet_ntoa(m_sHost.sin_addr), ntohs(m_sHost.sin_port));
        else
        {
            m_lstLogger->error("Could not close socket, error is {}", errno);
            return false;
        }
    }
    
    return true;
    
}

CNetworkDevice::~CNetworkDevice()
{
    /**
     * @brief Destroys the object but does not auto close the socket, frees up memory taken by buffer
     */
    delete m_lstCStrBuffer;
}
