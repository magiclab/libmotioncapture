#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <wchar.h>
#include <ctype.h>
#include <netdb.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <netinet/tcp.h>

#define SO_MAX_MSG_SIZE TCP_MAXSEG
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define FAR

#define SOCKET_TIMEOUT			SOCKET_ERROR - 1
#define NO_TIMEOUT				0xFFFF
#define OF_UDP_DEFAULT_TIMEOUT   NO_TIMEOUT

class UDPManager{
public:
    UDPManager(){
        m_hSocket= INVALID_SOCKET;
        m_dwTimeoutReceive = OF_UDP_DEFAULT_TIMEOUT;
        m_dwTimeoutSend = OF_UDP_DEFAULT_TIMEOUT;
        
        canGetRemoteAddress	= false;
        nonBlocking			= true;
    }
    
    ~UDPManager(){
        if (HasSocket()) Close();
    }
    
    bool Close(){
        if (m_hSocket == INVALID_SOCKET)
            return(false);
        if(close(m_hSocket)==SOCKET_ERROR){
            std::cout<<"ERROR closing socket"<<std::endl;
            return false;
        }
        m_hSocket= INVALID_SOCKET;
        
        return(true);
    }
    
    bool	HasSocket() const	{	return (m_hSocket)&&(m_hSocket != INVALID_SOCKET);	}
    
    bool Create(){
        if (m_hSocket != INVALID_SOCKET)
            return(false);
        m_hSocket =	socket(AF_INET,	SOCK_DGRAM,	0);
        if (m_hSocket != INVALID_SOCKET)
        {
            int unused = true;
            setsockopt(m_hSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&unused, sizeof(unused));
#ifdef __APPLE__   // MacOS/X requires an additional call
			setsockopt(m_hSocket, SOL_SOCKET, SO_REUSEPORT, (char*)&unused, sizeof(unused));
#endif
        }
        bool ret = m_hSocket !=	INVALID_SOCKET;
        if(!ret) std::cout<<"ERROR creating socket"<<std::endl;
        return ret;
    }
    
    bool SetNonBlocking(bool useNonBlocking){
        nonBlocking		= useNonBlocking;
        
		int arg			= nonBlocking;
		int retVal = ioctl(m_hSocket,FIONBIO,&arg);
        
        bool ret=(retVal >= 0);
        if(!ret) std::cout<<"ERROR setting non blocking\n";
        return ret;
    }
    
    bool Bind(unsigned short usPort){
        saServer.sin_family	= AF_INET;
        saServer.sin_addr.s_addr = INADDR_ANY;
        //Port MUST	be in Network Byte Order
        saServer.sin_port =	htons(usPort);
        int ret = ::bind(m_hSocket,(struct sockaddr*)&saServer,sizeof(struct sockaddr));
        if(ret == SOCKET_ERROR) std::cout<<"ERROR binding to port\n";
        
        return (ret == 0);
    }
    
    bool BindMcast(char *pMcast, unsigned short usPort){
        // bind to port
        if (!Bind(usPort))
        {
            std::cout<<"Warning: BindMcast() cannont bind to port "<<usPort<<"\n";
            return false;
        }
        
        // join the multicast group
        struct ip_mreq mreq;
        mreq.imr_multiaddr.s_addr = inet_addr(pMcast);
        mreq.imr_interface.s_addr = INADDR_ANY;
        
        if (setsockopt(m_hSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char FAR*) &mreq, sizeof (mreq)) == SOCKET_ERROR)
        {
            std::cout<<"ERROR BindMCast()\n";
            return false;
        }
        
        // multicast bind successful
        return true;
    }
    
    int PeekReceive(){
        if (m_hSocket == INVALID_SOCKET){
            std::cout<<"ERROR: INVALID_SOCKET\n";
            return SOCKET_ERROR;
        }
        
        if (m_dwTimeoutReceive	!= NO_TIMEOUT){
            auto ret = WaitReceive(m_dwTimeoutReceive,0);
            if(ret!=0){
                return ret;
            }
        }
        
        int size  = 0;
        int retVal = ioctl(m_hSocket,FIONREAD,&size);
        
        //	error
        if ( retVal != 0 )
        {
            std::cout<<"ERROR getting packet size\n";
            return SOCKET_ERROR;
        }
        
        return size;
    }
    
    int WaitReceive(time_t timeoutSeconds, time_t timeoutMicros){
        if (m_hSocket == INVALID_SOCKET) return SOCKET_ERROR;
        
        fd_set fd;
        FD_ZERO(&fd);
        FD_SET(m_hSocket, &fd);
        timeval	tv;
        tv.tv_sec = timeoutSeconds;
        tv.tv_usec = timeoutMicros;
        auto ret = select(m_hSocket+1,&fd,NULL,NULL,&tv);
        if(ret == 0){
            return SOCKET_TIMEOUT;
        }else if(ret < 0){
            return SOCKET_ERROR;
        }else{
            return 0;
        }
    }
    
    int Receive(unsigned char* pBuff, const int iSize){
        if (m_hSocket == INVALID_SOCKET){
            std::cout<<"ERROR: INVALID_SOCKET\n";
            return(SOCKET_ERROR);
            
        }
        
        if (m_dwTimeoutReceive	!= NO_TIMEOUT){
            auto ret = WaitReceive(m_dwTimeoutReceive,0);
            if(ret!=0){
                return ret;
            }
        }
        
        unsigned int	nLen= sizeof(sockaddr);
        
        int	ret=0;
        
        memset(pBuff, 0, iSize);
        ret= recvfrom(m_hSocket, pBuff,	iSize, 0, (sockaddr *)&saClient, &nLen);
        
        if (ret	> 0)
        {
            canGetRemoteAddress= true;
        }
        else
        {
            canGetRemoteAddress = false;
            
            //int SocketError = ofxNetworkCheckError();
            //if ( SocketError == OFXNETWORK_ERROR(WOULDBLOCK) )
            //    return 0;
        }
        
        return ret;
    }
protected:
    int m_hSocket;
    
    unsigned long m_dwTimeoutReceive;
    unsigned long m_dwTimeoutSend;
    
    struct sockaddr_in saServer;
    struct sockaddr_in saClient;
    
    bool nonBlocking;
    bool canGetRemoteAddress;
};