#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

#include "position.h"
#include "mpu9250_quaternion.h"

#include "UDPManager.h"
#include "CDPPacket.h"
#include "CDPFrame.h"

//#define PRINT_PACKETS
//#define GET_ALL_PKTS

typedef std::map<std::string,std::string>::const_iterator NameIteratorByHex;
typedef std::map<std::string,uint32_t>::const_iterator NameIteratorByDec;

class CDPClient{
public:
    CDPClient(){
        bConnected=false;
        toSec = 1.0;
    }
    
    void setup(std::map<std::string, std::string> _nameMap){
        nameTableHex = _nameMap;
        std::vector<uint32_t> _serials;
        for(NameIteratorByHex it=nameTableHex.begin();it!=nameTableHex.end();it++){
            uint32_t d = CDPFrameData::toDecSerial(it->second);
            _serials.push_back(d);
            nameTableDec[it->first] = d;
            nameTableDecRev[d] = it->first;
        }
        setSerials(_serials);
    }
    void setSerials(std::vector<uint32_t> _serials){cdpFrame.addSerials(_serials);}
    void setSerials(std::vector<std::string> _serials){cdpFrame.addSerials(_serials);}
    
    std::string getHexSerialForName(string n){
        return nameTableHex[n];
    }
    
    uint32_t getDecSerialForName(string n){
        return nameTableDec[n];
    }
    
    std::string getNameForSerial(uint32_t d){
        return nameTableDecRev[d];
    }
    
    void setTimer(){
#ifdef __APPLE__        
        timeStart = std::clock();
#else
        timeStart = clock();
#endif        
    }
    
    void connect(std::string host = "239.255.76.67", int port=7667){
        bConnected = udp.Create();
        bConnected = udp.BindMcast((char *)host.c_str(), port);
        bConnected = udp.SetNonBlocking(true);
        
        setTimer();        
    }
    
    void disconnect(){udp.Close();}
    
    void update(){
        while(!cdpFrame.isComplete()){
            if(checkTimeout()){
                std::cout<<"Frame Timeout"<<std::endl;
                cdpFrame.setTimeout();
            }else{
                getCDPData();
            }
        }
        cdpFrame.get(lastFrame);
        setTimer();
        cdpFrame.reset();
    }
    
    bool isConnected(){return bConnected;}
    
    bool isNewFrameReady(){
        return (lastFrame.isNew() && (lastFrame.getResult()==CDPFrameData::SUCCESS));
    }
    
    CDPFrameData & getLastFrame(){
        lastFrame.setNew(false);
        return lastFrame;
    }
    
    double toSec; //seconds until timeout
private:
    
    bool checkTimeout(){
#ifdef __APPLE__
        timeCur = std::clock() - timeStart;
#else
        timeCur = clock() - timeStart;
#endif        
        double sec = timeCur/(double)CLOCKS_PER_SEC;
        return (sec>toSec);
    }
    
    void getCDPData(){
        int _packet_size = udp.PeekReceive();
        unsigned char _rx_buffer[_packet_size];
        int r = udp.Receive(_rx_buffer,sizeof(_rx_buffer));
        if(r != SOCKET_ERROR && r != SOCKET_TIMEOUT){
            CDPPacket _packet(0);
            if (_packet.HasValidHeader(_rx_buffer)){
                _packet.ConvertPacketBuffer(_rx_buffer, _packet_size);
                for(auto _data_item :  _packet.GetDataItems()){
                    if(_data_item->GetType() == CDP_DATATYPE_POSITION){
                        parsePosition(_packet.GetSerialNumber(), _packet.GetSequence(), *_data_item.get());
                        
                    }
                    if(_data_item->GetType() == CDP_DATATYPE_QUAT){
                        parseQuat(_packet.GetSerialNumber(), _packet.GetSequence(), *_data_item.get());
                        
                    }
                    if(_data_item->GetType() == CDP_DATATYPE_MPU9250_QUATERNION){
                        parseQuat(_packet.GetSerialNumber(), _packet.GetSequence(), *_data_item.get());
                        
                    }
#ifdef GET_ALL_PKTS
                    if(_data_item->GetType() == CDP_DATATYPE_MPU9250_GYROSCOPE){
                    parseGyro(_packet.GetSerialNumber(), _packet.GetSequence(), *_data_item.get());
                    
                    }
                    if(_data_item->GetType() == CDP_DATATYPE_MPU9250_ACCELEROMETER){
                    parseAccel(_packet.GetSerialNumber(), _packet.GetSequence(), *_data_item.get());
                    
                    }
#endif                    
                }
            }
        }
    }
    
    void parsePosition(uint32_t serial_number, uint32_t sequence_number, const CdpDataItem &data_item){
        if (data_item.GetDataSize() != sizeof(cdp_data_position_t)) {
            std::cout<<"ERROR: Received position data of a bad size!" <<std::endl;
            return;
        }
        
        cdp_data_position_t* _pos = static_cast<cdp_data_position_t*>(data_item.GetData());
#ifdef PRINT_PACKETS
        std::cout << 0 << std::hex << serial_number << std::dec << " Pos (mm): (" << _pos->coordinates.x << ", " << _pos->coordinates.y << ", " << _pos->coordinates.z << ')' << std::endl;
#endif
        cdpFrame.setPosition(serial_number, _pos->coordinates.x,_pos->coordinates.y,_pos->coordinates.z);
    }
    
    void parseQuat(uint32_t serial_number, uint32_t sequence_number, const CdpDataItem& data_item){
        if (data_item.GetDataSize() != sizeof(quaternion_data)) {
            std::cout << "ERROR: Received MPU9250 Quat Data with bad size!" << std::endl;
            return;
        }
        
        quaternion_data* _quat_data = static_cast<quaternion_data*>(data_item.GetData());
        
        float xyzw[4];
        for(uint8_t i=0; i<4; i++) {
            xyzw[i] = _quat_data->xyzw[i];
            xyzw[i] /= (1 << 30);  //divide by 2^30 to get a value between -1 and 1 in float format
        }
#ifdef PRINT_PACKETS
        std::cout<< 0 << std::hex << serial_number <<"ofQuaternion "<< xyzw[0]<<" "<<  xyzw[1]<<" "<<  xyzw[2]<<" "<<  xyzw[3]<<endl;
#endif
        cdpFrame.setQuaternion(serial_number, xyzw[0],xyzw[1],xyzw[2],xyzw[3]);
    }
    
    CDPFrame cdpFrame;
    CDPFrameData lastFrame;
    UDPManager udp;
    std::map<std::string, std::string>nameTableHex;
    std::map<std::string,uint32_t>nameTableDec;
    std::map<uint32_t,std::string>nameTableDecRev;
#ifdef __APPLE__
    std::clock_t timeStart;
    std::clock_t timeCur;
#else
    clock_t timeStart;
    clock_t timeCur;
#endif    
    struct timespec lastTs;
    struct timeval timeout;
    bool bConnected;
};
