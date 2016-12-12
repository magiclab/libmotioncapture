#pragma once
#include <iostream>
#include <map>
#include <vector>

class CDPFrameSlot{
public:
    CDPFrameSlot(){
        reset();
    }
    
    void reset(){
        for(int i=0;i<3;i++)pos[i]=0.0;
        for(int i=0;i<4;i++)quat[i]=0.0;
        bPos=false;
        bQuat=false;
    }
    
    void setPosition(float x, float y, float z){
        pos[0]=x;
        pos[1]=y;
        pos[2]=z;
        bPos=true;
    }
    
    void setQuaternion(float x, float y, float z, float w){
        quat[0]=x;
        quat[1]=y;
        quat[2]=z;
        quat[3]=w;
        bQuat=true;
    }
    
    bool isComplete(){
        return (bPos && bQuat);
    }
    bool hasPosition(){return bPos;}
    bool hasQuaternion(){return bQuat;}
    
    float * getPosition(){return pos;}
    float * getQuaternion(){return quat;}
protected:
    float pos[3];
    float quat[4];
    float bPos;
    float bQuat;
};

class CDPFrameData{
public:
    enum FrameResult{
        SUCCESS,
        TIMEOUT,
        UNKNOWN
    };
    
    CDPFrameData(){reset();}
    
    void reset(){
        tags.clear();
        result = CDPFrameData::UNKNOWN;
        bNew = false;
    }
    
    void setNew(bool b=true){bNew=b;}
    bool isNew(){return bNew;}
    FrameResult getResult(){return result;}
    void setResult(FrameResult fr){result=fr;}
    
    std::map<uint32_t, CDPFrameSlot> tags;
protected:
    FrameResult result;
    bool bNew;
};

class CDPFrame{
public:
    
    typedef std::map<uint32_t, CDPFrameSlot>::iterator FrameIterator;
    
    CDPFrame(){
        clear();
        reset();
    }
    
    void reset(){
        for(FrameIterator it=serials.begin();it!=serials.end();it++){
            it->second.reset();
        }
        nFlagged= 0;
        bComplete = false;
        result = CDPFrameData::UNKNOWN;
    }
    
    void clear(){
        serials.clear();
        nFlagged = 0;
        bComplete = false;
    }
    
    void addSerials(std::vector<uint32_t> _serials){
        for(int i=0;i<_serials.size();i++){
            serials.insert(std::pair<uint32_t, CDPFrameSlot>(_serials[i],CDPFrameSlot()));
        }
    }
    
    void setPosition(uint32_t sId, float x, float y, float z){
        FrameIterator it = serials.find(sId);
        if(it!=serials.end()){
            if(!it->second.hasPosition()){
                it->second.setPosition(x, y, z);
                if(it->second.hasQuaternion()){
                    nFlagged++;
                    //bComplete=(nFlagged>=serials.size());
                    testComplete();
                }
            }
        }
    }
    
    void setQuaternion(uint32_t sId, float x, float y, float z, float w){
        FrameIterator it = serials.find(sId);
        if(it!=serials.end()){
            if(!it->second.hasQuaternion()){
                it->second.setQuaternion(x, y, z, w);
                if(it->second.hasPosition()){
                    nFlagged++;
                    //bComplete=(nFlagged>=serials.size());
                    testComplete();
                }
            }
        }
    }
    
    bool isComplete(){return bComplete;}
    
    void testComplete(){
        if(nFlagged>=serials.size()){
            bComplete=true;
            result = CDPFrameData::SUCCESS;
        }else{
            bComplete=false;
        }
    }
    
    void setTimeout(){
        bComplete=true;
        result = CDPFrameData::TIMEOUT;
    }
    
    void get(CDPFrameData & data){
        data.reset();
        data.setNew();
        data.tags.insert(serials.begin(), serials.end());
        data.setResult(result);
    }
protected:
    
    std::map<uint32_t, CDPFrameSlot> serials;
    int nFlagged;
    bool bComplete;
    CDPFrameData::FrameResult result;
};

std::ostream& operator<<(std::ostream& s, CDPFrameData const& data){
    s<<"FRAME ("<<data.tags.size()<<" tags):"<<std::endl;
    return s;
}