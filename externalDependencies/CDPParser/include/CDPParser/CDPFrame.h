#pragma once
#include <iostream>
#include <sstream>
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

typedef std::map<uint32_t, CDPFrameSlot>::const_iterator TagIterator;
typedef std::pair<uint32_t, CDPFrameSlot> CDPTag;

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
    
    static std::string toHexSerial(const uint32_t & tId){
        std::stringstream ss;
        ss<<std::hex<<tId;
        std::string s = ss.str();
        s.insert(2,":");
        s.insert(6,":");
        return s;
    }
    
    static uint32_t toDecSerial(const std::string & tId){
        uint32_t d;
        std::stringstream ss;
        std::string s = tId;
        s.erase(2,1);
        s.erase(4,1);
        ss<<std::hex<<s;
        ss>>d;
        return d;
    }
    
    static CDPTag getTag(std::map<uint32_t, CDPFrameSlot> & data, uint32_t ser){
        return CDPTag(ser, data[ser]);
    }
    
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
    
    void addSerials(std::vector<std::string> _serials){
        for(int i=0;i<_serials.size();i++){
            uint32_t d = CDPFrameData::toDecSerial(_serials[i]);
            serials.insert(std::pair<uint32_t, CDPFrameSlot>(d,CDPFrameSlot()));
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
    for(TagIterator it=data.tags.begin(); it!=data.tags.end();it++){
        s<<"    Tag["<<CDPFrameData::toHexSerial(it->first)<<"]:\n";
        CDPFrameSlot tag = it->second;
        s<<"        Pos: "<<tag.getPosition()[0]<<", "<<tag.getPosition()[1]<<", "<<tag.getPosition()[2]<<"\n";
        s<<"        Quat:"<<tag.getQuaternion()[0]<<", "<<tag.getQuaternion()[1]<<", "<<tag.getQuaternion()[2]<<", "<<tag.getQuaternion()[3]<<"\n";
    }
    return s;
}
