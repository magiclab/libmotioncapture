#include "libmotioncapture/cdp.h"

#include "CDPParser/CDPClient.h"

namespace libmotioncapture {
    class MotionCaptureCDPImpl{
    public:
        CDPClient client;
        std::string version;
    };
    
    void MotionCaptureCDP::setAxisRemapping(){
        axisOrder[0]=1;
        axisOrder[1]=0;
        axisOrder[2]=2;
        axisMultiplier[0]=-1;
        axisMultiplier[1]=1;
        axisMultiplier[2]=1;
    }
    
    MotionCaptureCDP::MotionCaptureCDP(std::map<std::string, std::string> _nameSerials){
        setAxisRemapping();
        pImpl = new MotionCaptureCDPImpl;
        pImpl->client.setup(_nameSerials);
        pImpl->client.connect();
        pImpl->version = "1.0";
    }
    
    const std::string & MotionCaptureCDP::version() const{
        return pImpl->version;
    }
    
    void MotionCaptureCDP::waitForNextFrame(){
        pImpl->client.update();
    }
    
    void MotionCaptureCDP::getObjects(std::vector<Object>& result) const{
        result.clear();
        CDPFrameData data = pImpl->client.getLastFrame();
        size_t count = data.tags.size();
        result.resize(count);
        map<uint32_t,CDPFrameSlot>::iterator it;
        int i = 0;
        for(it=data.tags.begin();it!=data.tags.end();it++){
            std::string name = pImpl->client.getNameForSerial(it->first);
            Eigen::Vector3f position = ptToEigen(it->second.getPosition());
            Eigen::Quaternionf rotation = quatToEigen(it->second.getQuaternion());
            result[i] = Object(name, position, rotation);
            //result[i] = toObject(CDPTag(it->first, it->second));
            i++;
        }
    }
    
    void MotionCaptureCDP::getObjectByName(const std::string & name, Object & result) const{
        CDPFrameData data = pImpl->client.getLastFrame();
        uint32_t d = pImpl->client.getDecSerialForName(name);
        std::string nn = pImpl->client.getNameForSerial(d);
        Eigen::Vector3f position = ptToEigen(data.tags[d].getPosition());
        Eigen::Quaternionf rotation = quatToEigen(data.tags[d].getQuaternion());
        result = Object(nn, position, rotation);
        //result = toObject(CDPTag(d,data.tags[d]));
    }
    
    MotionCaptureCDP::~MotionCaptureCDP(){
        delete pImpl;
    }
    
    bool MotionCaptureCDP::supportsObjectTracking() const{
        return true;
    }
    
    bool MotionCaptureCDP::supportsLatencyEstimate() const{
        return false;
    }
    
    bool MotionCaptureCDP::supportsPointCloud() const{
        return false;
    }
    
    Eigen::Vector3f MotionCaptureCDP::ptToEigen(float * pt) const{
        int id0 = axisOrder[0];
        int id1 = axisOrder[1];
        int id2 = axisOrder[2];
        return Eigen::Vector3f(pt[id0]*axisMultiplier[0],
                               pt[id1]*axisMultiplier[1],
                               pt[id2]*axisMultiplier[2]);
    }
    
    Eigen::Quaternionf MotionCaptureCDP::quatToEigen(float * quat) const{
        int id0 = axisOrder[0];
        int id1 = axisOrder[1];
        int id2 = axisOrder[2];
        float qx = quat[id0]*axisMultiplier[0];
        float qy = quat[id1]*axisMultiplier[1];
        float qz = quat[id2]*axisMultiplier[2];
        float qw = quat[3];
        return Eigen::Quaternionf(qw,qx,qy,qz);
    }
    
    /*Object toObject(CDPTag tag) {
        std::string name = pImpl->client.getNameForSerial(tag.first);
        Eigen::Vector3f position = ptToEigen(tag.second.getPosition());
        Eigen::Quaternionf rotation = quatToEigen(tag.second.getQuaternion());
        return Object(name, position, rotation);
    }*/
}
