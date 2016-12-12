#include "libmotioncapture/cdp.h"

#include "CDPParser/CDPClient.h"

namespace libmotioncapture {
    class MotionCaptureCDPImpl{
    public:
        CDPClient client;
        std::string version;
    };
    
    MotionCaptureCDP::MotionCaptureCDP(std::vector<uint32_t> _serials){
       
        
        pImpl = new MotionCaptureCDPImpl;
        pImpl->client.setSerials(_serials);
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
            std::stringstream ss;
            ss<<it->first;
            const std::string name = ss.str();
            
            Eigen::Vector3f position(
                it->second.getPosition()[0],
                it->second.getPosition()[1],
                it->second.getPosition()[2]);
            
            Eigen::Quaternionf rotation(
                it->second.getQuaternion()[3],
                it->second.getQuaternion()[0],
                it->second.getQuaternion()[1],
                it->second.getQuaternion()[2]);
            
            result[i] = Object(name, position, rotation);
            i++;
        }
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
}
