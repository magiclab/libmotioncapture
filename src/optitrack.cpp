#include "libmotioncapture/optitrack.h"

//NatNet
#include <NatNetLinux/NatNetClient.h>

namespace libmotioncapture {
    
    class MotionCaptureOptitrackImpl{
    public:
        NatNetClient client;
        std::string version;
    };
    
    MotionCaptureOptitrack::MotionCaptureOptitrack(std::string localIp,
                                                   std::string serverIp){
        
        axisMultiplier = Point3f(-1,1,1);
        axisOrder = Point3f(1,0,2);
        
        pImpl = new MotionCaptureOptitrackImpl;
        
        pImpl->client.connect(localIp, serverIp);
        pImpl->version = pImpl->client.getVersionString();
    }
    
    const std::string & MotionCaptureOptitrack::version() const{
        return pImpl->version;
    }
    
    void MotionCaptureOptitrack::waitForNextFrame(){
        pImpl->client.update();
    }
    
    Point3f MotionCaptureOptitrack::transformPoint(Point3f pt) const{
        Point3f p;
        p.x = pt[axisOrder.x]*axisMultiplier.x;
        p.y = pt[axisOrder.y]*axisMultiplier.y;
        p.z = pt[axisOrder.z]*axisMultiplier.z;
        return p;
    }
    
    Eigen::Vector3f MotionCaptureOptitrack::toEigen(Point3f pt) const{
        return Eigen::Vector3f(pt.x, pt.y, pt.z);
    }
    
    Eigen::Quaternionf MotionCaptureOptitrack::toEigen(Quaternion4f quat) const{
        float qx = quat.getAxis(axisOrder.x)*axisMultiplier.x;
        float qy = quat.getAxis(axisOrder.y)*axisMultiplier.y;
        float qz = quat.getAxis(axisOrder.z)*axisMultiplier.z;
        float qw = quat.qw;
        return Eigen::Quaternionf(qw,qx,qy,qz);
    }
    
    const std::string MotionCaptureOptitrack::getName(RigidBody rb) const{
        std::stringstream ss;
        ss<<"cf"<<rb.id();
        const std::string name = ss.str();
        return name;
    }
    
    Object MotionCaptureOptitrack::toObject(RigidBody rb) const{
        const std::string name = getName(rb);
        Point3f p = transformPoint(rb.location());
        Eigen::Vector3f position = toEigen(p);
        Eigen::Quaternionf rotation = toEigen(rb.orientation());
        return Object(name, position, rotation);
    }
    
    void MotionCaptureOptitrack::getObjects(std::vector<Object>& result) const{
        result.clear();
        std::vector<RigidBody> const & rbs = pImpl->client.getLastFrame().rigidBodies();
        size_t count = rbs.size();
        result.resize(count);
        for(size_t i=0;i<count;i++){
            result[i] = toObject(rbs[i]);
        }
    }
    
    void MotionCaptureOptitrack::getObjectByName(const std::string & name, libmotioncapture::Object & result) const{
        std::vector<RigidBody> const & rbs = pImpl->client.getLastFrame().rigidBodies();
        size_t count = rbs.size();
        for(size_t i=0;i<count;i++){
            const std::string curName = getName(rbs[i]);
            if(curName==name){
                result = toObject(rbs[i]);
                return;
            }
        }
    }
    
    void MotionCaptureOptitrack::getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr result) const{
        result->clear();
        std::vector<Point3f> const & markers = pImpl->client.getLastFrame().unIdMarkers();
        size_t count = markers.size();
        for(size_t i=0;i<count;++i){
            Point3f p = transformPoint(markers[i]);
            result->push_back(pcl::PointXYZ(p.x,p.y,p.z));
        }
    }
    
    void MotionCaptureOptitrack::getLatency(std::vector<libmotioncapture::LatencyInfo> & result) const{
        result.clear();
        
        result.clear();
	std::string nn = "";
	double dd = pImpl->client.getLastFrame().latency();
        result.emplace_back(libmotioncapture::LatencyInfo(nn, dd));
    }
    
    MotionCaptureOptitrack::~MotionCaptureOptitrack(){
        delete pImpl;
    }
    
    bool MotionCaptureOptitrack::supportsObjectTracking() const{
        return true;
    }
    
    bool MotionCaptureOptitrack::supportsLatencyEstimate() const{
        return true;
    }
    
    bool MotionCaptureOptitrack::supportsPointCloud() const{
        return true;
    }
}

