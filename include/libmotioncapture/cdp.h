#pragma once
#include <map>
#include "libmotioncapture/motioncapture.h"
//#include <CDPParser/CDPClient.h>

namespace libmotioncapture {
    class MotionCaptureCDPImpl;
    
    class MotionCaptureCDP : public MotionCapture{
    public:
        //map is <cf name, tag hex serial>
        MotionCaptureCDP(std::map<std::string, std::string> _nameSerials);
        ~MotionCaptureCDP();
        
        const std::string & version() const;
        
        
        virtual void waitForNextFrame();
        virtual void getObjects(std::vector<Object>& result) const;
        virtual void getObjectByName(const std::string & name, Object & result) const;
        
        virtual bool supportsObjectTracking() const;
        virtual bool supportsLatencyEstimate() const;
        virtual bool supportsPointCloud() const;

        virtual void getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr result) const{}
        virtual void getLatency(std::vector<LatencyInfo> & result) const{}
        
        void setAxisRemapping();
        //Object toObject(std::pair<uint32_t, CDPFrameSlot> tag) const;
        Eigen::Vector3f ptToEigen(float * pt) const;
        Eigen::Quaternionf quatToEigen(float * quat) const;
        
        float axisMultiplier[3];
        float axisOrder[3];
    private:
        MotionCaptureCDPImpl * pImpl;
    };
}
