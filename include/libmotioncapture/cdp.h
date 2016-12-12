#pragma once
#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {
    class MotionCaptureCDPImpl;
    
    class MotionCaptureCDP : public MotionCapture{
    public:
        MotionCaptureCDP(std::vector<uint32_t> _serials);
        ~MotionCaptureCDP();
        
        const std::string & version() const;
        
        
        virtual void waitForNextFrame();
        virtual void getObjects(std::vector<Object>& result) const;
        
        virtual bool supportsObjectTracking() const;
        virtual bool supportsLatencyEstimate() const;
        virtual bool supportsPointCloud() const;
        
    private:
        MotionCaptureCDPImpl * pImpl;
    };
}
