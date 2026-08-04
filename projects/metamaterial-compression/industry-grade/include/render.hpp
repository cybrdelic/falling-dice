#pragma once
#include "sim.hpp"
namespace meta {
struct Camera{Vec3 position,target,up{0,1,0};double verticalFov=36,aperture=0,focusDistance=1,shutterFraction=.25;};
struct RenderConfig{int width=540,height=720,samplesPerPixel=4,maxDepth=7,threads=0;double bevel=.00045,exposure=1.16;bool denoise=true,motionBlur=false;};
class PathTracer{public:explicit PathTracer(RenderConfig);void setScene(const std::vector<Snapshot>&);std::vector<Vec3>render(const Camera&,int)const;void writePPM(const std::filesystem::path&,const std::vector<Vec3>&)const;private:struct Impl;std::shared_ptr<Impl>impl_;};
Camera compressionCamera(int frame,int total,const FrameDiagnostics&,bool hero=false);
}
