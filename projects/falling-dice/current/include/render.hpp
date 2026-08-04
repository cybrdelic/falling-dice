#pragma once
#include "sim.hpp"

namespace tower {

struct Camera {
    Vec3 position;
    Vec3 target;
    Vec3 up{0, 1, 0};
    double verticalFov = 36;
    double aperture = 0;
    double focusDistance = 1;
    double shutterFraction = 0.44;
};

struct RenderConfig {
    int width = 360;
    int height = 640;
    int samplesPerPixel = 2;
    int maxDepth = 7;
    int threads = 0;
    double bevel = 0.00072;
    double exposure = 0.88;
    bool denoise = true;
    bool motionBlur = true;
    bool decorateModules = true;
};

class PathTracer {
public:
    explicit PathTracer(RenderConfig config);
    void setScene(const std::vector<Snapshot>& scene);
    std::vector<Vec3> render(const Camera& camera, int frameIndex) const;
    void writePPM(const std::filesystem::path& path,
                  const std::vector<Vec3>& pixels) const;

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

Camera cinematicCamera(int frame,
                       int totalFrames,
                       const FrameDiagnostics& diagnostics,
                       bool hero = false);

} // namespace tower
