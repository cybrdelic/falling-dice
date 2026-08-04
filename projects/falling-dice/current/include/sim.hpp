#pragma once
#include "core.hpp"

namespace tower {

constexpr int kTowerX = 10;
constexpr int kTowerZ = 5;
constexpr int kLayerCount = 20;
constexpr int kModulesPerLayer = kTowerX * kTowerZ;
constexpr int kModuleCount = kLayerCount * kModulesPerLayer;
constexpr int kCoilCount = 1;

enum class BodyKind : int32_t { Module = 0, CoilSegment = 1, Pedestal = 2, Indicator = 3 };
enum class AssemblyPhase : uint32_t { FixtureHold = 0, FixtureRetract = 1, PermanentCollapse = 2, Settling = 3, Reserved4 = 4, Reserved5 = 5, Complete = 6 };
const char* phaseName(AssemblyPhase phase);

struct Body {
    Vec3 p, v, w, force, torque;
    Quat q;
    Vec3 half{0.008, 0.008, 0.008};
    Vec3 invILocal{1, 1, 1};
    Vec3 initialP;
    Quat initialQ;
    Vec3 sleepReferenceMagneticForce{0, 0, 0};
    Vec3 sleepReferenceMagneticTorque{0, 0, 0};
    double rounding = 0.00155;
    double mass = 0.00485;
    double invMass = 1.0 / 0.00485;
    double strengthVariation = 1.0;
    double sleepTimer = 0;
    int material = 0;
    int id = 0;
    int moduleClass = 0;
    int layer = -1;
    int slot = -1;
    int contactCount = 0;
    int supportCount = 0;
    BodyKind kind = BodyKind::Module;
    bool anchored = false;
    bool collidable = true;
    bool sleeping = false;
};

struct Snapshot {
    Vec3 p;
    Quat q;
    Vec3 v, w, half;
    int material = 0;
    int id = 0;
    int kind = 0;
    int moduleClass = 0;
    int layer = -1;
    int slot = -1;
};

struct CoilState { Vec3 center, normal; double radius = 0; double current = 0; double turns = 0; int material = 4; };

struct FrameDiagnostics {
    double time = 0;
    AssemblyPhase phase = AssemblyPhase::FixtureHold;
    double globalMagnetization = 1;
    double meanMagnetization = 1;
    double permanentFieldFraction = 1;
    double collapseFraction = 0;
    double fractionBelow = 0;
    double meanSpeed = 0;
    double maxSpeed = 0;
    double meanSpin = 0;
    double maxSpin = 0;
    double kineticEnergy = 0;
    double maxPenetration = 0;
    double airborneFraction = 0;
    double meanNearestNeighbor = 0;
    double meanDipoleAlignment = 0;
    double magneticWork = 0;
    int closeMagneticPairs = 0;
    int snappedPolePairs = 0;
    int contactManifolds = 0;
    int clusterCount = 0;
    int largestCluster = 0;
};

struct SimulationResult {
    std::vector<std::vector<Snapshot>> frames;
    std::vector<FrameDiagnostics> diagnostics;
    std::array<CoilState, kCoilCount> coils{};
};

struct SimulationConfig {
    int fps = 30;
    int frames = 300;
    int baseSubsteps = 8;
    int maxSubsteps = 24;
    int velocityIterations = 16;
    int positionIterations = 8;
    uint64_t seed = 0x54484F5553414E44ULL;
    double preRollSeconds = 2.0;
    double gravity = 9.81;
    double airDrag = 0.008;
    double angularAirDrag = 0.012;
    double moduleDensity = 1180.0;
    double staticFriction = 0.58;
    double dynamicFriction = 0.44;
    double rollingFriction = 0.12;
    double torsionalFriction = 0.09;
    double restitution = 0.060;
    double contactSlop = 0.000025;
    double baumgarte = 0.0;
    double positionPercent = 0.86;
    double broadCell = 0.028;
    double contactBroadMargin = 0.0016;
    double permanentMagnetization = 0.0;
    double magneticK = 0.0;
    double poleSoftening = 0.0;
    double facePixelOffset = 0.0;
    double faceInteractionStart = 0.0;
    double farFadeStart = 0.0;
    double farCut = 0.0;
    double magneticNormalDamping = 0.0;
    double magneticTangentialDamping = 0.0;
    double magneticAngularDamping = 0.0;
    double captureGap = 0.0;
    double initialHold = 1.25;
    double releaseDuration = 0.32;
    double trapdoorTravel = 0.22;
    double collapseLatest = 2.00;
    double settlingDuration = 2.20;
    double settledHeight = 0.065;
    bool enableMotionSleep = true;
    double sleepLinearSpeed = 0.0030;
    double sleepAngularSpeed = 0.12;
    double sleepDelay = 0.22;
    double sleepPenetration = 0.00120;
    double wakeMagneticForceDelta = 1e9;
    double wakeMagneticTorqueDelta = 1e9;
    double wakeImpactNormalSpeed = 0.11;
    double wakeImpactTangentSpeed = 0.16;
    double wakeImpactPenetration = 0.00150;
    double wakeImpulse = 1e9;
};

class Simulator {
public:
    explicit Simulator(SimulationConfig config);
    size_t bodyCount() const { return bodies_.size(); }
    void loadState(const std::vector<Snapshot>& snapshots, AssemblyPhase phase = AssemblyPhase::Settling);
    SimulationResult run();

    struct ContactPoint { Vec3 point; double penetration = 0; double normalImpulse = 0; Vec3 tangentImpulse{0,0,0}; uint32_t feature = 0; };
    struct ContactManifold { int a = -1; int b = -1; Vec3 normal{0,1,0}; std::array<ContactPoint,4> points{}; int pointCount = 0; int lifetime = 0; };

private:
    SimulationConfig cfg_;
    std::vector<Body> bodies_;
    std::array<CoilState, kCoilCount> coils_{};
    int trapdoorBodyIndex_ = -1;
    std::unordered_map<uint64_t, ContactManifold, Int3Hash> manifoldCache_;
    std::unordered_map<uint64_t, uint16_t, Int3Hash> magneticFaceHistory_;
    std::unordered_map<uint64_t, uint16_t, Int3Hash> capturedFacePairs_;
    AssemblyPhase phase_ = AssemblyPhase::FixtureHold;
    double phaseStart_ = 0;
    double time_ = 0;
    double initialMeanHeight_ = 1;
    double accumulatedMagneticWork_ = 0;
    double lastMaxPenetration_ = 0;
    int lastCloseMagneticPairs_ = 0;
    int lastSnappedPolePairs_ = 0;
    int lastContactManifolds_ = 0;
    bool preRolling_ = false;
    bool stateLoaded_ = false;

    void initializeScene();
    void initializeModules();
    void initializePedestal();
    void enterPhase(AssemblyPhase phase);
    void updatePhase(double dt);
    int chooseSubsteps(double frameDt) const;
    double globalMagnetization() const;
    double bodyMagnetization(const Body& body) const;
    void step(double dt);
    void runPreRoll();
    std::vector<Snapshot> capture() const;
    FrameDiagnostics measure() const;
};

} // namespace tower
