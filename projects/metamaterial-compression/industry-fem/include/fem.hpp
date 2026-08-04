#pragma once
#include "core.hpp"

namespace meta {

struct MaterialModel {
    std::string name = "PETG_reference_bilinear";
    double density = 1270.0;
    double young = 2.05e9;
    double poisson = 0.38;
    double shearCorrection = 0.90;
    double yieldStress = 48.0e6;
    double hardening = 90.0e6;
    double plasticFailureStrain = 0.42;
    double damageStart = 0.20;
    double damageEnd = 0.42;
};

struct SolverConfig {
    int cellsX = 2;
    int cellsY = 5;
    int cellsZ = 2;
    double cellSize = 0.010;
    double radiusTop = 0.00055;
    double radiusBottom = 0.00080;
    double nodeBlendRadius = 0.00100;
    double imperfectionFraction = 0.010;
    double targetStrain = 0.58;
    int loadingSteps = 58;
    int unloadingSteps = 24;
    int maxLbfgsIterations = 160;
    int maxPlasticOuterIterations = 3;
    int lbfgsHistory = 12;
    int beamCurveSegments = 5;
    double forceTolerance = 2.5e-4;
    double relativeEnergyTolerance = 1.0e-10;
    double positionScale = 1.0;
    double rotationLengthScale = 0.008;
    double finiteDifferenceStep = 2.0e-6;
    double contactStiffnessScale = 70.0;
    double contactSearchMargin = 0.00075;
    double planeFriction = 0.08;
    uint64_t seed = 0x4D45544146454D34ULL;
    int threads = 0;
};

struct Node {
    Vec3 rest;
    Vec3 x;
    Vec3 rotationVector{0,0,0};
    double lumpedMass = 0.0;
    int ix = 0, iy = 0, iz = 0;
    bool cellCenter = false;
};

struct BeamState {
    double axialPlasticStrain = 0.0;
    Vec3 plasticCurvature{0,0,0};
    double accumulatedPlasticStrain = 0.0;
    double damage = 0.0;
    bool failed = false;
};

struct Beam {
    int a = -1;
    int b = -1;
    int cellLayer = 0;
    double restLength = 0.0;
    double radius = 0.001;
    Quat referenceFrame;
    BeamState state;
};

struct ContactPair { int beamA = -1; int beamB = -1; };

struct EquilibriumReport {
    bool converged = false;
    int iterations = 0;
    int lineSearchBacktracks = 0;
    double energy = 0.0;
    double gradientNorm = 0.0;
    double maxComponentGradient = 0.0;
    double topReaction = 0.0;
    double bottomReaction = 0.0;
    double maxPenetration = 0.0;
    int activeSelfContacts = 0;
};

struct StepMetrics {
    int step = 0;
    std::string phase;
    double platenY = 0.0;
    double displacement = 0.0;
    double engineeringStrain = 0.0;
    double reactionForce = 0.0;
    double nominalStress = 0.0;
    double externalWork = 0.0;
    double recoverableEnergy = 0.0;
    double plasticDissipation = 0.0;
    double contactEnergy = 0.0;
    double maxPenetration = 0.0;
    double maxDamage = 0.0;
    int failedBeams = 0;
    int activeSelfContacts = 0;
    int nonlinearIterations = 0;
    double residualNorm = 0.0;
    bool converged = false;
};

struct RenderSegment { Vec3 a; Vec3 b; double radius = 0.001; double damage = 0.0; int beam = -1; };
struct FrameState { std::vector<Vec3> nodePositions; std::vector<Vec3> nodeRotations; std::vector<RenderSegment> segments; StepMetrics metrics; };

struct SimulationResult {
    std::vector<FrameState> frames;
    std::vector<StepMetrics> metrics;
    double initialHeight = 0.0;
    double initialArea = 0.0;
    double specimenMass = 0.0;
    double peakForce = 0.0;
    double loadingWork = 0.0;
    double recoveredWork = 0.0;
    double dissipatedWork = 0.0;
    double specificEnergyAbsorption = 0.0;
    double efficiency = 0.0;
    double permanentSet = 0.0;
};

class NonlinearBeamLatticeSolver {
public:
    NonlinearBeamLatticeSolver(SolverConfig config, MaterialModel material);
    SimulationResult run();
    EquilibriumReport solveAtPlaten(double topY);
    FrameState captureFrame(const StepMetrics& metrics) const;
    void writeMetricsCSV(const std::filesystem::path& path, const SimulationResult& result) const;

    const std::vector<Node>& nodes() const { return nodes_; }
    const std::vector<Beam>& beams() const { return beams_; }
    const SolverConfig& config() const { return config_; }
    const MaterialModel& material() const { return material_; }
    double initialHeight() const { return initialHeight_; }
    double initialArea() const { return initialArea_; }
    double specimenMass() const { return specimenMass_; }

    double evaluateTotalEnergy(double topY, double* recoverable = nullptr,
                               double* contact = nullptr, double* topReaction = nullptr,
                               double* bottomReaction = nullptr, double* maxPenetration = nullptr,
                               int* activeContacts = nullptr) const;
    double beamEnergy(int beamIndex) const;

private:
    SolverConfig config_;
    MaterialModel material_;
    std::vector<Node> nodes_;
    std::vector<Beam> beams_;
    std::vector<std::pair<int,int>> beamPairs_;
    std::vector<int> freeDofs_;
    std::vector<int> dofToFree_;
    std::vector<int> bottomBoundaryNodes_;
    std::vector<int> topBoundaryNodes_;
    double initialHeight_ = 0.0;
    double initialArea_ = 0.0;
    double specimenMass_ = 0.0;
    double topY_ = 0.0;
    double previousTopY_ = 0.0;
    double externalWork_ = 0.0;
    double plasticDissipation_ = 0.0;
    mutable std::vector<ContactPair> activeContacts_;

    void buildBCC();
    void buildGaugeConstraints();
    void distributeMass();
    void buildPotentialContactPairs();
    std::vector<double> packVariables() const;
    void unpackVariables(const std::vector<double>& x);
    void projectGauge(std::vector<double>& x) const;
    double objective(const std::vector<double>& variables, std::vector<double>* gradient,
                     double topY, EquilibriumReport* report = nullptr);
    double beamLocalEnergy(int beamIndex, const std::array<double,12>& local) const;
    double beamPlaneContactEnergy(int beamIndex, const std::array<double,12>& local,
                                  double topY, double* topReaction = nullptr,
                                  double* bottomReaction = nullptr,
                                  double* maxPenetration = nullptr) const;
    double beamPairContactEnergy(int beamA, int beamB, const std::array<double,24>& local,
                                 double* maxPenetration = nullptr) const;
    void updatePlasticity(double& plasticIncrement);
    std::vector<RenderSegment> buildRenderSegments() const;
    Quat nodeOrientation(const Vec3& rotationVector) const;
    Quat endFrame(const Beam& beam, int nodeIndex, const Vec3& rotationVector) const;
    Vec3 curvePoint(const Beam& beam, double t, const Vec3& xa, const Vec3& ra,
                    const Vec3& xb, const Vec3& rb) const;
    Vec3 curveTangent(const Beam& beam, double t, const Vec3& xa, const Vec3& ra,
                      const Vec3& xb, const Vec3& rb) const;
    std::vector<ContactPair> detectActiveContacts() const;
    static double segmentSegmentDistanceSquared(const Vec3& p1, const Vec3& q1,
                                                const Vec3& p2, const Vec3& q2,
                                                double& s, double& t, Vec3& c1, Vec3& c2);
};

} // namespace meta
