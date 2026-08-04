#include "render.hpp"

#include <cstring>

namespace tower {
namespace {

constexpr uint32_t kCacheVersion = 10;

struct CacheHeader {
    char magic[8];
    uint32_t version = 0;
    uint32_t frames = 0;
    uint32_t bodies = 0;
    uint32_t diagnosticBytes = 0;
    uint32_t snapshotBytes = 0;
    uint32_t fps = 24;
};

struct DiskSnapshot {
    double p[3];
    double q[4];
    double v[3];
    double w[3];
    double half[3];
    int32_t material;
    int32_t id;
    int32_t kind;
    int32_t moduleClass;
    int32_t layer;
    int32_t slot;
};

struct DiskDiagnostics {
    double time;
    uint32_t phase;
    double globalMagnetization;
    double meanMagnetization;
    double permanentFieldFraction;
    double collapseFraction;
    double fractionBelow;
    double meanSpeed;
    double maxSpeed;
    double meanSpin;
    double maxSpin;
    double kineticEnergy;
    double maxPenetration;
    double airborneFraction;
    double meanNearestNeighbor;
    double meanDipoleAlignment;
    double magneticWork;
    int32_t closeMagneticPairs;
    int32_t snappedPolePairs;
    int32_t contactManifolds;
    int32_t clusterCount;
    int32_t largestCluster;
};

DiskSnapshot toDisk(const Snapshot& snapshot) {
    return {
        {snapshot.p.x, snapshot.p.y, snapshot.p.z},
        {snapshot.q.w, snapshot.q.x, snapshot.q.y, snapshot.q.z},
        {snapshot.v.x, snapshot.v.y, snapshot.v.z},
        {snapshot.w.x, snapshot.w.y, snapshot.w.z},
        {snapshot.half.x, snapshot.half.y, snapshot.half.z},
        snapshot.material,
        snapshot.id,
        snapshot.kind,
        snapshot.moduleClass,
        snapshot.layer,
        snapshot.slot
    };
}

Snapshot fromDisk(const DiskSnapshot& disk) {
    Snapshot snapshot;
    snapshot.p = {disk.p[0], disk.p[1], disk.p[2]};
    snapshot.q = {disk.q[0], disk.q[1], disk.q[2], disk.q[3]};
    snapshot.v = {disk.v[0], disk.v[1], disk.v[2]};
    snapshot.w = {disk.w[0], disk.w[1], disk.w[2]};
    snapshot.half = {disk.half[0], disk.half[1], disk.half[2]};
    snapshot.material = disk.material;
    snapshot.id = disk.id;
    snapshot.kind = disk.kind;
    snapshot.moduleClass = disk.moduleClass;
    snapshot.layer = disk.layer;
    snapshot.slot = disk.slot;
    return snapshot;
}

DiskDiagnostics toDisk(const FrameDiagnostics& d) {
    return {
        d.time,
        uint32_t(d.phase),
        d.globalMagnetization,
        d.meanMagnetization,
        d.permanentFieldFraction,
        d.collapseFraction,
        d.fractionBelow,
        d.meanSpeed,
        d.maxSpeed,
        d.meanSpin,
        d.maxSpin,
        d.kineticEnergy,
        d.maxPenetration,
        d.airborneFraction,
        d.meanNearestNeighbor,
        d.meanDipoleAlignment,
        d.magneticWork,
        d.closeMagneticPairs,
        d.snappedPolePairs,
        d.contactManifolds,
        d.clusterCount,
        d.largestCluster
    };
}

FrameDiagnostics fromDisk(const DiskDiagnostics& x) {
    FrameDiagnostics d;
    d.time = x.time;
    d.phase = AssemblyPhase(x.phase);
    d.globalMagnetization = x.globalMagnetization;
    d.meanMagnetization = x.meanMagnetization;
    d.permanentFieldFraction = x.permanentFieldFraction;
    d.collapseFraction = x.collapseFraction;
    d.fractionBelow = x.fractionBelow;
    d.meanSpeed = x.meanSpeed;
    d.maxSpeed = x.maxSpeed;
    d.meanSpin = x.meanSpin;
    d.maxSpin = x.maxSpin;
    d.kineticEnergy = x.kineticEnergy;
    d.maxPenetration = x.maxPenetration;
    d.airborneFraction = x.airborneFraction;
    d.meanNearestNeighbor = x.meanNearestNeighbor;
    d.meanDipoleAlignment = x.meanDipoleAlignment;
    d.magneticWork = x.magneticWork;
    d.closeMagneticPairs = x.closeMagneticPairs;
    d.snappedPolePairs = x.snappedPolePairs;
    d.contactManifolds = x.contactManifolds;
    d.clusterCount = x.clusterCount;
    d.largestCluster = x.largestCluster;
    return d;
}

void saveCache(const std::filesystem::path& path,
               const SimulationResult& result,
               int fps) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("cannot write cache " + path.string());

    CacheHeader header{};
    std::memcpy(header.magic, "DICE1K1", 7);
    header.version = kCacheVersion;
    header.frames = uint32_t(result.frames.size());
    header.bodies = result.frames.empty() ? 0 : uint32_t(result.frames.front().size());
    header.diagnosticBytes = sizeof(DiskDiagnostics);
    header.snapshotBytes = sizeof(DiskSnapshot);
    header.fps = uint32_t(fps);
    output.write(reinterpret_cast<const char*>(&header), sizeof(header));

    for (size_t frame = 0; frame < result.frames.size(); ++frame) {
        const DiskDiagnostics diagnostics = toDisk(result.diagnostics[frame]);
        output.write(reinterpret_cast<const char*>(&diagnostics), sizeof(diagnostics));
        for (const Snapshot& snapshot : result.frames[frame]) {
            const DiskSnapshot disk = toDisk(snapshot);
            output.write(reinterpret_cast<const char*>(&disk), sizeof(disk));
        }
    }
}

SimulationResult loadCache(const std::filesystem::path& path, int* fps = nullptr) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot read cache " + path.string());

    CacheHeader header{};
    input.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (std::strncmp(header.magic, "DICE1K1", 7) != 0 ||
        header.version != kCacheVersion ||
        header.snapshotBytes != sizeof(DiskSnapshot) ||
        header.diagnosticBytes != sizeof(DiskDiagnostics)) {
        throw std::runtime_error("unsupported or corrupt DICE1K cache");
    }
    if (fps) *fps = int(header.fps);

    SimulationResult result;
    result.frames.resize(header.frames);
    result.diagnostics.resize(header.frames);
    for (uint32_t frame = 0; frame < header.frames; ++frame) {
        DiskDiagnostics diagnostics{};
        input.read(reinterpret_cast<char*>(&diagnostics), sizeof(diagnostics));
        result.diagnostics[frame] = fromDisk(diagnostics);
        result.frames[frame].resize(header.bodies);
        for (uint32_t body = 0; body < header.bodies; ++body) {
            DiskSnapshot snapshot{};
            input.read(reinterpret_cast<char*>(&snapshot), sizeof(snapshot));
            result.frames[frame][body] = fromDisk(snapshot);
        }
    }
    if (!input) throw std::runtime_error("truncated DICE1K cache");
    return result;
}

void writeMetrics(const std::filesystem::path& path,
                  const SimulationResult& result) {
    std::ofstream output(path);
    if (!output) throw std::runtime_error("cannot write metrics " + path.string());

    output << "frame,time,phase,global_magnetization,mean_magnetization,"
              "permanent_field_fraction,collapse_fraction,fraction_below,"
              "mean_speed,max_speed,mean_spin,max_spin,kinetic_energy,"
              "max_penetration,airborne_fraction,mean_nearest_neighbor,"
              "mean_dipole_alignment,magnetic_work,close_magnetic_pairs,"
              "snapped_pole_pairs,contact_manifolds,cluster_count,largest_cluster\n";

    for (size_t frame = 0; frame < result.diagnostics.size(); ++frame) {
        const FrameDiagnostics& d = result.diagnostics[frame];
        output << frame << ',' << std::setprecision(12)
               << d.time << ',' << phaseName(d.phase) << ','
               << d.globalMagnetization << ',' << d.meanMagnetization << ','
               << d.permanentFieldFraction << ',' << d.collapseFraction << ','
               << d.fractionBelow << ',' << d.meanSpeed << ',' << d.maxSpeed << ','
               << d.meanSpin << ',' << d.maxSpin << ',' << d.kineticEnergy << ','
               << d.maxPenetration << ',' << d.airborneFraction << ','
               << d.meanNearestNeighbor << ',' << d.meanDipoleAlignment << ','
               << d.magneticWork << ',' << d.closeMagneticPairs << ','
               << d.snappedPolePairs << ',' << d.contactManifolds << ','
               << d.clusterCount << ',' << d.largestCluster << '\n';
    }
}

void writeFieldSlice(const std::filesystem::path& path,
                     const std::vector<Snapshot>& frame,
                     int width = 560,
                     int height = 560) {
    std::vector<Vec3> field(size_t(width) * size_t(height), Vec3(0));
    const double xMin = -1.45;
    const double xMax = 1.45;
    const double zMin = -1.15;
    const double zMax = 1.15;
    const double planeY = 0.095;

    parallelFor(height, 0, [&](int py) {
        const double z = zMax - (zMax - zMin) * (py + 0.5) / height;
        for (int px = 0; px < width; ++px) {
            const double x = xMin + (xMax - xMin) * (px + 0.5) / width;
            const Vec3 point{x, planeY, z};
            Vec3 B(0);
            for (int i = 0; i < std::min<int>(kModuleCount, int(frame.size())); ++i) {
                const Snapshot& snapshot = frame[size_t(i)];
                const Vec3 moment = 0.052 * normalize(snapshot.q.rotate({1, 0, 0}));
                const Vec3 r = point - snapshot.p;
                const double r2 = lengthSquared(r) + 0.00034;
                const double invR3 = 1.0 / std::pow(r2, 1.5);
                B += (3.0 * r * (dot(moment, r) / r2) - moment) * invR3;
            }
            field[size_t(py) * size_t(width) + size_t(px)] = B;
        }
    });

    std::vector<double> magnitudes;
    magnitudes.reserve(field.size());
    for (const Vec3& B : field) magnitudes.push_back(length(B));
    std::nth_element(
        magnitudes.begin(),
        magnitudes.begin() + ptrdiff_t(magnitudes.size() * 995 / 1000),
        magnitudes.end());
    const double scale = std::max(1e-10, magnitudes[magnitudes.size() * 995 / 1000]);

    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("cannot write field image " + path.string());
    output << "P6\n" << width << ' ' << height << "\n255\n";
    for (const Vec3& B : field) {
        const double magnitude = saturate(std::log1p(18.0 * length(B) / scale) / std::log(19.0));
        const double angle = 0.5 + 0.5 * std::atan2(B.z, B.x) / PI;
        const Vec3 cold{0.025, 0.075, 0.14};
        const Vec3 cyan{0.10, 0.72, 0.88};
        const Vec3 warm{0.95, 0.47, 0.11};
        const Vec3 directionColor = mix(cyan, warm, angle);
        const Vec3 color = mix(cold, directionColor, std::pow(magnitude, 0.68)) * magnitude;
        const unsigned char pixel[3] = {
            uint8_t(clamp(std::round(color.x * 255.0), 0.0, 255.0)),
            uint8_t(clamp(std::round(color.y * 255.0), 0.0, 255.0)),
            uint8_t(clamp(std::round(color.z * 255.0), 0.0, 255.0))
        };
        output.write(reinterpret_cast<const char*>(pixel), 3);
    }
}

struct Args {
    std::string mode = "all";
    std::filesystem::path output = "output";
    std::filesystem::path cache;
    std::filesystem::path sourceCache;
    int resumeFrame = 94;
    int width = 540;
    int height = 960;
    int spp = 8;
    int depth = 7;
    int threads = 0;
    int frameStart = 0;
    int frameEnd = -1;
    int frameStep = 1;
    int frames = 240;
    int fps = 30;
    int heroWidth = 1080;
    int heroHeight = 1920;
    int heroSpp = 96;
    bool noDenoise = false;
    bool noDecorations = false;
    bool noMotionBlur = false;
};

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string option = argv[i];
        auto requireValue = [&]() {
            if (++i >= argc) throw std::runtime_error("missing value for " + option);
            return std::string(argv[i]);
        };

        if (option == "--mode") args.mode = requireValue();
        else if (option == "--output") args.output = requireValue();
        else if (option == "--cache") args.cache = requireValue();
        else if (option == "--source-cache") args.sourceCache = requireValue();
        else if (option == "--resume-frame") args.resumeFrame = std::stoi(requireValue());
        else if (option == "--width") args.width = std::stoi(requireValue());
        else if (option == "--height") args.height = std::stoi(requireValue());
        else if (option == "--spp") args.spp = std::stoi(requireValue());
        else if (option == "--depth") args.depth = std::stoi(requireValue());
        else if (option == "--threads") args.threads = std::stoi(requireValue());
        else if (option == "--frame-start") args.frameStart = std::stoi(requireValue());
        else if (option == "--frame-end") args.frameEnd = std::stoi(requireValue());
        else if (option == "--frame-step") args.frameStep = std::stoi(requireValue());
        else if (option == "--frames") args.frames = std::stoi(requireValue());
        else if (option == "--fps") args.fps = std::stoi(requireValue());
        else if (option == "--hero-width") args.heroWidth = std::stoi(requireValue());
        else if (option == "--hero-height") args.heroHeight = std::stoi(requireValue());
        else if (option == "--hero-spp") args.heroSpp = std::stoi(requireValue());
        else if (option == "--no-denoise") args.noDenoise = true;
        else if (option == "--no-decorations") args.noDecorations = true;
        else if (option == "--no-motion-blur") args.noMotionBlur = true;
        else if (option == "--help") {
            std::cout
                << "thousand_dice --mode sim|continue|render|hero|all [options]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown option " + option);
        }
    }

    if (args.cache.empty()) args.cache = args.output / "simulation.dice1k";
    return args;
}

int findFirstFrame(const SimulationResult& result, AssemblyPhase phase) {
    for (int frame = 0; frame < int(result.diagnostics.size()); ++frame) {
        if (result.diagnostics[size_t(frame)].phase == phase) return frame;
    }
    return int(result.frames.size()) - 1;
}

[[maybe_unused]] int findLastFrame(const SimulationResult& result, AssemblyPhase phase) {
    int found = -1;
    for (int frame = 0; frame < int(result.diagnostics.size()); ++frame) {
        if (result.diagnostics[size_t(frame)].phase == phase) found = frame;
    }
    return found >= 0 ? found : int(result.frames.size()) - 1;
}

void renderFrames(const Args& args, const SimulationResult& result) {
    const std::filesystem::path directory = args.output / "frames";
    std::filesystem::create_directories(directory);

    RenderConfig config;
    config.width = args.width;
    config.height = args.height;
    config.samplesPerPixel = args.spp;
    config.maxDepth = args.depth;
    config.threads = args.threads;
    config.denoise = !args.noDenoise;
    config.decorateModules = !args.noDecorations;
    config.motionBlur = !args.noMotionBlur;
    PathTracer tracer(config);

    const int finalFrame = args.frameEnd < 0
        ? int(result.frames.size()) - 1
        : std::min(args.frameEnd, int(result.frames.size()) - 1);

    for (int frame = std::max(0, args.frameStart);
         frame <= finalFrame;
         frame += std::max(1, args.frameStep)) {
        tracer.setScene(result.frames[size_t(frame)]);
        const Camera camera = cinematicCamera(
            frame,
            int(result.frames.size()),
            result.diagnostics[size_t(frame)]);
        const std::vector<Vec3> image = tracer.render(camera, frame);
        std::ostringstream filename;
        filename << "frame_" << std::setw(4) << std::setfill('0') << frame << ".ppm";
        tracer.writePPM(directory / filename.str(), image);
    }
}

void renderHeroes(const Args& args, const SimulationResult& result) {
    std::filesystem::create_directories(args.output);

    RenderConfig config;
    config.width = args.heroWidth;
    config.height = args.heroHeight;
    config.samplesPerPixel = args.heroSpp;
    config.maxDepth = 9;
    config.threads = args.threads;
    config.denoise = true;
    config.motionBlur = false;
    config.decorateModules = true;
    PathTracer tracer(config);

    int collapsePeak = int(result.frames.size()) / 2;
    double bestTargetError = INF;
    for (int frame = 0; frame < int(result.diagnostics.size()); ++frame) {
        const double error = std::abs(result.diagnostics[size_t(frame)].collapseFraction - 0.72);
        if (error < bestTargetError) {
            bestTargetError = error;
            collapsePeak = frame;
        }
    }

    const std::vector<std::pair<std::string, int>> shots{
        {"hero_initial.ppm", 0},
        {"hero_release.ppm", findFirstFrame(result, AssemblyPhase::FixtureRetract)},
        {"hero_collapse.ppm", collapsePeak},
        {"hero_final.ppm", int(result.frames.size()) - 1}
    };

    for (const auto& [filename, unclampedFrame] : shots) {
        const int frame = std::clamp(
            unclampedFrame,
            0,
            int(result.frames.size()) - 1);
        tracer.setScene(result.frames[size_t(frame)]);
        const Camera camera = cinematicCamera(
            frame,
            int(result.frames.size()),
            result.diagnostics[size_t(frame)],
            true);
        const std::vector<Vec3> image = tracer.render(camera, 10000 + frame);
        tracer.writePPM(args.output / filename, image);
    }
}

} // namespace
} // namespace tower

int main(int argc, char** argv) {
    using namespace tower;

    try {
        const Args args = parseArgs(argc, argv);
        std::filesystem::create_directories(args.output);
        SimulationResult result;
        int fps = args.fps;

        if (args.mode == "sim" || args.mode == "all") {
            SimulationConfig config;
            config.frames = args.frames;
            config.fps = args.fps;
            Simulator simulator(config);
            result = simulator.run();
            saveCache(args.cache, result, args.fps);
            writeMetrics(args.output / "simulation_metrics.csv", result);
        } else if (args.mode == "continue") {
            if (args.sourceCache.empty()) {
                throw std::runtime_error("--source-cache is required for --mode continue");
            }
            int sourceFps = args.fps;
            const SimulationResult source = loadCache(args.sourceCache, &sourceFps);
            const int resumeFrame = std::clamp(
                args.resumeFrame,
                0,
                int(source.frames.size()) - 1);

            SimulationConfig config;
            config.frames = args.frames;
            config.fps = sourceFps;
            config.preRollSeconds = 0.0;
            Simulator simulator(config);
            simulator.loadState(
                source.frames[size_t(resumeFrame)],
                AssemblyPhase::Settling);
            const SimulationResult tail = simulator.run();

            result.coils = source.coils;
            result.frames.reserve(size_t(resumeFrame + 1) + tail.frames.size());
            result.diagnostics.reserve(size_t(resumeFrame + 1) + tail.diagnostics.size());
            for (int frame = 0; frame <= resumeFrame; ++frame) {
                result.frames.push_back(source.frames[size_t(frame)]);
                result.diagnostics.push_back(source.diagnostics[size_t(frame)]);
            }
            const double collapseFloor =
                source.diagnostics[size_t(resumeFrame)].collapseFraction;
            for (size_t frame = 1; frame < tail.frames.size(); ++frame) {
                result.frames.push_back(tail.frames[frame]);
                FrameDiagnostics diagnostics = tail.diagnostics[frame];
                diagnostics.collapseFraction = std::max(
                    collapseFloor, diagnostics.collapseFraction);
                result.diagnostics.push_back(diagnostics);
            }
            for (size_t frame = 0; frame < result.diagnostics.size(); ++frame) {
                result.diagnostics[frame].time = double(frame) / double(sourceFps);
            }
            fps = sourceFps;
            saveCache(args.cache, result, sourceFps);
            writeMetrics(args.output / "simulation_metrics.csv", result);
        } else {
            result = loadCache(args.cache, &fps);
        }

        if (args.mode == "render" || args.mode == "all") {
            renderFrames(args, result);
        }
        if (args.mode == "hero" || args.mode == "all") {
            renderHeroes(args, result);
        }
        if (args.mode == "field") {
            int collapseFrame = int(result.frames.size()) / 2;
            double bestError = INF;
            for (int frame = 0; frame < int(result.diagnostics.size()); ++frame) {
                const double error = std::abs(
                    result.diagnostics[size_t(frame)].collapseFraction - 0.72);
                if (error < bestError) {
                    bestError = error;
                    collapseFrame = frame;
                }
            }
            writeFieldSlice(args.output / "field_initial.ppm", result.frames.front());
            writeFieldSlice(
                args.output / "field_collapse.ppm",
                result.frames[size_t(collapseFrame)]);
            writeFieldSlice(args.output / "field_final.ppm", result.frames.back());
        }

        const FrameDiagnostics& final = result.diagnostics.back();
        std::cout
            << "[final] dice=" << kModuleCount
            << " phase=" << phaseName(final.phase)
            << " collapse=" << final.collapseFraction
            << " mean_speed=" << final.meanSpeed
            << " mean_spin=" << final.meanSpin
            << " contact_cluster=" << final.largestCluster
            << " max_penetration=" << final.maxPenetration
            << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
