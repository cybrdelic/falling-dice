#include "render.hpp"
#include <cstring>

namespace meta {
namespace {

constexpr uint32_t kCacheVersion = 3;

struct CacheHeader {
    char magic[8]{};
    uint32_t version = kCacheVersion;
    uint32_t frames = 0;
    uint32_t bodies = 0;
    uint32_t fps = 24;
    uint32_t snapshotBytes = 0;
    uint32_t diagnosticsBytes = 0;
};

struct DiskSnapshot {
    double p[3], q[4], v[3], w[3], half[3];
    int32_t material, id, kind, moduleClass, ring, slot;
};

struct DiskDiagnostics {
    double time, platenY, displacement, engineeringStrain, engineeringStress, reactionForce;
    double loadingWork, recoveredWork, dissipatedWork, specificEnergyAbsorption;
    double elasticEnergy, plasticDissipation, contactEnergy, residualRms, residualMax, maxPenetration;
    double meanPlasticStrain, maxPlasticStrain, meanPlasticRotation, maxPlasticRotation, permanentSet;
    int32_t activeContacts, plasticBeams, plasticHinges, iterations;
    uint32_t converged, loading;
};

DiskSnapshot toDisk(const Snapshot& s) {
    return {{s.p.x,s.p.y,s.p.z}, {s.q.w,s.q.x,s.q.y,s.q.z},
            {s.v.x,s.v.y,s.v.z}, {s.w.x,s.w.y,s.w.z},
            {s.half.x,s.half.y,s.half.z},
            s.material,s.id,s.kind,s.moduleClass,s.ring,s.slot};
}
Snapshot fromDisk(const DiskSnapshot& d) {
    Snapshot s;
    s.p={d.p[0],d.p[1],d.p[2]}; s.q={d.q[0],d.q[1],d.q[2],d.q[3]};
    s.v={d.v[0],d.v[1],d.v[2]}; s.w={d.w[0],d.w[1],d.w[2]};
    s.half={d.half[0],d.half[1],d.half[2]};
    s.material=d.material; s.id=d.id; s.kind=d.kind;
    s.moduleClass=d.moduleClass; s.ring=d.ring; s.slot=d.slot;
    return s;
}
DiskDiagnostics toDisk(const FrameDiagnostics& d) {
    return {d.time,d.platenY,d.displacement,d.engineeringStrain,d.engineeringStress,d.reactionForce,
            d.loadingWork,d.recoveredWork,d.dissipatedWork,d.specificEnergyAbsorption,
            d.elasticEnergy,d.plasticDissipation,d.contactEnergy,d.residualRms,d.residualMax,d.maxPenetration,
            d.meanPlasticStrain,d.maxPlasticStrain,d.meanPlasticRotation,d.maxPlasticRotation,d.permanentSet,
            d.activeContacts,d.plasticBeams,d.plasticHinges,d.iterations,
            uint32_t(d.converged),uint32_t(d.loading)};
}
FrameDiagnostics fromDisk(const DiskDiagnostics& d) {
    FrameDiagnostics x;
    x.time=d.time; x.platenY=d.platenY; x.displacement=d.displacement;
    x.engineeringStrain=d.engineeringStrain; x.engineeringStress=d.engineeringStress;
    x.reactionForce=d.reactionForce; x.loadingWork=d.loadingWork; x.recoveredWork=d.recoveredWork;
    x.dissipatedWork=d.dissipatedWork; x.specificEnergyAbsorption=d.specificEnergyAbsorption;
    x.elasticEnergy=d.elasticEnergy; x.plasticDissipation=d.plasticDissipation;
    x.contactEnergy=d.contactEnergy; x.residualRms=d.residualRms; x.residualMax=d.residualMax;
    x.maxPenetration=d.maxPenetration; x.meanPlasticStrain=d.meanPlasticStrain;
    x.maxPlasticStrain=d.maxPlasticStrain; x.meanPlasticRotation=d.meanPlasticRotation;
    x.maxPlasticRotation=d.maxPlasticRotation; x.permanentSet=d.permanentSet;
    x.activeContacts=d.activeContacts; x.plasticBeams=d.plasticBeams;
    x.plasticHinges=d.plasticHinges; x.iterations=d.iterations;
    x.converged=d.converged!=0; x.loading=d.loading!=0;
    return x;
}

void saveCache(const std::filesystem::path& path,
               const SimulationResult& result,
               int fps) {
    if (result.frames.empty()) throw std::runtime_error("cannot cache empty simulation");
    const size_t bodies = result.frames.front().size();
    for (const auto& frame : result.frames)
        if (frame.size() != bodies) throw std::runtime_error("cache requires stable body count");
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path,std::ios::binary);
    if(!out) throw std::runtime_error("cannot write cache " + path.string());
    CacheHeader h;
    std::memcpy(h.magic,"META3",5);
    h.frames=uint32_t(result.frames.size()); h.bodies=uint32_t(bodies); h.fps=uint32_t(fps);
    h.snapshotBytes=sizeof(DiskSnapshot); h.diagnosticsBytes=sizeof(DiskDiagnostics);
    out.write(reinterpret_cast<const char*>(&h),sizeof(h));
    for(size_t f=0;f<result.frames.size();++f){
        DiskDiagnostics d=toDisk(result.diagnostics[f]);
        out.write(reinterpret_cast<const char*>(&d),sizeof(d));
        for(const Snapshot&s:result.frames[f]){DiskSnapshot q=toDisk(s);out.write(reinterpret_cast<const char*>(&q),sizeof(q));}
    }
}

SimulationResult loadCache(const std::filesystem::path& path,int* fps=nullptr){
    std::ifstream in(path,std::ios::binary);if(!in)throw std::runtime_error("cannot read cache "+path.string());
    CacheHeader h{};in.read(reinterpret_cast<char*>(&h),sizeof(h));
    if(std::strncmp(h.magic,"META3",5)!=0||h.version!=kCacheVersion||
       h.snapshotBytes!=sizeof(DiskSnapshot)||h.diagnosticsBytes!=sizeof(DiskDiagnostics))
        throw std::runtime_error("unsupported or corrupt META3 cache");
    if(fps)*fps=int(h.fps);
    SimulationResult result;result.frames.resize(h.frames);result.diagnostics.resize(h.frames);
    for(uint32_t f=0;f<h.frames;++f){
        DiskDiagnostics d{};in.read(reinterpret_cast<char*>(&d),sizeof(d));result.diagnostics[f]=fromDisk(d);
        result.frames[f].resize(h.bodies);
        for(Snapshot&s:result.frames[f]){DiskSnapshot q{};in.read(reinterpret_cast<char*>(&q),sizeof(q));s=fromDisk(q);}
    }
    if(!in)throw std::runtime_error("truncated META3 cache");
    return result;
}

void writeMetrics(const std::filesystem::path& path,const SimulationResult& result){
    std::ofstream out(path);if(!out)throw std::runtime_error("cannot write metrics "+path.string());
    out << "frame,time,loading,platen_y_m,displacement_m,engineering_strain,engineering_stress_pa,"
           "reaction_force_n,loading_work_j,recovered_work_j,dissipated_work_j,sea_j_per_kg,"
           "elastic_energy_j,incremental_plastic_dissipation_j,contact_energy_j,residual_rms,residual_max,"
           "max_penetration_m,mean_plastic_strain,max_plastic_strain,mean_plastic_rotation_rad,"
           "max_plastic_rotation_rad,permanent_set,active_contacts,plastic_beams,plastic_hinges,iterations,converged\n";
    for(size_t f=0;f<result.diagnostics.size();++f){const auto&d=result.diagnostics[f];
        out<<f<<','<<std::setprecision(14)<<d.time<<','<<(d.loading?1:0)<<','<<d.platenY<<','<<d.displacement<<','
           <<d.engineeringStrain<<','<<d.engineeringStress<<','<<d.reactionForce<<','<<d.loadingWork<<','
           <<d.recoveredWork<<','<<d.dissipatedWork<<','<<d.specificEnergyAbsorption<<','<<d.elasticEnergy<<','
           <<d.plasticDissipation<<','<<d.contactEnergy<<','<<d.residualRms<<','<<d.residualMax<<','
           <<d.maxPenetration<<','<<d.meanPlasticStrain<<','<<d.maxPlasticStrain<<','<<d.meanPlasticRotation<<','
           <<d.maxPlasticRotation<<','<<d.permanentSet<<','<<d.activeContacts<<','<<d.plasticBeams<<','
           <<d.plasticHinges<<','<<d.iterations<<','<<(d.converged?1:0)<<'\n';
    }
}

struct Summary {
    double peakForce=0,peakStress=0,loadingWork=0,recoveredWork=0,dissipatedWork=0,sea=0;
    double maxPen=0,permanentSet=0,maxPlasticStrain=0,maxPlasticRotation=0;
    int unconverged=0,frames=0,peakForceFrame=0;
};
Summary summarize(const SimulationResult&r){Summary s;s.frames=int(r.diagnostics.size());for(int i=0;i<int(r.diagnostics.size());++i){const auto&d=r.diagnostics[size_t(i)];if(d.reactionForce>s.peakForce){s.peakForce=d.reactionForce;s.peakForceFrame=i;}s.peakStress=std::max(s.peakStress,d.engineeringStress);s.maxPen=std::max(s.maxPen,d.maxPenetration);s.maxPlasticStrain=std::max(s.maxPlasticStrain,d.maxPlasticStrain);s.maxPlasticRotation=std::max(s.maxPlasticRotation,d.maxPlasticRotation);if(!d.converged)++s.unconverged;}if(!r.diagnostics.empty()){const auto&f=r.diagnostics.back();s.loadingWork=f.loadingWork;s.recoveredWork=f.recoveredWork;s.dissipatedWork=f.dissipatedWork;s.sea=f.specificEnergyAbsorption;s.permanentSet=f.permanentSet;}return s;}

void writeSummary(const std::filesystem::path& path,const SimulationResult&r,const SimulationConfig&cfg,double massKg){
    Summary s=summarize(r);std::ofstream out(path);if(!out)throw std::runtime_error("cannot write summary");
    out<<"{\n"
       <<"  \"solver\": \"incremental quasi-static 6-DOF corotational beam/contact energy minimization\",\n"
       <<"  \"material\": \""<<cfg.material.name<<"\",\n"
       <<"  \"cells\": ["<<cfg.cellsX<<", "<<cfg.cellsY<<", "<<cfg.cellsZ<<"],\n"
       <<"  \"frames\": "<<s.frames<<",\n"
       <<"  \"mass_kg\": "<<std::setprecision(12)<<massKg<<",\n"
       <<"  \"peak_force_n\": "<<s.peakForce<<",\n"
       <<"  \"peak_stress_pa\": "<<s.peakStress<<",\n"
       <<"  \"loading_work_j\": "<<s.loadingWork<<",\n"
       <<"  \"recovered_work_j\": "<<s.recoveredWork<<",\n"
       <<"  \"dissipated_work_j\": "<<s.dissipatedWork<<",\n"
       <<"  \"specific_energy_absorption_j_per_kg\": "<<s.sea<<",\n"
       <<"  \"permanent_set\": "<<s.permanentSet<<",\n"
       <<"  \"max_penetration_m\": "<<s.maxPen<<",\n"
       <<"  \"unconverged_steps\": "<<s.unconverged<<"\n"
       <<"}\n";
}

struct Args{
    std::string mode="all";
    std::filesystem::path output="output",cache;
    int width=480,height=640,spp=4,depth=7,threads=0,start=0,end=-1,step=1,subframes=1;
    int heroWidth=768,heroHeight=768,heroSpp=48,stlQuality=14;
    bool noDenoise=false,quick=false,noUnload=false,skipStl=false;
    SimulationConfig sim;
};
Args parseArgs(int argc,char**argv){Args a;for(int n=1;n<argc;++n){std::string o=argv[n];auto value=[&](){if(++n>=argc)throw std::runtime_error("missing value for "+o);return std::string(argv[n]);};
    if(o=="--mode")a.mode=value();else if(o=="--output")a.output=value();else if(o=="--cache")a.cache=value();
    else if(o=="--width")a.width=std::stoi(value());else if(o=="--height")a.height=std::stoi(value());else if(o=="--spp")a.spp=std::stoi(value());else if(o=="--depth")a.depth=std::stoi(value());else if(o=="--threads")a.threads=std::stoi(value());
    else if(o=="--frame-start")a.start=std::stoi(value());else if(o=="--frame-end")a.end=std::stoi(value());else if(o=="--frame-step")a.step=std::stoi(value());else if(o=="--subframes")a.subframes=std::max(1,std::stoi(value()));
    else if(o=="--hero-width")a.heroWidth=std::stoi(value());else if(o=="--hero-height")a.heroHeight=std::stoi(value());else if(o=="--hero-spp")a.heroSpp=std::stoi(value());else if(o=="--stl-quality")a.stlQuality=std::stoi(value());
    else if(o=="--cells-x")a.sim.cellsX=std::stoi(value());else if(o=="--cells-y")a.sim.cellsY=std::stoi(value());else if(o=="--cells-z")a.sim.cellsZ=std::stoi(value());
    else if(o=="--load-steps")a.sim.loadSteps=std::stoi(value());else if(o=="--unload-steps")a.sim.unloadSteps=std::stoi(value());else if(o=="--max-strain")a.sim.maxStrain=std::stod(value());else if(o=="--max-iterations")a.sim.maxIterations=std::stoi(value());
    else if(o=="--young")a.sim.material.young=std::stod(value());else if(o=="--yield")a.sim.material.yield=std::stod(value());else if(o=="--hardening")a.sim.material.hardening=std::stod(value());else if(o=="--contact-stiffness")a.sim.contactStiffness=std::stod(value());
    else if(o=="--quick")a.quick=true;else if(o=="--no-unload")a.noUnload=true;else if(o=="--no-denoise")a.noDenoise=true;else if(o=="--skip-stl")a.skipStl=true;
    else if(o=="--help"){std::cout<<"metamaterial_compression --mode sim|render|hero|stl|validate|convergence|all [options]\n";std::exit(0);}else throw std::runtime_error("unknown option "+o);
  }
  if(a.quick){a.sim.cellsX=3;a.sim.cellsY=4;a.sim.cellsZ=3;a.sim.loadSteps=24;a.sim.unloadSteps=14;a.sim.maxIterations=95;a.width=360;a.height=480;a.spp=2;a.heroSpp=16;a.stlQuality=10;}
  if(a.noUnload)a.sim.unloadSteps=0;
  a.sim.threads=a.threads;a.sim.frames=a.sim.loadSteps+a.sim.unloadSteps+1;
  if(a.cache.empty())a.cache=a.output/"compression.meta3";
  return a;
}

FrameDiagnostics interpolateDiagnostics(const FrameDiagnostics&a,const FrameDiagnostics&b,double t){FrameDiagnostics d=a;auto L=[&](double x,double y){return lerp(x,y,t);};d.time=L(a.time,b.time);d.platenY=L(a.platenY,b.platenY);d.displacement=L(a.displacement,b.displacement);d.engineeringStrain=L(a.engineeringStrain,b.engineeringStrain);d.engineeringStress=L(a.engineeringStress,b.engineeringStress);d.reactionForce=L(a.reactionForce,b.reactionForce);d.loadingWork=L(a.loadingWork,b.loadingWork);d.recoveredWork=L(a.recoveredWork,b.recoveredWork);d.dissipatedWork=L(a.dissipatedWork,b.dissipatedWork);d.specificEnergyAbsorption=L(a.specificEnergyAbsorption,b.specificEnergyAbsorption);d.elasticEnergy=L(a.elasticEnergy,b.elasticEnergy);d.plasticDissipation=L(a.plasticDissipation,b.plasticDissipation);d.contactEnergy=L(a.contactEnergy,b.contactEnergy);d.residualRms=L(a.residualRms,b.residualRms);d.residualMax=L(a.residualMax,b.residualMax);d.maxPenetration=L(a.maxPenetration,b.maxPenetration);d.meanPlasticStrain=L(a.meanPlasticStrain,b.meanPlasticStrain);d.maxPlasticStrain=L(a.maxPlasticStrain,b.maxPlasticStrain);d.meanPlasticRotation=L(a.meanPlasticRotation,b.meanPlasticRotation);d.maxPlasticRotation=L(a.maxPlasticRotation,b.maxPlasticRotation);d.permanentSet=L(a.permanentSet,b.permanentSet);d.activeContacts=int(std::lround(L(a.activeContacts,b.activeContacts)));d.plasticBeams=int(std::lround(L(a.plasticBeams,b.plasticBeams)));d.plasticHinges=int(std::lround(L(a.plasticHinges,b.plasticHinges)));d.iterations=int(std::lround(L(a.iterations,b.iterations)));d.loading=t<.5?a.loading:b.loading;d.converged=a.converged&&b.converged;return d;}
std::vector<Snapshot> interpolateFrame(const std::vector<Snapshot>&a,const std::vector<Snapshot>&b,double t){if(a.size()!=b.size())throw std::runtime_error("interpolation body count mismatch");std::vector<Snapshot>r(a.size());for(size_t i=0;i<a.size();++i){r[i]=a[i];r[i].p=mix(a[i].p,b[i].p,t);r[i].q=slerp(a[i].q,b[i].q,t);r[i].v=mix(a[i].v,b[i].v,t);r[i].w=mix(a[i].w,b[i].w,t);r[i].half=mix(a[i].half,b[i].half,t);if(t>=.5)r[i].material=b[i].material;}return r;}
void renderFrames(const Args&a,const SimulationResult&r){
    std::filesystem::create_directories(a.output/"frames");RenderConfig c;c.width=a.width;c.height=a.height;c.samplesPerPixel=a.spp;c.maxDepth=a.depth;c.threads=a.threads;c.denoise=!a.noDenoise;c.motionBlur=false;PathTracer pt(c);
    int first=std::max(0,a.start),last=a.end<0?int(r.frames.size())-1:std::min(a.end,int(r.frames.size())-1),outFrame=first*a.subframes;
    for(int f=first;f<=last;f+=std::max(1,a.step)){int next=std::min(f+std::max(1,a.step),last);int count=(f==last)?1:a.subframes;for(int s=0;s<count;++s){double t=count==1?0.0:double(s)/count;std::vector<Snapshot>scene=t==0?r.frames[size_t(f)]:interpolateFrame(r.frames[size_t(f)],r.frames[size_t(next)],t);FrameDiagnostics diag=t==0?r.diagnostics[size_t(f)]:interpolateDiagnostics(r.diagnostics[size_t(f)],r.diagnostics[size_t(next)],t);pt.setScene(scene);auto image=pt.render(compressionCamera(outFrame,(last-first)*a.subframes+1,diag),outFrame);std::ostringstream name;name<<"frame_"<<std::setw(4)<<std::setfill('0')<<outFrame++<<".ppm";pt.writePPM(a.output/"frames"/name.str(),image);}}
}
void renderHeroes(const Args&a,const SimulationResult&r){
    RenderConfig c;c.width=a.heroWidth;c.height=a.heroHeight;c.samplesPerPixel=a.heroSpp;c.maxDepth=9;c.threads=a.threads;c.denoise=true;c.motionBlur=false;PathTracer pt(c);
    int peak=0;double maxF=-1;for(int i=0;i<int(r.diagnostics.size());++i)if(r.diagnostics[size_t(i)].loading&&r.diagnostics[size_t(i)].reactionForce>maxF){maxF=r.diagnostics[size_t(i)].reactionForce;peak=i;}
    int dens=0;double target=.82*maxF,best=INF;for(int i=peak;i<int(r.diagnostics.size());++i)if(r.diagnostics[size_t(i)].loading){double e=std::abs(r.diagnostics[size_t(i)].reactionForce-target);if(e<best){best=e;dens=i;}}
    int unload=int(r.frames.size())-1;std::array<int,5>frames{0,std::max(1,peak/2),peak,dens,unload};std::array<const char*,5>names{"hero_initial.ppm","hero_buckling.ppm","hero_peak.ppm","hero_densification.ppm","hero_unloaded.ppm"};
    for(int j=0;j<5;++j){int f=std::clamp(frames[j],0,int(r.frames.size())-1);pt.setScene(r.frames[size_t(f)]);auto image=pt.render(compressionCamera(f,int(r.frames.size()),r.diagnostics[size_t(f)],true),10000+f);pt.writePPM(a.output/names[j],image);}
}

bool finiteResult(const SimulationResult&r){for(const auto&d:r.diagnostics){const double values[]={d.time,d.platenY,d.displacement,d.engineeringStrain,d.engineeringStress,d.reactionForce,d.loadingWork,d.recoveredWork,d.dissipatedWork,d.specificEnergyAbsorption,d.elasticEnergy,d.plasticDissipation,d.contactEnergy,d.residualRms,d.residualMax,d.maxPenetration,d.meanPlasticStrain,d.maxPlasticStrain,d.meanPlasticRotation,d.maxPlasticRotation,d.permanentSet};for(double v:values)if(!std::isfinite(v))return false;}for(const auto&f:r.frames)for(const auto&s:f)if(!finite(s.p)||!finite(s.v)||!finite(s.w)||!std::isfinite(s.q.w)||!std::isfinite(s.q.x)||!std::isfinite(s.q.y)||!std::isfinite(s.q.z))return false;return true;}

void writeValidation(const std::filesystem::path&path,const Simulator&sim,const SimulationResult*result){
    double objectivity=INF,gradient=INF;bool obj=sim.rigidRotationTest(&objectivity),grad=sim.gradientCheck(&gradient);std::ofstream out(path);if(!out)throw std::runtime_error("cannot write validation report");
    out<<"Metamaterial compression engineering validation\n"
       <<"===============================================\n"
       <<"rigid_motion_objectivity_error="<<std::setprecision(12)<<objectivity<<" "<<(obj?"PASS":"FAIL")<<'\n'
       <<"finite_difference_gradient_relative_error="<<gradient<<" "<<(grad?"PASS":"FAIL")<<'\n'
       <<"nodes="<<sim.nodes().size()<<'\n'<<"beams="<<sim.beams().size()<<'\n'<<"mass_kg="<<sim.mass()<<'\n'<<"reference_area_m2="<<sim.area()<<'\n';
    if(result){Summary s=summarize(*result);bool finite=finiteResult(*result);double convergedFraction=result->diagnostics.empty()?0:1.0-double(s.unconverged)/result->diagnostics.size();bool penetration=s.maxPen<0.00025;bool energy=s.loadingWork+1e-10>=s.recoveredWork&&s.dissipatedWork>=-1e-10;out<<"finite_state="<<(finite?"PASS":"FAIL")<<'\n'<<"converged_step_fraction="<<convergedFraction<<'\n'<<"peak_force_n="<<s.peakForce<<'\n'<<"loading_work_j="<<s.loadingWork<<'\n'<<"recovered_work_j="<<s.recoveredWork<<'\n'<<"dissipated_work_j="<<s.dissipatedWork<<'\n'<<"specific_energy_absorption_j_per_kg="<<s.sea<<'\n'<<"permanent_set="<<s.permanentSet<<'\n'<<"max_contact_penetration_mm="<<s.maxPen*1000<<" "<<(penetration?"PASS":"WARN")<<'\n'<<"energy_accounting="<<(energy?"PASS":"FAIL")<<'\n'<<"overall_numerical_validation="<<((obj&&grad&&finite&&energy)?"PASS":"FAIL")<<'\n';}
}

void runConvergence(const Args&args){
    std::filesystem::create_directories(args.output);
    struct Case{const char*name;int steps,iterations;};
    std::array<Case,3>cases{{{"coarse",10,500},{"medium",14,500},{"fine",20,600}}};
    std::ofstream csv(args.output/"convergence.csv");
    csv<<"case,load_steps,unload_steps,max_iterations,peak_force_n,loading_work_j,dissipated_work_j,permanent_set,max_penetration_m,unconverged_steps\n";
    for(const Case&k:cases){
        SimulationConfig c=args.sim;
        c.cellsX=2;c.cellsY=4;c.cellsZ=2;c.maxStrain=.50;
        c.loadSteps=k.steps;c.unloadSteps=std::max(7,int(std::lround(.70*k.steps)));
        c.maxIterations=k.iterations;c.contactStiffness=1.0e6;
        c.frames=c.loadSteps+c.unloadSteps+1;
        std::cout<<"\n[convergence] running "<<k.name<<"\n";
        Simulator sim(c);SimulationResult r=sim.run();Summary s=summarize(r);
        csv<<k.name<<','<<k.steps<<','<<c.unloadSteps<<','<<k.iterations<<','<<std::setprecision(12)
           <<s.peakForce<<','<<s.loadingWork<<','<<s.dissipatedWork<<','<<s.permanentSet<<','<<s.maxPen<<','<<s.unconverged<<'\n';
        saveCache(args.output/(std::string("convergence_")+k.name+".meta3"),r,c.fps);
    }
}

} // namespace
} // namespace meta

int main(int argc,char**argv){using namespace meta;try{Args args=parseArgs(argc,argv);std::filesystem::create_directories(args.output);if(args.mode=="convergence"){runConvergence(args);return 0;}SimulationResult result;int fps=args.sim.fps;std::unique_ptr<Simulator>sim;
    if(args.mode=="sim"||args.mode=="all"){sim=std::make_unique<Simulator>(args.sim);result=sim->run();saveCache(args.cache,result,args.sim.fps);writeMetrics(args.output/"compression_metrics.csv",result);writeSummary(args.output/"summary.json",result,args.sim,sim->mass());if(!args.skipStl)sim->exportSTL(args.output/"graded_bcc_energy_absorber.stl",args.stlQuality);writeValidation(args.output/"VALIDATION.txt",*sim,&result);}
    else if(args.mode=="validate"||args.mode=="stl"){sim=std::make_unique<Simulator>(args.sim);if(args.mode=="stl")sim->exportSTL(args.output/"graded_bcc_energy_absorber.stl",args.stlQuality);writeValidation(args.output/"VALIDATION.txt",*sim,nullptr);if(args.mode=="validate")return 0;}
    else result=loadCache(args.cache,&fps);
    if(args.mode=="render"||args.mode=="all")renderFrames(args,result);
    if(args.mode=="hero"||args.mode=="all")renderHeroes(args,result);
    if(!result.diagnostics.empty()){Summary s=summarize(result);std::cout<<"[final] peak_force="<<s.peakForce<<" N loading_work="<<s.loadingWork<<" J recovered="<<s.recoveredWork<<" J dissipated="<<s.dissipatedWork<<" J SEA="<<s.sea<<" J/kg permanent_set="<<s.permanentSet<<" max_penetration="<<s.maxPen*1000<<" mm unconverged="<<s.unconverged<<'\n';}
    return 0;}catch(const std::exception&e){std::cerr<<"error: "<<e.what()<<'\n';return 1;}}
