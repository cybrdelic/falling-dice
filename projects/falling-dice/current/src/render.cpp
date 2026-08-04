#include "render.hpp"

namespace tower {
namespace {
struct Ray{Vec3 o,d;double tmin=1e-4,tmax=INF,time=.5,coneWidth=0.0;};
struct AABB{
    Vec3 lo{INF,INF,INF},hi{-INF,-INF,-INF};
    void add(const Vec3&p){lo=minVec(lo,p);hi=maxVec(hi,p);}void add(const AABB&b){lo=minVec(lo,b.lo);hi=maxVec(hi,b.hi);}Vec3 center()const{return .5*(lo+hi);}Vec3 extent()const{return hi-lo;}
    bool hitNear(const Ray&r,double mx,double&nearT)const{double a=r.tmin,b=std::min(r.tmax,mx);for(int k=0;k<3;k++){if(std::abs(r.d[k])<1e-15){if(r.o[k]<lo[k]||r.o[k]>hi[k])return false;continue;}double inv=1/r.d[k],n=(lo[k]-r.o[k])*inv,f=(hi[k]-r.o[k])*inv;if(n>f)std::swap(n,f);a=std::max(a,n);b=std::min(b,f);if(b<a)return false;}nearT=a;return true;}
};
struct Material{
    Vec3 base{.6,.6,.6},eta{.25,.3,.4},k{3.5,3.1,2.7};double roughX=.22,roughY=.34,metal=1,specProbability=.98;bool enamel=false;
};
struct Instance{
    Vec3 p0,p1,half;Quat q0,q1;int material=0,id=0,parent=0,moduleClass=0;bool module=false;Vec3 brushAxisLocal{1,0,0};
    Vec3 p(double t)const{return mix(p0,p1,t);}Quat q(double t)const{return slerp(q0,q1,t);}
};
struct Hit{double t=INF,footprint=0.0;Vec3 p,n,lp,ln,tangent,bitangent;int instance=-1,material=-1;bool floor=false;};
struct BVHNode{AABB b;int left=-1,right=-1,start=0,count=0;bool leaf()const{return count>0;}};
struct AreaLight{Vec3 center,u,v,emission;Vec3 normal()const{return normalize(cross(u,v));}double area()const{return 4*length(cross(u,v));}};
struct Feature{Vec3 n{0,0,0},albedo{0,0,0};double depth=0,hits=0;int instance=-2;};
struct Basis{Vec3 t,b,n;};
Basis makeBasis(const Vec3&n,const Vec3&preferred=Vec3{0,0,0}){
    Vec3 t=preferred-dot(preferred,n)*n;if(lengthSquared(t)<1e-10)t=orthogonal(n);else t=normalize(t);return{t,normalize(cross(n,t)),n};
}
Vec3 toWorld(const Basis&B,const Vec3&v){return B.t*v.x+B.b*v.y+B.n*v.z;}
Vec3 toLocal(const Basis&B,const Vec3&v){return{dot(v,B.t),dot(v,B.b),dot(v,B.n)};}

double sdfRoundedBox(const Vec3&p,const Vec3&core,double radius){Vec3 q=absVec(p)-core,out=maxVec(q,0.0);return length(out)+std::min(maxComponent(q),0.0)-radius;}
Vec3 roundedBoxNormal(const Vec3&p,const Vec3&core){Vec3 c=clampVec(p,-core,core),n=p-c;if(lengthSquared(n)>1e-16)return normalize(n);Vec3 d=absVec(absVec(p)-core);if(d.x<=d.y&&d.x<=d.z)return{p.x>=0?1.0:-1.0,0,0};if(d.y<=d.z)return{0,p.y>=0?1.0:-1.0,0};return{0,0,p.z>=0?1.0:-1.0};}

inline double scalarHash(int x,int y,int z){
    const uint64_t h=splitmix64(uint64_t(uint32_t(x))*0x9e3779b97f4a7c15ULL^
                                uint64_t(uint32_t(y))*0xbf58476d1ce4e5b9ULL^
                                uint64_t(uint32_t(z))*0x94d049bb133111ebULL);
    return double(uint32_t(h>>32))/4294967295.0;
}
inline double valueNoise(const Vec3&p){
    const int ix=int(std::floor(p.x)),iy=int(std::floor(p.y)),iz=int(std::floor(p.z));
    const double fx=p.x-ix,fy=p.y-iy,fz=p.z-iz;
    const double ux=fx*fx*(3-2*fx),uy=fy*fy*(3-2*fy),uz=fz*fz*(3-2*fz);
    auto h=[&](int dx,int dy,int dz){return scalarHash(ix+dx,iy+dy,iz+dz);};
    const double x00=mix(h(0,0,0),h(1,0,0),ux),x10=mix(h(0,1,0),h(1,1,0),ux);
    const double x01=mix(h(0,0,1),h(1,0,1),ux),x11=mix(h(0,1,1),h(1,1,1),ux);
    return mix(mix(x00,x10,uy),mix(x01,x11,uy),uz);
}
inline double fbm(Vec3 p){double a=.5,sum=0,norm=0;for(int i=0;i<5;++i){sum+=a*valueNoise(p);norm+=a;p=p*2.03+Vec3{17.1,9.7,13.4};a*=.5;}return sum/std::max(1e-9,norm);}
inline double gaussianLine(double x,double width){const double q=x/std::max(1e-9,width);return std::exp(-q*q);}
}

struct PathTracer::Impl{
    RenderConfig c;std::array<Material,24>materials{};Material floorMaterial;std::vector<AreaLight>lights;std::vector<Instance>instances;std::vector<int>indices;std::vector<AABB>instanceBounds;std::vector<Vec3>centroids;std::vector<BVHNode>nodes;int root=-1;

    explicit Impl(RenderConfig cfg):c(cfg){
        // Injection-molded urea dice: one coherent ivory production lot with
        // restrained, physically plausible batch variation. The previous
        // palette was too patchy and made the tower read as textured foam.
        materials[0]={{.820,.805,.758},{1.50,1.50,1.50},{0,0,0},.175,.190,0,.38,true};
        materials[1]={{.855,.845,.815},{1.50,1.50,1.50},{0,0,0},.165,.180,0,.40,true};
        materials[2]={{.782,.756,.700},{1.50,1.50,1.50},{0,0,0},.190,.205,0,.35,true};
        materials[3]={{.008,.009,.011},{1.50,1.50,1.50},{0,0,0},.255,.285,0,.20,true}; // pip inlay
        materials[4]={{.31,.025,.020},{1.50,1.50,1.50},{0,0,0},.24,.28,0,.24,true};

        // Collision-matched metrology-rig materials and a restrained workshop.
        materials[5]={{.34,.37,.41},{1.22,1.14,1.08},{3.35,3.55,3.78},.145,.31,1,.99,false}; // brushed stainless leaf
        materials[6]={{.245,.255,.270},{1.50,1.50,1.50},{0,0,0},.76,.80,0,.06,false}; // concrete wall
        materials[7]={{.018,.021,.026},{1.50,1.50,1.50},{0,0,0},.29,.34,0,.16,false}; // black powder coat
        materials[8]={{.030,.033,.038},{1.50,1.50,1.50},{0,0,0},.48,.54,0,.10,false}; // graphite work surface
        materials[9]={{.56,.37,.10},{1.50,1.50,1.50},{0,0,0},.31,.36,0,.19,false};
        materials[10]={{.014,.016,.020},{1.50,1.50,1.50},{0,0,0},.65,.72,0,.06,false}; // rubber
        materials[11]={{.69,.72,.76},{1.00,.94,.88},{3.75,3.98,4.20},.105,.27,1,.995,false}; // polished steel
        materials[12]={{.085,.095,.108},{1.20,1.12,1.05},{2.95,3.12,3.32},.22,.38,1,.95,false}; // dark steel frame
        materials[13]={{.16,.17,.19},{1.50,1.50,1.50},{0,0,0},.58,.63,0,.08,false};
        materials[14]={{.79,.80,.82},{1.50,1.50,1.50},{0,0,0},.32,.35,0,.20,false};
        materials[15]={{.055,.105,.135},{1.50,1.50,1.50},{0,0,0},.32,.38,0,.18,false};
        materials[16]={{.42,.24,.10},{1.50,1.50,1.50},{0,0,0},.45,.51,0,.13,false};
        materials[17]={{.026,.030,.037},{1.50,1.50,1.50},{0,0,0},.51,.57,0,.09,false};
        materials[18]=materials[0]; materials[19]=materials[8]; materials[20]=materials[11];
        materials[21]=materials[6]; materials[22]=materials[7]; materials[23]=materials[10];

        floorMaterial=materials[8];

        // Large photographic sources. They create continuous highlight bands
        // across the ivory resin rather than isolated sparkling point samples.
        lights.push_back({{-.58,.68,.40},{0,.34,0},{0,0,.46},{5.0,5.25,5.55}}); // cool-left key
        lights.push_back({{.16,.82,-.14},{.54,0,0},{0,0,.28},{1.55,1.62,1.75}}); // ceiling fill
        lights.push_back({{.52,.43,-.48},{.18,0,0},{0,.18,0},{2.25,1.55,.95}});  // warm rear rim
        lights.push_back({{-.34,.19,.55},{.17,0,0},{0,.10,.10},{.52,.61,.74}}); // frontal fill
    }

    void addStaticBox(const Vec3& p,
                      const Quat& q,
                      const Vec3& half,
                      int material,
                      int id,
                      bool module=false) {
        Instance i;
        i.p0=i.p1=p;
        i.q0=i.q1=q;
        i.half=half;
        i.material=material;
        i.id=id;
        i.parent=id;
        i.module=module;
        i.moduleClass=0;
        i.brushAxisLocal={1,0,0};
        instances.push_back(i);
    }

    // Conservative all-frame dice envelope measured from the deterministic
    // cache, expanded by 15 mm. Render-only solids are admitted only when
    // their world AABB lies completely outside this envelope. This turns the
    // former clipping fix into an invariant rather than a hand-placed guess.
    bool overlapsDiceSweptEnvelope(const Vec3& p, const Quat& q,
                                   const Vec3& half) const {
        const Vec3 ax=q.rotate({1,0,0}), ay=q.rotate({0,1,0}), az=q.rotate({0,0,1});
        const Vec3 e{std::abs(ax.x)*half.x+std::abs(ay.x)*half.y+std::abs(az.x)*half.z,
                     std::abs(ax.y)*half.x+std::abs(ay.y)*half.y+std::abs(az.y)*half.z,
                     std::abs(ax.z)*half.x+std::abs(ay.z)*half.y+std::abs(az.z)*half.z};
        const Vec3 lo=p-e, hi=p+e;
        const Vec3 sweptLo{-0.340,-0.018,-0.345};
        const Vec3 sweptHi{ 0.297, 0.407, 0.276};
        return lo.x<=sweptHi.x && hi.x>=sweptLo.x &&
               lo.y<=sweptHi.y && hi.y>=sweptLo.y &&
               lo.z<=sweptHi.z && hi.z>=sweptLo.z;
    }

    void addSafeStaticBox(const Vec3& p, const Quat& q, const Vec3& half,
                          int material, int id) {
        if(overlapsDiceSweptEnvelope(p,q,half))
            throw std::runtime_error("render-only prop enters dice swept envelope: id="+
                                     std::to_string(id));
        addStaticBox(p,q,half,material,id,false);
    }

    void addBeam(const Vec3& a,
                 const Vec3& b,
                 double radius,
                 int material,
                 int id) {
        const Vec3 axis=normalize(b-a);
        const double lengthValue=length(b-a);
        Vec3 up=std::abs(axis.y)<.90?Vec3{0,1,0}:Vec3{0,0,1};
        Vec3 z=normalize(cross(axis,up));
        Vec3 y=normalize(cross(z,axis));
        addStaticBox(.5*(a+b),quatFromAxes(axis,y,z),
                     {0.5*lengthValue,radius,radius},material,id,false);
    }

    AABB boundsAt(const Instance&i,double t)const{
        Quat q=i.q(t);Vec3 x=q.rotate({1,0,0}),y=q.rotate({0,1,0}),z=q.rotate({0,0,1});
        Vec3 e{std::abs(x.x)*i.half.x+std::abs(y.x)*i.half.y+std::abs(z.x)*i.half.z,
               std::abs(x.y)*i.half.x+std::abs(y.y)*i.half.y+std::abs(z.y)*i.half.z,
               std::abs(x.z)*i.half.x+std::abs(y.z)*i.half.y+std::abs(z.z)*i.half.z};
        AABB b;b.add(i.p(t)-e-Vec3(2e-4));b.add(i.p(t)+e+Vec3(2e-4));return b;
    }
    int buildNode(int start,int end){
        int ni=int(nodes.size());nodes.push_back({});AABB b,cb;for(int k=start;k<end;k++){int j=indices[size_t(k)];b.add(instanceBounds[size_t(j)]);cb.add(centroids[size_t(j)]);}nodes[size_t(ni)].b=b;int n=end-start;
        if(n<=6){nodes[size_t(ni)].start=start;nodes[size_t(ni)].count=n;return ni;}
        Vec3 ex=cb.extent();int axis=ex.y>ex.x&&ex.y>=ex.z?1:(ex.z>ex.x&&ex.z>=ex.y?2:0);int mid=(start+end)/2;
        std::nth_element(indices.begin()+start,indices.begin()+mid,indices.begin()+end,[&](int a,int b2){return centroids[size_t(a)][axis]<centroids[size_t(b2)][axis];});
        nodes[size_t(ni)].left=buildNode(start,mid);nodes[size_t(ni)].right=buildNode(mid,end);return ni;
    }
    void rebuildBVH(){
        indices.resize(instances.size());std::iota(indices.begin(),indices.end(),0);instanceBounds.resize(instances.size());centroids.resize(instances.size());
        for(size_t i=0;i<instances.size();i++){instanceBounds[i]=boundsAt(instances[i],0);instanceBounds[i].add(boundsAt(instances[i],1));centroids[i]=instanceBounds[i].center();}
        nodes.clear();nodes.reserve(instances.size()*2);root=indices.empty()?-1:buildNode(0,int(indices.size()));
    }

    static bool localAABB(const Vec3&o,const Vec3&d,const Vec3&half,double&t0,double&t1){
        t0=0;t1=INF;for(int a=0;a<3;a++){if(std::abs(d[a])<1e-15){if(o[a]<-half[a]||o[a]>half[a])return false;continue;}double inv=1/d[a],n=(-half[a]-o[a])*inv,f=(half[a]-o[a])*inv;if(n>f)std::swap(n,f);t0=std::max(t0,n);t1=std::min(t1,f);if(t1<t0)return false;}return true;
    }
    bool roundedBoxHit(const Instance& i,
                       const Ray& r,
                       double maximumT,
                       double& t,
                       Vec3& localPoint,
                       Vec3& localNormal,
                       Vec3& worldTangent) const {
        const Quat q = i.q(r.time);
        const Vec3 position = i.p(r.time);
        const Vec3 origin = q.inverseRotate(r.o - position);
        const Vec3 direction = q.inverseRotate(r.d);
        double tEnter = 0;
        double tExit = INF;
        if (!localAABB(origin, direction, i.half, tEnter, tExit)) return false;
        tEnter = std::max(tEnter, r.tmin);
        tExit = std::min({tExit, r.tmax, maximumT});
        if (tExit < tEnter) return false;

        const double minimumHalf = std::min({i.half.x, i.half.y, i.half.z});
        const double bevel = std::min(c.bevel, 0.46 * minimumHalf);
        const Vec3 core = maxVec(i.half - Vec3(bevel), 0.0);
        const double epsilon = 2.0e-6 * (1.0 + tEnter);

        // Nearly every primary or shadow ray hits a planar face.  Accept those
        // analytically at the outer-box entry rather than sphere tracing through
        // dozens of iterations.  Only the small edge/corner region uses the
        // rounded-box SDF refinement below.
        Vec3 point = origin + tEnter * direction;
        const Vec3 absolutePoint = absVec(point);
        const bool flatX = std::abs(absolutePoint.x - i.half.x) < 2.5e-6 &&
            absolutePoint.y <= core.y + 2.5e-6 &&
            absolutePoint.z <= core.z + 2.5e-6;
        const bool flatY = std::abs(absolutePoint.y - i.half.y) < 2.5e-6 &&
            absolutePoint.x <= core.x + 2.5e-6 &&
            absolutePoint.z <= core.z + 2.5e-6;
        const bool flatZ = std::abs(absolutePoint.z - i.half.z) < 2.5e-6 &&
            absolutePoint.x <= core.x + 2.5e-6 &&
            absolutePoint.y <= core.y + 2.5e-6;
        if (flatX || flatY || flatZ) {
            t = tEnter;
            localPoint = point;
            if (flatX) localNormal = {point.x >= 0 ? 1.0 : -1.0, 0, 0};
            else if (flatY) localNormal = {0, point.y >= 0 ? 1.0 : -1.0, 0};
            else localNormal = {0, 0, point.z >= 0 ? 1.0 : -1.0};
            worldTangent = normalize(q.rotate(i.brushAxisLocal));
            return true;
        }

        // Edge and corner rays use a bounded Newton solve followed by a small
        // fixed bracket search.  The former sphere marcher could take its full
        // iteration budget for every candidate in a dense collapsed pile; a few
        // scanlines then became hundreds of times slower.  This solver has a
        // strict, small evaluation budget and is stable for grazing rays.
        auto acceptHit = [&](double candidateT) {
            candidateT = clamp(candidateT, tEnter, tExit);
            const Vec3 candidatePoint = origin + candidateT * direction;
            if (sdfRoundedBox(candidatePoint, core, bevel) > 2.0e-5) return false;
            t = candidateT;
            localPoint = candidatePoint;
            localNormal = roundedBoxNormal(candidatePoint, core);
            worldTangent = normalize(q.rotate(i.brushAxisLocal));
            return true;
        };

        double currentT = tEnter;
        for (int iteration = 0; iteration < 6; ++iteration) {
            point = origin + currentT * direction;
            const double distance = sdfRoundedBox(point, core, bevel);
            if (distance <= epsilon && acceptHit(currentT)) return true;
            const Vec3 gradient = roundedBoxNormal(point, core);
            const double denominator = dot(gradient, direction);
            double step = distance;
            if (denominator < -1.0e-6) step = distance / (-denominator);
            const double remaining = tExit - currentT;
            if (remaining <= 1.0e-7) break;
            step = clamp(step, 2.0e-6, std::max(2.0e-6, 0.55 * remaining));
            currentT += step;
        }

        double previousT = tEnter;
        double previousDistance = sdfRoundedBox(origin + previousT * direction, core, bevel);
        double bestT = previousT;
        double bestDistance = previousDistance;
        constexpr int probeCount = 6;
        for (int probe = 1; probe <= probeCount; ++probe) {
            const double probeT = mix(tEnter, tExit, double(probe) / probeCount);
            const double probeDistance = sdfRoundedBox(
                origin + probeT * direction, core, bevel);
            if (probeDistance < bestDistance) {
                bestDistance = probeDistance;
                bestT = probeT;
            }
            if (probeDistance <= 0.0 && previousDistance > 0.0) {
                double lo = previousT;
                double hi = probeT;
                for (int refinement = 0; refinement < 6; ++refinement) {
                    const double mid = 0.5 * (lo + hi);
                    if (sdfRoundedBox(origin + mid * direction, core, bevel) > 0.0)
                        lo = mid;
                    else
                        hi = mid;
                }
                if (acceptHit(hi)) return true;
            }
            previousT = probeT;
            previousDistance = probeDistance;
        }
        if (bestDistance <= 2.0e-5 && acceptHit(bestT)) return true;
        return false;
    }

    bool intersect(const Ray&r,Hit&h)const{
        bool found=false;if(root>=0){std::array<int,256>stack{};std::array<double,256>nearStack{};double rn=0;if(nodes[size_t(root)].b.hitNear(r,h.t,rn)){int sp=0;stack[size_t(sp)]=root;nearStack[size_t(sp++)]=rn;
            while(sp){--sp;int ni=stack[size_t(sp)];if(nearStack[size_t(sp)]>h.t)continue;const auto&node=nodes[size_t(ni)];if(node.leaf()){
                for(int k=0;k<node.count;k++){int j=indices[size_t(node.start+k)];double t;Vec3 lp,ln,tan;if(roundedBoxHit(instances[size_t(j)],r,h.t,t,lp,ln,tan)){found=true;h.t=t;h.p=r.o+t*r.d;Quat q=instances[size_t(j)].q(r.time);h.n=normalize(q.rotate(ln));if(dot(h.n,r.d)>0)h.n=-h.n;h.lp=lp;h.ln=ln;h.instance=j;h.material=instances[size_t(j)].material;h.floor=false;h.footprint=std::max(1e-7,r.coneWidth*h.t);Basis B=makeBasis(h.n,tan);h.tangent=B.t;h.bitangent=B.b;}}
            }else if(sp+2<int(stack.size())){double ln=0,rn2=0;bool hl=nodes[size_t(node.left)].b.hitNear(r,h.t,ln),hr=nodes[size_t(node.right)].b.hitNear(r,h.t,rn2);if(hl&&hr){if(ln<rn2){stack[size_t(sp)]=node.right;nearStack[size_t(sp++)]=rn2;stack[size_t(sp)]=node.left;nearStack[size_t(sp++)]=ln;}else{stack[size_t(sp)]=node.left;nearStack[size_t(sp++)]=ln;stack[size_t(sp)]=node.right;nearStack[size_t(sp++)]=rn2;}}else if(hl){stack[size_t(sp)]=node.left;nearStack[size_t(sp++)]=ln;}else if(hr){stack[size_t(sp)]=node.right;nearStack[size_t(sp++)]=rn2;}}
            }
        }}
        if(std::abs(r.d.y)>1e-12){
            double t=-r.o.y/r.d.y;
            const Vec3 floorPoint=r.o+t*r.d;
            const bool pedestalTop=std::abs(floorPoint.x)<.233 &&
                                    std::abs(floorPoint.z)<.163;
            if(!pedestalTop&&t>=r.tmin&&t<h.t&&t<=r.tmax){
                found=true;h.t=t;h.p=floorPoint;h.n={0,1,0};
                if (dot(h.n, r.d) > 0) h.n = -h.n;
                h.instance = -1;
                h.material = -1;
                h.floor=true;h.footprint=std::max(1e-7,r.coneWidth*h.t);
                h.tangent={1,0,0};h.bitangent={0,0,-1};
            }
        }
        return found;
    }
    bool blocked(const Ray& r) const {
        // Shadow rays only need an any-hit answer.  Calling the full closest-hit
        // traversal forced every softbox sample to walk the entire dense pile,
        // which made a few collapse-frame scanlines hundreds of times slower.
        // Return immediately at the first geometric blocker instead.
        if (root >= 0) {
            std::array<int, 256> stack{};
            int sp = 0;
            double nearT = 0;
            if (nodes[size_t(root)].b.hitNear(r, r.tmax, nearT)) {
                stack[size_t(sp++)] = root;
            }
            while (sp) {
                const int nodeIndex = stack[size_t(--sp)];
                const BVHNode& node = nodes[size_t(nodeIndex)];
                double nodeNear = 0;
                if (!node.b.hitNear(r, r.tmax, nodeNear)) continue;
                if (node.leaf()) {
                    for (int k = 0; k < node.count; ++k) {
                        const int instanceIndex = indices[size_t(node.start + k)];
                        double t = INF;
                        Vec3 localPoint, localNormal, tangent;
                        if (roundedBoxHit(
                                instances[size_t(instanceIndex)],
                                r,
                                r.tmax,
                                t,
                                localPoint,
                                localNormal,
                                tangent)) {
                            return true;
                        }
                    }
                } else if (sp + 2 < int(stack.size())) {
                    stack[size_t(sp++)] = node.left;
                    stack[size_t(sp++)] = node.right;
                }
            }
        }
        if (std::abs(r.d.y) > 1e-12) {
            const double t = -r.o.y / r.d.y;
            const Vec3 floorPoint=r.o+t*r.d;
            const bool pedestalTop=std::abs(floorPoint.x)<.233 &&
                                    std::abs(floorPoint.z)<.163;
            if (!pedestalTop && t >= r.tmin && t <= r.tmax) return true;
        }
        return false;
    }

    Vec3 environment(const Vec3&d)const{
        const Vec3 floorBounce{.020,.023,.029};
        const Vec3 wall{.105,.116,.135};
        const Vec3 ceiling{.055,.064,.082};
        Vec3 color=mix(floorBounce,wall,smootherstep(-.48,.10,d.y));
        color=mix(color,ceiling,smootherstep(.18,.96,d.y));
        const Vec3 window=normalize(Vec3{-.82,.34,.46});
        const Vec3 practical=normalize(Vec3{.58,.44,-.68});
        color+=Vec3{.32,.40,.52}*std::pow(saturate(dot(d,window)),96);
        color+=Vec3{.17,.095,.048}*std::pow(saturate(dot(d,practical)),125);
        return color;
    }
    Material materialFor(const Hit&h)const{
        if(h.floor){
            Material m=floorMaterial;
            const double macro=fbm(Vec3{h.p.x*2.2,0.0,h.p.z*1.65}+Vec3{4.1,0,8.3});
            const double fineVisibility=1.0-smoothstep(.0009,.0055,h.footprint);
            const double fine=valueNoise(Vec3{h.p.x*19.0,0.0,h.p.z*14.0});
            const double tone=.82+.18*macro+.035*(fine-.5)*fineVisibility;
            m.base=Vec3{.032,.036,.043}*tone;
            m.roughX=clamp(.47+.055*(macro-.5),.42,.55);
            m.roughY=clamp(.53+.050*(macro-.5),.48,.60);
            m.specProbability=.095;
            return m;
        }

        Material m=materials[size_t(std::clamp(h.material,0,23))];
        if(h.instance<0)return m;
        const Instance&i=instances[size_t(h.instance)];

        if(i.module&&c.decorateModules){
            const Vec3 an=absVec(h.ln);
            double u=0,v=0;
            int value=1;
            if(an.x>=an.y&&an.x>=an.z){
                u=h.lp.y; v=h.lp.z; value=h.ln.x>=0?1:6;
            }else if(an.y>=an.z){
                u=h.lp.x; v=h.lp.z; value=h.ln.y>=0?2:5;
            }else{
                u=h.lp.x; v=h.lp.y; value=h.ln.z>=0?3:4;
            }

            constexpr double a=0.00320;
            constexpr double radius=0.00116;
            const double footprint=std::max(1e-7,h.footprint);
            const double feather=clamp(.70*footprint,0.00016,0.00062);
            const double pipVisibility=1.0-smoothstep(.00042,.00138,footprint);
            std::array<Vec3,6> pips{};
            int count=0;
            auto add=[&](double x,double y){pips[size_t(count++)]={x,y,0};};
            if(value==1){add(0,0);}
            else if(value==2){add(-a,a);add(a,-a);}
            else if(value==3){add(-a,a);add(0,0);add(a,-a);}
            else if(value==4){add(-a,-a);add(-a,a);add(a,-a);add(a,a);}
            else if(value==5){add(-a,-a);add(-a,a);add(0,0);add(a,-a);add(a,a);}
            else {add(-a,-a);add(-a,0);add(-a,a);add(a,-a);add(a,0);add(a,a);}

            double pipMask=0;
            double cavityRim=0;
            for(int k=0;k<count;++k){
                const double d=std::sqrt(sq(u-pips[size_t(k)].x)+sq(v-pips[size_t(k)].y));
                pipMask=std::max(pipMask,1.0-smoothstep(radius-feather,radius+feather,d));
                const double rimIn=smoothstep(radius-.00018,radius+.00002,d);
                const double rimOut=1.0-smoothstep(radius+.00002,radius+.00034+feather,d);
                cavityRim=std::max(cavityRim,rimIn*rimOut);
            }
            const double distanceFade=(1.0-.18*smoothstep(.58,1.05,h.t));
            pipMask*=pipVisibility*distanceFade;
            cavityRim*=pipVisibility*distanceFade;
            if(cavityRim>0){
                m.base*=1.0-.18*cavityRim;
                m.roughX=clamp(m.roughX+.035*cavityRim,.14,.34);
                m.roughY=clamp(m.roughY+.040*cavityRim,.15,.36);
            }
            if(pipMask>0){
                const Material pip=materials[3];
                m.base=mix(m.base,pip.base,pipMask);
                m.roughX=mix(m.roughX,pip.roughX,pipMask);
                m.roughY=mix(m.roughY,pip.roughY,pipMask);
                m.specProbability=mix(m.specProbability,pip.specProbability,pipMask);
                m.metal=0;
            }

            const uint32_t hash=hash32(uint64_t(i.parent)*0x9e3779b97f4a7c15ULL+
                                       uint64_t(i.material)*37);
            const double lot=.996+.008*double((hash>>12)&255)/255.0;
            const double microVisibility=1.0-smoothstep(.00016,.00095,h.footprint);
            const double microNoise=valueNoise(h.lp*150.0+Vec3(double(i.parent%97)))-.5;
            const double micro=1.0+.0025*microNoise*microVisibility;
            m.base*=lot*micro;
            const double roughLot=double((hash>>22)&255)/255.0-.5;
            m.roughX=clamp(m.roughX+.006*roughLot,.145,.27);
            m.roughY=clamp(m.roughY+.007*roughLot,.155,.29);
            // Very restrained mold parting line; visible only in macro shots.
            const double seam=gaussianLine(h.lp.y,0.000055)*
                smootherstep(.0060,.0075,std::max(std::abs(h.lp.x),std::abs(h.lp.z)));
            m.roughX=clamp(m.roughX+.035*seam,.14,.32);
            m.roughY=clamp(m.roughY+.040*seam,.15,.34);
            m.base*=1.0-.010*seam;
            return m;
        }

        if(h.material==8 || h.material==19){
            const double macro=fbm(h.p*2.0+Vec3{2.7,8.1,5.4});
            m.base=Vec3{.030,.034,.041}*(.86+.17*macro);
            m.roughX=clamp(.46+.05*(macro-.5),.41,.54);
            m.roughY=clamp(.53+.05*(macro-.5),.48,.60);
            m.specProbability=.095;
        }else if(h.material==6 || h.material==21){
            const double n=fbm(h.p*3.2);
            const double fineVisibility=1.0-smoothstep(.0008,.0045,h.footprint);
            const double fine=valueNoise(h.p*18.0);
            m.base*=.91+.13*n;
            m.roughX=clamp(.74+.035*(fine-.5)*fineVisibility,.69,.80);
            m.roughY=m.roughX;
        }else if(h.material==5 || h.material==11 || h.material==20){
            const double brushVisibility=1.0-smoothstep(.00010,.00072,h.footprint);
            const double brush=.5+.5*std::sin(680.0*h.lp.x+
                3.0*valueNoise(h.lp*70.0));
            m.roughX=clamp(m.roughX+.020*(brush-.5)*brushVisibility,.09,.34);
            m.roughY=clamp(m.roughY+.036*(.5-brush)*brushVisibility,.19,.56);
        }
        return m;
    }

    void applyPipNormal(Hit& h) const {
        if(h.instance<0 || !c.decorateModules) return;
        const Instance& i=instances[size_t(h.instance)];
        if(!i.module) return;
        const Vec3 an=absVec(h.ln);
        double u=0,v=0;
        int value=1;
        Vec3 localU,localV;
        if(an.x>=an.y&&an.x>=an.z){
            u=h.lp.y;v=h.lp.z;value=h.ln.x>=0?1:6;
            localU={0,1,0};localV={0,0,1};
        }else if(an.y>=an.z){
            u=h.lp.x;v=h.lp.z;value=h.ln.y>=0?2:5;
            localU={1,0,0};localV={0,0,1};
        }else{
            u=h.lp.x;v=h.lp.y;value=h.ln.z>=0?3:4;
            localU={1,0,0};localV={0,1,0};
        }
        constexpr double a=.00325;
        constexpr double radius=.00118;
        constexpr double depth=.00031;
        std::array<Vec3,6> centers{};int count=0;
        auto add=[&](double x,double y){centers[size_t(count++)]={x,y,0};};
        if(value==1){add(0,0);}else if(value==2){add(-a,a);add(a,-a);}else if(value==3){add(-a,a);add(0,0);add(a,-a);}else if(value==4){add(-a,-a);add(-a,a);add(a,-a);add(a,a);}else if(value==5){add(-a,-a);add(-a,a);add(0,0);add(a,-a);add(a,a);}else{add(-a,-a);add(-a,0);add(-a,a);add(a,-a);add(a,0);add(a,a);}
        double best=INF,du=0,dv=0;
        for(int k=0;k<count;++k){double x=u-centers[size_t(k)].x,y=v-centers[size_t(k)].y,d2=x*x+y*y;if(d2<best){best=d2;du=x;dv=y;}}
        if(best>=radius*radius) return;
        const double normalVisibility=1.0-smoothstep(.00028,.00102,h.footprint);
        if(normalVisibility<=1e-4) return;
        const double r=std::sqrt(std::max(1e-18,best));
        const double edge=smoothstep(radius,.76*radius,r);
        const double gradientScale=(2.0*depth/(radius*radius))*edge*normalVisibility;
        const Quat q=i.q(.5);
        const Vec3 worldU=normalize(q.rotate(localU));
        const Vec3 worldV=normalize(q.rotate(localV));
        h.n=normalize(h.n-gradientScale*(du*worldU+dv*worldV));
        Basis basis=makeBasis(h.n,worldU);
        h.tangent=basis.t;h.bitangent=basis.b;
    }

    static Vec3 conductorFresnel(double cosTheta,const Vec3&eta,const Vec3&kk){
        cosTheta=saturate(cosTheta);Vec3 c2(cosTheta*cosTheta),eta2=eta*eta,k2=kk*kk,t0=eta2+k2;
        Vec3 two=2*cosTheta*eta;Vec3 rs=(t0-two+c2)/(t0+two+c2);
        Vec3 rp=(t0*c2-two+Vec3(1))/(t0*c2+two+Vec3(1));return .5*(rs+rp);
    }
    static double dielectricFresnel(double c,double etaI=1,double etaT=1.5){
        c=clamp(c,-1,1);bool entering=c>0;if(!entering){std::swap(etaI,etaT);c=std::abs(c);}double sinT=etaI/etaT*std::sqrt(std::max(0.0,1-c*c));if(sinT>=1)return 1;double ct=std::sqrt(std::max(0.0,1-sinT*sinT));double rs=(etaT*c-etaI*ct)/(etaT*c+etaI*ct),rp=(etaI*c-etaT*ct)/(etaI*c+etaT*ct);return .5*(rs*rs+rp*rp);
    }
    static double anisotropicD(const Vec3&h,const Basis&B,double ax,double ay){
        Vec3 l=toLocal(B,h);if(l.z<=0)return 0;double s=sq(l.x/ax)+sq(l.y/ay)+sq(l.z);return 1.0/(PI*ax*ay*s*s+1e-15);
    }
    static double smithG1(const Vec3&w,const Basis&B,double ax,double ay){
        Vec3 l=toLocal(B,w);if(l.z<=0)return 0;double tan2=(sq(ax*l.x)+sq(ay*l.y))/std::max(1e-12,sq(l.z));return 2.0/(1.0+std::sqrt(1.0+tan2));
    }
    Vec3 evalBSDF(const Material&m,const Basis&B,const Vec3&v,const Vec3&l,double&pdf)const{
        double nv=std::max(0.0,dot(B.n,v)),nl=std::max(0.0,dot(B.n,l));if(nv<=0||nl<=0){pdf=0;return Vec3(0);}Vec3 h=normalize(v+l);double nh=std::max(0.0,dot(B.n,h)),vh=std::max(0.0,dot(v,h));
        double ax=std::max(.014,sq(m.roughX)),ay=std::max(.014,sq(m.roughY));double D=anisotropicD(h,B,ax,ay),G=smithG1(v,B,ax,ay)*smithG1(l,B,ax,ay);
        Vec3 F=m.metal>.5?conductorFresnel(vh,m.eta,m.k):Vec3(dielectricFresnel(vh));Vec3 spec=F*(D*G/std::max(1e-9,4*nv*nl));
        spec*=1.0+.24*std::sqrt(m.roughX*m.roughY); // rough-conductor multiscatter compensation
        Vec3 diffuse=(1-m.metal)*m.base*(Vec3(1)-F)*(1/PI);double pSpec=clamp(m.specProbability,.08,.995),pd=nl/PI,ps=D*nh/std::max(1e-9,4*vh);pdf=(1-pSpec)*pd+pSpec*ps;return diffuse+spec;
    }
    bool sampleBSDF(const Material&m,const Basis&B,const Vec3&v,RNG&rng,Vec3&l,Vec3&f,double&pdf)const{
        double pSpec=clamp(m.specProbability,.08,.995);if(rng.uniform()>pSpec){double rr=std::sqrt(rng.uniform()),phi=2*PI*rng.uniform();l=normalize(toWorld(B,{rr*std::cos(phi),rr*std::sin(phi),std::sqrt(std::max(0.0,1-rr*rr))}));}
        else{
            double ax=std::max(.014,sq(m.roughX)),ay=std::max(.014,sq(m.roughY));double u=rng.uniform(),phi=2*PI*rng.uniform();double tanTheta=std::sqrt(u/std::max(1e-12,1-u));Vec3 hs{ax*tanTheta*std::cos(phi),ay*tanTheta*std::sin(phi),1};Vec3 h=normalize(toWorld(B,normalize(hs)));if(dot(v,h)<0)h=-h;l=normalize(2*dot(v,h)*h-v);
            if(dot(B.n,l)<=0){double rr=std::sqrt(rng.uniform()),q=2*PI*rng.uniform();l=normalize(toWorld(B,{rr*std::cos(q),rr*std::sin(q),std::sqrt(std::max(0.0,1-rr*rr))}));}
        }
        f=evalBSDF(m,B,v,l,pdf);return pdf>1e-12&&maxComponent(f)>0;
    }
    static double misWeight(double a,double b){double aa=a*a,bb=b*b;return aa/(aa+bb+1e-30);}
    Vec3 sampleDirectLight(const Hit&h,const Material&m,const Basis&B,const Vec3&v,RNG&rng,int li,double selectionPdf)const{
        const auto&light=lights[size_t(li)];
        Vec3 lp=light.center+(2*rng.uniform()-1)*light.u+(2*rng.uniform()-1)*light.v;
        Vec3 to=lp-h.p;double d2=lengthSquared(to),d=std::sqrt(d2);
        if(d2<1e-12)return Vec3(0);
        Vec3 wi=to/d;
        double nl=std::max(0.0,dot(B.n,wi));
        double cl=std::max(0.0,dot(light.normal(),-wi));
        if(nl<=0||cl<=0)return Vec3(0);
        Ray shadow{h.p+B.n*3e-4,wi,1e-4,d-7e-4,.5,0.0};
        if(blocked(shadow))return Vec3(0);
        double pl=selectionPdf*d2/(cl*light.area()),pb=0;
        Vec3 f=evalBSDF(m,B,v,wi,pb);
        if(pl<=1e-12)return Vec3(0);
        return light.emission*f*(nl*misWeight(pl,pb)/pl);
    }
    Vec3 directLight(const Hit&h,const Material&m,const Basis&B,const Vec3&v,RNG&rng,int depth)const{
        // The first visible bounce dominates this all-metal scene. Sampling every
        // softbox there removes the salt-and-pepper light-selection noise that a
        // denoiser cannot distinguish from tiny metallic features. Later bounces
        // retain weighted one-light MIS for cost control.
        if(depth==0){
            Vec3 sum(0);
            for(int li=0;li<int(lights.size());++li){
                sum+=sampleDirectLight(h,m,B,v,rng,li,1.0);
            }
            return sum;
        }
        double total=0;std::array<double,8>w{};
        for(int i=0;i<int(lights.size());i++){
            w[size_t(i)]=lights[size_t(i)].area()*luminance(lights[size_t(i)].emission);
            total+=w[size_t(i)];
        }
        double pick=rng.uniform()*total;int li=0;
        for(;li<int(lights.size())-1;li++)if((pick-=w[size_t(li)])<=0)break;
        return sampleDirectLight(h,m,B,v,rng,li,w[size_t(li)]/total);
    }
    Vec3 trace(Ray ray,RNG&rng,Feature&feat)const{
        Vec3 out(0),throughput(1);
        for(int depth=0;depth<c.maxDepth;depth++){
            Hit h;if(!intersect(ray,h)){out+=throughput*environment(ray.d);break;}
            applyPipNormal(h);
            Material m=materialFor(h);
            if(depth>0){
                m.roughX=std::max(m.roughX,.30);
                m.roughY=std::max(m.roughY,.32);
                m.specProbability=std::min(m.specProbability,.55);
            }
            Basis B{h.tangent,h.bitangent,h.n};
            if(depth==0){feat.n=h.n;feat.albedo=m.base;feat.depth=h.t;feat.hits=1;feat.instance=h.instance;}
            Vec3 v=-ray.d;out+=throughput*directLight(h,m,B,v,rng,depth);
            Vec3 l,f;double pdf;if(!sampleBSDF(m,B,v,rng,l,f,pdf))break;
            double nl=std::max(0.0,dot(B.n,l));throughput=throughput*f*(nl/pdf);
            if(!finite(throughput)||maxComponent(throughput)<1e-7)break;
            ray={h.p+B.n*3e-4,l,1e-4,INF,ray.time,std::min(0.035,ray.coneWidth*1.35+2e-5)};
            if(depth>=2){double q=clamp(maxComponent(throughput),.07,.96);if(rng.uniform()>q)break;throughput/=q;}
        }
        return out;
    }
    static std::vector<Vec3> denoiseImage(const std::vector<Vec3>& input,
                                                const std::vector<Feature>& features,
                                                int W,
                                                int H,
                                                int threads) {
        std::vector<Vec3> a = input;
        std::vector<Vec3> b(input.size());

        auto compatible = [&](const Feature& f0, const Feature& f1,
                              double normalThreshold) {
            if ((f0.hits > 0.0) != (f1.hits > 0.0)) return false;
            if (f0.hits <= 0.0) return true;
            // The analytic tabletop is a single planar instance. Its depth
            // changes rapidly under perspective, so depth gating would prevent
            // the filter from collecting enough samples and leave visible
            // Monte Carlo stipple. Instance identity is the stronger guide.
            if (f0.instance == -1 && f1.instance == -1) return true;
            // Mixed-coverage edge pixels are already supersampled. Do not let
            // the denoiser smear them across a neighboring die or backdrop.
            if (f0.instance == -3 || f1.instance == -3)
                return f0.instance == f1.instance;
            if (f0.instance != f1.instance) return false;
            const double nd = dot(f0.n, f1.n);
            if (nd < normalThreshold) return false;
            // Within one watertight die or set object, instance identity plus
            // normal and albedo are more reliable than perspective depth.
            const double ad = lengthSquared(f0.albedo - f1.albedo);
            return ad < 0.032;
        };

        // Guided 5x5 percentile clamp. It removes both isolated bright paths
        // and isolated black samples before filtering. Unlike the former BM3D
        // and unsharp chain, the clamp never crosses an instance boundary, so
        // pips, silhouettes, and contact gaps remain intact.
        parallelFor(H, threads, [&](int y) {
            for (int x = 0; x < W; ++x) {
                const size_t id = size_t(y) * W + x;
                const Feature& f0 = features[id];
                std::array<double, 25> values{};
                int count = 0;
                for (int oy = -2; oy <= 2; ++oy) {
                    for (int ox = -2; ox <= 2; ++ox) {
                        const int xx = std::clamp(x + ox, 0, W - 1);
                        const int yy = std::clamp(y + oy, 0, H - 1);
                        const size_t j = size_t(yy) * W + xx;
                        if (!compatible(f0, features[j], 0.94)) continue;
                        values[size_t(count++)] = luminance(input[j]);
                    }
                }
                if (count < 4) {
                    b[id] = input[id];
                    continue;
                }
                std::sort(values.begin(), values.begin() + count);
                const double q10 = values[size_t((count - 1) / 10)];
                const double q50 = values[size_t((count - 1) / 2)];
                const double q90 = values[size_t((count - 1) * 9 / 10)];
                const double spread = std::max(0.002, q90 - q10);
                const double lower = std::max(0.0, q10 - 0.30 * spread - 0.015);
                const double upper = q90 + 0.55 * spread + 0.035 + 0.25 * q50;
                const double current = luminance(input[id]);
                if (current < lower && current > 1e-12)
                    b[id] = input[id] * (lower / current);
                else if (current > upper && current > 1e-12)
                    b[id] = input[id] * (upper / current);
                else
                    b[id] = input[id];
            }
        });
        a.swap(b);

        constexpr std::array<double, 5> kernel{1.0, 4.0, 6.0, 4.0, 1.0};
        constexpr std::array<int, 4> steps{1, 2, 4, 8};
        constexpr std::array<double, 4> blends{0.68, 0.54, 0.39, 0.24};

        // Four-pass instance-aware a-trous reconstruction. Broad plaster,
        // resin, and wood regions receive enough support to remove salt-and-
        // pepper grain, while object IDs, pip albedo, normals, and depth prevent
        // the characteristic waxy edge bleeding of generic video denoisers.
        for (int pass = 0; pass < int(steps.size()); ++pass) {
            const int step = steps[size_t(pass)];
            const double blend = blends[size_t(pass)];
            parallelFor(H, threads, [&](int y) {
                for (int x = 0; x < W; ++x) {
                    const size_t id = size_t(y) * W + x;
                    const Feature& f0 = features[id];
                    if (f0.instance == -3) {
                        b[id] = a[id];
                        continue;
                    }
                    Vec3 sum(0);
                    double weightSum = 0.0;
                    for (int ky = -2; ky <= 2; ++ky) {
                        for (int kx = -2; kx <= 2; ++kx) {
                            const int xx = std::clamp(x + kx * step, 0, W - 1);
                            const int yy = std::clamp(y + ky * step, 0, H - 1);
                            const size_t j = size_t(yy) * W + xx;
                            const Feature& fj = features[j];
                            if (!compatible(f0, fj, pass < 2 ? 0.91 : 0.95))
                                continue;

                            double weight = kernel[size_t(kx + 2)] *
                                            kernel[size_t(ky + 2)];
                            if (f0.hits > 0.0) {
                                const double nd = saturate(dot(f0.n, fj.n));
                                weight *= std::pow(nd, pass < 2 ? 18.0 : 42.0);
                                const double depthScale =
                                    0.00055 + (pass < 2 ? 0.0040 : 0.0024) *
                                                    std::max(0.05, f0.depth);
                                const double dd =
                                    std::abs(f0.depth - fj.depth) / depthScale;
                                weight *= std::exp(-0.5 * dd * dd);
                                const double ad =
                                    lengthSquared(f0.albedo - fj.albedo);
                                weight *= std::exp(-70.0 * ad);
                            }
                            sum += weight * a[j];
                            weightSum += weight;
                        }
                    }
                    const Vec3 filtered =
                        weightSum > 1e-12 ? sum / weightSum : a[id];
                    const double localBlend =
                        f0.instance == -1 ? std::min(.92, blend + .18) : blend;
                    b[id] = mix(a[id], filtered, localBlend);
                }
            });
            a.swap(b);
        }
        return a;
    }


    void addInstance(const Snapshot&s,const Vec3&localOffset,const Quat&localQ,const Vec3&half,int material,int decoration){
        const double shutterDt=c.motionBlur?.18/30.0:0;Quat q1=integrateOrientation(s.q,s.w,shutterDt);Vec3 parentP1=s.p+s.v*shutterDt;
        Instance i;i.q0=normalized(s.q*localQ);i.q1=normalized(q1*localQ);i.p0=s.p+s.q.rotate(localOffset);i.p1=parentP1+q1.rotate(localOffset);i.half=half;i.material=material;i.id=s.id*16+decoration;i.parent=s.id;i.moduleClass=s.moduleClass;i.module=BodyKind(s.kind)==BodyKind::Module;i.brushAxisLocal={1,0,0};instances.push_back(i);
    }
    void setScene(const std::vector<Snapshot>&s){
        instances.clear();
        instances.reserve(s.size()+180);
        const Snapshot* supportLeaf=nullptr;

        for(const auto&x:s){
            const BodyKind kind=BodyKind(x.kind);
            if(kind==BodyKind::Module){
                const uint64_t h=splitmix64(uint64_t(x.id+1)*0x9e3779b97f4a7c15ULL);
                auto unit=[&](int shift){return double(uint32_t(h>>shift))/4294967295.0;};
                const Vec3 offset{(unit(8)-.5)*.000040,(unit(16)-.5)*.000026,
                                  (unit(24)-.5)*.000040};
                const Vec3 rawAxis{unit(5)-.5,unit(13)-.5,unit(21)-.5};
                const Vec3 axis=lengthSquared(rawAxis)>1e-12?normalize(rawAxis):Vec3{0,1,0};
                const Quat microQ=quatAxisAngle(axis,(unit(1)-.5)*.0010);
                const int lot=(h%100)<92?0:((h%100)<98?1:2);
                // A 0.10 mm render clearance hides the solver's collision skin
                // without changing the visible 16 mm manufacturing scale.
                const Vec3 visualHalf=maxVec(x.half-Vec3(.00010),.0005);
                addInstance(x,offset,microQ,visualHalf,lot,0);
            }else if(kind==BodyKind::Pedestal){
                if(x.half.y>.020){
                    // Exact collision body: flush black fixture base.
                    addInstance(x,{0,0,0},Quat{},x.half,7,0);
                }else{
                    supportLeaf=&x;
                    // Exact collision body: brushed-steel moving leaf.
                    addInstance(x,{0,0,0},Quat{},x.half,12,0);
                }
            }
        }

        // Telescoping sleeve beneath the physical leaf. It exists only while
        // the leaf is above the work surface and is rejected automatically if
        // any current-frame die AABB could intersect it.
        if(supportLeaf){
            const double sleeveTop=supportLeaf->p.y-supportLeaf->half.y-.00035;
            if(sleeveTop>.0015){
                const Vec3 sleeveP{supportLeaf->p.x,.5*sleeveTop,supportLeaf->p.z};
                const Vec3 sleeveHalf{.026,.5*sleeveTop,.021};
                const Quat sleeveQ{};
                bool clear=true;
                Instance candidate;
                candidate.p0=candidate.p1=sleeveP;
                candidate.q0=candidate.q1=sleeveQ;
                candidate.half=sleeveHalf;
                const AABB sleeveBounds=boundsAt(candidate,0.0);
                for(const Instance& die:instances){
                    if(!die.module) continue;
                    const AABB dieBounds=boundsAt(die,0.0);
                    const bool overlap=sleeveBounds.lo.x<=dieBounds.hi.x && sleeveBounds.hi.x>=dieBounds.lo.x &&
                                       sleeveBounds.lo.y<=dieBounds.hi.y && sleeveBounds.hi.y>=dieBounds.lo.y &&
                                       sleeveBounds.lo.z<=dieBounds.hi.z && sleeveBounds.hi.z>=dieBounds.lo.z;
                    if(overlap){clear=false;break;}
                }
                if(clear){
                    const double postX=.050, postZ=.026;
                    int postId=300050;
                    for(double px:{-postX,postX}) for(double pz:{-postZ,postZ})
                        addStaticBox({sleeveP.x+px,sleeveP.y,sleeveP.z+pz},sleeveQ,
                                     {.0065,sleeveHalf.y,.0065},12,postId++,false);
                    addStaticBox(sleeveP,sleeveQ,{.018,sleeveHalf.y,.014},7,300054,false);
                }
            }
        }

        // Everything below is renderer-only set dressing and is forced outside
        // the measured, expanded all-frame dice envelope by addSafeStaticBox.
        addSafeStaticBox({0,-.065,.61},Quat{}, {.78,.065,.035},17,310000); // front bench fascia
        addSafeStaticBox({-.745,-.065,.16},Quat{}, {.035,.065,.49},17,310001);
        addSafeStaticBox({ .745,-.065,.16},Quat{}, {.035,.065,.49},17,310002);

        // Deep concrete workshop wall and baseboard, placed far behind the pile.
        addSafeStaticBox({0,.43,-1.08},Quat{}, {1.50,.53,.035},6,310010);
        addSafeStaticBox({0,.050,-1.025},Quat{}, {.84,.050,.025},7,310011);

        // Rear-mounted metrology gantry. Columns sit behind and laterally
        // outside the complete dice sweep; the crossbar is above it.
        addSafeStaticBox({-.72,.26,-.52},Quat{}, {.028,.26,.028},7,310020);
        addSafeStaticBox({ .72,.26,-.52},Quat{}, {.028,.26,.028},7,310021);
        addSafeStaticBox({0,.515,-.52},Quat{}, {.748,.027,.030},7,310022);
        addSafeStaticBox({0,.265,-.535},Quat{}, {.430,.215,.012},17,310023);

        // A compact control box and physical reference ruler provide scale.
        addSafeStaticBox({.82,.12,-.575},Quat{}, {.075,.110,.045},7,310030);
        addSafeStaticBox({.82,.150,-.528},Quat{}, {.011,.011,.004},9,310031);
        addSafeStaticBox({.455,.003,.135},Quat{}, {.115,.003,.018},11,310040);
        for(int tick=0;tick<=12;++tick){
            const double x=.345+tick*(.220/12.0);
            const double depth=(tick%6==0)?.012:((tick%3==0)?.008:.005);
            addSafeStaticBox({x,.0068,.135},Quat{}, {.00055,.00065,.5*depth},7,310041+tick);
        }
        addSafeStaticBox({-.43,.024,.105},Quat{}, {.050,.024,.034},15,310060);
        addSafeStaticBox({-.43,.050,.105},Quat{}, {.036,.003,.025},11,310061);

        rebuildBVH();
    }

    Ray cameraRay(const Camera&cam,double x,double y,double aspect,RNG&rng)const{
        Vec3 f=normalize(cam.target-cam.position),right=normalize(cross(f,cam.up)),up=cross(right,f);double scale=std::tan(cam.verticalFov*PI/360);Vec3 d=normalize(f+((2*x-1)*aspect*scale)*right+((1-2*y)*scale)*up),o=cam.position;
        if(cam.aperture>0){double a=2*PI*rng.uniform(),rad=cam.aperture*std::sqrt(rng.uniform());Vec3 lens=rad*(std::cos(a)*right+std::sin(a)*up),fp=cam.position+d*(cam.focusDistance/std::max(1e-9,dot(d,f)));o+=lens;d=normalize(fp-o);}double time=c.motionBlur?rng.uniform():.5;
        const double coneWidth=2.0*scale/std::max(1,c.height);
        return{o,d,1e-4,INF,time,coneWidth};
    }
    std::vector<Vec3> render(const Camera& cam, int frameIndex) const {
        const int W = c.width;
        const int H = c.height;
        const int spp = std::clamp(c.samplesPerPixel, 1, 128);
        std::vector<Vec3> pixels(size_t(W) * H, Vec3(0));
        std::vector<Feature> features(size_t(W) * H);
        const double aspect = double(W) / H;
        const auto start = std::chrono::steady_clock::now();
        std::atomic<int> done{0};

        parallelFor(H, c.threads, [&](int y) {
            for (int x = 0; x < W; ++x) {
                std::array<Vec3, 128> sampleColors{};
                std::array<double, 128> sampleLuminance{};
                Feature merged;
                const int strataX = std::max(1, int(std::ceil(std::sqrt(double(spp)))));
                const int strataY = std::max(1, int(std::ceil(double(spp) / strataX)));

                for (int sample = 0; sample < spp; ++sample) {
                    // The sequence uses a frame-stable, stratified tent filter.
                    // Geometry motion changes the signal; the Monte Carlo pattern
                    // itself does not crawl from frame to frame.
                    const uint64_t seed = splitmix64(
                        uint64_t(y) * 0xbf58476d1ce4e5b9ULL ^
                        uint64_t(x) * 0x94d049bb133111ebULL ^
                        uint64_t(sample) * 0x632be59bd9b4e019ULL ^
                        0x6a09e667f3bcc909ULL);
                    RNG rng(seed);
                    const int sx = sample % strataX;
                    const int sy = sample / strataX;
                    auto tent = [](double u) {
                        return u < .5 ? std::sqrt(2 * u) - 1.0
                                      : 1.0 - std::sqrt(2.0 - 2.0 * u);
                    };
                    const double ux = (sx + rng.uniform()) / strataX;
                    const double uy = (sy + rng.uniform()) / strataY;
                    const double jx = .5 + .58 * tent(ux);
                    const double jy = .5 + .58 * tent(uy);

                    Feature sampleFeature;
                    Vec3 color = trace(
                        cameraRay(cam, (x + jx) / W, (y + jy) / H, aspect, rng),
                        rng,
                        sampleFeature);
                    if (!finite(color)) color = Vec3(0);
                    double lum = std::max(0.0, luminance(color));
                    // First-stage path clamp. The later robust estimator is
                    // neighborhood-independent, so it cannot erase thin pips or
                    // bevels by borrowing from another object.
                    if (lum > 6.0) {
                        color *= 6.0 / lum;
                        lum = 6.0;
                    }
                    sampleColors[size_t(sample)] = color;
                    sampleLuminance[size_t(sample)] = lum;

                    if (sampleFeature.hits) {
                        if (merged.hits == 0.0) merged.instance = sampleFeature.instance;
                        else if (merged.instance != sampleFeature.instance) merged.instance = -3;
                        merged.n += sampleFeature.n;
                        merged.albedo += sampleFeature.albedo;
                        merged.depth += sampleFeature.depth;
                        merged.hits += 1.0;
                    }
                }

                // Per-pixel winsorized mean. This removes isolated firefly paths
                // before spatial reconstruction while retaining broad legitimate
                // highlights. It also avoids the black/white salt-and-pepper
                // pattern produced by sharpening a low-sample estimate.
                std::array<double, 128> ordered = sampleLuminance;
                const int middle = spp / 2;
                std::nth_element(
                    ordered.begin(),
                    ordered.begin() + middle,
                    ordered.begin() + spp);
                const double median = ordered[size_t(middle)];
                const double upper = 0.36 + 4.25 * median;
                Vec3 sum(0);
                for (int sample = 0; sample < spp; ++sample) {
                    Vec3 color = sampleColors[size_t(sample)];
                    const double lum = sampleLuminance[size_t(sample)];
                    if (lum > upper && lum > 1e-12) color *= upper / lum;
                    sum += color;
                }

                const size_t id = size_t(y) * W + x;
                pixels[id] = sum / double(spp);
                if (merged.hits) {
                    merged.n = normalize(merged.n);
                    merged.albedo /= merged.hits;
                    merged.depth /= merged.hits;
                    merged.hits = 1.0;
                }
                features[id] = merged;
            }
            const int rows = done.fetch_add(1) + 1;
            if (rows % 48 == 0 || rows == H) {
                std::cout << "[render] frame " << frameIndex << " rows "
                          << rows << "/" << H << "     \r" << std::flush;
            }
        });

        std::cout << "[render] frame " << frameIndex << " " << std::fixed
                  << std::setprecision(2)
                  << std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start)
                         .count()
                  << "s                         \n";
        return c.denoise ? denoiseImage(pixels, features, W, H, c.threads)
                         : pixels;
    }
    static double linearToSrgb(double x){x=saturate(x);return x<=.0031308?12.92*x:1.055*std::pow(x,1/2.4)-.055;}
    static Vec3 aces(Vec3 x){auto f=[](double v){return saturate((v*(2.51*v+.03))/(v*(2.43*v+.59)+.14));};return{f(x.x),f(x.y),f(x.z)};}
    void writePPM(const std::filesystem::path&p,const std::vector<Vec3>&pixels)const{
        std::ofstream out(p,std::ios::binary);if(!out)throw std::runtime_error("cannot write "+p.string());out<<"P6\n"<<c.width<<" "<<c.height<<"\n255\n";for(Vec3 c0:pixels){Vec3 x=aces(c0*c.exposure);x={linearToSrgb(x.x),linearToSrgb(x.y),linearToSrgb(x.z)};unsigned char q[3]={uint8_t(clamp(std::round(x.x*255),0.0,255.0)),uint8_t(clamp(std::round(x.y*255),0.0,255.0)),uint8_t(clamp(std::round(x.z*255),0.0,255.0))};out.write(reinterpret_cast<char*>(q),3);}
    }
};

PathTracer::PathTracer(RenderConfig c):impl_(std::make_shared<Impl>(c)){}
void PathTracer::setScene(const std::vector<Snapshot>&s){impl_->setScene(s);}std::vector<Vec3>PathTracer::render(const Camera&c,int f)const{return impl_->render(c,f);}void PathTracer::writePPM(const std::filesystem::path&p,const std::vector<Vec3>&v)const{impl_->writePPM(p,v);}

Camera cinematicCamera(int frame,
                       int total,
                       const FrameDiagnostics& diagnostics,
                       bool hero) {
    (void)total; (void)diagnostics;
    Camera camera;
    // Three intentional locked-off tripod shots. There is no interpolation,
    // tracking, lens breathing, or per-frame camera noise.
    if(frame<=51){
        camera.position={.585,.292,.710};
        camera.target={-.005,.198,-.010};
        camera.verticalFov=hero?33.0:35.0;
    }else if(frame<=74){
        camera.position={.540,.205,.640};
        camera.target={-.010,.140,-.012};
        camera.verticalFov=hero?37.0:39.0;
    }else{
        camera.position={.485,.178,.565};
        camera.target={-.018,.058,-.018};
        camera.verticalFov=hero?34.0:36.0;
    }
    camera.focusDistance=length(camera.target-camera.position);
    camera.aperture=hero?.00055:0.0;
    camera.shutterFraction=.18;
    return camera;
}

} // namespace tower
