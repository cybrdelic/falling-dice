#include "render.hpp"

namespace meta {
namespace {
struct Ray{Vec3 o,d;double tmin=1e-4,tmax=INF,time=.5;};
struct AABB{
    Vec3 lo{INF,INF,INF},hi{-INF,-INF,-INF};
    void add(const Vec3&p){lo=minVec(lo,p);hi=maxVec(hi,p);}void add(const AABB&b){lo=minVec(lo,b.lo);hi=maxVec(hi,b.hi);}Vec3 center()const{return .5*(lo+hi);}Vec3 extent()const{return hi-lo;}
    bool hitNear(const Ray&r,double mx,double&nearT)const{double a=r.tmin,b=std::min(r.tmax,mx);for(int k=0;k<3;k++){if(std::abs(r.d[k])<1e-15){if(r.o[k]<lo[k]||r.o[k]>hi[k])return false;continue;}double inv=1/r.d[k],n=(lo[k]-r.o[k])*inv,f=(hi[k]-r.o[k])*inv;if(n>f)std::swap(n,f);a=std::max(a,n);b=std::min(b,f);if(b<a)return false;}nearT=a;return true;}
};
struct Material{
    Vec3 base{.6,.6,.6},eta{.25,.3,.4},k{3.5,3.1,2.7};double roughX=.22,roughY=.34,metal=1,specProbability=.98;bool enamel=false;
};
struct Instance{
    Vec3 p0,p1,half;Quat q0,q1;int material=0,id=0,parent=0,kind=0;Vec3 brushAxisLocal{1,0,0};
    Vec3 p(double t)const{return mix(p0,p1,t);}Quat q(double t)const{return slerp(q0,q1,t);}
};
struct Hit{double t=INF;Vec3 p,n,lp,ln,tangent,bitangent;int instance=-1,material=-1;bool floor=false;};
struct BVHNode{AABB b;int left=-1,right=-1,start=0,count=0;bool leaf()const{return count>0;}};
struct AreaLight{Vec3 center,u,v,emission;Vec3 normal()const{return normalize(cross(u,v));}double area()const{return 4*length(cross(u,v));}};
struct Feature{Vec3 n{0,0,0},albedo{0,0,0};double depth=0,hits=0;};
struct Basis{Vec3 t,b,n;};
Basis makeBasis(const Vec3&n,const Vec3&preferred=Vec3{0,0,0}){
    Vec3 t=preferred-dot(preferred,n)*n;if(lengthSquared(t)<1e-10)t=orthogonal(n);else t=normalize(t);return{t,normalize(cross(n,t)),n};
}
Vec3 toWorld(const Basis&B,const Vec3&v){return B.t*v.x+B.b*v.y+B.n*v.z;}
Vec3 toLocal(const Basis&B,const Vec3&v){return{dot(v,B.t),dot(v,B.b),dot(v,B.n)};}

double sdfRoundedBox(const Vec3&p,const Vec3&core,double radius){Vec3 q=absVec(p)-core,out=maxVec(q,0.0);return length(out)+std::min(maxComponent(q),0.0)-radius;}
Vec3 roundedBoxNormal(const Vec3&p,const Vec3&core){Vec3 c=clampVec(p,-core,core),n=p-c;if(lengthSquared(n)>1e-16)return normalize(n);Vec3 d=absVec(absVec(p)-core);if(d.x<=d.y&&d.x<=d.z)return{p.x>=0?1.0:-1.0,0,0};if(d.y<=d.z)return{0,p.y>=0?1.0:-1.0,0};return{0,0,p.z>=0?1.0:-1.0};}
}

struct PathTracer::Impl{
    RenderConfig c;std::array<Material,10>materials{};Material floorMaterial;std::vector<AreaLight>lights;std::vector<Instance>instances;std::vector<int>indices;std::vector<AABB>instanceBounds;std::vector<Vec3>centroids;std::vector<BVHNode>nodes;int root=-1;

    explicit Impl(RenderConfig cfg):c(cfg){
        // Engineered polymer, damage states, machined platens, and frame.
        // Matte engineering polymer with a deformation-state palette.  The
        // healthy material stays cyan; local plastic zones move through amber
        // to red so hinge formation remains readable without an overlay.
        materials[0]={{.035,.36,.40},{1.48,1.48,1.48},{0,0,0},.40,.52,0,.18,true};
        materials[1]={{.025,.54,.58},{1.48,1.48,1.48},{0,0,0},.36,.47,0,.20,true};
        materials[2]={{.78,.105,.018},{1.48,1.48,1.48},{0,0,0},.40,.50,0,.19,true};
        materials[3]={{.98,.31,.018},{1.48,1.48,1.48},{0,0,0},.34,.44,0,.22,true};
        materials[4]={{.045,.31,.34},{1.48,1.48,1.48},{0,0,0},.38,.50,0,.18,true};
        materials[5]={{.105,.125,.150},{1.52,1.52,1.52},{0,0,0},.36,.50,.28,.58,false};
        materials[6]={{.025,.032,.043},{1.35,1.25,1.15},{2.25,2.45,2.65},.31,.46,1,.98,false};
        materials[7]={{.055,.065,.078},{1.5,1.5,1.5},{0,0,0},.43,.56,.08,.34,false};
        materials[8]=materials[6];materials[9]=materials[5];
        floorMaterial={{.010,.014,.021},{1.5,1.5,1.5},{0,0,0},.58,.70,.01,.14,false};
        lights.push_back({{-.040,.125,.060},{.060,0,0},{0,0,.048},{20.0,19.0,17.0}});
        lights.push_back({{.062,.072,.050},{0,.030,0},{.026,0,0},{7.0,9.5,12.0}});
        lights.push_back({{-.060,.075,-.060},{0,.028,0},{.022,0,0},{7.5,4.5,2.6}});
        // Camera-side inspection softbox.  Its normal faces -Z, revealing the
        // interior fold pattern that would otherwise sit in platen shadow.
        lights.push_back({{.045,.055,.105},{0,.026,0},{.030,0,0},{5.0,7.0,8.5}});
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
    bool roundedBoxHit(const Instance&i,const Ray&r,double mx,double&t,Vec3&lp,Vec3&ln,Vec3&worldTangent)const{
        Quat q=i.q(r.time);Vec3 pos=i.p(r.time);Vec3 o=q.inverseRotate(r.o-pos),d=q.inverseRotate(r.d);double t0,t1;if(!localAABB(o,d,i.half,t0,t1))return false;
        t0=std::max(t0,r.tmin);t1=std::min({t1,r.tmax,mx});if(t1<t0)return false;
        double minHalf=std::min({i.half.x,i.half.y,i.half.z}),bevel=std::min(c.bevel,.46*minHalf);Vec3 core=maxVec(i.half-Vec3(bevel),0.0);double tt=t0;
        for(int iter=0;iter<48&&tt<=t1;iter++){Vec3 p=o+tt*d;double sd=sdfRoundedBox(p,core,bevel);if(sd<8e-5){t=tt;lp=p;ln=roundedBoxNormal(p,core);worldTangent=normalize(q.rotate(i.brushAxisLocal));return true;}tt+=std::max(5e-5,.78*sd);}return false;
    }
    bool sphereHit(const Instance&i,const Ray&r,double mx,double&t,Vec3&lp,Vec3&ln,Vec3&worldTangent)const{
        Quat q=i.q(r.time);Vec3 pos=i.p(r.time);Vec3 o=q.inverseRotate(r.o-pos),d=q.inverseRotate(r.d);
        const double radius=i.half.x;double b=dot(o,d),cc=dot(o,o)-radius*radius,disc=b*b-cc;
        if (disc < 0) return false;
        const double root = std::sqrt(disc);
        double tt = -b - root;
        if (tt < r.tmin) tt = -b + root;
        if (tt < r.tmin || tt > r.tmax || tt >= mx) return false;
        t = tt;
        lp = o + tt * d;
        ln = normalize(lp);
        worldTangent = normalize(q.rotate(orthogonal(ln)));
        return true;
    }
    bool capsuleHit(const Instance&i,const Ray&r,double mx,double&t,Vec3&lp,Vec3&ln,Vec3&worldTangent)const{
        Quat q=i.q(r.time);Vec3 pos=i.p(r.time);Vec3 o=q.inverseRotate(r.o-pos),d=q.inverseRotate(r.d);
        const double h=i.half.x,radius=i.half.y;double best=std::min(mx,r.tmax);bool found=false;Vec3 bestP,bestN;
        // Finite cylinder around the local X axis.
        double A=d.y*d.y+d.z*d.z,B=2*(o.y*d.y+o.z*d.z),C=o.y*o.y+o.z*o.z-radius*radius;
        if(A>1e-18){double disc=B*B-4*A*C;if(disc>=0){double root=std::sqrt(disc);for(double tt:{(-B-root)/(2*A),(-B+root)/(2*A)}){if(tt>=r.tmin&&tt<best){double x=o.x+tt*d.x;if(x>=-h&&x<=h){Vec3 p=o+tt*d;best=tt;bestP=p;bestN=normalize(Vec3{0,p.y,p.z});found=true;}}}}}
        // Hemispherical end caps.
        for(double ex:{-h,h}){Vec3 center{ex,0,0},oc=o-center;double b=dot(oc,d),cc=dot(oc,oc)-radius*radius,disc=b*b-cc;if(disc<0)continue;double root=std::sqrt(disc);for(double tt:{-b-root,-b+root}){if(tt<r.tmin||tt>=best)continue;Vec3 p=o+tt*d;if((ex<0&&p.x<=-h)||(ex>0&&p.x>=h)){best=tt;bestP=p;bestN=normalize(p-center);found=true;}}}
        if (!found) return false;
        t = best;
        lp = bestP;
        ln = bestN;
        worldTangent = normalize(q.rotate({1, 0, 0}));
        return true;
    }
    bool intersect(const Ray&r,Hit&h)const{
        bool found=false;if(root>=0){std::array<int,256>stack{};std::array<double,256>nearStack{};double rn=0;if(nodes[size_t(root)].b.hitNear(r,h.t,rn)){int sp=0;stack[size_t(sp)]=root;nearStack[size_t(sp++)]=rn;
            while(sp){--sp;int ni=stack[size_t(sp)];if(nearStack[size_t(sp)]>h.t)continue;const auto&node=nodes[size_t(ni)];if(node.leaf()){
                for(int k=0;k<node.count;k++){
                    int j=indices[size_t(node.start+k)];const Instance&inst=instances[size_t(j)];double t;Vec3 lp,ln,tan;bool hit=false;
                    if(inst.kind==int(BodyKind::Strut))hit=capsuleHit(inst,r,h.t,t,lp,ln,tan);
                    else if(inst.kind==int(BodyKind::Joint))hit=sphereHit(inst,r,h.t,t,lp,ln,tan);
                    else hit=roundedBoxHit(inst,r,h.t,t,lp,ln,tan);
                    if(hit){found=true;h.t=t;h.p=r.o+t*r.d;Quat q=inst.q(r.time);h.n=normalize(q.rotate(ln));if(dot(h.n,r.d)>0)h.n=-h.n;h.lp=lp;h.ln=ln;h.instance=j;h.material=inst.material;h.floor=false;Basis B=makeBasis(h.n,tan);h.tangent=B.t;h.bitangent=B.b;}
                }
            }else if(sp+2<int(stack.size())){double ln=0,rn2=0;bool hl=nodes[size_t(node.left)].b.hitNear(r,h.t,ln),hr=nodes[size_t(node.right)].b.hitNear(r,h.t,rn2);if(hl&&hr){if(ln<rn2){stack[size_t(sp)]=node.right;nearStack[size_t(sp++)]=rn2;stack[size_t(sp)]=node.left;nearStack[size_t(sp++)]=ln;}else{stack[size_t(sp)]=node.left;nearStack[size_t(sp++)]=ln;stack[size_t(sp)]=node.right;nearStack[size_t(sp++)]=rn2;}}else if(hl){stack[size_t(sp)]=node.left;nearStack[size_t(sp++)]=ln;}else if(hr){stack[size_t(sp)]=node.right;nearStack[size_t(sp++)]=rn2;}}
            }
        }}
        if(std::abs(r.d.y)>1e-12){double t=(-.016-r.o.y)/r.d.y;if(t>=r.tmin&&t<h.t&&t<=r.tmax){found=true;h.t=t;h.p=r.o+t*r.d;h.n={0,1,0};if(dot(h.n,r.d)>0)h.n=-h.n;h.instance=-1;h.material=-1;h.floor=true;h.tangent={1,0,0};h.bitangent={0,0,-1};}}
        return found;
    }
    bool blocked(const Ray&r)const{Hit h;h.t=r.tmax;return intersect(r,h);}

    Vec3 environment(const Vec3&d)const{
        double h=saturate(.5*(d.y+1));
        Vec3 horizon{.045,.055,.072},zenith{.003,.005,.010},ground{.008,.011,.017};
        Vec3 c=d.y>=0?mix(horizon,zenith,std::pow(h,.72)):mix(horizon,ground,saturate(-d.y));
        Vec3 a=normalize(Vec3{-.62,.68,.39}),b=normalize(Vec3{.74,.38,-.55});
        c+=Vec3{.26,.34,.48}*std::pow(saturate(dot(d,a)),80);
        c+=Vec3{.23,.13,.065}*std::pow(saturate(dot(d,b)),130);
        return c;
    }
    Material materialFor(const Hit&h)const{
        if(h.floor)return floorMaterial;
        Material m=materials[size_t(std::clamp(h.material,0,9))];
        if(h.instance<0)return m;
        const Instance&i=instances[size_t(h.instance)];uint32_t hash=hash32(uint64_t(i.parent)*0x9e3779b97f4a7c15ULL+uint64_t(i.material)*37);
        if(i.material<=3){
            // Longitudinal brushing with micro-lot variation. Roughness is
            // anisotropic: scratches run along the module's machined X axis.
            double phase=double(hash&1023)*.0157;double brush=.5+.5*std::sin(92*h.lp.x+34*h.lp.z+phase);
            m.roughX=clamp(m.roughX+.020*(brush-.5),.07,.42);m.roughY=clamp(m.roughY+.055*(brush-.5),.11,.52);
            double lot=.955+.085*double((hash>>12)&255)/255.0;m.base*=lot;
            // Laser-welded longitudinal channels rather than copied symbols.
            double nx=h.lp.x/std::max(1e-8,i.half.x);if(std::abs(std::abs(nx)-.62)<.027){m.base*=.72;m.roughX+=.04;m.roughY+=.09;}
        }
        return m;
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
    Vec3 directLight(const Hit&h,const Material&m,const Basis&B,const Vec3&v,RNG&rng)const{
        double total=0;std::array<double,8>w{};for(int i=0;i<int(lights.size());i++){w[size_t(i)]=lights[size_t(i)].area()*luminance(lights[size_t(i)].emission);total+=w[size_t(i)];}
        double pick=rng.uniform()*total;int li=0;for(;li<int(lights.size())-1;li++)if((pick-=w[size_t(li)])<=0)break;const auto&light=lights[size_t(li)];double selection=w[size_t(li)]/total;
        Vec3 lp=light.center+(2*rng.uniform()-1)*light.u+(2*rng.uniform()-1)*light.v,to=lp-h.p;double d2=lengthSquared(to),d=std::sqrt(d2);if(d2<1e-12)return Vec3(0);Vec3 wi=to/d;
        double nl=std::max(0.0,dot(B.n,wi)),cl=std::max(0.0,dot(light.normal(),-wi));if(nl<=0||cl<=0)return Vec3(0);Ray shadow{h.p+B.n*5e-5,wi,1e-4,d-1e-4,.5};if(blocked(shadow))return Vec3(0);
        double pl=selection*d2/(cl*light.area()),pb=0;Vec3 f=evalBSDF(m,B,v,wi,pb);if(pl<=1e-12)return Vec3(0);return light.emission*f*(nl*misWeight(pl,pb)/pl);
    }
    Vec3 trace(Ray ray,RNG&rng,Feature&feat)const{
        Vec3 out(0),throughput(1);for(int depth=0;depth<c.maxDepth;depth++){Hit h;if(!intersect(ray,h)){out+=throughput*environment(ray.d);break;}Material m=materialFor(h);Basis B{h.tangent,h.bitangent,h.n};if(depth==0){feat.n=h.n;feat.albedo=m.base;feat.depth=h.t;feat.hits=1;}
            Vec3 v=-ray.d;out+=throughput*directLight(h,m,B,v,rng);Vec3 l,f;double pdf;if(!sampleBSDF(m,B,v,rng,l,f,pdf))break;double nl=std::max(0.0,dot(B.n,l));throughput=throughput*f*(nl/pdf);if(!finite(throughput)||maxComponent(throughput)<1e-7)break;ray={h.p+B.n*5e-5,l,1e-4,INF,ray.time};if(depth>=2){double q=clamp(maxComponent(throughput),.07,.96);if(rng.uniform()>q)break;throughput/=q;}}
        return out;
    }
    static std::vector<Vec3> denoiseImage(const std::vector<Vec3>&input,const std::vector<Feature>&features,int W,int H,int threads){
        std::vector<Vec3>a=input,b(input.size());constexpr double K[5]={1,4,6,4,1};for(int it=0;it<4;it++){int step=1<<it;parallelFor(H,threads,[&](int y){for(int x=0;x<W;x++){size_t id=size_t(y)*W+x;Vec3 c0=a[id],sum(0);double ws=0;const auto&f0=features[id];for(int ky=-2;ky<=2;ky++)for(int kx=-2;kx<=2;kx++){int xx=std::clamp(x+kx*step,0,W-1),yy=std::clamp(y+ky*step,0,H-1);size_t j=size_t(yy)*W+xx;double ww=K[kx+2]*K[ky+2];const auto&fj=features[j];if(f0.hits&&fj.hits){double wn=std::pow(std::max(0.0,dot(f0.n,fj.n)),56),wd=std::exp(-std::abs(f0.depth-fj.depth)/(.022*std::max(1.0,f0.depth)+.025)),wa=std::exp(-lengthSquared(f0.albedo-fj.albedo)/.045),wc=std::exp(-lengthSquared(c0-a[j])/(.12+.10*luminance(c0)));ww*=wn*wd*wa*wc;}else if(f0.hits!=fj.hits)ww=0;else ww*=std::exp(-lengthSquared(c0-a[j])/.08);sum+=ww*a[j];ws+=ww;}b[id]=ws>1e-12?sum/ws:c0;}});a.swap(b);}return a;
    }

    void addInstance(const Snapshot&s,const Vec3&localOffset,const Quat&localQ,const Vec3&half,int material,int decoration){
        const double shutterDt=c.motionBlur?.72/24.0:0;Quat q1=integrateOrientation(s.q,s.w,shutterDt);Vec3 parentP1=s.p+s.v*shutterDt;
        Instance i;i.q0=normalized(s.q*localQ);i.q1=normalized(q1*localQ);i.p0=s.p+s.q.rotate(localOffset);i.p1=parentP1+q1.rotate(localOffset);i.half=half;i.material=material;i.id=s.id*16+decoration;i.parent=s.id;i.kind=s.kind;i.brushAxisLocal={1,0,0};instances.push_back(i);
    }
    void setScene(const std::vector<Snapshot>&s){
        instances.clear();instances.reserve(s.size());
        for(const auto&x:s)addInstance(x,{0,0,0},Quat{},x.half,std::clamp(x.material,0,9),0);
        rebuildBVH();
    }
    Ray cameraRay(const Camera&cam,double x,double y,double aspect,RNG&rng)const{
        Vec3 f=normalize(cam.target-cam.position),right=normalize(cross(f,cam.up)),up=cross(right,f);double scale=std::tan(cam.verticalFov*PI/360);Vec3 d=normalize(f+((2*x-1)*aspect*scale)*right+((1-2*y)*scale)*up),o=cam.position;
        if(cam.aperture>0){double a=2*PI*rng.uniform(),rad=cam.aperture*std::sqrt(rng.uniform());Vec3 lens=rad*(std::cos(a)*right+std::sin(a)*up),fp=cam.position+d*(cam.focusDistance/std::max(1e-9,dot(d,f)));o+=lens;d=normalize(fp-o);}double time=c.motionBlur?rng.uniform():.5;return{o,d,1e-4,INF,time};
    }
    std::vector<Vec3> render(const Camera&cam,int frameIndex)const{
        int W=c.width,H=c.height;std::vector<Vec3>pixels(size_t(W)*H,Vec3(0));std::vector<Feature>features(size_t(W)*H);double aspect=double(W)/H;auto start=std::chrono::steady_clock::now();std::atomic<int>done{0};
        parallelFor(H,c.threads,[&](int y){for(int x=0;x<W;x++){
            std::vector<Vec3> samples(size_t(c.samplesPerPixel));
            std::vector<double> sampleLuminance(size_t(c.samplesPerPixel));
            Feature merged;
            for(int s=0;s<c.samplesPerPixel;s++){
                uint64_t seed=splitmix64(uint64_t(frameIndex)*0x9e3779b97f4a7c15ULL^uint64_t(y)*0xbf58476d1ce4e5b9ULL^uint64_t(x)*0x94d049bb133111ebULL^uint64_t(s)*0x632be59bd9b4e019ULL);
                RNG rng(seed);double jx=s?rng.uniform():.5,jy=s?rng.uniform():.5;Feature sf;
                Vec3 col=trace(cameraRay(cam,(x+jx)/W,(y+jy)/H,aspect,rng),rng,sf);
                samples[size_t(s)]=col;sampleLuminance[size_t(s)]=luminance(col);
                if(sf.hits){merged.n+=sf.n;merged.albedo+=sf.albedo;merged.depth+=sf.depth;merged.hits++;}
            }
            std::vector<double> sorted=sampleLuminance;
            std::nth_element(sorted.begin(),sorted.begin()+sorted.size()/2,sorted.end());
            const double median=sorted[sorted.size()/2];
            const double cap=std::max(2.0,8.0*median);
            Vec3 sum(0);
            for(size_t s=0;s<samples.size();++s){Vec3 col=samples[s];double l=sampleLuminance[s];if(l>cap)col*=cap/l;sum+=col;}
            pixels[size_t(y)*W+x]=sum/double(c.samplesPerPixel);
            if(merged.hits){merged.n=normalize(merged.n);merged.albedo/=merged.hits;merged.depth/=merged.hits;}
            features[size_t(y)*W+x]=merged;
        }
            int n=done.fetch_add(1)+1;if(n%48==0||n==H)std::cout<<"[render] frame "<<frameIndex<<" rows "<<n<<"/"<<H<<"     \r"<<std::flush;});
        std::cout<<"[render] frame "<<frameIndex<<" "<<std::fixed<<std::setprecision(2)<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<"s                         \n";return c.denoise?denoiseImage(pixels,features,W,H,c.threads):pixels;
    }
    static double linearToSrgb(double x){x=saturate(x);return x<=.0031308?12.92*x:1.055*std::pow(x,1/2.4)-.055;}
    static Vec3 aces(Vec3 x){auto f=[](double v){return saturate((v*(2.51*v+.03))/(v*(2.43*v+.59)+.14));};return{f(x.x),f(x.y),f(x.z)};}
    void writePPM(const std::filesystem::path&p,const std::vector<Vec3>&pixels)const{
        std::ofstream out(p,std::ios::binary);if(!out)throw std::runtime_error("cannot write "+p.string());out<<"P6\n"<<c.width<<" "<<c.height<<"\n255\n";for(Vec3 c0:pixels){Vec3 x=aces(c0*c.exposure);x={linearToSrgb(x.x),linearToSrgb(x.y),linearToSrgb(x.z)};unsigned char q[3]={uint8_t(clamp(std::round(x.x*255),0.0,255.0)),uint8_t(clamp(std::round(x.y*255),0.0,255.0)),uint8_t(clamp(std::round(x.z*255),0.0,255.0))};out.write(reinterpret_cast<char*>(q),3);}
    }
};

PathTracer::PathTracer(RenderConfig c):impl_(std::make_shared<Impl>(c)){}
void PathTracer::setScene(const std::vector<Snapshot>&s){impl_->setScene(s);}std::vector<Vec3>PathTracer::render(const Camera&c,int f)const{return impl_->render(c,f);}void PathTracer::writePPM(const std::filesystem::path&p,const std::vector<Vec3>&v)const{impl_->writePPM(p,v);}

Camera compressionCamera(int frame,int total,const FrameDiagnostics&d,bool hero){
    (void)frame;(void)total;(void)d;
    Camera c;
    c.position=hero?Vec3{.067,.052,.118}:Vec3{.076,.057,.137};
    c.target={0,.0335,0};
    c.verticalFov=hero?27.0:29.5;
    c.focusDistance=length(c.target-c.position);
    c.aperture=0.0;
    c.shutterFraction=.22;
    return c;
}

} // namespace meta
