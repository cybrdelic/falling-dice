#include "sim.hpp"
#include "marching_cubes_table.hpp"

namespace meta { namespace {
struct ScalarReturn{double force=0,p=0,a=0,dg=0,energy=0,hard=0;bool yielded=false;};
ScalarReturn scalarReturn(double total,double pc,double ac,double k,double y,double H){ScalarReturn r;double trial=k*(total-pc),f=std::abs(trial)-(y+H*ac);if(f>0){double s=trial>=0?1:-1;r.dg=f/std::max(1e-20,k+H);r.p=pc+s*r.dg;r.a=ac+r.dg;r.yielded=true;}else{r.p=pc;r.a=ac;}r.force=k*(total-r.p);r.energy=.5*k*sq(total-r.p);r.hard=.5*H*sq(r.a);return r;}
struct VectorReturn{Vec3 force,p;double a=0,dg=0,energy=0,hard=0;bool yielded=false;};
VectorReturn vectorReturn(const Vec3&t,const Vec3&pc,double ac,double k,double y,double H){VectorReturn r;Vec3 trial=k*(t-pc);double n=length(trial),f=n-(y+H*ac);if(f>0&&n>1e-20){Vec3 d=trial/n;r.dg=f/std::max(1e-20,k+H);r.p=pc+r.dg*d;r.a=ac+r.dg;r.yielded=true;}else{r.p=pc;r.a=ac;}r.force=k*(t-r.p);r.energy=.5*k*lengthSquared(t-r.p);r.hard=.5*H*sq(r.a);return r;}
struct ContactShape{Vec3 a,b;double r=0;int n0=-1,n1=-1,beam=-1;bool capsule=false;};
struct ClosestPair{double s=0,t=0;Vec3 pa,pb;};
ClosestPair closestSegments(const Vec3&p1,const Vec3&q1,const Vec3&p2,const Vec3&q2){
 Vec3 d1=q1-p1,d2=q2-p2,r=p1-p2;double a=dot(d1,d1),e=dot(d2,d2),f=dot(d2,r),s=0,t=0;
 if(a<=1e-20&&e<=1e-20)return{0,0,p1,p2};
 if(a<=1e-20){s=0;t=saturate(f/e);}else{double c=dot(d1,r);if(e<=1e-20){t=0;s=saturate(-c/a);}else{double b=dot(d1,d2),den=a*e-b*b;s=std::abs(den)>1e-20?saturate((b*f-c*e)/den):0;t=(b*s+f)/e;if(t<0){t=0;s=saturate(-c/a);}else if(t>1){t=1;s=saturate((b-c)/a);}}}
 return{s,t,p1+s*d1,p2+t*d2};
}
ClosestPair closestShapes(const ContactShape&a,const ContactShape&b){
 if(a.capsule&&b.capsule)return closestSegments(a.a,a.b,b.a,b.b);
 if(a.capsule&&!b.capsule){Vec3 d=a.b-a.a;double s=saturate(dot(b.a-a.a,d)/std::max(1e-30,lengthSquared(d)));Vec3 p=a.a+s*d;return{s,0,p,b.a};}
 if(!a.capsule&&b.capsule){Vec3 d=b.b-b.a;double t=saturate(dot(a.a-b.a,d)/std::max(1e-30,lengthSquared(d)));Vec3 p=b.a+t*d;return{0,t,a.a,p};}
 return{0,0,a.a,b.a};
}
void addShapeGradient(std::vector<Vec3>&g,const ContactShape&s,double u,const Vec3&v){if(s.capsule){g[size_t(s.n0)]+=(1-u)*v;g[size_t(s.n1)]+=u*v;}else g[size_t(s.n0)]+=v;}
bool localShapes(const ContactShape& a,
                 const ContactShape& b,
                 const std::unordered_set<uint64_t>& connected) {
    if (a.beam >= 0 && a.beam == b.beam) return true;
    const int aa[2] = {a.n0, a.n1};
    const int bb[2] = {b.n0, b.n1};
    const int na = a.capsule ? 2 : 1;
    const int nb = b.capsule ? 2 : 1;
    for (int i = 0; i < na; ++i) {
        for (int j = 0; j < nb; ++j) {
            if (aa[i] == bb[j]) return true;
            if (connected.contains(pairKey(aa[i], bb[j]))) return true;
        }
    }
    return false;
}

Quat alignX(const Vec3&d){Vec3 x=normalize(d),y=orthogonal(x),z=normalize(cross(x,y));y=normalize(cross(z,x));return quatFromAxes(x,y,z);}
struct Hist{std::vector<double>s,y;double rho=0;};
double vdot(const std::vector<double>&a,const std::vector<double>&b){double r=0;for(size_t i=0;i<a.size();i++)r+=a[i]*b[i];return r;}
double vrms(const std::vector<double>&a,const std::vector<uint8_t>&l){double s=0;size_t n=0;for(size_t i=0;i<a.size();i++)if(!l[i]){s+=a[i]*a[i];n++;}return n?std::sqrt(s/n):0;}
double vmax(const std::vector<double>&a,const std::vector<uint8_t>&l){double m=0;for(size_t i=0;i<a.size();i++)if(!l[i])m=std::max(m,std::abs(a[i]));return m;}
void clampDirection(std::vector<double>&p,const SimulationConfig&c,const std::vector<uint8_t>&locked){double mt=c.maxTranslationStep/c.translationScale;for(size_t i=0;i<p.size()/6;i++){size_t b=6*i;Vec3 t{p[b],p[b+1],p[b+2]};double lt=length(t);if(lt>mt){t*=mt/lt;p[b]=t.x;p[b+1]=t.y;p[b+2]=t.z;}Vec3 r{p[b+3],p[b+4],p[b+5]};double lr=length(r);if(lr>c.maxRotationStep){r*=c.maxRotationStep/lr;p[b+3]=r.x;p[b+4]=r.y;p[b+5]=r.z;}}for(size_t i=0;i<p.size();i++)if(locked[i])p[i]=0;}
}

Simulator::Simulator(SimulationConfig c):cfg_(std::move(c)){cfg_.translationScale=cfg_.cellSize;build();distributeMass();baseX_.resize(nodes_.size());baseR_.resize(nodes_.size());for(size_t i=0;i<nodes_.size();i++){baseX_[i]=nodes_[i].x;baseR_[i]=nodes_[i].R;}}

void Simulator::build(){
 int nx=cfg_.cellsX,ny=cfg_.cellsY,nz=cfg_.cellsZ;
 int corners=(nx+1)*(ny+1)*(nz+1);
 auto corner=[&](int i,int j,int k){return(j*(nz+1)+k)*(nx+1)+i;};
 auto center=[&](int i,int j,int k){return corners+(j*nz+k)*nx+i;};
 width_=nx*cfg_.cellSize;height_=ny*cfg_.cellSize;depth_=nz*cfg_.cellSize;area_=width_*depth_;
 double jr=cfg_.jointScale*cfg_.maxRadius;Vec3 origin{-.5*width_,jr,-.5*depth_};RNG rng(cfg_.seed);
 nodes_.clear();beams_.clear();connected_.clear();
 nodes_.reserve(size_t(corners+nx*ny*nz+16*nx*ny*nz));
 // Corner and BCC-centre nodes define the nominal unit-cell topology.  Small
 // geometric deviations represent measured print/manufacturing imperfection;
 // no force is injected later to make the structure buckle.
 for(int j=0;j<=ny;j++)for(int k=0;k<=nz;k++)for(int i=0;i<=nx;i++){
  Node n;n.x0=origin+Vec3{i*cfg_.cellSize,j*cfg_.cellSize,k*cfg_.cellSize};
  double yf=double(j)/ny,env=std::sin(PI*yf),sg=((i+2*k+3*j)&1)?1.0:-1.0;
  n.x0.x+=cfg_.imperfection*env*(.55*sg+.45*rng.signedUniform());
  n.x0.z+=cfg_.imperfection*env*(.35*sg+.65*rng.signedUniform());
  n.x=n.x0;n.layer=j;nodes_.push_back(n);
 }
 for(int j=0;j<ny;j++)for(int k=0;k<nz;k++)for(int i=0;i<nx;i++){
  Node n;n.x0=origin+Vec3{(i+.5)*cfg_.cellSize,(j+.5)*cfg_.cellSize,(k+.5)*cfg_.cellSize};
  double yf=(j+.5)/double(ny),env=std::sin(PI*yf),sg=((i+k+j)&1)?1.0:-1.0;
  // Coherent, alternating layer offsets turn the BCC core into a graded
  // accordion topology.  The trigger is geometric and printable: no lateral
  // force or post-threshold displacement is applied during the solve.
  double sx=(j&1)?-1.0:1.0,sz=((j/2)&1)?-1.0:1.0;
  n.x0.x+=cfg_.cellSize*(.082*(1-.32*yf)*sx+.018*env*sg)
          +1.2*cfg_.imperfection*env*rng.signedUniform();
  n.x0.z+=cfg_.cellSize*(.032*(.65+.35*(1-yf))*sz-.012*env*sg)
          +1.2*cfg_.imperfection*env*rng.signedUniform();
  n.x=n.x0;n.layer=j;nodes_.push_back(n);
 }
 std::unordered_set<uint64_t>physicalPairs;
 auto radius=[&](int a,int b,int layer,double scale){
  Vec3 m=.5*(nodes_[size_t(a)].x0+nodes_[size_t(b)].x0);
  double y=saturate((m.y-jr)/height_);
  double radial=saturate(std::sqrt(m.x*m.x+m.z*m.z)/(.72*std::max(width_,depth_)));
  // Lower trigger band is deliberately slender; upper layers gradually thicken
  // so collapse propagates upward rather than occurring as one global mode.
  double grade=.58+.42*std::pow(1-y,.72);grade*=.94+.12*radial;
  if(layer==0)grade*=cfg_.triggerScale; else if(layer==1)grade*=std::sqrt(cfg_.triggerScale); else if(layer==2)grade*=.88;
  return scale*lerp(cfg_.minRadius,cfg_.maxRadius,saturate(grade));
 };
 auto addPhysical=[&](int a,int b,int layer,int family,double scale=1.0){
  uint64_t key=pairKey(a,b);if(a==b||!physicalPairs.insert(key).second)return;
  Vec3 pa=nodes_[size_t(a)].x0,pb=nodes_[size_t(b)].x0,d=pb-pa;
  Vec3 tangent=normalize(d);
  uint64_t h=splitmix64(cfg_.seed^key^uint64_t(family+31)*0x9e3779b97f4a7c15ULL);
  Vec3 helper{double(int((h>>0)&1023)-511),double(int((h>>10)&1023)-511),double(int((h>>20)&1023)-511)};
  Vec3 axis=cross(tangent,helper);if(lengthSquared(axis)<1e-12)axis=orthogonal(tangent);else axis=normalize(axis);
  Vec3 axis2=normalize(cross(tangent,axis));
  double layerFactor=layer==0?2.60:(layer==1?1.60:1.0);
  double amplitude=cfg_.memberImperfection*layerFactor*(.55+.90*double((h>>32)&65535)/65535.0);
  int segments=std::clamp(cfg_.elementsPerStrut,2,5);
  std::vector<int> chain;chain.reserve(size_t(segments+1));chain.push_back(a);
  for(int s=1;s<segments;s++){
   double t=double(s)/segments;
   Node mid;mid.x0=(1-t)*pa+t*pb
      +std::sin(PI*t)*amplitude*axis
      +.22*std::sin(2*PI*t)*amplitude*axis2;
   mid.x=mid.x0;mid.layer=layer;chain.push_back(int(nodes_.size()));nodes_.push_back(mid);
  }
  chain.push_back(b);
  double r=radius(a,b,layer,scale);
  auto element=[&](int u,int v){Beam e;e.a=u;e.b=v;e.layer=layer;e.family=family;Vec3 ed=nodes_[size_t(v)].x0-nodes_[size_t(u)].x0;e.L0=length(ed);e.e0=normalize(ed);e.radius=r;e.area=PI*r*r;e.I=.25*PI*std::pow(r,4);e.J=2*e.I;beams_.push_back(e);connected_.insert(pairKey(u,v));};
  for(size_t s=0;s+1<chain.size();s++)element(chain[s],chain[s+1]);
  connected_.insert(key); // suppress self-contact across the same physical strut
 };
 int off[8][3]={{0,0,0},{1,0,0},{0,1,0},{1,1,0},{0,0,1},{1,0,1},{0,1,1},{1,1,1}};
 for(int j=0;j<ny;j++)for(int k=0;k<nz;k++)for(int i=0;i<nx;i++){
  int c=center(i,j,k);for(int q=0;q<8;q++)addPhysical(c,corner(i+off[q][0],j+off[q][1],k+off[q][2]),j,q);
 }
 // No artificial vertical frame is added around the specimen.  The BCC core
 // carries the load, so the exterior is free to develop the lateral folds and
 // shear bands that the graded trigger geometry selects.
 // Thin face grids distribute platen load without creating infinitely rigid end plates.
 for(int face:{0,ny}){
  for(int k=0;k<=nz;k++)for(int i=0;i<nx;i++)addPhysical(corner(i,face,k),corner(i+1,face,k),std::clamp(face,0,ny-1),30,.62);
  for(int i=0;i<=nx;i++)for(int k=0;k<nz;k++)addPhysical(corner(i,face,k),corner(i,face,k+1),std::clamp(face,0,ny-1),31,.62);
 }
 top0_=jr+height_+jr;
}

void Simulator::distributeMass(){mass_=0;for(Node&n:nodes_)n.mass=0;for(const Beam&e:beams_){double m=cfg_.material.density*e.area*e.L0;nodes_[size_t(e.a)].mass+=.5*m;nodes_[size_t(e.b)].mass+=.5*m;mass_+=m;}double jr=cfg_.jointScale*cfg_.maxRadius,jm=cfg_.material.density*(4.0/3.0)*PI*std::pow(jr,3)*.28;for(Node&n:nodes_){n.mass+=jm;mass_+=jm;}locked_.assign(nodes_.size()*6,0);int a=-1,b=-1;double da=INF,db=INF;for(int i=0;i<int(nodes_.size());i++)if(nodes_[size_t(i)].layer==0){Vec3 p=nodes_[size_t(i)].x0;double qa=sq(p.x+.5*width_)+sq(p.z+.5*depth_),qb=sq(p.x-.5*width_)+sq(p.z+.5*depth_);if(qa<da){da=qa;a=i;}if(qb<db){db=qb;b=i;}}if(a<0||b<0)throw std::runtime_error("failed to create anti-drift constraints");locked_[size_t(6*a)]=1;locked_[size_t(6*a+2)]=1;locked_[size_t(6*b+2)]=1;}
std::vector<double>Simulator::zeroVars()const{return std::vector<double>(nodes_.size()*6,0);}
void Simulator::state(const std::vector<double>&z,std::vector<Vec3>&x,std::vector<Quat>&R)const{x.resize(nodes_.size());R.resize(nodes_.size());for(size_t i=0;i<nodes_.size();i++){size_t b=6*i;Vec3 t{z[b],z[b+1],z[b+2]},r{z[b+3],z[b+4],z[b+5]};for(int d=0;d<3;d++){if(locked_[b+size_t(d)])t[d]=0;if(locked_[b+size_t(d+3)])r[d]=0;}x[i]=baseX_[i]+cfg_.translationScale*t;R[i]=normalized(quatExp(r)*baseR_[i]);}}

Objective Simulator::objective(const std::vector<double>&z,double top,bool trials)const{Objective o;o.grad.assign(nodes_.size()*6,0);if(trials)o.trial.resize(beams_.size());std::vector<Vec3>x,gx(nodes_.size(),Vec3(0)),gr(nodes_.size(),Vec3(0));std::vector<Quat>R;state(z,x,R);double E=cfg_.material.young,G=E/(2*(1+cfg_.material.nu)),sy=cfg_.material.yield,H=cfg_.material.hardening;
 for(size_t bi=0;bi<beams_.size();bi++){const Beam&e=beams_[bi];Vec3 d=x[size_t(e.b)]-x[size_t(e.a)];double L=length(d);if(L<1e-10){o.energy+=1e8*sq(1e-10-L);continue;}Vec3 t=d/L;TrialBeam tr;double eps=(L-e.L0)/e.L0;auto ax=scalarReturn(eps,e.plastic.epsp,e.plastic.alpha,E,sy,H);tr.strain=eps;tr.stress=ax.force;tr.next=e.plastic;tr.next.epsp=ax.p;tr.next.alpha=ax.a;tr.axialPlastic=ax.yielded;double uax=e.area*e.L0*(ax.energy+ax.hard),dax=e.area*e.L0*sy*ax.dg;o.energy+=uax+dax;o.d.elastic+=uax;o.d.plastic+=dax;double N=e.area*ax.force;gx[size_t(e.a)]-=N*t;gx[size_t(e.b)]+=N*t;
  Vec3 da=R[size_t(e.a)].rotate(e.e0),db=R[size_t(e.b)].rotate(e.e0);double cs=.5*cfg_.shearCorrection*G*e.area*e.L0;Vec3 ma=da-t,mb=db-t;double us=.5*cs*(lengthSquared(ma)+lengthSquared(mb));o.energy+=us;o.d.elastic+=us;Vec3 gt=cs*(2*t-da-db),trans=(gt-dot(gt,t)*t)/L;gx[size_t(e.a)]-=trans;gx[size_t(e.b)]+=trans;gr[size_t(e.a)]-=cs*cross(da,t);gr[size_t(e.b)]-=cs*cross(db,t);
  Quat Q=normalized(R[size_t(e.a)].conjugate()*R[size_t(e.b)]);Vec3 phi=quatLog(Q),axis=normalize(R[size_t(e.a)].inverseRotate(t));double tw=dot(phi,axis),ptw=dot(e.plastic.rotp,axis);Vec3 bend=phi-tw*axis,pb=e.plastic.rotp-ptw*axis;double kb=E*e.I/e.L0,kt=G*e.J/e.L0,my=sy*e.I/e.radius,ty=(sy/std::sqrt(3.0))*e.J/e.radius,hb=cfg_.material.bendHardening*kb,ht=cfg_.material.torsionHardening*kt;auto br=vectorReturn(bend,pb,e.plastic.alphaB,kb,my,hb);auto tor=scalarReturn(tw,ptw,e.plastic.alphaT,kt,ty,ht);tr.next.rotp=br.p+tor.p*axis;tr.next.alphaB=br.a;tr.next.alphaT=tor.a;tr.moment=br.force+tor.force*axis;tr.bendPlastic=br.yielded;tr.torsionPlastic=tor.yielded;double ur=br.energy+br.hard+tor.energy+tor.hard,dr=my*br.dg+ty*tor.dg;o.energy+=ur+dr;o.d.elastic+=ur;o.d.plastic+=dr;tr.elastic=uax+us+ur;tr.dissipation=dax+dr;Vec3 loc=leftJacobianInverseTransposeApply(phi,tr.moment),world=R[size_t(e.a)].rotate(loc);gr[size_t(e.a)]-=world;gr[size_t(e.b)]+=world;if(trials)o.trial[bi]=tr;}
 for(size_t i=0;i<nodes_.size();i++){double u=nodes_[i].mass*cfg_.gravity*x[i].y;o.energy+=u;gx[i].y+=nodes_[i].mass*cfg_.gravity;}
 Vec3 cen(0);for(auto&p:x)cen+=p;cen/=double(x.size());o.energy+=.5*cfg_.centeringStiffness*(sq(cen.x)+sq(cen.z));for(auto&g:gx){g.x+=cfg_.centeringStiffness*cen.x/x.size();g.z+=cfg_.centeringStiffness*cen.z/x.size();}
 double jr=cfg_.jointScale*cfg_.maxRadius;
 // Exact sphere/capsule broad phase.  Every beam is treated as a swept
 // circular section; unlike the former point-sample model, contact location
 // and force interpolation are continuous along the strut centreline.
 std::vector<ContactShape> shapes;shapes.reserve(nodes_.size()+beams_.size());
 for(int i=0;i<int(nodes_.size());i++)shapes.push_back({x[size_t(i)],x[size_t(i)],jr,i,-1,-1,false});
 for(int bi=0;bi<int(beams_.size());bi++){const Beam&e=beams_[size_t(bi)];shapes.push_back({x[size_t(e.a)],x[size_t(e.b)],e.radius,e.a,e.b,bi,true});}
 double broadCell=std::max(2.8*jr,.72*cfg_.cellSize);
 std::unordered_map<int64_t,std::vector<int>,Int3Hash>grid;grid.reserve(shapes.size()*3);
 for(int si=0;si<int(shapes.size());si++){
  const ContactShape&s=shapes[size_t(si)];Vec3 lo=minVec(s.a,s.b)-Vec3(s.r+cfg_.contactMargin),hi=maxVec(s.a,s.b)+Vec3(s.r+cfg_.contactMargin);
  int i0=int(std::floor(lo.x/broadCell)),j0=int(std::floor(lo.y/broadCell)),k0=int(std::floor(lo.z/broadCell));
  int i1=int(std::floor(hi.x/broadCell)),j1=int(std::floor(hi.y/broadCell)),k1=int(std::floor(hi.z/broadCell));
  for(int j=j0;j<=j1;j++)for(int k=k0;k<=k1;k++)for(int i=i0;i<=i1;i++)grid[cellKey(i,j,k)].push_back(si);
 }
 std::unordered_set<uint64_t>tested;tested.reserve(shapes.size()*8);
 for(const auto&[cellKeyValue,list]:grid){(void)cellKeyValue;for(size_t ia=0;ia<list.size();ia++)for(size_t ib=ia+1;ib<list.size();ib++){
  int ai=list[ia],bi=list[ib];uint64_t key=pairKey(ai,bi);if(!tested.insert(key).second)continue;const ContactShape&a=shapes[size_t(ai)],&b=shapes[size_t(bi)];if(localShapes(a,b,connected_))continue;
  ClosestPair cp=closestShapes(a,b);Vec3 delta=cp.pb-cp.pa;double dist=length(delta),target=a.r+b.r+cfg_.contactMargin;if(dist>=target)continue;
  Vec3 n=dist>1e-12?delta/dist:normalize(Vec3{.37+.001*ai,.73,-.41+.001*bi});double pen=target-dist,mag=cfg_.contactStiffness*pen,en=.5*cfg_.contactStiffness*pen*pen;
  o.energy+=en;o.d.contact+=en;o.d.maxPen=std::max(o.d.maxPen,pen);o.d.contacts++;addShapeGradient(gx,a,cp.s,mag*n);addShapeGradient(gx,b,cp.t,-mag*n);
 }}
 // Lubricated rigid platens: only normal contact is included.  The joint
 // spheres form the actual platen interface and supply the measured reaction.
 for(size_t i=0;i<nodes_.size();i++){
  double pb=jr-x[i].y;if(pb>0){double m=cfg_.contactStiffness*pb,en=.5*cfg_.contactStiffness*pb*pb;o.energy+=en;o.d.contact+=en;o.d.maxPen=std::max(o.d.maxPen,pb);o.d.contacts++;gx[i].y-=m;}
  double pt=x[i].y+jr-top;if(pt>0){double m=cfg_.contactStiffness*pt,en=.5*cfg_.contactStiffness*pt*pt;o.energy+=en;o.d.contact+=en;o.d.reaction+=m;o.d.maxPen=std::max(o.d.maxPen,pt);o.d.contacts++;gx[i].y+=m;}
 }
 for(size_t i=0;i<nodes_.size();i++){size_t b=6*i;o.grad[b]=cfg_.translationScale*gx[i].x;o.grad[b+1]=cfg_.translationScale*gx[i].y;o.grad[b+2]=cfg_.translationScale*gx[i].z;o.grad[b+3]=gr[i].x;o.grad[b+4]=gr[i].y;o.grad[b+5]=gr[i].z;}for(size_t i=0;i<o.grad.size();i++)if(locked_[i])o.grad[i]=0;if(!std::isfinite(o.energy)){o.energy=INF;std::fill(o.grad.begin(),o.grad.end(),0);}return o;}

OptStats Simulator::minimize(double top,std::vector<double>&z,Objective&final){
 OptStats st;std::vector<Hist>hist;hist.reserve(cfg_.history);
 Objective cur=objective(z,top,false),best=cur;std::vector<double>bestZ=z;
 auto converged=[&](const Objective&o,double rms,double mx){double rel=rms/std::max(1e-6,cfg_.translationScale*std::max(1.0,o.d.reaction));bool forceOK=(rms<=cfg_.gradientRmsTolerance&&mx<=cfg_.gradientMaxTolerance)||rel<1.5e-3;return forceOK&&o.d.maxPen<=cfg_.penetrationTolerance;};
 auto lineSearch=[&](const std::vector<double>&p,double dg,std::vector<double>&candidateZ,Objective&candidate)->bool{
  double step=1.0;for(int bt=0;bt<cfg_.maxLineSearch;bt++){for(size_t i=0;i<z.size();i++){candidateZ[i]=z[i]+step*p[i];if(locked_[i])candidateZ[i]=0;}candidate=objective(candidateZ,top,false);st.backtracks++;if(std::isfinite(candidate.energy)&&candidate.energy<=cur.energy+cfg_.armijo*step*dg)return true;step*=.5;if(step<cfg_.minLineStep)break;}return false;
 };
 for(int it=0;it<cfg_.maxIterations;it++){
  st.iterations=it+1;st.rms=vrms(cur.grad,locked_);st.max=vmax(cur.grad,locked_);st.energy=cur.energy;
  if(converged(cur,st.rms,st.max)){st.converged=true;break;}
  std::vector<double>p=cur.grad,alpha(hist.size());
  for(int h=int(hist.size())-1;h>=0;h--){alpha[size_t(h)]=hist[size_t(h)].rho*vdot(hist[size_t(h)].s,p);for(size_t i=0;i<p.size();i++)p[i]-=alpha[size_t(h)]*hist[size_t(h)].y[i];}
  double H0=.10;if(!hist.empty()){const auto&h=hist.back();double sy=vdot(h.s,h.y),yy=vdot(h.y,h.y);if(sy>1e-20&&yy>1e-20)H0=clamp(sy/yy,1e-7,1e4);}for(double&v:p)v*=H0;
  for(size_t h=0;h<hist.size();h++){double beta=hist[h].rho*vdot(hist[h].y,p);for(size_t i=0;i<p.size();i++)p[i]+=hist[h].s[i]*(alpha[h]-beta);}
  for (double& v : p) v = -v;
  for (size_t i = 0; i < p.size(); ++i) if (locked_[i]) p[i] = 0;
  clampDirection(p, cfg_, locked_);
  double dg=vdot(cur.grad,p);if(!(dg<0)){hist.clear();p=cur.grad;for(double&v:p)v=-v;for(size_t i=0;i<p.size();i++)if(locked_[i])p[i]=0;clampDirection(p,cfg_,locked_);dg=vdot(cur.grad,p);}
  std::vector<double>zc(z.size());Objective cand;bool ok=lineSearch(p,dg,zc,cand);
  if(!ok){
   // Contact active-set changes can invalidate the quasi-Newton metric.  Retry
   // from a true steepest-descent direction with its own Armijo search.
   hist.clear();
   p = cur.grad;
   for (double& v : p) v = -v;
   for (size_t i = 0; i < p.size(); ++i) if (locked_[i]) p[i] = 0;
   clampDirection(p, cfg_, locked_);
   dg = vdot(cur.grad, p);
   ok = lineSearch(p, dg, zc, cand);
  }
  if(!ok)break;
  std::vector<double>sv(z.size()),yv(z.size());for(size_t i=0;i<z.size();i++){sv[i]=zc[i]-z[i];yv[i]=cand.grad[i]-cur.grad[i];}
  double sy=vdot(sv,yv),ss=vdot(sv,sv),yy=vdot(yv,yv);if(sy>1e-12*std::sqrt(std::max(1e-30,ss*yy))){if(int(hist.size())==cfg_.history)hist.erase(hist.begin());hist.push_back({std::move(sv),std::move(yv),1/sy});}else hist.clear();
  z=std::move(zc);cur=std::move(cand);if(cur.energy<best.energy){best=cur;bestZ=z;}
 }
 if(!st.converged&&best.energy<cur.energy){z=bestZ;cur=best;}
 final=objective(z,top,true);st.rms=vrms(final.grad,locked_);st.max=vmax(final.grad,locked_);st.energy=final.energy;st.converged=converged(final,st.rms,st.max);return st;
}
void Simulator::commit(const std::vector<double>&z,const Objective&o){std::vector<Vec3>x;std::vector<Quat>R;state(z,x,R);if(o.trial.size()!=beams_.size())throw std::runtime_error("missing constitutive trial states");for(size_t i=0;i<nodes_.size();i++){nodes_[i].x=x[i];nodes_[i].R=R[i];baseX_[i]=x[i];baseR_[i]=R[i];}for(size_t i=0;i<beams_.size();i++)beams_[i].plastic=o.trial[i].next;}
double Simulator::topY(double s,bool loading)const{double compressed=top0_-cfg_.maxStrain*height_;return loading?lerp(top0_,compressed,saturate(s)):lerp(compressed,top0_,saturate(s));}

std::vector<Snapshot> Simulator::capture(double top,
                                         const std::vector<Vec3>* prevX,
                                         const std::vector<Quat>* prevR) const {
    std::vector<Snapshot> out;
    // One visible primitive for every physical beam and joint, followed by
    // rigid machine geometry.  Nothing in the render is allowed to intersect
    // the specimen unless it also exists in the mechanics model.
    out.reserve(beams_.size() + nodes_.size() + 10);
    const double dt = std::max(1e-6, cfg_.stepTime);

    auto nodeVelocity = [&](int i) {
        if (!prevX || prevX->size() != nodes_.size()) return Vec3(0);
        return (nodes_[size_t(i)].x - (*prevX)[size_t(i)]) / dt;
    };
    auto nodeOmega = [&](int i) {
        if (!prevR || prevR->size() != nodes_.size()) return Vec3(0);
        Quat dq = normalized(nodes_[size_t(i)].R * (*prevR)[size_t(i)].conjugate());
        return quatLog(dq) / dt;
    };

    for (size_t bi = 0; bi < beams_.size(); ++bi) {
        const Beam& e = beams_[bi];
        const Vec3 a = nodes_[size_t(e.a)].x;
        const Vec3 b = nodes_[size_t(e.b)].x;
        const Vec3 d = b - a;
        const double L = std::max(1e-8, length(d));
        const double pAxial = std::abs(e.plastic.epsp);
        const double pRot = length(e.plastic.rotp);
        const double severity = std::max(pAxial / 0.10, pRot / 0.42);
        int material = 0;
        if (severity > 0.28) material = 1;
        if (severity > 0.62) material = 2;
        if (severity > 1.05) material = 3;

        Snapshot s;
        s.p = 0.5 * (a + b);
        s.q = alignX(d);
        s.v = 0.5 * (nodeVelocity(e.a) + nodeVelocity(e.b));
        s.w = 0.5 * (nodeOmega(e.a) + nodeOmega(e.b));
        s.half = {0.5 * L, e.radius, e.radius};
        s.material = material;
        s.id = int(bi);
        s.kind = int(BodyKind::Strut);
        s.moduleClass = e.family;
        s.ring = e.layer;
        s.slot = int(bi);
        out.push_back(s);
    }

    const double jointRadius = cfg_.jointScale * cfg_.maxRadius;
    for (size_t ni = 0; ni < nodes_.size(); ++ni) {
        Snapshot s;
        s.p = nodes_[ni].x;
        s.q = nodes_[ni].R;
        s.v = nodeVelocity(int(ni));
        s.w = nodeOmega(int(ni));
        s.half = Vec3(jointRadius);
        s.material = 4;
        s.id = int(beams_.size() + ni);
        s.kind = int(BodyKind::Joint);
        s.moduleClass = nodes_[ni].layer;
        s.ring = nodes_[ni].layer;
        s.slot = int(ni);
        out.push_back(s);
    }

    int id = int(beams_.size() + nodes_.size());
    auto box = [&](Vec3 p, Vec3 half, int material, BodyKind kind) {
        Snapshot s;
        s.p = p;
        s.q = Quat{};
        s.half = half;
        s.material = material;
        s.id = id++;
        s.kind = int(kind);
        out.push_back(s);
    };

    // Platen collision planes are y=0 and y=top.  The visible solids terminate
    // at exactly those planes, so the rendered machine and mechanics agree.
    // Compact metrology platens: large enough to maintain a uniform boundary
    // condition, but not so oversized that the specimen disappears visually.
    box({0, -0.0030, 0}, {0.026, 0.0030, 0.026}, 5, BodyKind::Platen);
    box({0, top + 0.0030, 0}, {0.026, 0.0030, 0.026}, 5, BodyKind::Platen);
    box({0, -0.0095, 0}, {0.036, 0.0035, 0.036}, 6, BodyKind::Machine);

    // The load frame is real geometry, but it sits behind the specimen and
    // remains secondary to the deformation in the technical macro view.
    const double columnY = 0.054;
    box({-0.037, columnY, -0.049}, {0.0026, 0.061, 0.0026}, 6, BodyKind::Machine);
    box({ 0.037, columnY, -0.049}, {0.0026, 0.061, 0.0026}, 6, BodyKind::Machine);
    box({0, 0.111, -0.049}, {0.042, 0.0032, 0.0036}, 6, BodyKind::Machine);
    box({0, top + 0.0080, -0.001}, {0.0105, 0.0025, 0.0105}, 7, BodyKind::Machine);

    return out;
}

FrameDiagnostics Simulator::diagnostics(double time,
                                        double top,
                                        bool loading,
                                        const Objective& o,
                                        const OptStats& stats) const {
    FrameDiagnostics d;
    d.time = time;
    d.platenY = top;
    d.displacement = top0_ - top;
    d.engineeringStrain = d.displacement / std::max(1e-12, height_);
    d.reactionForce = std::max(0.0, o.d.reaction - reactionOffset_);
    d.engineeringStress = d.reactionForce / std::max(1e-12, area_);
    d.loadingWork = loadingWork_;
    d.recoveredWork = recoveredWork_;
    d.dissipatedWork = std::max(0.0, loadingWork_ - recoveredWork_);
    d.specificEnergyAbsorption = d.dissipatedWork / std::max(1e-12, mass_);
    d.elasticEnergy = o.d.elastic;
    d.plasticDissipation = o.d.plastic;
    d.contactEnergy = o.d.contact;
    d.residualRms = stats.rms;
    d.residualMax = stats.max;
    d.maxPenetration = o.d.maxPen;
    d.activeContacts = o.d.contacts;
    d.iterations = stats.iterations;
    d.converged = stats.converged;
    d.loading = loading;

    double sumAxial = 0.0;
    double maxAxial = 0.0;
    double sumRot = 0.0;
    double maxRot = 0.0;
    int plasticBeams = 0;
    int plasticHinges = 0;
    for (const Beam& e : beams_) {
        const double pa = std::abs(e.plastic.epsp);
        const double pr = length(e.plastic.rotp);
        sumAxial += pa;
        maxAxial = std::max(maxAxial, pa);
        sumRot += pr;
        maxRot = std::max(maxRot, pr);
        if (pa > 1e-5) ++plasticBeams;
        if (pr > 1e-4) ++plasticHinges;
    }
    if (!beams_.empty()) {
        d.meanPlasticStrain = sumAxial / beams_.size();
        d.maxPlasticStrain = maxAxial;
        d.meanPlasticRotation = sumRot / beams_.size();
        d.maxPlasticRotation = maxRot;
    }
    d.plasticBeams = plasticBeams;
    d.plasticHinges = plasticHinges;

    double ymin = INF, ymax = -INF;
    for (const Node& n : nodes_) {
        ymin = std::min(ymin, n.x.y);
        ymax = std::max(ymax, n.x.y);
    }
    const double currentHeight = std::max(0.0, ymax - ymin);
    d.permanentSet = std::max(0.0, 1.0 - currentHeight / std::max(1e-12, height_));
    return d;
}

SimulationResult Simulator::run() {
    SimulationResult result;
    result.frames.reserve(size_t(cfg_.loadSteps + cfg_.unloadSteps + 24));
    result.diagnostics.reserve(result.frames.capacity());

    loadingWork_ = 0.0;
    recoveredWork_ = 0.0;
    prevDisp_ = 0.0;
    prevForce_ = 0.0;
    reactionOffset_ = 0.0;
    double time = 0.0;
    bool first = true;
    double currentTop = top0_;
    int acceptedSteps = 0;

    std::function<void(double,bool,int)> advanceTo;
    advanceTo = [&](double targetTop, bool loading, int refinement) {
        std::vector<Vec3> previousX(nodes_.size());
        std::vector<Quat> previousR(nodes_.size());
        for (size_t i = 0; i < nodes_.size(); ++i) {
            previousX[i] = nodes_[i].x;
            previousR[i] = nodes_[i].R;
        }

        std::vector<double> z = zeroVars();
        Objective finalObjective;
        OptStats stats = minimize(targetTop, z, finalObjective);
        const bool acceptable = stats.converged &&
            finalObjective.d.maxPen <= cfg_.penetrationTolerance &&
            std::isfinite(finalObjective.energy);

        if (!acceptable && refinement < cfg_.maxAdaptiveRefinements &&
            std::abs(targetTop - currentTop) > 2.5e-5) {
            const double midpoint = 0.5 * (currentTop + targetTop);
            advanceTo(midpoint, loading, refinement + 1);
            advanceTo(targetTop, loading, refinement + 1);
            return;
        }
        if (!acceptable) {
            std::ostringstream message;
            message << "quasi-static equilibrium failed at platen=" << targetTop
                    << " m, residual=" << stats.rms
                    << ", penetration=" << finalObjective.d.maxPen * 1000.0
                    << " mm after " << stats.iterations << " iterations";
            throw std::runtime_error(message.str());
        }

        commit(z, finalObjective);
        if (first) reactionOffset_ = finalObjective.d.reaction;
        const double correctedForce = std::max(0.0, finalObjective.d.reaction - reactionOffset_);
        const double displacement = top0_ - targetTop;
        if (!first) {
            const double dd = displacement - prevDisp_;
            const double work = 0.5 * (prevForce_ + correctedForce) * dd;
            if (dd >= 0.0) loadingWork_ += std::max(0.0, work);
            else recoveredWork_ += std::max(0.0, -work);
        }
        prevDisp_ = displacement;
        prevForce_ = correctedForce;
        currentTop = targetTop;

        result.frames.push_back(capture(targetTop,
            first ? nullptr : &previousX,
            first ? nullptr : &previousR));
        result.diagnostics.push_back(diagnostics(time, targetTop, loading,
                                                finalObjective, stats));
        const FrameDiagnostics& d = result.diagnostics.back();
        std::cout << "[equilibrium] " << std::setw(3) << acceptedSteps++
                  << " " << (loading ? "load  " : "unload")
                  << " strain=" << std::fixed << std::setprecision(4)
                  << d.engineeringStrain
                  << " force=" << std::setprecision(3) << d.reactionForce << " N"
                  << " residual=" << std::scientific << std::setprecision(2)
                  << d.residualRms
                  << " pen=" << std::fixed << std::setprecision(3)
                  << d.maxPenetration * 1000.0 << " mm"
                  << " iter=" << d.iterations
                  << (refinement ? " [REFINED]" : " [OK]") << '\n';
        first = false;
        time += cfg_.stepTime;
    };

    // Establish gravity/contact equilibrium and tare the load cell.
    advanceTo(top0_, true, 0);
    for (int step = 1; step <= cfg_.loadSteps; ++step) {
        double s = double(step) / std::max(1, cfg_.loadSteps);
        advanceTo(topY(s, true), true, 0);
    }
    for (int step = 1; step <= cfg_.unloadSteps; ++step) {
        double s = double(step) / std::max(1, cfg_.unloadSteps);
        advanceTo(topY(s, false), false, 0);
    }
    return result;
}
double Simulator::beamPotential(const std::vector<Vec3>& x,
                                const std::vector<Quat>& R) const {
    if (x.size() != nodes_.size() || R.size() != nodes_.size())
        throw std::runtime_error("beamPotential state size mismatch");
    const double E = cfg_.material.young;
    const double G = E / (2.0 * (1.0 + cfg_.material.nu));
    const double sy = cfg_.material.yield;
    const double H = cfg_.material.hardening;
    double total = 0.0;
    for (const Beam& e : beams_) {
        Vec3 d = x[size_t(e.b)] - x[size_t(e.a)];
        double L = std::max(1e-12, length(d));
        Vec3 t = d / L;
        double eps = (L - e.L0) / e.L0;
        auto ax = scalarReturn(eps, e.plastic.epsp, e.plastic.alpha,
                               E, sy, H);
        total += e.area * e.L0 * (ax.energy + ax.hard);

        Vec3 da = R[size_t(e.a)].rotate(e.e0);
        Vec3 db = R[size_t(e.b)].rotate(e.e0);
        double cs = 0.5 * cfg_.shearCorrection * G * e.area * e.L0;
        total += 0.5 * cs * (lengthSquared(da - t) + lengthSquared(db - t));

        Quat Q = normalized(R[size_t(e.a)].conjugate() * R[size_t(e.b)]);
        Vec3 phi = quatLog(Q);
        Vec3 axis = normalize(R[size_t(e.a)].inverseRotate(t));
        double tw = dot(phi, axis);
        double ptw = dot(e.plastic.rotp, axis);
        Vec3 bend = phi - tw * axis;
        Vec3 pb = e.plastic.rotp - ptw * axis;
        double kb = E * e.I / e.L0;
        double kt = G * e.J / e.L0;
        double my = sy * e.I / e.radius;
        double ty = (sy / std::sqrt(3.0)) * e.J / e.radius;
        auto br = vectorReturn(bend, pb, e.plastic.alphaB,
                               kb, my, cfg_.material.bendHardening * kb);
        auto tor = scalarReturn(tw, ptw, e.plastic.alphaT,
                                kt, ty, cfg_.material.torsionHardening * kt);
        total += br.energy + br.hard + tor.energy + tor.hard;
    }
    return total;
}

bool Simulator::rigidRotationTest(double* error) const {
    std::vector<Vec3> x(nodes_.size());
    std::vector<Quat> R(nodes_.size(), Quat{});
    for (size_t i = 0; i < nodes_.size(); ++i) {
        const Node& n = nodes_[i];
        double yf = saturate((n.x0.y - cfg_.jointScale * cfg_.maxRadius) /
                             std::max(1e-12, height_));
        x[i] = n.x0;
        x[i].y -= 0.018 * height_ * yf;
        x[i].x += 0.00018 * std::sin(2.0 * PI * yf + 0.17 * i);
        R[i] = quatAxisAngle({0, 0, 1}, 0.035 * yf);
    }
    const double e0 = beamPotential(x, R);
    Quat Q = quatAxisAngle(normalize(Vec3{0.37, 0.81, -0.44}), 1.137);
    Vec3 shift{0.081, -0.024, 0.053};
    std::vector<Vec3> xr(x.size());
    std::vector<Quat> Rr(R.size());
    for (size_t i = 0; i < x.size(); ++i) {
        xr[i] = Q.rotate(x[i]) + shift;
        Rr[i] = normalized(Q * R[i]);
    }
    const double e1 = beamPotential(xr, Rr);
    const double rel = std::abs(e1 - e0) / std::max({1e-12, std::abs(e0), std::abs(e1)});
    if (error) *error = rel;
    return rel < 1e-9;
}

bool Simulator::gradientCheck(double* relativeError) const {
    std::vector<double> z = zeroVars();
    RNG rng(cfg_.seed ^ 0x475241444348454bULL);
    for (size_t i = 0; i < nodes_.size(); ++i) {
        const size_t b = 6 * i;
        z[b + 0] = 0.003 * rng.signedUniform();
        z[b + 1] = 0.100 + 0.003 * rng.signedUniform(); // lift off lower platen
        z[b + 2] = 0.003 * rng.signedUniform();
        z[b + 3] = 0.004 * rng.signedUniform();
        z[b + 4] = 0.004 * rng.signedUniform();
        z[b + 5] = 0.004 * rng.signedUniform();
        for (int d = 0; d < 6; ++d) if (locked_[b + size_t(d)]) z[b + size_t(d)] = 0.0;
    }
    const double top = top0_ + 0.020;
    Objective a = objective(z, top, false);
    double numerator = 0.0, denominator = 0.0;
    int checked = 0;
    const int desired = 42;
    for (size_t k = 0; k < z.size() && checked < desired; ++k) {
        if (locked_[k]) continue;
        // Deterministic sparse coverage over both translational and rotational DOFs.
        if ((splitmix64(k * 0x9e3779b97f4a7c15ULL + cfg_.seed) & 15ULL) > 1ULL) continue;
        const double h = (k % 6 < 3) ? 2e-6 : 1e-6;
        std::vector<double> zp = z, zm = z;
        zp[k] += h;
        zm[k] -= h;
        double fp = objective(zp, top, false).energy;
        double fm = objective(zm, top, false).energy;
        double numeric = (fp - fm) / (2.0 * h);
        double analytic = a.grad[k];
        numerator += sq(numeric - analytic);
        denominator += sq(numeric) + sq(analytic);
        ++checked;
    }
    const double rel = std::sqrt(numerator / std::max(1e-30, denominator));
    if (relativeError) *relativeError = rel;
    // Rotation gradients use a corotational first-order tangent; a few percent
    // finite-difference discrepancy is acceptable away from the converged state.
    return checked >= 24 && rel < 0.075;
}

namespace {
struct Triangle { Vec3 a, b, c; };

inline double pointSegmentDistance(const Vec3& p, const Vec3& a, const Vec3& b) {
    Vec3 ab = b - a;
    double t = dot(p - a, ab) / std::max(1e-30, lengthSquared(ab));
    t = saturate(t);
    return length(p - (a + t * ab));
}

struct QuantizedVertex {
    int64_t x = 0, y = 0, z = 0;
    bool operator==(const QuantizedVertex&) const = default;
};
struct QuantizedVertexHash {
    size_t operator()(const QuantizedVertex& v) const noexcept {
        uint64_t h = splitmix64(uint64_t(v.x));
        h ^= splitmix64(uint64_t(v.y) + 0x9e3779b97f4a7c15ULL);
        h ^= splitmix64(uint64_t(v.z) + 0xbf58476d1ce4e5b9ULL);
        return size_t(h);
    }
};
struct EdgeKey {
    QuantizedVertex a, b;
    bool operator==(const EdgeKey&) const = default;
};
struct EdgeKeyHash {
    size_t operator()(const EdgeKey& e) const noexcept {
        QuantizedVertexHash h;
        return h(e.a) ^ (h(e.b) * 0x9e3779b97f4a7c15ULL);
    }
};

QuantizedVertex quantizeVertex(const Vec3& p, double q = 1e-8) {
    return {int64_t(std::llround(p.x / q)),
            int64_t(std::llround(p.y / q)),
            int64_t(std::llround(p.z / q))};
}
bool qless(const QuantizedVertex& a, const QuantizedVertex& b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
}
EdgeKey edgeKey(const Vec3& a, const Vec3& b) {
    QuantizedVertex qa = quantizeVertex(a), qb = quantizeVertex(b);
    if (qless(qb, qa)) std::swap(qa, qb);
    return {qa, qb};
}

Vec3 interpolateIso(const Vec3& a, const Vec3& b, double va, double vb) {
    double den = va - vb;
    double t = std::abs(den) > 1e-30 ? va / den : 0.5;
    t = clamp(t, 0.0, 1.0);
    Vec3 p = a + t * (b - a);
    // Grid-edge interpolation is deterministic; explicit quantization removes
    // binary-rounding cracks in the emitted STL.
    const double q = 1e-8;
    p.x = std::round(p.x / q) * q;
    p.y = std::round(p.y / q) * q;
    p.z = std::round(p.z / q) * q;
    return p;
}


} // namespace

void Simulator::exportSTL(const std::filesystem::path& path,
                          int radialSegments) const {
    if (nodes_.empty() || beams_.empty()) throw std::runtime_error("cannot mesh empty lattice");
    std::filesystem::create_directories(path.parent_path());

    // The parameter is retained for CLI compatibility but now controls the
    // implicit-union voxel size, not independent cylinder tessellation.
    const double voxel = clamp(0.006 / std::max(8, radialSegments), 0.00030, 0.00072);
    const double jointRadius = cfg_.jointScale * cfg_.maxRadius;
    Vec3 lo(INF), hi(-INF);
    for (const Node& n : nodes_) {
        lo = minVec(lo, n.x0 - Vec3(jointRadius + 2.5 * voxel));
        hi = maxVec(hi, n.x0 + Vec3(jointRadius + 2.5 * voxel));
    }
    auto ceilCount = [&](double extent) { return int(std::ceil(extent / voxel)) + 1; };
    const int nx = ceilCount(hi.x - lo.x);
    const int ny = ceilCount(hi.y - lo.y);
    const int nz = ceilCount(hi.z - lo.z);
    if (int64_t(nx) * ny * nz > 12000000LL)
        throw std::runtime_error("implicit STL grid exceeds safety limit");
    hi = lo + Vec3{(nx - 1) * voxel, (ny - 1) * voxel, (nz - 1) * voxel};

    auto index = [=](int i, int j, int k) {
        return (size_t(j) * size_t(nz) + size_t(k)) * size_t(nx) + size_t(i);
    };
    std::vector<float> field(size_t(nx) * size_t(ny) * size_t(nz),
                             std::numeric_limits<float>::infinity());

    auto rasterSphere = [&](const Vec3& c, double r) {
        Vec3 b0 = c - Vec3(r + 1.8 * voxel), b1 = c + Vec3(r + 1.8 * voxel);
        int i0 = std::max(0, int(std::floor((b0.x - lo.x) / voxel)));
        int j0 = std::max(0, int(std::floor((b0.y - lo.y) / voxel)));
        int k0 = std::max(0, int(std::floor((b0.z - lo.z) / voxel)));
        int i1 = std::min(nx - 1, int(std::ceil((b1.x - lo.x) / voxel)));
        int j1 = std::min(ny - 1, int(std::ceil((b1.y - lo.y) / voxel)));
        int k1 = std::min(nz - 1, int(std::ceil((b1.z - lo.z) / voxel)));
        for (int j = j0; j <= j1; ++j) for (int k = k0; k <= k1; ++k)
            for (int i = i0; i <= i1; ++i) {
                Vec3 p = lo + Vec3{i * voxel, j * voxel, k * voxel};
                float sdf = float(length(p - c) - r);
                float& f = field[index(i, j, k)];
                if (sdf < f) f = sdf;
            }
    };
    auto rasterCapsule = [&](const Vec3& a, const Vec3& b, double r) {
        Vec3 b0 = minVec(a, b) - Vec3(r + 1.8 * voxel);
        Vec3 b1 = maxVec(a, b) + Vec3(r + 1.8 * voxel);
        int i0 = std::max(0, int(std::floor((b0.x - lo.x) / voxel)));
        int j0 = std::max(0, int(std::floor((b0.y - lo.y) / voxel)));
        int k0 = std::max(0, int(std::floor((b0.z - lo.z) / voxel)));
        int i1 = std::min(nx - 1, int(std::ceil((b1.x - lo.x) / voxel)));
        int j1 = std::min(ny - 1, int(std::ceil((b1.y - lo.y) / voxel)));
        int k1 = std::min(nz - 1, int(std::ceil((b1.z - lo.z) / voxel)));
        for (int j = j0; j <= j1; ++j) for (int k = k0; k <= k1; ++k)
            for (int i = i0; i <= i1; ++i) {
                Vec3 p = lo + Vec3{i * voxel, j * voxel, k * voxel};
                float sdf = float(pointSegmentDistance(p, a, b) - r);
                float& f = field[index(i, j, k)];
                if (sdf < f) f = sdf;
            }
    };

    for (const Beam& e : beams_)
        rasterCapsule(nodes_[size_t(e.a)].x0, nodes_[size_t(e.b)].x0, e.radius);
    for (const Node& n : nodes_) rasterSphere(n.x0, jointRadius);

    std::vector<Triangle> triangles;
    triangles.reserve(beams_.size() * 100);
    constexpr int cornerOffset[8][3] = {
        {0,0,0},{1,0,0},{1,1,0},{0,1,0},
        {0,0,1},{1,0,1},{1,1,1},{0,1,1}};
    for (int j = 0; j < ny - 1; ++j) {
        for (int k = 0; k < nz - 1; ++k) {
            for (int i = 0; i < nx - 1; ++i) {
                std::array<Vec3, 8> p;
                std::array<double, 8> v;
                int cubeIndex = 0;
                for (int c = 0; c < 8; ++c) {
                    int ii = i + cornerOffset[c][0];
                    int jj = j + cornerOffset[c][1];
                    int kk = k + cornerOffset[c][2];
                    p[size_t(c)] = lo + Vec3{ii * voxel, jj * voxel, kk * voxel};
                    v[size_t(c)] = field[index(ii, jj, kk)];
                    if (v[size_t(c)] < 0.0) cubeIndex |= (1 << c);
                }
                const signed char* row = mc::triTable[cubeIndex];
                if (row[0] < 0) continue;
                std::array<Vec3, 12> edgePoint;
                std::array<uint8_t, 12> edgeReady{};
                auto edgeVertex = [&](int edge) -> Vec3 {
                    if (!edgeReady[size_t(edge)]) {
                        int a = mc::edgeVertices[edge][0];
                        int b = mc::edgeVertices[edge][1];
                        edgePoint[size_t(edge)] = interpolateIso(
                            p[size_t(a)], p[size_t(b)], v[size_t(a)], v[size_t(b)]);
                        edgeReady[size_t(edge)] = 1;
                    }
                    return edgePoint[size_t(edge)];
                };
                for (int q = 0; q < 16 && row[q] >= 0; q += 3) {
                    if (q + 2 >= 16 || row[q + 1] < 0 || row[q + 2] < 0) break;
                    Triangle t{edgeVertex(row[q]), edgeVertex(row[q + 1]), edgeVertex(row[q + 2])};
                    triangles.push_back(t);
                }
            }
        }
    }

    // Remove zero-area triangles and enforce positive signed volume.
    triangles.erase(std::remove_if(triangles.begin(), triangles.end(), [](const Triangle& t) {
        return lengthSquared(cross(t.b - t.a, t.c - t.a)) < 1e-22;
    }), triangles.end());
    double signedVolume = 0.0;
    for (const Triangle& t : triangles) signedVolume += dot(t.a, cross(t.b, t.c)) / 6.0;
    if (signedVolume < 0.0) {
        for (Triangle& t : triangles) std::swap(t.b, t.c);
        signedVolume = -signedVolume;
    }

    // Classic marching cubes is face-consistent but has a small set of
    // ambiguous topology cases.  Detect every resulting boundary loop and
    // close only those sub-voxel loops before release.  This is a deterministic
    // topology-repair stage, followed by a strict edge-incidence audit.
    struct EdgeRecord { int count = 0; QuantizedVertex from, to; };
    size_t initialBoundaryEdges = 0;
    size_t repairedLoops = 0;
    auto dequantize = [](const QuantizedVertex& q) {
        constexpr double h = 1e-8;
        return Vec3{q.x * h, q.y * h, q.z * h};
    };
    for (int repairPass = 0; repairPass < 5; ++repairPass) {
        std::unordered_map<EdgeKey, EdgeRecord, EdgeKeyHash> records;
        records.reserve(triangles.size() * 2);
        auto registerEdge = [&](const Vec3& a, const Vec3& b) {
            QuantizedVertex qa = quantizeVertex(a), qb = quantizeVertex(b);
            EdgeKey key{qa, qb};
            if (qless(qb, qa)) key = {qb, qa};
            EdgeRecord& r = records[key];
            if (r.count == 0) { r.from = qa; r.to = qb; }
            ++r.count;
        };
        for (const Triangle& t : triangles) {
            registerEdge(t.a, t.b); registerEdge(t.b, t.c); registerEdge(t.c, t.a);
        }
        std::unordered_map<QuantizedVertex, QuantizedVertex, QuantizedVertexHash> nextBoundary;
        for (const auto& [key, r] : records) {
            (void)key;
            if (r.count == 1) nextBoundary[r.to] = r.from;
        }
        if (repairPass == 0) initialBoundaryEdges = nextBoundary.size();
        if (nextBoundary.empty()) break;
        size_t loopsThisPass = 0;
        std::unordered_set<QuantizedVertex, QuantizedVertexHash> used;
        for (const auto& [startVertex, ignored] : nextBoundary) {
            (void)ignored;
            if (used.contains(startVertex)) continue;
            std::vector<QuantizedVertex> loop;
            QuantizedVertex current = startVertex;
            for (size_t guard = 0; guard <= nextBoundary.size(); ++guard) {
                if (used.contains(current) && !(current == startVertex)) break;
                used.insert(current);
                loop.push_back(current);
                auto it = nextBoundary.find(current);
                if (it == nextBoundary.end()) break;
                current = it->second;
                if (current == startVertex) break;
            }
            if (loop.size() < 3 || !(current == startVertex)) continue;
            ++loopsThisPass;
            ++repairedLoops;
            std::vector<Vec3> pLoop;
            pLoop.reserve(loop.size());
            Vec3 centroid(0);
            for (const QuantizedVertex& q : loop) { Vec3 p = dequantize(q); pLoop.push_back(p); centroid += p; }
            centroid /= double(pLoop.size());
            centroid = dequantize(quantizeVertex(centroid));
            if (pLoop.size() == 3) triangles.push_back({pLoop[0], pLoop[1], pLoop[2]});
            else for (size_t i = 0; i < pLoop.size(); ++i)
                triangles.push_back({pLoop[i], pLoop[(i + 1) % pLoop.size()], centroid});
        }
        if (loopsThisPass == 0) break;
    }

    signedVolume = 0.0;
    for (const Triangle& t : triangles) signedVolume += dot(t.a, cross(t.b, t.c)) / 6.0;
    if (signedVolume < 0.0) {
        for (Triangle& t : triangles) std::swap(t.b, t.c);
        signedVolume = -signedVolume;
    }

    std::unordered_map<EdgeKey, int, EdgeKeyHash> edges;
    edges.reserve(triangles.size() * 2);
    for (const Triangle& t : triangles) {
        ++edges[edgeKey(t.a, t.b)];
        ++edges[edgeKey(t.b, t.c)];
        ++edges[edgeKey(t.c, t.a)];
    }
    size_t boundary = 0, nonmanifold = 0;
    for (const auto& [edge, count] : edges) {
        (void)edge;
        if (count == 1) ++boundary;
        else if (count != 2) ++nonmanifold;
    }

    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot write STL " + path.string());
    std::array<char, 80> header{};
    std::string title = "Metamaterial engineering v2 - watertight implicit union";
    std::copy(title.begin(), title.begin() + std::min(title.size(), header.size()), header.begin());
    out.write(header.data(), header.size());
    uint32_t triangleCount = uint32_t(triangles.size());
    out.write(reinterpret_cast<const char*>(&triangleCount), sizeof(triangleCount));
    auto writeFloat = [&](float x) { out.write(reinterpret_cast<const char*>(&x), sizeof(x)); };
    for (const Triangle& t : triangles) {
        Vec3 n = normalize(cross(t.b - t.a, t.c - t.a));
        for (double v : {n.x, n.y, n.z}) writeFloat(float(v));
        for (const Vec3& p : {t.a, t.b, t.c}) {
            writeFloat(float(p.x * 1000.0));
            writeFloat(float(p.y * 1000.0));
            writeFloat(float(p.z * 1000.0));
        }
        uint16_t attr = 0;
        out.write(reinterpret_cast<const char*>(&attr), sizeof(attr));
    }
    out.close();

    std::filesystem::path report = path;
    report.replace_extension(".validation.txt");
    std::ofstream validation(report);
    validation << "Implicit-union STL validation\n"
               << "voxel_mm=" << voxel * 1000.0 << '\n'
               << "grid=" << nx << 'x' << ny << 'x' << nz << '\n'
               << "triangles=" << triangles.size() << '\n'
               << "unique_edges=" << edges.size() << '\n'
               << "initial_ambiguous_boundary_edges=" << initialBoundaryEdges << '\n'
               << "repaired_boundary_loops=" << repairedLoops << '\n'
               << "boundary_edges=" << boundary << '\n'
               << "nonmanifold_edges=" << nonmanifold << '\n'
               << "signed_volume_mm3=" << std::abs(signedVolume) * 1e9 << '\n'
               << "watertight=" << ((boundary == 0 && nonmanifold == 0) ? "PASS" : "FAIL") << '\n';
    if (boundary != 0 || nonmanifold != 0)
        throw std::runtime_error("implicit STL failed watertight edge audit");

}

} // namespace meta
