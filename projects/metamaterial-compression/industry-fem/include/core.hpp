#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
namespace meta {
constexpr double PI=3.1415926535897932384626433832795;
constexpr double INF=std::numeric_limits<double>::infinity();
inline double clamp(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
inline double saturate(double x){return clamp(x,0.0,1.0);}
inline double lerp(double a,double b,double t){return a*(1.0-t)+b*t;}
inline double sq(double x){return x*x;}
inline double smoothstep(double a,double b,double x){if(std::abs(b-a)<1e-15)return x>=b?1.0:0.0;double t=saturate((x-a)/(b-a));return t*t*(3.0-2.0*t);}
inline double smootherstep(double a,double b,double x){if(std::abs(b-a)<1e-15)return x>=b?1.0:0.0;double t=saturate((x-a)/(b-a));return t*t*t*(t*(t*6.0-15.0)+10.0);}
template<class F>void parallelFor(int count,int requestedThreads,F&&fn){if(count<=0)return;int threads=requestedThreads>0?requestedThreads:int(std::thread::hardware_concurrency());threads=std::clamp(threads,1,count);if(threads==1){for(int i=0;i<count;++i)fn(i);return;}std::atomic<int>next{0};std::vector<std::thread>pool;pool.reserve(size_t(threads));for(int t=0;t<threads;++t)pool.emplace_back([&]{for(;;){int i=next.fetch_add(1,std::memory_order_relaxed);if(i>=count)break;fn(i);}});for(auto&t:pool)t.join();}
struct Vec3{double x=0,y=0,z=0;Vec3()=default;explicit Vec3(double s):x(s),y(s),z(s){}Vec3(double X,double Y,double Z):x(X),y(Y),z(Z){}double&operator[](int i){return(&x)[i];}const double&operator[](int i)const{return(&x)[i];}Vec3 operator-()const{return{-x,-y,-z};}Vec3&operator+=(const Vec3&b){x+=b.x;y+=b.y;z+=b.z;return*this;}Vec3&operator-=(const Vec3&b){x-=b.x;y-=b.y;z-=b.z;return*this;}Vec3&operator*=(double s){x*=s;y*=s;z*=s;return*this;}Vec3&operator/=(double s){return*this*=1.0/s;}};
inline Vec3 operator+(Vec3 a,const Vec3&b){return a+=b;}inline Vec3 operator-(Vec3 a,const Vec3&b){return a-=b;}inline Vec3 operator*(Vec3 a,double s){return a*=s;}inline Vec3 operator*(double s,Vec3 a){return a*=s;}inline Vec3 operator/(Vec3 a,double s){return a/=s;}inline Vec3 operator*(const Vec3&a,const Vec3&b){return{a.x*b.x,a.y*b.y,a.z*b.z};}inline Vec3 operator/(const Vec3&a,const Vec3&b){return{a.x/b.x,a.y/b.y,a.z/b.z};}
inline double dot(const Vec3&a,const Vec3&b){return a.x*b.x+a.y*b.y+a.z*b.z;}inline Vec3 cross(const Vec3&a,const Vec3&b){return{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}inline double lengthSquared(const Vec3&v){return dot(v,v);}inline double length(const Vec3&v){return std::sqrt(lengthSquared(v));}inline bool finite(double x){return std::isfinite(x);}inline bool finite(const Vec3&v){return finite(v.x)&&finite(v.y)&&finite(v.z);}inline Vec3 normalize(const Vec3&v){double l2=lengthSquared(v);return finite(l2)&&l2>1e-30?v/std::sqrt(l2):Vec3{0,1,0};}inline Vec3 absVec(const Vec3&v){return{std::abs(v.x),std::abs(v.y),std::abs(v.z)};}inline Vec3 minVec(const Vec3&a,const Vec3&b){return{std::min(a.x,b.x),std::min(a.y,b.y),std::min(a.z,b.z)};}inline Vec3 maxVec(const Vec3&a,const Vec3&b){return{std::max(a.x,b.x),std::max(a.y,b.y),std::max(a.z,b.z)};}inline Vec3 maxVec(const Vec3&a,double s){return{std::max(a.x,s),std::max(a.y,s),std::max(a.z,s)};}inline Vec3 clampVec(const Vec3&p,const Vec3&lo,const Vec3&hi){return{clamp(p.x,lo.x,hi.x),clamp(p.y,lo.y,hi.y),clamp(p.z,lo.z,hi.z)};}inline double maxComponent(const Vec3&v){return std::max(v.x,std::max(v.y,v.z));}inline double luminance(const Vec3&c){return.2126*c.x+.7152*c.y+.0722*c.z;}inline Vec3 mix(const Vec3&a,const Vec3&b,double t){return a*(1.0-t)+b*t;}inline Vec3 smoothLimit(const Vec3&value,double limit){double m=length(value);if(m<=1e-14||limit<=0)return Vec3(0);return value*((limit*std::tanh(m/limit))/m);}
struct Quat{double w=1,x=0,y=0,z=0;Quat()=default;Quat(double W,double X,double Y,double Z):w(W),x(X),y(Y),z(Z){}Quat conjugate()const{return{w,-x,-y,-z};}Quat&normalizeInPlace(){double n=std::sqrt(w*w+x*x+y*y+z*z);if(n<1e-30){w=1;x=y=z=0;}else{w/=n;x/=n;y/=n;z/=n;}return*this;}Vec3 rotate(const Vec3&v)const{Vec3 qv{x,y,z};Vec3 t=2.0*cross(qv,v);return v+w*t+cross(qv,t);}Vec3 inverseRotate(const Vec3&v)const{return conjugate().rotate(v);}};
inline Quat operator+(const Quat&a,const Quat&b){return{a.w+b.w,a.x+b.x,a.y+b.y,a.z+b.z};}inline Quat operator*(const Quat&a,const Quat&b){return{a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};}inline Quat operator*(const Quat&q,double s){return{q.w*s,q.x*s,q.y*s,q.z*s};}inline Quat operator*(double s,const Quat&q){return q*s;}inline double dot(const Quat&a,const Quat&b){return a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z;}inline Quat normalized(Quat q){return q.normalizeInPlace();}
inline Quat quatAxisAngle(Vec3 axis,double angle){axis=normalize(axis);double s=std::sin(.5*angle);return normalized({std::cos(.5*angle),axis.x*s,axis.y*s,axis.z*s});}
inline Quat quatFromAxes(const Vec3&x,const Vec3&y,const Vec3&z){double m00=x.x,m01=y.x,m02=z.x,m10=x.y,m11=y.y,m12=z.y,m20=x.z,m21=y.z,m22=z.z;Quat q;double tr=m00+m11+m22;if(tr>0){double s=std::sqrt(tr+1)*2;q.w=.25*s;q.x=(m21-m12)/s;q.y=(m02-m20)/s;q.z=(m10-m01)/s;}else if(m00>m11&&m00>m22){double s=std::sqrt(1+m00-m11-m22)*2;q.w=(m21-m12)/s;q.x=.25*s;q.y=(m01+m10)/s;q.z=(m02+m20)/s;}else if(m11>m22){double s=std::sqrt(1+m11-m00-m22)*2;q.w=(m02-m20)/s;q.x=(m01+m10)/s;q.y=.25*s;q.z=(m12+m21)/s;}else{double s=std::sqrt(1+m22-m00-m11)*2;q.w=(m10-m01)/s;q.x=(m02+m20)/s;q.y=(m12+m21)/s;q.z=.25*s;}return normalized(q);}
inline Quat nlerp(Quat a,Quat b,double t){if(dot(a,b)<0)b={-b.w,-b.x,-b.y,-b.z};return normalized(a*(1-t)+b*t);}inline Quat slerp(Quat a,Quat b,double t){double c=dot(a,b);if(c<0){b={-b.w,-b.x,-b.y,-b.z};c=-c;}if(c>.9995)return nlerp(a,b,t);double th=std::acos(clamp(c,-1,1)),s=std::sin(th);return normalized(a*(std::sin((1-t)*th)/s)+b*(std::sin(t*th)/s));}
inline Vec3 quatErrorVector(const Quat&current,const Quat&target){Quat e=normalized(target*current.conjugate());if(e.w<0)e={-e.w,-e.x,-e.y,-e.z};double vl=std::sqrt(e.x*e.x+e.y*e.y+e.z*e.z);if(vl<1e-12)return Vec3(0);double angle=2*std::atan2(vl,clamp(e.w,-1,1));return(angle/vl)*Vec3{e.x,e.y,e.z};}inline Quat integrateOrientation(const Quat&q,const Vec3&w,double dt){double s=length(w);return s<1e-12?q:normalized(quatAxisAngle(w/s,s*dt)*q);}
inline uint64_t splitmix64(uint64_t x){x+=0x9e3779b97f4a7c15ULL;x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL;x=(x^(x>>27))*0x94d049bb133111ebULL;return x^(x>>31);}inline uint32_t hash32(uint64_t x){return uint32_t(splitmix64(x)>>32);}struct RNG{uint64_t state;explicit RNG(uint64_t seed):state(splitmix64(seed)){}uint32_t u32(){state=splitmix64(state);return uint32_t(state>>32);}double uniform(){return(double(u32())+.5)/4294967296.0;}double signedUniform(){return 2*uniform()-1;}};
struct Int3Hash{size_t operator()(int64_t k)const noexcept{return size_t(splitmix64(uint64_t(k)));}};inline int64_t cellKey(int x,int y,int z){constexpr int64_t B=1LL<<20,M=(1LL<<21)-1;return((int64_t(x)+B)&M)|(((int64_t(y)+B)&M)<<21)|(((int64_t(z)+B)&M)<<42);}inline uint64_t pairKey(int a,int b){if(a>b)std::swap(a,b);return(uint64_t(uint32_t(a))<<32)|uint32_t(b);}inline Vec3 orthogonal(const Vec3&n){Vec3 h=std::abs(n.z)<.8?Vec3{0,0,1}:Vec3{0,1,0};return normalize(cross(h,n));}
}
