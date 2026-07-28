#include "my_epuck_frontier_candidates/candidate_utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <tuple>
namespace my_epuck_frontier_candidates {
uint64_t fnv1a64(const void*d,size_t n,uint64_t h){auto*p=static_cast<const uint8_t*>(d);for(size_t i=0;i<n;i++){h^=p[i];h*=1099511628211ULL;}return h;}
template<class T>static void add(uint64_t&h,const T&v){h=fnv1a64(&v,sizeof(v),h);}
uint64_t map_checksum(const nav_msgs::msg::OccupancyGrid&m){uint64_t h=14695981039346656037ULL;add(h,m.info.resolution);add(h,m.info.width);add(h,m.info.height);add(h,m.info.origin.position.x);add(h,m.info.origin.position.y);add(h,m.info.origin.position.z);add(h,m.info.origin.orientation.x);add(h,m.info.origin.orientation.y);add(h,m.info.origin.orientation.z);add(h,m.info.origin.orientation.w);if(!m.data.empty())h=fnv1a64(m.data.data(),m.data.size(),h);return h;}
uint64_t stable_frontier_id(const frontier_exploration_ros2::FrontierCandidate&f,const frontier_exploration_ros2::OccupancyGrid2d&m,double q){q=std::max(q,1e-6);std::vector<std::pair<int64_t,int64_t>> pts;pts.reserve(f.cells.size());for(auto c:f.cells){auto w=m.mapToWorld(c.x,c.y);pts.emplace_back(std::llround(w.first/q),std::llround(w.second/q));}std::sort(pts.begin(),pts.end());uint64_t h=14695981039346656037ULL;int64_t vals[]={std::llround(f.centroid_x/q),std::llround(f.centroid_y/q),std::llround(f.min_x/q),std::llround(f.min_y/q),std::llround(f.max_x/q),std::llround(f.max_y/q)};for(auto v:vals)add(h,v);size_t step=std::max<size_t>(1,pts.size()/16);for(size_t i=0;i<pts.size();i+=step){add(h,pts[i].first);add(h,pts[i].second);}add(h,pts.size());return h;}
std::optional<double> path_length(const nav_msgs::msg::Path&p,double rx,double ry,double gx,double gy,double tol){if(p.poses.empty())return{};for(auto&s:p.poses)if(!std::isfinite(s.pose.position.x)||!std::isfinite(s.pose.position.y))return{};auto&last=p.poses.back().pose.position;if(std::hypot(last.x-gx,last.y-gy)>tol)return{};if(p.poses.size()==1){if(std::hypot(rx-gx,ry-gy)<=tol)return 0.;return{};}double total=0;for(size_t i=1;i<p.poses.size();i++)total+=std::hypot(p.poses[i].pose.position.x-p.poses[i-1].pose.position.x,p.poses[i].pose.position.y-p.poses[i-1].pose.position.y);return total;}
bool clearance_ok(const frontier_exploration_ros2::OccupancyGrid2d&m,double wx,double wy,double clear,int threshold){int cx,cy;if(!m.worldToMap(wx,wy,cx,cy))return false;int r=int(std::ceil(clear/m.resolution()));for(int y=cy-r;y<=cy+r;y++)for(int x=cx-r;x<=cx+r;x++){if(!m.inside(x,y))return false;if(std::hypot((x-cx)*m.resolution(),(y-cy)*m.resolution())<=clear&&m.cost(x,y)>=threshold)return false;}return true;}
double normalized_value(double v,double lo,double hi){return hi-lo<1e-9?0.:((v-lo)/(hi-lo));}
}
