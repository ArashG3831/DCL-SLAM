// Adapted from frontier-exploration-ros2 v1.6.0; project modifications described in THIRD_PARTY.md.
// Copyright 2026 Mert Güler. Licensed under Apache-2.0.
#include "frontier_exploration_ros2/frontier_search.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
namespace frontier_exploration_ros2 {
int OccupancyGrid2d::width() const{return int(map_->info.width);} int OccupancyGrid2d::height() const{return int(map_->info.height);}
bool OccupancyGrid2d::inside(int x,int y) const{return x>=0&&y>=0&&x<width()&&y<height();}
int OccupancyGrid2d::cost(int x,int y) const{return int(map_->data[size_t(y)*map_->info.width+size_t(x)]);}
double OccupancyGrid2d::resolution() const{return map_->info.resolution;}
static double yaw(const geometry_msgs::msg::Quaternion&q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
std::pair<double,double> OccupancyGrid2d::mapToWorld(int x,int y) const{
 const auto&o=map_->info.origin; double a=yaw(o.orientation),gx=(x+.5)*resolution(),gy=(y+.5)*resolution();
 return {o.position.x+std::cos(a)*gx-std::sin(a)*gy,o.position.y+std::sin(a)*gx+std::cos(a)*gy};}
bool OccupancyGrid2d::worldToMap(double wx,double wy,int&x,int&y) const{
 const auto&o=map_->info.origin; double a=yaw(o.orientation),dx=wx-o.position.x,dy=wy-o.position.y;
 double gx=std::cos(a)*dx+std::sin(a)*dy,gy=-std::sin(a)*dx+std::cos(a)*dy;
 x=int(std::floor(gx/resolution())); y=int(std::floor(gy/resolution())); return inside(x,y);}
void FrontierCache::reset(int w,int h){visited.assign(size_t(w*h),0);frontier.assign(size_t(w*h),0);}
static const int N4[4][2]={{1,0},{-1,0},{0,1},{0,-1}}; static const int N8[8][2]={{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
std::vector<FrontierCandidate> get_frontier(const OccupancyGrid2d&m,double rx,double ry,int free_t,int min_cells,bool diag,FrontierCache&c){
 c.reset(m.width(),m.height()); int sx,sy;if(!m.worldToMap(rx,ry,sx,sy))return{}; auto idx=[&](int x,int y){return size_t(y*m.width()+x);};
 std::queue<Cell> q; q.push({sx,sy});c.visited[idx(sx,sy)]=1;std::vector<FrontierCandidate> out;
 while(!q.empty()){Cell p=q.front();q.pop(); for(auto&d:N4){int x=p.x+d[0],y=p.y+d[1];if(!m.inside(x,y))continue;auto i=idx(x,y);
   if(m.cost(x,y)>=0&&m.cost(x,y)<=free_t&&!c.visited[i]){c.visited[i]=1;q.push({x,y});continue;}
   if(m.cost(x,y)!=-1||c.frontier[i])continue;bool adjacent=false;for(auto&e:N4){int ax=x+e[0],ay=y+e[1];if(m.inside(ax,ay)&&m.cost(ax,ay)>=0&&m.cost(ax,ay)<=free_t){adjacent=true;break;}}if(!adjacent)continue;
   FrontierCandidate f;std::queue<Cell> fq;fq.push({x,y});c.frontier[i]=1;while(!fq.empty()){Cell u=fq.front();fq.pop();f.cells.push_back(u);auto w=m.mapToWorld(u.x,u.y);f.centroid_x+=w.first;f.centroid_y+=w.second;
    int nn=diag?8:4;for(int k=0;k<nn;k++){auto&e=diag?N8[k]:N4[k];int vx=u.x+e[0],vy=u.y+e[1];if(!m.inside(vx,vy))continue;auto vi=idx(vx,vy);if(m.cost(vx,vy)!=-1||c.frontier[vi])continue;bool a=false;for(auto&z:N4){int ax=vx+z[0],ay=vy+z[1];if(m.inside(ax,ay)&&m.cost(ax,ay)>=0&&m.cost(ax,ay)<=free_t){a=true;break;}}if(a){c.frontier[vi]=1;fq.push({vx,vy});}}
   } if(int(f.cells.size())<min_cells)continue;f.centroid_x/=f.cells.size();f.centroid_y/=f.cells.size();f.min_x=f.min_y=std::numeric_limits<double>::infinity();f.max_x=f.max_y=-f.min_x;
   std::vector<uint8_t> seen(size_t(m.width()*m.height()),0);for(auto u:f.cells){auto w=m.mapToWorld(u.x,u.y);f.min_x=std::min(f.min_x,w.first);f.max_x=std::max(f.max_x,w.first);f.min_y=std::min(f.min_y,w.second);f.max_y=std::max(f.max_y,w.second);for(auto&e:N8){int ax=u.x+e[0],ay=u.y+e[1];if(m.inside(ax,ay)&&m.cost(ax,ay)>=0&&m.cost(ax,ay)<=free_t&&!seen[idx(ax,ay)]){seen[idx(ax,ay)]=1;f.approach_cells.push_back({ax,ay});}}}out.push_back(std::move(f));
  }}return out;}
std::optional<Cell> choose_accessible_frontier_goal(const FrontierCandidate&f,const OccupancyGrid2d&m,double rx,double ry,double min_d){std::optional<Cell>b;double bd=1e99;for(auto c:f.approach_cells){auto w=m.mapToWorld(c.x,c.y);if(std::hypot(w.first-rx,w.second-ry)<min_d)continue;double d=std::hypot(w.first-f.centroid_x,w.second-f.centroid_y);if(d<bd){bd=d;b=c;}}return b;}
}
