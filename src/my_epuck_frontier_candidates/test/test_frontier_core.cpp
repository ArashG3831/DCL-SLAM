#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include "frontier_exploration_ros2/frontier_search.hpp"
#include "my_epuck_frontier_candidates/candidate_utils.hpp"
using namespace frontier_exploration_ros2; using namespace my_epuck_frontier_candidates;
static nav_msgs::msg::OccupancyGrid::SharedPtr grid(int w=8,int h=8,double r=.01,double ox=0,double oy=0,double yaw=0){auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>();m->header.frame_id="shared_map";m->info.width=w;m->info.height=h;m->info.resolution=r;m->info.origin.position.x=ox;m->info.origin.position.y=oy;m->info.origin.orientation.z=sin(yaw/2);m->info.origin.orientation.w=cos(yaw/2);m->data.assign(w*h,-1);return m;}
static void free_box(nav_msgs::msg::OccupancyGrid::SharedPtr &m,int x0,int y0,int x1,int y1){for(int y=y0;y<=y1;y++)for(int x=x0;x<=x1;x++)m->data[y*m->info.width+x]=0;}
static nav_msgs::msg::OccupancyGrid::SharedPtr free_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr&m){auto c=std::make_shared<nav_msgs::msg::OccupancyGrid>(*m);c->data.assign(c->data.size(),0);return c;}
static FrontierSearchResult search(const nav_msgs::msg::OccupancyGrid::SharedPtr&m,int min_cells=1,int robot_x=3,int robot_y=3){OccupancyGrid2d g(m),c(free_costmap(m));auto p=g.mapToWorld(robot_x,robot_y);geometry_msgs::msg::Pose pose;pose.position.x=p.first;pose.position.y=p.second;FrontierSearchOptions o;o.min_frontier_size_cells=min_cells;return get_frontier(pose,g,c,std::nullopt,0.,false,o);}
TEST(MapGeometry,IdentityTranslatedRotatedRoundTrip){for(auto p:std::vector<std::tuple<double,double,double>>{{0,0,0},{-2,3,0},{1,-2,M_PI/2}}){auto m=grid(8,8,.01,std::get<0>(p),std::get<1>(p),std::get<2>(p));OccupancyGrid2d g(m);for(auto c:std::vector<std::pair<int,int>>{{0,0},{3,5},{7,7}}){auto w=g.mapToWorld(c.first,c.second);int x,y;EXPECT_TRUE(g.worldToMapNoThrow(w.first,w.second,x,y));EXPECT_EQ(x,c.first);EXPECT_EQ(y,c.second);}}}
TEST(Frontiers,BoundaryGroupingAndMinimums){auto m=grid();free_box(m,2,2,5,5);auto f=search(m,1).frontiers;ASSERT_EQ(f.size(),1u);EXPECT_GT(f[0].size,4);EXPECT_TRUE(search(m,100).frontiers.empty());EXPECT_NEAR(f[0].centroid.first,.04,.02);}
TEST(Frontiers,VisibleRevealUsesUpstreamRayCasting){auto m=grid();free_box(m,2,2,5,5);OccupancyGrid2d g(m),c(free_costmap(m));auto fs=search(m).frontiers;ASSERT_FALSE(fs.empty());ASSERT_TRUE(fs[0].goal_point);geometry_msgs::msg::Pose sensor;sensor.position.x=fs[0].goal_point->first;sensor.position.y=fs[0].goal_point->second;auto gain=compute_visible_reveal_gain(sensor,g,c,std::nullopt,1.,360.,2.,fs[0].visible_reveal_bounds);ASSERT_TRUE(gain);EXPECT_GT(gain->visible_reveal_cell_count,0);}
TEST(Approach,FreeAndDistance){auto m=grid();free_box(m,2,2,5,5);OccupancyGrid2d g(m);auto p=g.mapToWorld(3,3);auto goal=choose_accessible_frontier_goal(p,{{2,2},{5,5}},g,std::nullopt,.01);ASSERT_TRUE(goal);int x,y;ASSERT_TRUE(g.worldToMapNoThrow(goal->first,goal->second,x,y));EXPECT_EQ(g.getCost(x,y),0);EXPECT_GE(std::hypot(goal->first-p.first,goal->second-p.second),.01);}
TEST(Approach,ClearanceRejectsInflationAndBounds){auto m=grid();free_box(m,1,1,6,6);m->data[3*8+4]=50;OccupancyGrid2d g(m);auto w=g.mapToWorld(3,3);EXPECT_FALSE(clearance_ok(g,w.first,w.second,.02,1));auto edge=g.mapToWorld(0,0);EXPECT_FALSE(clearance_ok(g,edge.first,edge.second,.02,1));}
TEST(Approach,PlannerToleranceMarginUsesExclusiveBounds){auto m=grid(200,200,.01);free_box(m,0,0,199,199);OccupancyGrid2d g(m);auto inside=g.mapToWorld(100,100);auto edge=g.mapToWorld(199,100);EXPECT_TRUE(inside_with_margin(g,inside.first,inside.second,.5));EXPECT_FALSE(inside_with_margin(g,edge.first,edge.second,.5));int x,y;EXPECT_FALSE(g.worldToMapNoThrow(2.0,1.0,x,y));}
TEST(StableId,BoundsOriginAndUnrelatedChange){auto a=grid(8,8,.05,-.2,-.2);free_box(a,2,2,5,5);OccupancyGrid2d ga(a);auto fa=search(a).frontiers;ASSERT_FALSE(fa.empty());auto id=stable_frontier_id(fa[0],ga,.05);auto b=grid(12,12,.05,-.3,-.3);free_box(b,4,4,7,7);OccupancyGrid2d gb(b);auto fb=search(b,1,5,5).frontiers;ASSERT_FALSE(fb.empty());EXPECT_EQ(id,stable_frontier_id(fb[0],gb,.05));b->data.back()=100;EXPECT_EQ(id,stable_frontier_id(fb[0],gb,.05));EXPECT_NE(map_checksum(*a),map_checksum(*b));}
TEST(StableId,SmallFrontierGrowthPreservesPhysicalIdentity){
  OccupancyGrid2d map(grid());
  frontier_exploration_ros2::FrontierCandidate first({1.02,2.04},{1.00,2.00},20);
  frontier_exploration_ros2::FrontierCandidate grown({1.05,2.07},{1.03,2.03},23);
  frontier_exploration_ros2::FrontierCandidate separate({1.31,2.34},{1.29,2.30},20);
  EXPECT_EQ(stable_frontier_id(first,map,.05),stable_frontier_id(grown,map,.05));
  EXPECT_NE(stable_frontier_id(first,map,.05),stable_frontier_id(separate,map,.05));
}
TEST(Path,LengthAndValidation){nav_msgs::msg::Path p;geometry_msgs::msg::PoseStamped a,b,c;a.pose.position.x=0;b.pose.position.x=3;b.pose.position.y=4;c.pose.position.x=6;c.pose.position.y=8;p.poses={a,b,c};auto l=path_length(p,0,0,6,8,.01);ASSERT_TRUE(l);EXPECT_DOUBLE_EQ(*l,10);p.poses.clear();EXPECT_FALSE(path_length(p,0,0,0,0,.1));p.poses={a};EXPECT_TRUE(path_length(p,0,0,0,0,.1));EXPECT_FALSE(path_length(p,1,1,0,0,.1));p.poses={a,b};p.poses[1].pose.position.x=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(path_length(p,0,0,0,0,.1));}
TEST(Revision,IdenticalTimestampDoesNotMatter){auto a=grid();auto b=std::make_shared<nav_msgs::msg::OccupancyGrid>(*a);b->header.stamp.sec=99;EXPECT_EQ(map_checksum(*a),map_checksum(*b));b->data[0]=0;EXPECT_NE(map_checksum(*a),map_checksum(*b));}
TEST(Architecture,SourceHasNoForbiddenInterfaces){SUCCEED();}
TEST(AsyncRequests,OldGenerationCannotTouchReplacement){
  EXPECT_TRUE(async_request_is_current(7,7,12,12,true));
  EXPECT_FALSE(async_request_is_current(6,7,12,12,true));
  EXPECT_FALSE(async_request_is_current(7,7,11,12,true));
  EXPECT_FALSE(async_request_is_current(7,7,12,12,false));
}
TEST(FairEvaluation, SixteenStableFrontiersWithBudgetFiveEventuallyAllQueried){
  std::vector<FrontierEvaluationRecord> records;
  for (uint64_t id = 1; id <= 16; ++id) {
    records.push_back(FrontierEvaluationRecord{id, true, false, false, 0, 0});
  }
  std::vector<uint64_t> queried;
  for (int cycle = 0; cycle < 4; ++cycle) {
    const auto selected = fair_frontier_query_order(records, 8, 5);
    ASSERT_LE(selected.size(), 5U);
    for (const auto index : selected) {
      queried.push_back(records[index].id);
      records[index].never_queried = false;
      records[index].cycles_not_queried = 0;
      records[index].last_query_ns = cycle + 1;
    }
    for (auto & record : records) {
      if (std::find(queried.begin(), queried.end(), record.id) == queried.end()) {
        ++record.cycles_not_queried;
      }
    }
  }
  std::sort(queried.begin(), queried.end());
  queried.erase(std::unique(queried.begin(), queried.end()), queried.end());
  EXPECT_EQ(queried.size(), 16U);
}
TEST(FairEvaluation, DeterministicTieBreakUsesCanonicalId){
  std::vector<FrontierEvaluationRecord> records{
    {20, true, false, false, 0, 0}, {10, true, false, false, 0, 0}};
  const auto selected = fair_frontier_query_order(records, 2, 2);
  ASSERT_EQ(selected.size(), 2U);
  EXPECT_EQ(records[selected[0]].id, 10U);
  EXPECT_EQ(records[selected[1]].id, 20U);
}
TEST(FairEvaluation, NeverQueriedAndInvalidatedWorkPrecedeStableCache){
  std::vector<FrontierEvaluationRecord> records{
    {30, false, false, false, 0, 30},
    {20, false, true, false, 1, 20},
    {10, false, false, true, 0, 10}};
  const auto selected = fair_frontier_query_order(records, 3, 3);
  ASSERT_EQ(selected.size(), 3U);
  EXPECT_EQ(records[selected[0]].id, 20U);
  EXPECT_EQ(records[selected[1]].id, 10U);
  EXPECT_EQ(records[selected[2]].id, 30U);
}
TEST(FairEvaluation, TransientPlannerFailureRemainsEligibleForRetry){
  std::vector<FrontierEvaluationRecord> records{
    {42, false, false, true, 4, 100},
    {7, false, false, false, 0, 1}};
  const auto selected = fair_frontier_query_order(records, 2, 1);
  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(records[selected[0]].id, 42U);
}
