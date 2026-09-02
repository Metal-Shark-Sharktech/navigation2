//  Copyright 2020 Anshumaan Singh
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"
#include "nav2_theta_star_planner/theta_star_planner.hpp"
#include "nav2_theta_star_planner/parameter_handler.hpp"

/// class created to access the protected members of the ThetaStar class
/// u is used as shorthand for use
class test_theta_star : public nav2_theta_star_planner::ThetaStar
{
public:
  explicit test_theta_star(nav2_theta_star_planner::Parameters * params)
  : ThetaStar(params) {}
  int getSizeOfNodePosition()
  {
    return static_cast<int>(node_position_.size());
  }

  bool ulosCheck(const int & x0, const int & y0, const int & x1, const int & y1, double & sl_cost)
  {
    return losCheck(x0, y0, x1, y1, sl_cost);
  }

  bool uwithinLimits(const int & cx, const int & cy) {return withinLimits(cx, cy);}

  double ugetNormalizedCost(const int & cx, const int & cy) {return getNormalizedCost(cx, cy);}

  double ugetTraversalCost(const int & ax, const int & ay, const int & bx, const int & by)
  {
    return getTraversalCost(ax, ay, bx, by);
  }

  bool uisGoal(const tree_node & this_node) {return isGoal(this_node);}

  void uinitializePosn(int size_inc = 0)
  {
    node_position_.reserve(size_x_ * size_y_); initializePosn(size_inc);
  }

  void uaddIndex(const int & cx, const int & cy) {addIndex(cx, cy, &nodes_data_[0]);}

  tree_node * ugetIndex(const int & cx, const int & cy) {return getIndex(cx, cy);}

  tree_node * test_getIndex() {return &nodes_data_[0];}

  void uaddToNodesData(const int & id) {addToNodesData(id);}

  void uresetContainers() {nodes_data_.clear(); resetContainers();}

  bool runAlgo(
    std::vector<coordsW> & path,
    std::function<bool()> cancel_checker = [] () {return false;})
  {
    if (!isUnsafeToPlan()) {
      return generatePath(path, cancel_checker);
    }
    return false;
  }
};

// Tests meant to test the algorithm itself and its helper functions
TEST(ThetaStarTest, test_theta_star) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  auto planner_ = std::make_unique<test_theta_star>(params);
  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(50, 50, 1.0, 0.0, 0.0, 0);
  for (int i = 7; i <= 14; i++) {
    for (int j = 7; j <= 14; j++) {
      planner_->costmap_->setCost(i, j, 253);
    }
  }
  coordsM s = {5, 5}, g = {18, 18};
  int size_x = 20, size_y = 20;
  planner_->size_x_ = size_x;
  planner_->size_y_ = size_y;
  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = s.x;
  start.pose.position.y = s.y;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = g.x;
  goal.pose.position.y = g.y;
  goal.pose.orientation.w = 1.0;
  /// Check if the setStartAndGoal function works properly
  planner_->setStartAndGoal(start, goal);
  EXPECT_TRUE(planner_->src_.x == s.x && planner_->src_.y == s.y);
  EXPECT_TRUE(planner_->dst_.x == g.x && planner_->dst_.y == g.y);
  /// Check if the initializePosn function works properly
  planner_->uinitializePosn(size_x * size_y);
  EXPECT_EQ(planner_->getSizeOfNodePosition(), (size_x * size_y));

  /// Check if the withinLimits function works properly
  EXPECT_TRUE(planner_->uwithinLimits(18, 18));
  EXPECT_FALSE(planner_->uwithinLimits(120, 140));

  tree_node n = {g.x, g.y, 120, 0, NULL, false, 20};
  n.parent_id = &n;
  /// Check if the isGoal function works properly
  EXPECT_TRUE(planner_->uisGoal(n));           // both (x,y) are the goal coordinates
  n.x = 25;
  EXPECT_FALSE(planner_->uisGoal(n));          // only y coordinate matches with that of goal
  n.x = g.x;
  n.y = 20;
  EXPECT_FALSE(planner_->uisGoal(n));          // only x coordinate matches with that of goal
  n.x = 30;
  EXPECT_FALSE(planner_->uisGoal(n));          // both (x, y) are different from the goal coordinate

  /// Check if the isSafe functions work properly
  EXPECT_TRUE(planner_->isSafe(5, 5));         // cost at this point is 0
  EXPECT_FALSE(planner_->isSafe(10, 10));      // cost at this point is 253 (>LETHAL_COST)

  /// Check if the functions addIndex & getIndex work properly
  coordsM c = {18, 18};
  planner_->uaddToNodesData(0);
  planner_->uaddIndex(c.x, c.y);
  tree_node * c_node = planner_->ugetIndex(c.x, c.y);
  EXPECT_EQ(c_node, planner_->test_getIndex());

  double sl_cost = 0.0;
  /// Checking for the case where the losCheck should return the value as true
  EXPECT_TRUE(planner_->ulosCheck(2, 2, 7, 20, sl_cost));
  /// and as false
  EXPECT_FALSE(planner_->ulosCheck(2, 2, 18, 18, sl_cost));

  planner_->uresetContainers();
  std::vector<coordsW> path;
  /// Check if the planner returns a path for the case where a path exists
  EXPECT_TRUE(planner_->runAlgo(path));
  EXPECT_GT(static_cast<int>(path.size()), 0);
  /// and where it doesn't exist
  path.clear();
  planner_->src_ = {10, 10};
  EXPECT_FALSE(planner_->runAlgo(path));
  EXPECT_EQ(static_cast<int>(path.size()), 0);
}

// Smoke tests meant to detect issues arising from the plugin part rather than the algorithm
TEST(ThetaStarPlanner, test_theta_star_planner) {
  nav2::LifecycleNode::SharedPtr life_node =
    std::make_shared<nav2::LifecycleNode>("ThetaStarPlannerTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  geometry_msgs::msg::PoseStamped start, goal, viapoint;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal = start;
  viapoint = start;
  auto planner_2d = std::make_unique<nav2_theta_star_planner::ThetaStarPlanner>();
  planner_2d->configure(life_node, "test", nullptr, costmap_ros);
  planner_2d->activate();

  auto dummy_cancel_checker = []() {
      return false;
    };

  std::vector<geometry_msgs::msg::PoseStamped> viapoints{viapoint};
  nav_msgs::msg::Path path = planner_2d->createPlan(
    start, goal, viapoints, dummy_cancel_checker);
  EXPECT_GT(static_cast<int>(path.poses.size()), 0);

  // test if the goal is unsafe
  for (int i = 7; i <= 14; i++) {
    for (int j = 7; j <= 14; j++) {
      costmap_ros->getCostmap()->setCost(i, j, 254);
    }
  }
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;

  EXPECT_THROW(planner_2d->createPlan(start, goal, viapoints, dummy_cancel_checker),
    nav2_core::GoalOccupied);

  planner_2d->deactivate();
  planner_2d->cleanup();

  planner_2d.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  life_node.reset();
  costmap_ros.reset();
}

TEST(ThetaStarPlanner, test_theta_star_reconfigure)
{
  nav2::LifecycleNode::SharedPtr life_node =
    std::make_shared<nav2::LifecycleNode>("ThetaStarPlannerTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto planner = std::make_unique<nav2_theta_star_planner::ThetaStarPlanner>();
  try {
    // Expect to throw due to invalid prims file in param
    planner->configure(life_node, "test", nullptr, costmap_ros);
  } catch (...) {
  }
  planner->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    life_node->get_node_base_interface(), life_node->get_node_topics_interface(),
    life_node->get_node_graph_interface(),
    life_node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.how_many_corners", 8),
      rclcpp::Parameter("test.w_euc_cost", 1.0),
      rclcpp::Parameter("test.w_traversal_cost", 2.0),
      rclcpp::Parameter("test.use_final_approach_orientation", false),
      rclcpp::Parameter("test.allow_unknown", false),
      rclcpp::Parameter("test.terminal_checking_interval", 100)});

  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(),
    results);

  EXPECT_EQ(life_node->get_parameter("test.how_many_corners").as_int(), 8);
  EXPECT_EQ(
    life_node->get_parameter("test.w_euc_cost").as_double(),
    1.0);
  EXPECT_EQ(life_node->get_parameter("test.w_traversal_cost").as_double(), 2.0);
  EXPECT_EQ(life_node->get_parameter("test.use_final_approach_orientation").as_bool(), false);
  EXPECT_EQ(life_node->get_parameter("test.allow_unknown").as_bool(), false);
  EXPECT_EQ(life_node->get_parameter("test.terminal_checking_interval").as_int(), 100);

  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(),
    results);

  // Try setting invalid value for how_many_corners
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.how_many_corners", 5)});
  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(),
    results);
  EXPECT_EQ(life_node->get_parameter("test.how_many_corners").as_int(), 8);

  // Try setting invalid value for w_euc_cost
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.w_euc_cost", -1.0)});
  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(),
    results);
  EXPECT_EQ(life_node->get_parameter("test.w_euc_cost").as_double(), 1.0);
}

TEST(ThetaStarTest, test_unknown_cost_agrees_between_cost_sites) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarUnknownTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  auto planner_ = std::make_unique<test_theta_star>(params);

  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(2, 1, 1.0, 0.0, 0.0, UNKNOWN_COST);
  params->w_traversal_cost = 2.0;
  params->allow_unknown = true;

  // test if a line of sight check charges an unknown cell as an expansion step does
  double sl_cost = 0.0;
  ASSERT_TRUE(planner_->ulosCheck(0, 0, 1, 0, sl_cost));
  EXPECT_DOUBLE_EQ(sl_cost, planner_->ugetTraversalCost(0, 0, 1, 0));

  // test if that charge is the one for a near-obstacle cell
  planner_->costmap_->setCost(1, 0, OCCUPIED_COST - 1);
  EXPECT_DOUBLE_EQ(planner_->ugetNormalizedCost(0, 0), planner_->ugetNormalizedCost(1, 0));

  delete planner_->costmap_;
}

// Traversal cost is charged per unit distance, so equal-length lines cost the same at any bearing
TEST(ThetaStarTest, test_los_cost_is_direction_independent) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarDirectionTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  auto planner_ = std::make_unique<test_theta_star>(params);

  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(10, 10, 1.0, 0.0, 0.0, 100);
  params->w_traversal_cost = 2.0;

  const int len = 8;
  double axial = 0.0, diagonal = 0.0, oblique = 0.0;
  ASSERT_TRUE(planner_->ulosCheck(1, 1, 1 + len, 1, axial));
  ASSERT_TRUE(planner_->ulosCheck(1, 1, 1 + len, 1 + len, diagonal));
  // the staircase equals the chord at 0 and 45 degrees, so an oblique bearing is needed too
  ASSERT_TRUE(planner_->ulosCheck(1, 1, 1 + len, 1 + len / 2, oblique));

  EXPECT_NEAR(axial / len, diagonal / std::hypot(len, len), 1e-9);
  EXPECT_NEAR(axial / len, oblique / std::hypot(len, len / 2), 1e-9);

  delete planner_->costmap_;
}

/// Free space carries no traversal cost, so the traversal term contributes nothing to a line
/// that crosses only free cells, and the cost of such a line is charged solely by w_euc_cost.
/// The safety cutoff also becomes consistent: the line-of-sight check admits exactly the same
/// set of cells as isSafe, where the 26 + 0.9 remap made it stop a cost level earlier.
TEST(ThetaStarTest, test_free_space_carries_no_traversal_cost) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarFreeSpaceTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  auto planner_ = std::make_unique<test_theta_star>(params);

  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(10, 10, 1.0, 0.0, 0.0, 0);
  params->w_traversal_cost = 2.0;

  const int len = 8;
  double sl_cost = 0.0;
  ASSERT_TRUE(planner_->ulosCheck(1, 1, 1 + len, 1, sl_cost));
  EXPECT_DOUBLE_EQ(sl_cost, 0.0);

  // a uniform non-free cost is charged at its analytic density per unit distance
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      planner_->costmap_->setCost(i, j, 126);
    }
  }
  sl_cost = 0.0;
  ASSERT_TRUE(planner_->ulosCheck(1, 1, 1 + len, 1, sl_cost));
  EXPECT_NEAR(sl_cost / len, 2.0 * (126.0 / 252.0) * (126.0 / 252.0), 1e-9);

  // a cell at the highest non-obstacle cost is crossed by a line of sight, as isSafe admits it
  planner_->costmap_->setCost(5, 1, MAX_NON_OBSTACLE_COST);
  EXPECT_TRUE(planner_->isSafe(5, 1));
  sl_cost = 0.0;
  EXPECT_TRUE(planner_->ulosCheck(1, 1, 1 + len, 1, sl_cost));

  delete planner_->costmap_;
}

/// A step is charged by both of its ends, so it costs the same in either direction and its cost
/// scales with its length. Reading only the destination cell charged the whole step at one cell's
/// cost, which made the same pair of cells cost different amounts depending on which one the
/// search reached first.
TEST(ThetaStarTest, test_step_traversal_cost_is_symmetric) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarSymmetryTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  auto planner_ = std::make_unique<test_theta_star>(params);

  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(50, 50, 1.0, 0.0, 0.0, 100);
  params->w_traversal_cost = 2.0;

  /// A step between two cells of markedly different cost, where the endpoint convention shows.
  planner_->costmap_->setCost(11, 10, 200);
  const double forward = planner_->ugetTraversalCost(10, 10, 11, 10);
  EXPECT_DOUBLE_EQ(forward, planner_->ugetTraversalCost(11, 10, 10, 10));

  /// and it is the mean of the two cells' normalized costs, not either one of them alone.
  const double lo = planner_->ugetNormalizedCost(10, 10);
  const double hi = planner_->ugetNormalizedCost(11, 10);
  EXPECT_NEAR(forward, params->w_traversal_cost * 0.5 * (lo + hi), 1e-9);
  EXPECT_GT(forward, params->w_traversal_cost * lo);
  EXPECT_LT(forward, params->w_traversal_cost * hi);

  /// Over uniform cost the mean is that cost, charged over the length of the step.
  const double axial = planner_->ugetTraversalCost(20, 20, 21, 20);
  EXPECT_NEAR(axial, params->w_traversal_cost * planner_->ugetNormalizedCost(20, 20), 1e-9);
  EXPECT_NEAR(planner_->ugetTraversalCost(20, 20, 21, 21), M_SQRT2 * axial, 1e-9);

  delete planner_->costmap_;
}

/// The line-of-sight check charges both ends of every step it takes, so the far end of the line
/// is charged. Charging only the near end left the last cell of a line free, and disagreed with
/// the expansion step, which charged its destination.
TEST(ThetaStarTest, test_los_charges_both_ends_of_the_line) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarLosEndpointTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  auto planner_ = std::make_unique<test_theta_star>(params);

  /// Free everywhere but the far end of the line, so the far cell is the only thing to charge.
  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(50, 50, 1.0, 0.0, 0.0, 0);
  params->w_traversal_cost = 2.0;
  planner_->costmap_->setCost(9, 5, 200);

  double forward = 0.0;
  ASSERT_TRUE(planner_->ulosCheck(5, 5, 9, 5, forward));

  /// It is charged for its half of the last step, and for nothing else.
  EXPECT_NEAR(
    forward, params->w_traversal_cost * 0.5 * planner_->ugetNormalizedCost(9, 5), 1e-9);

  /// An axial line crosses the same cells in either direction, so it now costs the same either
  /// way; charging one end alone made it depend on which end the check started from.
  double backward = 0.0;
  ASSERT_TRUE(planner_->ulosCheck(9, 5, 5, 5, backward));
  EXPECT_NEAR(forward, backward, 1e-9);

  delete planner_->costmap_;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
