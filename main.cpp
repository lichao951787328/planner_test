/*
 * @description: 
 * @param : 
 * @return: 
 */


// goal pose: x 0.537 y -0.249 yaw -1.571
// 4 points: 
// point 1: 1.053 -0.343
// point 2: 0.074 -0.547
// point 3: 0.115 -0.743
// point 4: 1.094 -0.539
#include <foot_step_planning_ljh.h>
#include "matplotlibcpp.h"
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <array>
using namespace std;
namespace plt = matplotlibcpp;
double PI = 3.1415026;
array<Eigen::Vector2d, 4> generate_rect(double goal_x, double goal_y, double theta)
{
  Eigen::Vector3d goal(goal_x, goal_y, 0.);
  double rect_width = 1; double rect_depth = 0.3;
  Eigen::AngleAxisd rotation_vector(theta, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d goal_direct = rotation_vector.matrix() * Eigen::Vector3d::UnitX();
  Eigen::Matrix4d trans_matrix = Eigen::Matrix4d::Identity();
  trans_matrix.block<3,3>(0,0) = rotation_vector.matrix();
  trans_matrix.block<3,1>(0,3) = Eigen::Vector3d(goal_x, goal_y, 0);
  std::cout<<"transform :"<<trans_matrix<<endl;
  Eigen::Vector2d right_down, right_top, left_top, left_down;
  Eigen::AngleAxisd rotation_right(- PI/2.0, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d right_direct = rotation_right.matrix() * goal_direct;
  right_down = goal.head(2) +      (right_direct * rect_width/2.0).head(2);
  right_top = right_down + (goal_direct * rect_depth ).head(2);
  left_top = right_top -   (right_direct * rect_width).head(2);
  left_down = left_top -   (goal_direct * rect_depth ).head(2);
  array<Eigen::Vector2d, 4> rect;
  rect.at(0) = right_down;
  rect.at(1) = right_top;
  rect.at(2) = left_top;
  rect.at(3) = left_down;
  return rect;
}

array<Eigen::Vector3f, 4> computeStepMarker(Eigen::Vector3f step)
{
  array<Eigen::Vector3f, 4> Zero_step;
  Zero_step.at(0) = Eigen::Vector3f(0.15, 0.05, 0.0);
  Zero_step.at(1) = Eigen::Vector3f(0.15,  - 0.09, 0.0);
  Zero_step.at(2) = Eigen::Vector3f(- 0.09, - 0.09, 0.0);
  Zero_step.at(3) = Eigen::Vector3f(-0.09, 0.05, 0.0);

  Eigen::AngleAxisf r_v(step(2), Eigen::Vector3f(0,0,1));
  array<Eigen::Vector3f, 4> step_real;
  step_real.at(0) = r_v.matrix() * Zero_step.at(0) + Eigen::Vector3f(step.x(), step.y(), 0.0);
  step_real.at(1) = r_v.matrix() * Zero_step.at(1) + Eigen::Vector3f(step.x(), step.y(), 0.0);
  step_real.at(2) = r_v.matrix() * Zero_step.at(2) + Eigen::Vector3f(step.x(), step.y(), 0.0);
  step_real.at(3) = r_v.matrix() * Zero_step.at(3) + Eigen::Vector3f(step.x(), step.y(), 0.0);
  return step_real;
}

size_t step_index = 1;
void showSteps(vector<Eigen::Vector3f> steps)
{
  step_index = 1;
  for (auto & tmpstep : steps)
  {
    vector<float> x_, y_;
    cout<<tmpstep.transpose()<<endl;
    array<Eigen::Vector3f, 4> footCor = computeStepMarker(tmpstep);
    for (auto & point2d : footCor)
    {
      x_.emplace_back(point2d.x());
      y_.emplace_back(point2d.y());
      // cout<<"in for "<<point2d.transpose()<<endl;
    }
    x_.emplace_back(*x_.begin());
    y_.emplace_back(*y_.begin());
    plt::plot(x_, y_);
    plt::text(tmpstep.x(), tmpstep.y(), std::to_string(step_index));
    step_index++;
  }
  plt::set_aspect(1);
  plt::show();
}


int main(int argc, char** argv)
{
  // 假设终点
  double x = 1.2;
  double y = - 0.7;
  double theta =  - 35/(180/3.1415926);
  
  array<Eigen::Vector2d, 4> rect = generate_rect(x, y, theta);
  // vector<double> x_v, y_v;
  // for (auto & point : rect)
  // {
  //   x_v.emplace_back(point.x());
  //   y_v.emplace_back(point.y());
  // }
  // x_v.emplace_back(x_v.front());
  // y_v.emplace_back(y_v.front());
  // plt::plot(x_v, y_v);
  
  Eigen::AngleAxisd rotation_vector(theta, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d goal_direct = rotation_vector.matrix() * Eigen::Vector3d::UnitX();
  Eigen::Vector2d walk_goal = (goal_direct *(-0.16) + Eigen::Vector3d(x, y, 0)).head(2);

  Eigen::Vector2d end = walk_goal  + (goal_direct * 0.05).head(2);

  // vector<double> arrow_x, arrow_y;
  // arrow_x.emplace_back(walk_goal.x());arrow_x.emplace_back(end.x());
  // arrow_y.emplace_back(walk_goal.y());arrow_y.emplace_back(end.y());

  // plt::plot(arrow_x, arrow_y);
  // plt::set_aspect(1);
  // plt::show();
  vector<double> input;
  input.emplace_back(walk_goal.x());
  input.emplace_back(walk_goal.y());
  input.emplace_back(theta);
  for (auto & rect_point : rect)
  {
    input.emplace_back(rect_point.x());
    input.emplace_back(rect_point.y());
  }
  std::vector<std::pair<Eigen::Vector3d, bool> >  steps = foot_step_planning(input, 0, 0.015);
  cout<<"steps size "<<steps.size()<<endl;
  steps.pop_back();
  steps.pop_back();
  vector<Eigen::Vector3f> input_steps;
  for (auto & step : steps)
  {
    input_steps.emplace_back(Eigen::Vector3f(step.first.x(), step.first.y(), step.first.z()));
  }

  if (input_steps.size() % 2 == 1)// 补左脚
  {    
    float angle = PI/2 + input_steps.back().z();
    cout<<"angle = "<<angle<<endl;
    Eigen::AngleAxisf r_v(angle, Eigen::Vector3f::UnitZ());
    Eigen::Vector3f normal = r_v.matrix() * Eigen::Vector3f::UnitX();
    Eigen::Vector3f left_cor = 0.16 * normal + Eigen::Vector3f(input_steps.back().x(), input_steps.back().y(), 0.);
    input_steps.emplace_back(Eigen::Vector3f(left_cor(0), left_cor(1), input_steps.back().z()));
  }
  else// 补右脚
  {
    float angle = - PI/2 + input_steps.back().z();
    Eigen::AngleAxisf r_v(angle, Eigen::Vector3f::UnitZ());
    Eigen::Vector3f normal = r_v.matrix() * Eigen::Vector3f::UnitX();
    Eigen::Vector3f right_cor = 0.16 * normal + Eigen::Vector3f(input_steps.back().x(), input_steps.back().y(), 0.0);
    input_steps.emplace_back(Eigen::Vector3f(right_cor(0), right_cor(1), input_steps.back().z()));
  }

  showSteps(input_steps);

  return 0;
}