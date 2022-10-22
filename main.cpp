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
#include <fstream>
#include <array>
using namespace std;
namespace plt = matplotlibcpp;
float PI = 180/57.3;
array<Eigen::Vector2f, 4> generate_rect(float goal_x, float goal_y, float theta)
{
  Eigen::Vector3f goal(goal_x, goal_y, 0.);
  float rect_width = 1; float rect_depth = 0.3;
  Eigen::AngleAxisf rotation_vector(theta, Eigen::Vector3f::UnitZ());
  Eigen::Vector3f goal_direct = rotation_vector.matrix() * Eigen::Vector3f::UnitX();
  Eigen::Matrix4f trans_matrix = Eigen::Matrix4f::Identity();
  trans_matrix.block<3,3>(0,0) = rotation_vector.matrix();
  trans_matrix.block<3,1>(0,3) = Eigen::Vector3f(goal_x, goal_y, 0);
  std::cout<<"transform :"<<trans_matrix<<endl;
  Eigen::Vector2f right_down, right_top, left_top, left_down;
  Eigen::AngleAxisf rotation_right(- PI/2.0, Eigen::Vector3f::UnitZ());
  Eigen::Vector3f right_direct = rotation_right.matrix() * goal_direct;

  right_down = goal.head(2) + (right_direct * rect_width/2.0).head(2) + (goal_direct * (0.01 + 0.16)).head(2);
  right_top = right_down + (goal_direct * rect_depth ).head(2);
  left_top = right_top -   (right_direct * rect_width).head(2);
  left_down = left_top -   (goal_direct * rect_depth ).head(2);
  array<Eigen::Vector2f, 4> rect;
  rect.at(0) = right_down;
  rect.at(1) = right_top;
  rect.at(2) = left_top;
  rect.at(3) = left_down;
  return rect;
}

struct step_foot
{
  Eigen::Vector3f position;
  float yaw;
  bool is_left;
  step_foot(){}
  step_foot(Eigen::Vector3f & p, float yaw_, bool is_left_)
  {
    position = p; yaw = yaw_; is_left = is_left_;
  }
};

array<Eigen::Vector3f, 4> computeStepMarker(step_foot step)
{
  array<Eigen::Vector3f, 4> Zero_step;
  Zero_step.at(0) = Eigen::Vector3f(0.16, 0.05, 0.0);
  Zero_step.at(1) = Eigen::Vector3f(0.16,  - 0.09, 0.0);
  Zero_step.at(2) = Eigen::Vector3f(- 0.09, - 0.09, 0.0);
  Zero_step.at(3) = Eigen::Vector3f(-0.09, 0.05, 0.0);

  Eigen::AngleAxisf r_v(step.yaw, Eigen::Vector3f(0,0,1));
  array<Eigen::Vector3f, 4> step_real;
  step_real.at(0) = r_v.matrix() * Zero_step.at(0) + step.position;
  step_real.at(1) = r_v.matrix() * Zero_step.at(1) + step.position;
  step_real.at(2) = r_v.matrix() * Zero_step.at(2) + step.position;
  step_real.at(3) = r_v.matrix() * Zero_step.at(3) + step.position;
  return step_real;
}

size_t step_index = 1;
void showSteps(vector<step_foot> steps)
{
  step_index = 1;
  for (auto & tmpstep : steps)
  {
    vector<float> x_, y_;
    array<Eigen::Vector3f, 4> footCor = computeStepMarker(tmpstep);
    for (auto & point3d : footCor)
    {
      x_.emplace_back(point3d.x());
      y_.emplace_back(point3d.y());
    }
    x_.emplace_back(x_.front());
    y_.emplace_back(y_.front());
    plt::plot(x_, y_);
    plt::text(tmpstep.position.x(), tmpstep.position.y(), std::to_string(step_index));
    step_index++;
  }
  plt::set_aspect(1);
  plt::show();
}

struct stepClimbed
{
  float width, depth, height, yaw, center_x, center_y;
  array<Eigen::Vector3f, 4> rect_points;// anticlockwise

  stepClimbed(float width_p, float depth_p, float height_p)
  {
    width = width_p; depth = depth_p; height = height_p;
  }

  void initial_position(float x, float y, float angle)
  {
    center_x = x; center_y = y; yaw = angle;
    computeStepMarker();
  }
  void computeStepMarker()
  {
    array<Eigen::Vector3f, 4> Zero_step;
    Zero_step.at(0) = Eigen::Vector3f(- depth/2, - width/2, height);// right down
    Zero_step.at(1) = Eigen::Vector3f(depth/2, - width/2, height);// right top
    Zero_step.at(2) = Eigen::Vector3f(depth/2, width/2, height);// left top
    Zero_step.at(3) = Eigen::Vector3f(- depth/2, width/2, height);// left down
    Eigen::AngleAxisf r_v(yaw, Eigen::Vector3f(0,0,1));
    for (size_t i = 0; i < 4; i++)
    {
      rect_points.at(i) = r_v.matrix() * Zero_step.at(i) + Eigen::Vector3f(center_x, center_y, 0.0);
    }
  }
  void show()
  {
    vector<float> x_v, y_v;
    for (auto & point2d : rect_points)
    {
      x_v.emplace_back(point2d.x());
      y_v.emplace_back(point2d.y());
    }
    x_v.emplace_back(x_v.front());
    y_v.emplace_back(y_v.front());
    plt::plot(x_v, y_v);
    plt::set_aspect(1);
    // plt::show();
  }
};


int main(int argc, char** argv)
{
  // 台阶参数
  float step_width = 0.5; float step_depth = 0.35; float step_height = 0.04;
  // 第一组参数 先上台阶再转弯
  // Eigen::Vector3f step_center(0.4, -0.05, step_height);
  // float pyaw = - 5/57.3;

  // 第二段参数
  // Eigen::Vector3f step_center(0.55, -0.08, step_height);
  // float pyaw = - 8/57.3;

  Eigen::Vector3f step_center(0.55, -0.00, step_height);
  float pyaw = - 0/57.3;

 

  stepClimbed step_climbed(step_width, step_depth, step_height);
  step_climbed.initial_position(step_center.x(), step_center.y(), pyaw);
  step_climbed.show();

  // 规划第一段
  Eigen::Vector3f first_stage_goal = Eigen::Vector3f::Zero(); 
  Eigen::AngleAxisf first_stage_rotation(pyaw, Eigen::Vector3f::UnitZ());
  Eigen::Vector3f first_stage_direct = first_stage_rotation.matrix() * Eigen::Vector3f::UnitX();
  first_stage_goal.head(2) = (step_center - first_stage_direct*(0.16 + step_depth/2 + 0.02)).head(2);// 0.02 为防止误差导致与台阶碰撞的补偿量
  // 记录第一阶段的步态点
  vector<double> first_stage_input;
  first_stage_input.emplace_back(first_stage_goal.x());
  first_stage_input.emplace_back(first_stage_goal.y());
  first_stage_input.emplace_back(pyaw);
  for (auto & rect_point : step_climbed.rect_points)
  {
    first_stage_input.emplace_back(rect_point.x());
    first_stage_input.emplace_back(rect_point.y());
  }
  std::vector<std::pair<Eigen::Vector3d, bool> >  first_stage_steps = foot_step_planning(first_stage_input, 0, 0.015);
  vector<step_foot> first_stage_output_steps;
  for (auto & step : first_stage_steps)
  {
    Eigen::Vector3f position(step.first.x(), step.first.y(), 0.0);
    step_foot tmpstep(position, (float)step.first.z(), step.second);
    first_stage_output_steps.emplace_back(tmpstep);
  }

  // showSteps(first_stage_output_steps);

  // 规划台阶 台阶上落两步
  step_foot & last_step = first_stage_output_steps.back();
  Eigen::Vector3f last_step_direct = (Eigen::AngleAxisf(last_step.yaw, Eigen::Vector3f::UnitZ())).matrix() * Eigen::Vector3f::UnitX();
  last_step.position = last_step.position + last_step_direct * (0.25 + 0.02);//0.25 脚板长度
  last_step.position.z() = step_height;

  step_foot step_on_step;
  step_on_step.is_left = !first_stage_output_steps.back().is_left;
  step_on_step.yaw = first_stage_output_steps.back().yaw;
  Eigen::Vector3f broadwise_direct;
  if (step_on_step.is_left)//补左脚
  {
    broadwise_direct = Eigen::AngleAxisf(PI/2 + step_on_step.yaw, Eigen::Vector3f::UnitZ()).matrix() * Eigen::Vector3f::UnitX();
  }
  else
  {
    broadwise_direct = Eigen::AngleAxisf(- PI/2 + step_on_step.yaw, Eigen::Vector3f::UnitZ()).matrix() * Eigen::Vector3f::UnitX();
  }
  step_on_step.position = first_stage_output_steps.back().position + broadwise_direct * 0.16 + last_step_direct*(step_depth - 0.25);
  first_stage_output_steps.emplace_back(step_on_step);

  // 再补落地的第一脚
  step_foot step_on_ground;
  step_on_ground.is_left = !first_stage_output_steps.back().is_left;
  step_on_ground.yaw = first_stage_output_steps.back().yaw;
  step_on_ground.position = first_stage_output_steps.back().position - broadwise_direct * 0.16 + last_step_direct * (0.25 + 0.02);
  step_on_ground.position.z() = 0.0;
  first_stage_output_steps.emplace_back(step_on_ground);
  
  // 规划第二段
  Eigen::Vector3f second_stage_start = first_stage_output_steps.back().position + broadwise_direct * 0.08;
  second_stage_start.z() = 0.0;
  // 假设终点
  float x = 1.3;
  float y = - 0.3;
  float theta =  - 60/57.3;
  // 第一组参数
  // float x = 1.4;
  // float y = - 0.28;
  // float theta =  - 28/57.3;

  // 第二组参数
  // float x = 1.3;
  // float y = - 0.25;
  // float theta =  - 30/57.3;
  Eigen::Vector3f goal(x, y, 0.0);
  Eigen::Vector3f second_stage_goal = goal - second_stage_start;
  cout<<"second_stage_goal: "<<second_stage_goal.transpose()<<endl;
  cout<<"second theta "<<theta - pyaw<<endl;
  vector<double> second_stage_input;
  second_stage_input.emplace_back((double)second_stage_goal.x());
  second_stage_input.emplace_back((double)second_stage_goal.y());
  second_stage_input.emplace_back((double)(theta - pyaw));
  array<Eigen::Vector2f, 4> goal_rect = generate_rect(second_stage_goal.x(), second_stage_goal.y(), theta - pyaw);
  for (auto & rect_point : goal_rect)
  {
    second_stage_input.emplace_back((double)rect_point.x());
    second_stage_input.emplace_back((double)rect_point.y());
  }
  std::vector<std::pair<Eigen::Vector3d, bool> >  second_stage_steps = foot_step_planning(second_stage_input, 0, 0.0);
  // cout<<"...."<<endl;
  // cout<<second_stage_steps.at(0).first.transpose()<<endl;
  // cout<<second_stage_steps.at(1).first.transpose()<<endl;
  vector<step_foot> second_stage_output_steps;
  for (auto & step : second_stage_steps)
  {
    Eigen::Vector3f position(step.first.x(), step.first.y(), 0.0);
    step_foot tmpstep(position, (float)step.first.z(), step.second);
    second_stage_output_steps.emplace_back(tmpstep);
  }
  
  // showSteps(second_stage_output_steps);
  vector<step_foot> result_steps = first_stage_output_steps;
  second_stage_output_steps.erase(second_stage_output_steps.begin());
  if (first_stage_output_steps.back().is_left)
  {
    second_stage_output_steps.erase(second_stage_output_steps.begin());
  }
  Eigen::Matrix4f rotation;
  Eigen::AngleAxisf r2i(pyaw, Eigen::Vector3f::UnitZ());
  rotation.block<3,3>(0,0) = r2i.matrix();
  rotation.block<3,1>(0,3) = second_stage_start;
  
  for (auto & step : second_stage_output_steps)
  {
    step_foot tmpstep;
    Eigen::Vector4f qi; qi.head(3) = step.position; qi(3) = 1;
    tmpstep.position = (rotation * qi).head(3);
    tmpstep.is_left = step.is_left;
    tmpstep.yaw = step.yaw + pyaw;
    result_steps.emplace_back(tmpstep);
  }
  ofstream ofile;
  ofile.open("/home/humanoid/turn_right_planner/planner_test/steps.txt");
  for (auto & tmpstep : result_steps)
  {
    ofile<<tmpstep.is_left<<" "<<tmpstep.position.x()<<" "<<tmpstep.position.y()<<" "<<tmpstep.position.z()<<" "<<tmpstep.yaw<<endl;
  }
  // result_steps.pop_back();
  // result_steps.pop_back();
  showSteps(result_steps);
  return 0;
}