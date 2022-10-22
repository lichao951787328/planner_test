/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <vector>
using namespace std;

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

int main(int argc, char** argv)
{
  ifstream ifile;
  ifile.open("/home/humanoid/turn_right_planner/planner_test/steps.txt");
  if (ifile.bad())
  {
    cout<<"error open file"<<endl;
  }
  int index = 1;
  vector<step_foot> steps;
  step_foot tmpstep;
  while (!ifile.eof())
  {
    switch (index%5)
    {
    case 1:
      ifile >> tmpstep.is_left;
      index++;
      break;
    
    case 2:
      ifile >> tmpstep.position.x();
      index++;
      break;

    case 3:
      ifile >> tmpstep.position.y();
      index++;
      break;
    case 4:
      ifile >> tmpstep.position.z();
      index++;
      break;

    case 0:
      ifile >> tmpstep.yaw;
      index++;
      steps.emplace_back(tmpstep);
      break;

    default:
      break;
    }
  }

  for (auto & step : steps)
  {
    cout<<step.is_left<<" "<<step.position.transpose()<<" "<<step.yaw<<endl;
  }
  
  
  return 0;
}