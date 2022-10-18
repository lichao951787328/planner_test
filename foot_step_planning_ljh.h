/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <Eigen/Core>
#include <vector>

// param 参数分别为 goalX， goalY， goalyaw, point1.x ... point4.y
// clockwise_ 1为顺时针，0为逆时针
std::vector<std::pair<Eigen::Vector3d, bool> > foot_step_planning(std::vector<double> param_dis, int clockwise_, double start_x_);