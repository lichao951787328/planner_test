#include <foot_step_planning_ljh.h>
#include <FootstepPlannerLJH/AStarFootstepPlanner.h>
#include <FootstepPlannerLJH/parameters.h>
#include <FootstepPlannerLJH/StepConstraints/StepConstraintCheck.h>
#include <FootstepPlannerLJH/StepConstraints/StepConstraintCheck.h>
#include <assert.h>
#include <glog/logging.h>
#include <iostream>
#define PAI 3.1415926
// param 参数分别为 goalX， goalY， goalyaw, point1.x ... point4.y
// clockwise_ 1为顺时针，0为逆时针
std::vector<std::pair<Eigen::Vector3d, bool> > foot_step_planning(std::vector<double> param_dis, int clockwise_, double start_x_)
{
    
    assert(param_dis.size() == 11 && "your entered param is wrong");
    assert(clockwise_ == 0 || clockwise_ == 1 && "points direct is wrong");
    CHECK(param_dis.size() == 11);
    CHECK(clockwise_ == 0 || clockwise_ == 1);
    std::cout<<"in function foot_step_planning:"<<std::endl;
    std::cout<<"goal pose: x "<<param_dis.at(0)<<" y "<<param_dis.at(1)<<" yaw "<<param_dis.at(2)<<std::endl;
    std::cout<<"4 points: "<<std::endl;
    std::cout<<"point 1: "<<param_dis.at(3)<<" "<<param_dis.at(4)<<std::endl;
    std::cout<<"point 2: "<<param_dis.at(5)<<" "<<param_dis.at(6)<<std::endl;
    std::cout<<"point 3: "<<param_dis.at(7)<<" "<<param_dis.at(8)<<std::endl;
    std::cout<<"point 4: "<<param_dis.at(9)<<" "<<param_dis.at(10)<<std::endl;
    ljh::path::footstep_planner::StepConstraintCheck checker;
    ljh::path::footstep_planner::LatticePoint latticepoint;
    ljh::path::footstep_planner::parameters param;

    latticepoint.setGridSizeXY(latticepoint, 0.01);
    latticepoint.setYawDivision(latticepoint, 72);

    param.SetEdgeCostDistance(param, 4.0);
    param.SetEdgeCostYaw(param, 4.0);
    param.SetEdgeCostStaticPerStep(param, 1.4);
    param.SetDebugFlag(param, false);
    param.SetMaxStepYaw(param, PAI/10);//PAI/11
    param.SetMinStepYaw(param, -PAI/10);//-p1/11        

    param.SetFinalTurnProximity(param, 0.3);
    param.SetGoalDistanceProximity(param, 0.04);
    param.SetGoalYawProximity(param, 4.0/180.0 * PAI);
    param.SetFootPolygonExtendedLength(param, 0.025);
    
    param.SetHWPOfWalkDistacne(param,1.3);
    param.SetHWPOfPathDistance(param,2.50);
    
    param.SetHWPOfFinalTurnDistacne(param,1.30);
    param.SetHWPOfFinalWalkDistacne(param,1.30);

    param.SetMaxStepLength(param, 0.14);//0.12
    param.SetMinStepLength(param,-0.14);//-0.12
    param.SetMaxStepWidth(param,0.22);//0.22
    param.SetMinStepWidth(param,0.16);
    param.SetMaxStepReach(param,sqrt((param.MaxStepWidth-param.MinStepWidth) * (param.MaxStepWidth-param.MinStepWidth) + param.MaxStepLength * param.MaxStepLength));

    std:: cout<< "gridSizeXY is "<<latticepoint.getGridSizeXY(latticepoint)<<std::endl;
    std:: cout<< "gridSizeYaw is "<<latticepoint.getGridSizeYaw(latticepoint)<<std::endl;

    std:: cout<< "EdgeCost Weight Distance is "<<param.getEdgeCostDistance(param)<<std::endl;
    std:: cout<< "EdgeCost Weight Yaw is "<<param.getEdgeCostYaw(param)<<std::endl; 
    // define the initial and final pose

    double startX = start_x_;
    double startY = 0.0;
    double startZ = 0.0;
    double startYaw = 0.0/180.0 * PAI;

    // double goalX = 0.8;
    // double goalY = 0.0;
    // double goalZ = 0.0;
    // double goalYaw = -60.0/180.0 * PAI;

    double goalX = param_dis.at(0);
    double goalY = param_dis.at(1);
    double goalZ = 0.0;
    double goalYaw = param_dis.at(2);

    ljh::mathlib::Pose2D<double> goalPose2D(goalX,goalY,goalYaw);
    ljh::mathlib::Pose3D<double> goalPose(goalX,goalY,goalZ,goalYaw,0.0,0.0);
    ljh::mathlib::Pose3D<double> startPose(startX,startY,startZ,startYaw,0.0,0.0);

    // double xFromGoalToStair = 0.16+0.005;
    // double xLenOfStair = 0.5;
    // double yLenOfStair = 0.5;

    // Point2D<double> p0(xFromGoalToStair,yLenOfStair/2);
    // Point2D<double> p1(xFromGoalToStair+xLenOfStair,yLenOfStair/2);
    // Point2D<double> p2(xFromGoalToStair+xLenOfStair,-yLenOfStair/2);
    // Point2D<double> p3(xFromGoalToStair,-yLenOfStair/2);

    Point2D<double> p0(param_dis.at(3), param_dis.at(4));
    Point2D<double> p1(param_dis.at(5), param_dis.at(6));
    Point2D<double> p2(param_dis.at(7), param_dis.at(8));
    Point2D<double> p3(param_dis.at(9), param_dis.at(10));

    std::vector<Point2D<double> > stairBuffer({p0,p1,p2,p3});
    // No Need of the transformation!!!!!!!!!!!
    // for(int i=0;i<4;i++)
    // {
    //     stairBuffer[i].setPoint2D(goalX + cos(goalYaw)*stairBuffer[i].getX() - sin(goalYaw)*stairBuffer[i].getY(),
    //                               goalY + sin(goalYaw)*stairBuffer[i].getX() + cos(goalYaw)*stairBuffer[i].getY());
    // }
    param.SetStairAlignMode(param,true);
    param.SetStairPolygon(param,stairBuffer,4,clockwise_);// 1 表示点为顺时针
    

    std::cout<<"initilize start! "<<std::endl;
    ljh::path::footstep_planner::AStarFootstepPlanner footstepPlanner;
    std::cout<<"initilize start 2! "<<std::endl;
    footstepPlanner.initialize(goalPose2D,goalPose,startPose);
    std::cout<<"initilize over! "<<std::endl;

    footstepPlanner.doAStarSearch();
    footstepPlanner.calFootstepSeries();
    auto Out = footstepPlanner.getOrCalAccurateFootstepSeries();
    std::cout<<"the size of the footsteps: "<<Out.size()<<std::endl;
    std::cout<< "First x y yaw"<<Out.at(0).getX() <<" "<<Out.at(0).getY() <<" "<<Out.at(0).getYaw()<<std::endl;
    std::vector<std::pair<Eigen::Vector3d, bool>> steps;
    for (auto & iter_footstep : Out)
    {
        Eigen::Vector3d foot_x_y_yaw(iter_footstep.getX(), iter_footstep.getY(), iter_footstep.getYaw());
        bool is_left = iter_footstep.getStepFlag() == 0 ? true : false;
        steps.emplace_back(std::make_pair(foot_x_y_yaw, is_left));
    }
    
    return steps;

}