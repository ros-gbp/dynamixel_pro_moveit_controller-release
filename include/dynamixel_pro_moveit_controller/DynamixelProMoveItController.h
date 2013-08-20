/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef DYNAMIXEL_PRO_MOVEIT_CONTROLLER 
#define DYNAMIXEL_PRO_MOVEIT_CONTROLLER 

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/controller_manager/controller_manager.h>
#include <map>
#include <vector>
#include <boost/thread.hpp>

namespace DynamixelProMoveIt
{

class DynamixelProMoveItController : public moveit_controller_manager::MoveItControllerHandle 
{
public:
    DynamixelProMoveItController();
    virtual ~DynamixelProMoveItController();

    virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory);
    virtual bool cancelExecution();
    virtual bool waitForExecution(const ros::Duration &timeout); 
    virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus();

    std::vector<std::string>& getJoints();
private:
    int signum(double val);
    void manageTrajectoryExecution();

    boost::thread *worker;
    ros::AsyncSpinner spinner;
    
    volatile bool ready, running, clean;
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);


    std::vector<std::string> joints;
    boost::mutex state_mutex;
    std::vector<double> positions;

    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

    int curr_step;

    struct waypoint
    {
        double accel;
        double velocity;
        double position;
    };

    void execute_waypoint_p(std::string joint_name, waypoint wp,
        sensor_msgs::JointState &out);

    void execute_waypoint_v(std::string joint_name, waypoint wp,
        sensor_msgs::JointState &out);

    //TODO FINISHME!!!!!!!!!!
    bool waypoint_reached_p(int index, waypoint wp);
    bool waypoint_reached_v(int index, waypoint wp);



    typedef std::map<std::string, waypoint> waypoint_step ;
    std::vector< waypoint_step > waypoints;
};

}

#endif //DYNAMIXEL_PRO_MOVEIT_CONTROLLER 

