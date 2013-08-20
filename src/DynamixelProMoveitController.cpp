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

#include "dynamixel_pro_moveit_controller/DynamixelProMoveItController.h"
#include <sensor_msgs/JointState.h>

using namespace DynamixelProMoveIt;

DynamixelProMoveItController::DynamixelProMoveItController()
 : MoveItControllerHandle("DynamixelProMoveItController")
 , spinner(1)
{
    ROS_ERROR("You are calling an unimplemented function %s", __PRETTY_FUNCTION__);
    ready = false;

    sub = nh.subscribe("joint_states", 1, &DynamixelProMoveItController::joint_state_callback, this);
    pub = nh.advertise<sensor_msgs::JointState>("joint_commands", 1000);

    ros::Rate r(10);// poll at 10hz
    while (!ready) //wait for our first jointstate message so we know what joints are available to us
    {
        ros::spinOnce();
        r.sleep();
    }

    running = true;
    curr_step = -1;
    worker = new boost::thread( boost::bind(&DynamixelProMoveItController::manageTrajectoryExecution, this));

    spinner.start();
}

DynamixelProMoveItController::~DynamixelProMoveItController()
{
    clean = false;
    running = false;

    while (!clean)
    {
        ros::Duration(0.1).sleep();
    }
}

void DynamixelProMoveItController::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    boost::mutex::scoped_lock lck(state_mutex);

    if (!ready) // do the init
    {
        for (int i = 0; i < msg->name.size(); i++)
            joints.push_back(msg->name[i]);
    }
    else
    {
        for (int i = 0; i < msg->name.size(); i++)
        {
            if (joints.size() != msg->name.size() || joints[i] != msg->name[i])
            {
                joints.clear();
                for (int i = 0; i < msg->name.size(); i++)
                    joints.push_back(msg->name[i]);
            }
        }
    }

    positions.clear();
    for (int i = 0; i < msg->position.size(); i++)
        positions.push_back(msg->position[i]);

    ready = true;

}

bool DynamixelProMoveItController::sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
{
    ROS_ERROR("You are calling %s", __PRETTY_FUNCTION__);

    if (curr_step != 0 && curr_step != waypoints.size())
        ROS_ERROR("YOU ARENT NOT DONE WITH THE PREVIOUS TRAJECTORY!");
    
    waypoints.clear();
    std::vector<std::string> names;
    for (int i = 0; i < trajectory.joint_trajectory.joint_names.size(); i++)
        names.push_back(trajectory.joint_trajectory.joint_names[i]);

    //iterate over points
    for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
    {
        waypoint_step mp;

        //iterate over joints
        for (int j = 0; j < names.size(); j++)
        {
            waypoint wp;
            wp.accel    = trajectory.joint_trajectory.points[i].accelerations[j];
            wp.velocity = trajectory.joint_trajectory.points[i].velocities[j];
            wp.position = trajectory.joint_trajectory.points[i].positions[j];
            mp[names[j]]= wp;
        }
        waypoints.push_back(mp);
    }

    curr_step = 0;

    return true;
}

bool DynamixelProMoveItController::cancelExecution()
{
    ROS_ERROR("You are calling an unimplemented function %s", __PRETTY_FUNCTION__);
    return true;
}

bool DynamixelProMoveItController::waitForExecution(const ros::Duration &timeout)
{
    timeout.sleep();
    ROS_ERROR("You are calling an unimplemented function %s", __PRETTY_FUNCTION__);
    return true;
}

moveit_controller_manager::ExecutionStatus DynamixelProMoveItController::getLastExecutionStatus()
{
    ROS_ERROR("You are calling an unimplemented function %s", __PRETTY_FUNCTION__);

    if (curr_step < 0)
        return moveit_controller_manager::ExecutionStatus::UNKNOWN;
    else if (curr_step < waypoints.size())
        return moveit_controller_manager::ExecutionStatus::RUNNING;
    else
        return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
}

std::vector<std::string>& DynamixelProMoveItController::getJoints()
{
    ROS_ERROR("You are calling an unimplemented function %s", __PRETTY_FUNCTION__);
    return joints;
}

int DynamixelProMoveItController::signum(double val)
{
    if (val > 0)
        return 1;
    else if (val < 0 )
        return -1;
    else 
        return 0; 

}

void DynamixelProMoveItController::execute_waypoint_p(std::string joint_name,
    waypoint wp, sensor_msgs::JointState &out)
{
    out.name.push_back(joint_name);
    out.position.push_back(wp.position);
}

void DynamixelProMoveItController::execute_waypoint_v(std::string joint_name,
    waypoint wp, sensor_msgs::JointState &out)
{
    out.name.push_back(joint_name);

    out.velocity.push_back(wp.velocity);
}

bool DynamixelProMoveItController::waypoint_reached_p(int index, waypoint wp)
{
    return fabs(positions[index] - wp.position) < 0.2; 
}

bool DynamixelProMoveItController::waypoint_reached_v(int index, waypoint wp)
{
    //when signum vel != signum(p_target - p_current) 
    //forex, say v > 0, p_c = 1, p_t = 2; p_t > p_c => p_t - p_c > 0 == signum v
    return signum(wp.position - positions[index]) != signum(wp.velocity);
}

void DynamixelProMoveItController::manageTrajectoryExecution()
{
    while (running)
    {
        if (curr_step == 0) //time to start a new trajectory
        {
            for (curr_step = 0; curr_step < waypoints.size(); curr_step++)
            {
                std::vector<bool> completions;
                waypoint_step &step = waypoints[curr_step];
                sensor_msgs::JointState command;
                
                for (waypoint_step::iterator iter = step.begin(); iter != step.end(); iter++)
                {
                   
                    //Sometimes you might want to move to the starting position
                    //first before starting the sequence 
                    //and at the end you want to maintain the end position
                    if (curr_step == 0 || curr_step >= waypoints.size() - 1)
                    {
                        command.name.push_back(iter->first);
                        command.position.push_back(iter->second.position);// go to the starting position
                    }
                    else
                    {
                        //works but motion is ugly
                        execute_waypoint_p(iter->first, iter->second, command);

                        //Needs work, but if tweaked should work better
                        //execute_waypoint_v(iter->first,iter->second,command);
                    }
                    completions.push_back(false);
                }

                //publish all the commands.
                pub.publish(command);

                //make sure that you're actually publishing the state messages
                //at 200 hz or faster. This is only waiting for state completion
                //but you might want to go faster anyways.
                ros::Rate rate(200);
                int numCompletionsLeft = completions.size();

                double *error = new double[completions.size()];
                double *last_error= new double[completions.size()];

                while (numCompletionsLeft > 0)
                {
                    //can't have the sleep inside the scoped lock
                    {
                        boost::mutex::scoped_lock lck(state_mutex);
                        for (int i = 0; i < completions.size(); i++)
                        {
                            //the below option should work better, but is
                            //untested, as I don't have a working arm right now
                            //if (!completions[i] && waypoint_reached_v(i,
                            //    step[joints[i]]))
                            if (!completions[i] && waypoint_reached_p(i,
                                step[joints[i]]))
                            {
                                completions[i] = true;
                                numCompletionsLeft--;
                            }
                        }
                    }
                    rate.sleep();
                }
                //ROS_INFO("Done with step %d/%d", curr_step, (int) waypoints.size() - 1);
            }
        }
        else
        {
            ros::Duration(0.02).sleep();
        }
    }
}

