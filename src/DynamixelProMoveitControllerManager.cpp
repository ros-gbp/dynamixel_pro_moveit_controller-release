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
#include "dynamixel_pro_moveit_controller/DynamixelProMoveItControllerManager.h"
#include <pluginlib/class_list_macros.h>

using namespace DynamixelProMoveIt;

DynamixelProMoveItControllerManager::DynamixelProMoveItControllerManager()
{
    controller = new DynamixelProMoveItController();
    moveit_controller_manager::MoveItControllerHandlePtr tmp(controller);
    controllerPtr = tmp;
}

DynamixelProMoveItControllerManager::~DynamixelProMoveItControllerManager()
{
}

moveit_controller_manager::MoveItControllerHandlePtr DynamixelProMoveItControllerManager::getControllerHandle(const std::string &name)
{
    return controllerPtr;
}

void DynamixelProMoveItControllerManager::getControllersList(std::vector<std::string> &names)
{
    names.push_back("DynamixelProMoveItControllerManager");
}

void DynamixelProMoveItControllerManager::getActiveControllers(std::vector<std::string> &names)
{
    names.push_back("DynamixelProMoveItControllerManager");
}

void DynamixelProMoveItControllerManager::getControllerJoints(const std::string &name, std::vector<std::string> &joints)
{
    joints = controller->getJoints(); //this will copy over all the elements
}

//Not yet properly implemented 
moveit_controller_manager::MoveItControllerManager::ControllerState DynamixelProMoveItControllerManager::getControllerState(const std::string &name)
{
    moveit_controller_manager::MoveItControllerManager::ControllerState ret;
    ret.active_  = false;
    ret.default_ = false;
    return ret;
}

bool DynamixelProMoveItControllerManager::switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate)
{
    //there's not really much for us to do here
    return true;
}

PLUGINLIB_EXPORT_CLASS(DynamixelProMoveIt::DynamixelProMoveItControllerManager,
    moveit_controller_manager::MoveItControllerManager);

