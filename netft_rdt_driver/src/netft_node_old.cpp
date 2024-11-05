/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/** 
 * Simple stand-alone ROS node that takes data from NetFT sensor and
 * Publishes it ROS topic
 */

#include "ros/ros.h"
#include "netft_rdt_driver/netft_rdt_driver.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include <unistd.h>
#include <iostream>
#include <memory>
#include <boost/program_options.hpp>
#include <actionlib/server/simple_action_server.h>
#include <netft_rdt_driver/ZeroAction.h>

namespace po = boost::program_options;
using namespace std;

geometry_msgs::Wrench offsets;
bool setzero = false;
const int total_setzero_cnt = 100;
int setzero_cnt = 0;
bool accepted_goal = false;
netft_rdt_driver::ZeroResult result_;

bool zero(std_srvs::Empty::Request  &req,
          std_srvs::Empty::Response &res){
  setzero = true;
  
  return true;
}

void goal_cb(){
  setzero = true;
  setzero_cnt = 0;
  result_.success = true;
  accepted_goal = true;
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "netft_node");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("zero", zero);

  actionlib::SimpleActionServer<netft_rdt_driver::ZeroAction> actionserver(nh, "zero_driver", false);
  actionserver.registerGoalCallback(&goal_cb);
  result_.success = false;
  actionserver.start();

  float pub_rate_hz;
  string address;
  string frame_id;

  po::options_description desc("Options");
  desc.add_options()
    ("help", "display help")
    ("rate", po::value<float>(&pub_rate_hz)->default_value(1000.0), "set publish rate (in hertz)")
    ("wrench", "publish older Wrench message type instead of WrenchStamped")
    ("address", po::value<string>(&address), "IP address of NetFT box")
    ("frame_id", po::value<string>(&frame_id)->default_value("link_ft"), "frame_id of FT sensor")
    ;
     
  po::positional_options_description p;
  p.add("address",  1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << endl;
    //usage(progname);
    exit(EXIT_SUCCESS);
  }      

  if (!vm.count("address"))
  {
    cout << desc << endl;
    cerr << "Please specify address of NetFT" << endl;
    exit(EXIT_FAILURE);
  }

  bool publish_wrench = false;
  if (vm.count("wrench"))
  {
    publish_wrench = true;
    ROS_WARN("Publishing NetFT data as geometry_msgs::Wrench is deprecated");
  }

  std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> netft(new netft_rdt_driver::NetFTRDTDriver(address, frame_id));
  ros::Publisher pub;
  if (publish_wrench)
  {
    pub = nh.advertise<geometry_msgs::Wrench>("netft_data", 100);
  }
  else 
  {
    pub = nh.advertise<geometry_msgs::WrenchStamped>("netft_data", 100);
  }
  ros::Rate pub_rate(pub_rate_hz);
  geometry_msgs::WrenchStamped data;

  ros::Duration diag_pub_duration(1.0);
  ros::Publisher diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 2);
  diagnostic_msgs::DiagnosticArray diag_array;
  diag_array.status.reserve(1);
  diagnostic_updater::DiagnosticStatusWrapper diag_status;
  ros::Time last_diag_pub_time(ros::Time::now());

  while (ros::ok())
  {
    if (netft->waitForNewData())
    {
      netft->getData(data);
      if(setzero){
        printf("Calibrating %d / %d\n", setzero_cnt, total_setzero_cnt);
        if(accepted_goal){
          accepted_goal = false;
          actionserver.acceptNewGoal();
        }
        if(actionserver.isPreemptRequested()){
          actionserver.setPreempted();
          result_.success = false;
          setzero = false;
        }
        if(setzero_cnt == 0){
          offsets = data.wrench;
          setzero_cnt++;
        }
        else if(setzero_cnt < total_setzero_cnt){
          geometry_msgs::Wrench tmp_wrench = data.wrench;
          offsets.force.x += tmp_wrench.force.x;
          offsets.force.y += tmp_wrench.force.y;
          offsets.force.z += tmp_wrench.force.z;
          offsets.torque.x += tmp_wrench.torque.x;
          offsets.torque.y += tmp_wrench.torque.y;
          offsets.torque.z += tmp_wrench.torque.z;
          setzero_cnt++;
        }
        else{
          offsets.force.x /= total_setzero_cnt;
          offsets.force.y /= total_setzero_cnt;
          offsets.force.z /= total_setzero_cnt;
          offsets.torque.x /= total_setzero_cnt;
          offsets.torque.y /= total_setzero_cnt;
          offsets.torque.z /= total_setzero_cnt;
          setzero_cnt = 0;
          setzero = false;
          if(result_.success){
            actionserver.setSucceeded(result_);
          }
        }
      }
      else{
        data.wrench.force.x -= offsets.force.x;
        data.wrench.force.y -= offsets.force.y;
        data.wrench.force.z -= offsets.force.z;
        data.wrench.torque.x -= offsets.torque.x;
        data.wrench.torque.y -= offsets.torque.y;
        data.wrench.torque.z -= offsets.torque.z;
        if (publish_wrench) 
        {
          //geometry_msgs::Wrench(data.wrench);
          pub.publish(data.wrench);
        }
        else 
        {
          pub.publish(data);
        }
      }
    }
    
    ros::Time current_time(ros::Time::now());
    if ( (current_time - last_diag_pub_time) > diag_pub_duration )
    {
      diag_array.status.clear();
      netft->diagnostics(diag_status);
      diag_array.status.push_back(diag_status);
      diag_array.header.stamp = ros::Time::now();
      diag_pub.publish(diag_array);
      last_diag_pub_time = current_time;
    }
    
    ros::spinOnce();
    pub_rate.sleep();
  }
  
  return 0;
}
