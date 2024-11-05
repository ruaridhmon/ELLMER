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
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the distribution.
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
 * ROS node that takes data from NetFT sensor and publishes it to a ROS topic,
 * with the ability to zero a specific axis.
 */

#include "ros/ros.h"
#include "netft_rdt_driver/netft_rdt_driver.h"
#include "netft_rdt_driver/ZeroAxis.h"  // Change to your package's path
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include <memory>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;

geometry_msgs::Wrench offsets;
bool setzero = false;
string zero_axis;
const int total_setzero_cnt = 100;
int setzero_cnt = 0;

bool zero(netft_rdt_driver::ZeroAxis::Request &req,
          netft_rdt_driver::ZeroAxis::Response &res) {
    setzero = true;
    zero_axis = req.axis; // Axis to be zeroed
    setzero_cnt = 0;      // Reset the counter for zeroing process

    // No need to reinitialize all offsets to zero here
    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "netft_node");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("zero_axis", zero);

    float pub_rate_hz;
    string address;
    string frame_id;

    po::options_description desc("Options");
    desc.add_options()
        ("help", "display help")
        ("rate", po::value<float>(&pub_rate_hz)->default_value(1000.0), "set publish rate (in hertz)")
        ("wrench", "publish older Wrench message type instead of WrenchStamped")
        ("address", po::value<string>(&address), "IP address of NetFT box")
        ("frame_id", po::value<string>(&frame_id)->default_value("link_ft"), "frame_id of FT sensor");

    po::positional_options_description p;
    p.add("address", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << endl;
        exit(EXIT_SUCCESS);
    }

    if (!vm.count("address")) {
        cout << desc << endl;
        cerr << "Please specify address of NetFT" << endl;
        exit(EXIT_FAILURE);
    }

    bool publish_wrench = false;
    if (vm.count("wrench")) {
        publish_wrench = true;
        ROS_WARN("Publishing NetFT data as geometry_msgs::Wrench is deprecated");
    }

    std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> netft(new netft_rdt_driver::NetFTRDTDriver(address, frame_id));
    ros::Publisher pub;
    if (publish_wrench) {
        pub = nh.advertise<geometry_msgs::Wrench>("netft_data", 100);
    } else {
        pub = nh.advertise<geometry_msgs::WrenchStamped>("netft_data", 100);
    }

    ros::Rate pub_rate(pub_rate_hz);
    geometry_msgs::WrenchStamped data;

    while (ros::ok()) {
        if (netft->waitForNewData()) {
            netft->getData(data);
            if (setzero) {
                if (setzero_cnt < total_setzero_cnt) {
                    if (zero_axis == "all" || zero_axis == "x") offsets.force.x += data.wrench.force.x;
                    if (zero_axis == "all" || zero_axis == "y") offsets.force.y += data.wrench.force.y;
                    if (zero_axis == "all" || zero_axis == "z") offsets.force.z += data.wrench.force.z;
                    if (zero_axis == "all" || zero_axis == "tx") offsets.torque.x += data.wrench.torque.x;
                    if (zero_axis == "all" || zero_axis == "ty") offsets.torque.y += data.wrench.torque.y;
                    if (zero_axis == "all" || zero_axis == "tz") offsets.torque.z += data.wrench.torque.z;
                    setzero_cnt++;
                } else {
                    float total = static_cast<float>(total_setzero_cnt);
                    if (zero_axis == "all" || zero_axis == "x") offsets.force.x /= total;
                    if (zero_axis == "all" || zero_axis == "y") offsets.force.y /= total;
                    if (zero_axis == "all" || zero_axis == "z") offsets.force.z /= total;
                    if (zero_axis == "all" || zero_axis == "tx") offsets.torque.x /= total;
                    if (zero_axis == "all" || zero_axis == "ty") offsets.torque.y /= total;
                    if (zero_axis == "all" || zero_axis == "tz") offsets.torque.z /= total;
                    setzero_cnt = 0;
                    setzero = false;
                }
            } else {
                // Apply the offsets to the wrench data
                data.wrench.force.x -= offsets.force.x;
                data.wrench.force.y -= offsets.force.y;
                data.wrench.force.z -= offsets.force.z;
                data.wrench.torque.x -= offsets.torque.x;
                data.wrench.torque.y -= offsets.torque.y;
                data.wrench.torque.z -= offsets.torque.z;

                if (publish_wrench) {
                    pub.publish(data.wrench);
                } else {
                    pub.publish(data);
                }
            }
        }

        ros::spinOnce();
        pub_rate.sleep();
    }

    return 0;
}