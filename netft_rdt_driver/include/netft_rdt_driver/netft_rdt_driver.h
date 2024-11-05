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

#ifndef NETFT_RDT_DRIVER
#define NETFT_RDT_DRIVER

#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <string>

#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "geometry_msgs/WrenchStamped.h"

namespace netft_rdt_driver
{

class NetFTRDTDriver
{
public:
  // Start receiving data from NetFT device
  NetFTRDTDriver(const std::string &address, const std::string &frame_id);

  ~NetFTRDTDriver();

  //! Get newest RDT data from netFT device
  void getData(geometry_msgs::WrenchStamped &data);

  //! Add device diagnostics status wrapper
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d);

  //! Wait for new NetFT data to arrive.  
  // Returns true if new data has arrived, false it function times out
  bool waitForNewData(void);

protected:
  void recvThreadFunc(void);

  //! Asks NetFT to start streaming data.
  void startStreaming(void);

  enum {RDT_PORT=49152};
  std::string address_;
  std::string frame_id_;

  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_;
  boost::mutex mutex_;
  boost::thread recv_thread_;
  boost::condition condition_;
  volatile bool stop_recv_thread_;
  //! True if recv loop is still running
  bool recv_thread_running_;
  //! Set if recv thread exited because of error
  std::string recv_thread_error_msg_; 

  //! Newest data received from netft device
  geometry_msgs::WrenchStamped new_data_;
  //! Count number of received <good> packets
  unsigned packet_count_;
  //! Count of lost RDT packets using RDT sequence number
  unsigned lost_packets_;
  //! Counts number of out-of-order (or duplicate) received packets
  unsigned out_of_order_count_;
  //! Incremental counter for wrench header
  unsigned seq_counter_;

  //! Scaling factor for converting raw force values from device into Newtons
  double force_scale_;
  //! Scaling factor for converting raw torque values into Newton*meters
  double torque_scale_;

  //! Packet count last time diagnostics thread published output
  unsigned diag_packet_count_;
  //! Last time diagnostics was published
  ros::Time last_diag_pub_time_;
  
  //! to keep track of out-of-order or duplicate packet
  uint32_t last_rdt_sequence_;
  //! to keep track of any error codes reported by netft
  uint32_t system_status_;
};


} // end namespace netft_rdt_driver


#endif // NETFT_RDT_DRIVER
