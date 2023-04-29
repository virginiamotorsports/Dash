/** @file ros2socketcan.h
 *
 *  @ingroup ROS2CAN_bridge
 *  @author Philipp Wuestenberg
 *  @brief  bidirectional ROS2 to CAN interface with topics and service
 */

#ifndef __fastdash_H__
#define __fastdash_H__

//#include <termios.h> //n
//#include <fstream> //n
//#include <sstream>//n
//#include <chrono>//n
//#include <iostream>//n
//#include <stdio.h>
//#include <stdint.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <string.h>
//#include <net/if.h>
//#include <sys/types.h>
//#include <sys/socket.h>
//#include <sys/ioctl.h>


//define pin names because numbers are confusing
#define a_pin 17
#define b_pin 27
#define c_pin 22
#define d_pin 5
#define e_pin 6
#define f_pin 26
#define g_pin 23

//#include <linux/can.h>
#include <linux/can/raw.h>
#include <filesystem>
#include <ctime>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <sstream>
#include <pigpio.h>

#include <boost/asio.hpp>
//#include <boost/asio/io_service.hpp>
//#include <boost/asio/signal_set.hpp>
//#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
// #include "can_msgs/srv/can_request.hpp"
#include <std_msgs/msg/string.hpp>
#include "dash_msgs/msg/imu_in.hpp"
#include "dash_msgs/msg/motec_report.hpp"
#include "dash_msgs/msg/teensy_rear.hpp"
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>

// #include "log.h"

const std::string version = "1.00 from: " + std::string(__DATE__) + " " + std::string(__TIME__);
const std::string programdescr = "ROS 2 to CAN-Bus Bridge\nVersion: " + version;

/**
 * @brief The ros2socketcan bridge connects a canbus with the ROS2 topic system. 
 * @details A nodes is provided, which provides the bridge from a ROS topic to the CAN bus and from the CAN bus to a ROS topic. The node functions as a bidirectional bridge and provides a service to publish a message and receive the answer with the fitting message id. 
 * 
 */
class ros2socketcan : public rclcpp::Node
{
    public:
        /**
         * @brief constructor for ros2socketcan class
         * @details Within the constructor the topic and service naming is done. 
         */
        ros2socketcan(std::string can_socket = "can0");//boost::asio::io_service& ios);
        
        /**
         * @brief Within the Init() fucntin the ROS and CAN setup is done.
         * @details Within the Init() function the ROS2 publisher, subscriber and the service server is initialized. In addition the socketcan interface is configured and assigned to the socket. The Init function is necessary as the topics need a fully constructed node class to be added to.
         */
        // void Init(const char* can_socket = "can0");//boost::asio::io_service& ios);
        /**
         * @brief destructor
         */
        ~ros2socketcan();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        const char *homedir;
        dash_msgs::msg::ImuIn imu_msg;
        dash_msgs::msg::MotecReport motec_msg;
        dash_msgs::msg::TeensyRear teensy_msg;
        std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
        rosbag2_storage::StorageOptions storage_options_;
        rosbag2_cpp::ConverterOptions converter_options_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr test_pub_;
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;

        // rclcpp::service::Service<can_msgs::srv::CanRequest>::SharedPtr server_ros2can_;
        
        can_msgs::msg::Frame current_frame;
        
        /**
         * @brief The CanSendConfirm function is needed by the .async_write_some function and is called as confirmation for a successfull send process.
         */
        void CanSendConfirm();

        void start_bag();

        void end_bag();

        void update_7seg();
        
        /**
         * @brief The CanPublisher is listening to a ROS2 Topic and calls the CanSend Method.
         */
        void CanPublisher(const can_msgs::msg::Frame::SharedPtr msg);
        
        /**
         * @brief The CanSend method sends a ROS message to the CAN bus.
         * @details The CanSend function is Called by the CanPublisher and ther ros2can_srv. It converts the ROS message to a can_frame and adds the CAN Flags to the message ID.  
         */
        void CanSend(const can_msgs::msg::Frame msg);
        
        /**
         * @brief The CanListener listens to the CAN Bus and publishes the message to a ROS2 Topic.
         * @details The CanListener function is Called by the .async_read_some when a Message is received on the Can Socket. It converts the message to a ROS Message and publishes it to a ROS2 Topic. Afterwards .async_read_some must be called again to wait for further CAN Messages.
         */
        void CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream);
        
        /**
         * @brief The ros2can_service provides the possibility to send a can message and wait for a specific can message with a give CAN Message ID.
         */
        // void ros2can_srv(
        // const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        // const std::shared_ptr<can_msgs::srv::CanRequest::Request> request,
        // std::shared_ptr<can_msgs::srv::CanRequest::Response> response);
        
        /**
         * @biref The Stop method is needed as the interuped handler must be configered to the asio libary.
         */
        void stop();
        
        boost::asio::io_service ios;
        boost::asio::posix::basic_stream_descriptor<> stream;
        boost::asio::signal_set signals;

        struct sockaddr_can addr;
        struct can_frame frame;
        struct can_frame rec_frame;
        struct ifreq ifr;

        int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        std::stringstream topicname_receive;
        std::stringstream topicname_transmit;
        std::stringstream servername;
};
#endif