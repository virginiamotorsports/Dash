# include "fastdash.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

ros2socketcan::ros2socketcan(std::string can_socket): Node("datalogger"), stream(ios), signals(ios, SIGINT, SIGTERM)
{
    printf("Using can socket %s\n", can_socket.c_str());
    rclcpp::Time time_stamp = this->now();
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    std::string s = "data_logged_test";
    writer_->open(s);

    const char* canname = can_socket.c_str();
        
    topicname_receive 	<< "CAN/" << canname << "/" << "receive";
    topicname_transmit 	<< "CAN/" << canname << "/" << "transmit";
      
    rclcpp::executors::MultiThreadedExecutor exec;
    
    publisher_ 		= this->create_publisher<can_msgs::msg::Frame>(topicname_receive.str(), 1);
    // test_pub_ 		= this->create_publisher<can_msgs::msg::Frame>(topicname_transmit.str(), 1);
    // subscription_ 	= this->create_subscription<can_msgs::msg::Frame>(topicname_transmit.str(), std::bind(&ros2socketcan::CanPublisher, this, _1));
    
    strcpy(ifr.ifr_name, can_socket.c_str());
    ioctl(natsock, SIOCGIFINDEX, &ifr);
    
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if(bind(natsock,(struct sockaddr *)&addr,sizeof(addr))<0)
    {
        perror("Error in socket bind");
    }

    stream.assign(natsock);
    
    std::cout << "ROS2 to CAN-Bus topic:" << subscription_->get_topic_name() 	<< std::endl;
    std::cout << "CAN-Bus to ROS2 topic:" << publisher_->get_topic_name() 	<< std::endl;
    
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&ros2socketcan::CanListener, this,std::ref(rec_frame),std::ref(stream)));
    
    signals.async_wait(std::bind(&ros2socketcan::stop, this));
    
    boost::system::error_code ec;
    
    std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
    std::thread bt(std::bind(run, &ios));
    bt.detach();
}

void ros2socketcan::stop()
{
    printf("\nEnd of Listener Thread. Please press strg+c again to stop the whole program.\n");
    ios.stop();
    signals.clear();
}

ros2socketcan::~ros2socketcan(){printf("\nEnd of Publisher Thread. \n");}

// void ros2socketcan::CanSend(const can_msgs::msg::Frame msg)
// {
//     struct can_frame frame1;
    
//     frame1.can_id = msg.id;
    
//     if (msg.is_extended == 1)
//     {
//         frame1.can_id  = frame1.can_id + CAN_EFF_FLAG;
//     }
    
//     if (msg.is_error == 1)
//     {
//         frame1.can_id  = frame1.can_id + CAN_ERR_FLAG;
//     }
    
//     if (msg.is_rtr == 1)
//     {
//         frame1.can_id  = frame1.can_id + CAN_RTR_FLAG;
//     }
    
//     frame1.can_dlc = msg.dlc;

//     for(int i=0;i<(int)frame1.can_dlc;i++)
//     {
//         frame1.data[i] = msg.data[i];
//     }
     
//     printf("S | %x | %s | ", frame1.can_id, frame1.data);
//     for(int j=0;j<(int)frame1.can_dlc;j++)
//     {
//         printf("%i ", frame1.data[j]);
//     }
//     printf("\n");
    
//     stream.async_write_some(boost::asio::buffer(&frame1, sizeof(frame1)),std::bind(&ros2socketcan::CanSendConfirm, this));
// }


// void ros2socketcan::CanPublisher(const can_msgs::msg::Frame::SharedPtr msg)
// {

//     can_msgs::msg::Frame msg1;
//     msg1.id  = msg->id;
//     msg1.dlc = msg->dlc;
//     msg1.is_extended = msg->is_extended;
//     msg1.is_rtr = msg->is_rtr;
//     msg1.is_error = msg->is_error;
//     msg1.data= msg->data;
    
//     CanSend(msg1);
    
// }

// void ros2socketcan::CanSendConfirm(void)
// {
//     //std::cout << "Message sent" << std::endl;
// }

void ros2socketcan::CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream)
{
    
    can_msgs::msg::Frame frame;
    
    std::stringstream s;
    
    frame.id = rec_frame.can_id; 
    frame.dlc = int(rec_frame.can_dlc);
    
    printf("R | %x | ", rec_frame.can_id);
    for(int i=0; i<rec_frame.can_dlc; i++)
    {
         frame.data[i]=rec_frame.data[i];
         s << rec_frame.data[i];
    }
    current_frame = frame;
    std::cout << s.str() << " | ";
    
    for(int j=0;j<(int)rec_frame.can_dlc;j++)
    {
        printf("%i ", rec_frame.data[j]);
    }
    printf("\n");  
    
    publisher_->publish(frame);
  
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&ros2socketcan::CanListener,this, std::ref(rec_frame),std::ref(stream)));
    
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2socketcan>());
  rclcpp::shutdown();
  return 0;
}