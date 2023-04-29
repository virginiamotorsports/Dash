# include "fastdash.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
#define DEBUG false
// #define BAG (motec_msg.engine_rpm > 200)

ros2socketcan::ros2socketcan(std::string can_socket): Node("datalogger"), stream(ios), signals(ios, SIGINT, SIGTERM)
{
    printf("Using can socket %s\n", can_socket.c_str());
    int i = 0;
    i += gpioInitialise(); // this initializes the library. i dunno. it just does. 
	i += gpioSetMode(a_pin, PI_OUTPUT); // set the relevant pins to output mode. 
    i += gpioSetMode(b_pin,  PI_OUTPUT);
    i += gpioSetMode(c_pin, PI_OUTPUT);
    i += gpioSetMode(d_pin, PI_OUTPUT);
    i += gpioSetMode(e_pin, PI_OUTPUT);
    i += gpioSetMode(f_pin, PI_OUTPUT);
    i += gpioSetMode(g_pin, PI_OUTPUT);

    if(i != 0){
        printf("GPIO init failed.\n");
    }

    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    if ((this->homedir = getenv("HOME")) == NULL) {
        this->homedir = getpwuid(getuid())->pw_dir;
    }

    rclcpp::Time time_stamp = this->now();

    motec_msg.header = std_msgs::msg::Header();
    motec_msg.header.stamp = builtin_interfaces::msg::Time();
    motec_msg.header.stamp.sec = 32;

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
    printf("here");
    stream.assign(natsock);
    
    // std::cout << "ROS2 to CAN-Bus topic:" << subscription_->get_topic_name() 	<< std::endl;
    // std::cout << "CAN-Bus to ROS2 topic:" << publisher_->get_topic_name() 	<< std::endl;
    
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
    gpioTerminate();
//     if(BAG)
//         writer_->close();
}

void ros2socketcan::start_bag(){
    
    time_t currentTime = time(0);
    tm* currentDate = localtime(&currentTime);
    int year = currentDate->tm_year + 1900;
    int month = currentDate->tm_mon + 1;
    int day = currentDate->tm_mday;
    int hour = currentDate->tm_hour;
    int min = currentDate->tm_min;
    int sec = currentDate->tm_sec;
    std::stringstream ss;
    ss << this->homedir << "/bags/robag2_test_" << year << "_" << month << "_" << day << "_" << hour << "_" << min << "_" << sec;
    std::string currentDateString = ss.str();
    std::filesystem::path s = currentDateString;
    // Storage Options
    storage_options_.uri = s;
    storage_options_.storage_id = "sqlite3";
    // Converter Options
    converter_options_.input_serialization_format = "cdr";
    converter_options_.output_serialization_format = "cdr";

    writer_->open(storage_options_, converter_options_);
}

void ros2socketcan::write_to_bag(){
    rclcpp::Time time_stamp = this->now();

    // writer_->write(motec_msg, "chatter", "std_msgs/msg/String", time_stamp);
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
    if(DEBUG)
        printf("R | %x | ", rec_frame.can_id);
    for(int i=0; i<rec_frame.can_dlc; i++)
    {
         frame.data[i]=rec_frame.data[i];
         s << rec_frame.data[i];
    }
    current_frame = frame;
    std::cout << s.str() << " | ";
    if(DEBUG){
        for(int j=0;j<(int)rec_frame.can_dlc;j++)
        {
            printf("%i ", rec_frame.data[j]);
        }
        printf("\n");  
    }
    
    publisher_->publish(frame);
  
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&ros2socketcan::CanListener,this, std::ref(rec_frame),std::ref(stream)));
    
}


void update_7seg(int gear){
    switch(gear)
    {
        case 0:
        {
            gpioWrite(a_pin,1);
            gpioWrite(b_pin,1);
            gpioWrite(c_pin,1);
            gpioWrite(d_pin,1);
            gpioWrite(e_pin,1);
            gpioWrite(f_pin,1);
            gpioWrite(g_pin,0);
            break;
        }
        case 1:
        {
            gpioWrite(a_pin,0);
            gpioWrite(b_pin,1);
            gpioWrite(c_pin,1);
            gpioWrite(d_pin,0);
            gpioWrite(e_pin,0);
            gpioWrite(f_pin,0);
            gpioWrite(g_pin,0);
            break;
        }
        case 2:
        {
            gpioWrite(a_pin,1);
            gpioWrite(b_pin,1);
            gpioWrite(c_pin,0);
            gpioWrite(d_pin,1);
            gpioWrite(e_pin,1);
            gpioWrite(f_pin,0);
            gpioWrite(g_pin,1);
            break;
        }
        case 3:
        {
            gpioWrite(a_pin,1);
            gpioWrite(b_pin,1);
            gpioWrite(c_pin,1);
            gpioWrite(d_pin,1);
            gpioWrite(e_pin,0);
            gpioWrite(f_pin,0);
            gpioWrite(g_pin,1);
            break;
        }
        case 4:
        {
            gpioWrite(a_pin,0);
            gpioWrite(b_pin,1);
            gpioWrite(c_pin,1);
            gpioWrite(d_pin,0);
            gpioWrite(e_pin,0);
            gpioWrite(f_pin,1);
            gpioWrite(g_pin,1);
            break;
        }
        case 5:
        {
            gpioWrite(a_pin,1);
            gpioWrite(b_pin,1);
            gpioWrite(c_pin,1);
            gpioWrite(d_pin,1);
            gpioWrite(e_pin,1);
            gpioWrite(f_pin,1);
            gpioWrite(g_pin,1);
            break;
        }
        case 6:
        {
            gpioWrite(a_pin,1);
            gpioWrite(b_pin,0);
            gpioWrite(c_pin,1);
            gpioWrite(d_pin,1);
            gpioWrite(e_pin,1);
            gpioWrite(f_pin,1);
            gpioWrite(g_pin,0);
            break;
        }
        case 7:
        {
            gpioWrite(a_pin,1);
            gpioWrite(b_pin,1);
            gpioWrite(c_pin,1);
            gpioWrite(d_pin,0);
            gpioWrite(e_pin,0);
            gpioWrite(f_pin,0);
            gpioWrite(g_pin,0);
            break;
        }
        case 8:
        {
            gpioWrite(a_pin,1);
            gpioWrite(b_pin,1);
            gpioWrite(c_pin,1);
            gpioWrite(d_pin,1);
            gpioWrite(e_pin,1);
            gpioWrite(f_pin,1);
            gpioWrite(g_pin,1);
            break;
        }
        default:
        {
            gpioWrite(a_pin,1);
            gpioWrite(b_pin,1);
            gpioWrite(c_pin,1);
            gpioWrite(d_pin,1);
            gpioWrite(e_pin,1);
            gpioWrite(f_pin,1);
            gpioWrite(g_pin,0);
            break;
        }
    }
}

int get_gear(double Mph, double RPM){
    const double Tire_diameter = 0.521;
    const double Primary_Gear_Ratio = 2.073;
    const double Differential_Ratio = 3.370;

    double Gear_Trans_Ratio = (RPM * Tire_diameter * 6 * M_PI) / (Mph * Primary_Gear_Ratio * Differential_Ratio * 161);
    
    if (Gear_Trans_Ratio < 2.000){ // >= 2.583?
        return 1;
    } else if (Gear_Trans_Ratio < 1.667){ // >= 2.000?
        return 2;
    } else if (Gear_Trans_Ratio < 1.444){ // >= 1.667?
        return 3;
    } else if (Gear_Trans_Ratio < 1.286){ // >= 1.444?
        return 4;
    } else if (Gear_Trans_Ratio < 1.150){ // >= 1.286?
        return 5;
    } else if (Gear_Trans_Ratio < 1.0){
        return 6;
    } else
        return 0;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2socketcan>());
  rclcpp::shutdown();
  return 0;
}