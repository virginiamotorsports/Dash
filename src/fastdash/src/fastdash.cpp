# include "fastdash.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
#define DEBUG false
#define BRAKE "break_temps"
#define MOTEC "motec_report"
#define SUSP "suspension_data"


fastdash::fastdash(std::string can_socket): Node("datalogger"), stream(ios), signals(ios, SIGINT, SIGTERM)
{
    std::string s1 = "Using can socket " +  can_socket + "\n";
    RCLCPP_INFO(this->get_logger(), s1.c_str());
    // int i = 0;

    // i += gpioInitialise(); // this initializes the library. i dunno. it just does. 
    // if(i != 0){
    //     printf("GPIO init failed. %d\n", i);
    // }
	// i += gpioSetMode(a_pin, PI_OUTPUT); // set the relevant pins to output mode. 
    // i += gpioSetMode(b_pin,  PI_OUTPUT);
    // i += gpioSetMode(c_pin, PI_OUTPUT);
    // i += gpioSetMode(d_pin, PI_OUTPUT);
    // i += gpioSetMode(e_pin, PI_OUTPUT);
    // i += gpioSetMode(f_pin, PI_OUTPUT);
    // i += gpioSetMode(g_pin, PI_OUTPUT);

    

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    std::chrono::seconds bag_hyster(10);
    this->stop_timer = create_wall_timer(bag_hyster, std::bind(&fastdash::stop_bag, this));
    this->stop_timer->cancel();

    if ((this->homedir = getenv("HOME")) == NULL) {
        this->homedir = getpwuid(getuid())->pw_dir;
    }


    const char* canname = can_socket.c_str();
        
    topicname_receive 	<< "CAN/" << canname << "/" << "receive";
    topicname_transmit 	<< "CAN/" << canname << "/" << "transmit";
      
    rclcpp::executors::MultiThreadedExecutor exec;
    
    publisher_ 		= this->create_publisher<can_msgs::msg::Frame>(topicname_receive.str(), 1);
    // test_pub_ 		= this->create_publisher<can_msgs::msg::Frame>(topicname_transmit.str(), 1);
    // subscription_ 	= this->create_subscription<can_msgs::msg::Frame>(topicname_transmit.str(), std::bind(&fastdash::CanPublisher, this, _1));
    
    strcpy(ifr.ifr_name, can_socket.c_str());
    ioctl(natsock, SIOCGIFINDEX, &ifr);
    
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if(bind(natsock,(struct sockaddr *)&addr,sizeof(addr))<0)
    {
        perror("Error in socket bind");
    }
    stream.assign(natsock);

    // initalize_topics();
    start_bag();
    
    // std::cout << "ROS2 to CAN-Bus topic:" << subscription_->get_topic_name() 	<< std::endl;
    // std::cout << "CAN-Bus to ROS2 topic:" << publisher_->get_topic_name() 	<< std::endl;
    
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&fastdash::CanListener, this,std::ref(rec_frame),std::ref(stream)));
    
    signals.async_wait(std::bind(&fastdash::stop, this));
    
    boost::system::error_code ec;
    
    std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
    std::thread bt(std::bind(run, &ios));
    bt.detach();
}

void fastdash::stop()
{
    printf("\nEnd of Listener Thread. Please press strg+c again to stop the whole program.\n");
    stop_bag();
    ios.stop();
    signals.clear();
    gpioTerminate();
//     if(BAG)
//         writer_->close();
}

void fastdash::start_bag(){
    std::stringstream ss;
    ss << this->homedir << "/bags/rosbag2_recording_0";
    struct stat sb;
    int count = 1;
    std::string filename = ss.str();
    char c_filename[sizeof(filename)];
    for(int i = 0; i < (int)sizeof(filename); i++){
        c_filename[i] = filename[i];
    }

    while (stat(c_filename, &sb) == 0){
        c_filename[sizeof(c_filename) - 1] = '0' + count;
        count++;
    }
    std::filesystem::path s = c_filename;
    // Storage Options
    storage_options_.uri = s;
    storage_options_.storage_id = "sqlite3";
    // Converter Options
    converter_options_.input_serialization_format = "cdr";
    converter_options_.output_serialization_format = "cdr";

    writer_->open(storage_options_, converter_options_);
    writer_->create_topic({BRAKE, "dash_msgs/msg/BrakeTemp", rmw_get_serialization_format(),""});
    writer_->create_topic({MOTEC, "dash_msgs/msg/MotecReport", rmw_get_serialization_format(),""});
    writer_->create_topic({SUSP, "dash_msgs/msg/SuspensionReport", rmw_get_serialization_format(),""});
    curr_bag_state = true;

    // std::string s1 = "Starting bag at " + filename + "\n";
    // RCLCPP_INFO(this->get_logger(), s1.c_str());
}

void fastdash::stop_bag(){
    // writer_->close();
    stop_timer->cancel();
    curr_bag_state = false;

    std::string s1 = "Stopping bag\n";
    RCLCPP_INFO(this->get_logger(), s1.c_str());
}

// void fastdash::initalize_topics(){
//     rclcpp::Time time_stamp = this->now();

//     motec_msg.header = std_msgs::msg::Header();
//     motec_msg.header.stamp = time_stamp;

//     brake_msg.header = std_msgs::msg::Header();
//     brake_msg.header.stamp = time_stamp;
// }


fastdash::~fastdash(){printf("\nEnd of Publisher Thread. \n");}

// void fastdash::CanSend(const can_msgs::msg::Frame msg)
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
    
//     stream.async_write_some(boost::asio::buffer(&frame1, sizeof(frame1)),std::bind(&fastdash::CanSendConfirm, this));
// }

void fastdash::CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream)
{
    
    can_msgs::msg::Frame frame;
    
    frame.id = rec_frame.can_id; 
    frame.dlc = int(rec_frame.can_dlc);
    for(int i=0; i<rec_frame.can_dlc; i++)
    {
         frame.data[i]=rec_frame.data[i];
    }

    switch(frame.id){
        case(120):{
            motec_msg.engine_rpm = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0);
            if(motec_msg.engine_rpm > 800 && prev_bag_state == false){
                if(!this->stop_timer->is_canceled())
                    this->stop_timer->cancel();
                else{
                    start_bag();
                }
                prev_bag_state = true;
            }
            else if(motec_msg.engine_rpm < 600 && prev_bag_state == true){
                this->stop_timer->reset();
                prev_bag_state = false;
            }
            break;
        }
        case(0x4C4):{
            brake_msg.front_left[0] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.front_left[1] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.front_left[2] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.front_left[3] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4C5):{
            brake_msg.front_left[4] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.front_left[5] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.front_left[6] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.front_left[7] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4C6):{
            brake_msg.front_left[8] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.front_left[9] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.front_left[10] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.front_left[11] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4C7):{
            brake_msg.front_left[12] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.front_left[13] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.front_left[14] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.front_left[15] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4C8):{
            brake_msg.front_left_sensor_temp = (frame.data[0]);
            break;
        }
        case(0x4C9):{
            brake_msg.rear_left[0] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.rear_left[1] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.rear_left[2] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.rear_left[3] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4CA):{
            brake_msg.rear_left[4] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.rear_left[5] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.rear_left[6] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.rear_left[7] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4CB):{
            brake_msg.rear_left[8] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.rear_left[9] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.rear_left[10] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.rear_left[11] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4CC):{
            brake_msg.rear_left[12] = (((((short)frame.data[0]) << 8) | frame.data[1]) / 10.0 - 100.);
            brake_msg.rear_left[13] = (((((short)frame.data[2]) << 8) | frame.data[3]) / 10.0 - 100.);
            brake_msg.rear_left[14] = (((((short)frame.data[4]) << 8) | frame.data[5]) / 10.0 - 100.);
            brake_msg.rear_left[15] = (((((short)frame.data[6]) << 8) | frame.data[7]) / 10.0 - 100.);
            break;
        }
        case(0x4CD):{
            brake_msg.rear_left_sensor_temp = (frame.data[0]);
            // uint8_t* serialized_data = reinterpret_cast<uint8_t*>(brake_msg);
            if(curr_bag_state){
                writer_->write(brake_msg, BRAKE, now());
            }
            break;
        }
    }
    
    // publisher_->publish(frame);
  
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&fastdash::CanListener,this, std::ref(rec_frame),std::ref(stream)));
    
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
  rclcpp::spin(std::make_shared<fastdash>());
  rclcpp::shutdown();
  return 0;
}