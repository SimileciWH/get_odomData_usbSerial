#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "getodomdata_serial/mobileRobot_msgs.h"
#include "getodomdata_serial/mobileRobot_velocity.h"
#include "string.h"


serial::Serial ros_ser;

typedef union
{
    int si;
    char sc[4];
}typechange;

bool transferData(getodomdata_serial::mobileRobot_msgs &encoder_data, uint8_t *serial_data)
{
    static int left_encoder, right_encoder;
    typechange l_tmp_left, r_tmp_right;
    try
    {
        encoder_data.encoder_name1="left_encoder";
        l_tmp_left.sc[0] = *(serial_data+5);
        l_tmp_left.sc[1] = *(serial_data+6);
        l_tmp_left.sc[2] = *(serial_data+7);
        l_tmp_left.sc[3] = *(serial_data+8);
        left_encoder = l_tmp_left.si;
        std::cout<<"left_encoder: "<<left_encoder<<std::endl;
        encoder_data.left_enc = left_encoder;


        encoder_data.encoder_name2="right_encoder";
        r_tmp_right.sc[0] = *(serial_data+15);
        r_tmp_right.sc[1] = *(serial_data+16);
        r_tmp_right.sc[2] = *(serial_data+17);
        r_tmp_right.sc[3] = *(serial_data+18);
        right_encoder = r_tmp_right.si*(-1);
        std::cout<<"right_encoder: "<<right_encoder<<std::endl;
        encoder_data.right_enc = right_encoder;
        return true;
    }
    catch (double)
    {
        return false;
    }  
}

void readEncoder_serial_pub(uint8_t *lcmd_buffer, uint8_t *rcmd_buffer, getodomdata_serial::mobileRobot_msgs &encoder_data, uint8_t *serial_data, ros::Publisher enc_pub)
{
    // send lcmd && rcmd
    ros_ser.write(lcmd_buffer,10);
    ros_ser.write(rcmd_buffer,10);
    ROS_INFO_STREAM("Write to serial port");
    if(ros_ser.available())
    {
        ROS_INFO_STREAM("Reading from serial port");
        //获取串口数据
        if (ros_ser.available() == 20)
        {
        ros_ser.read(serial_data, 20);
        }
        if (transferData(encoder_data, serial_data) == true)
        //将串口数据发布到主题sensor
        enc_pub.publish(encoder_data);
        else
        ROS_INFO_STREAM("transfer data failure, cannot publish topic...");
    }
}

void vtarget_callback(getodomdata_serial::mobileRobot_velocity msg)
// subscribe left && right velocity and send to robot through serial port
{
    std::cout << "name1:" << msg.name1 << "---speed:" << msg.left_vtarget << std::endl;
    std::cout << "name2:" << msg.name2 << "---speed:" << msg.right_vtarget << std::endl;
    // here to send wheel_vtarget to mobile robot through serial port

    
    //  ros_ser.write(msg->data);
 }


int main (int argc, char** argv)
{
    // initial parameters
    std::string node_name;
    static uint8_t lcmd_buffer[10]={0x02, 0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb};
    static uint8_t rcmd_buffer[10]={0x01, 0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc};
    uint8_t serial_data[20];
    getodomdata_serial::mobileRobot_msgs encoder_data;
    


    // initial parameter service


     ros::init(argc, argv, "serial_node");
     node_name=ros::this_node::getName();
     ROS_INFO("%s is starting...", node_name.c_str());
     ros::NodeHandle n;

     //订阅主题command
     ros::Subscriber vtarget_sub = n.subscribe("/wheel_vtarget", 1000, vtarget_callback);
     //发布主题sensor
     ros::Publisher enc_pub = n.advertise<getodomdata_serial::mobileRobot_msgs>("encoder", 1000);

     try 
     {
         ros_ser.setPort("/dev/ttyUSB0");
         ros_ser.setBaudrate(115200);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_ser.setTimeout(to);
         ros_ser.open();
     }
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
     }
     if(ros_ser.isOpen())
     {
         ROS_INFO_STREAM("Serial Port opened");
     }
     else
     {
         return -1;
     }
     ros::Rate loop_rate(10);
     while(ros::ok())
     {
        ros::spinOnce();

        // get left && right encoder and publisher /encoder topic
        readEncoder_serial_pub(lcmd_buffer, rcmd_buffer, encoder_data, serial_data, enc_pub);

        loop_rate.sleep();
     }
     return 0;
 }
