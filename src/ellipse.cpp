#include "ros/ros.h"
//
#include "sbgwrapper.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sbg_ellipse");

    // Nodes (Private) handlers
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    std::string uart_port;
    int uart_baud_rate;
    private_nh.param<std::string>("uart_port", uart_port, "/dev/ttyUSB1");
    private_nh.param<int>("uart_baud_rate", uart_baud_rate, 115200);
    //
    ROS_INFO_STREAM("uart_port: " << uart_port);
    ROS_INFO_STREAM("uart_baud_rate: " << uart_baud_rate);

    // ********************* Initialize the SBG  *********************
    SBGWrapperPtr ptr_sbgwrapper;
    ptr_sbgwrapper.reset(new SBGWrapper(uart_port, uart_baud_rate, n, private_nh));
    SBGWrapper& sbgwrapper = *ptr_sbgwrapper;

    sbgwrapper.initialize();

    ROS_INFO("CONNEXTION SET-UP");

    // ****************************** SBG Config ******************************
    //
    sbgwrapper.set_configuration(SBGConfiguration::build_configuration_for_log_efk_nav());
    sbgwrapper.set_configuration(SBGConfiguration::build_configuration_for_log_efk_quat());
    sbgwrapper.set_configuration(SBGConfiguration::build_configuration_for_log_ship_motion());
    //
    sbgwrapper.save_settings();

    ROS_INFO("CONFIGURATION DONE");

    // ************************** SBG Callback for data ************************
    sbgwrapper.set_callback_for_logs();

    ROS_INFO("START RECEIVING DATA");   

    ros::Rate loop_rate(25);
    while (ros::ok())
    {
        sbgwrapper.handle_logs();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
