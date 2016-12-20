#include "ros/ros.h"
//
#include "sbgwrapper.h"
//
#include "wrapper.h"

void test_sbglogparser(SBGLogtoROSMsg& log_parser)
{    
    //
    const SbgLogEkfQuatData quat_data = {0, {1, 2, 3, 4}, {5, 6, 7}, 8};
    const SbgLogEkfNavData nav_data = {0, {1, 2, 3}, {5, 6, 7}, {8, 9, 10}, 11,
                                       {12, 13, 14}, 15};
    const SbgLogShipMotionData ship_motion_data = {0, 11, 12, {1, 2, 3},
                                                   {5, 6, 7}, {8, 9, 10}};
    // Cast à l'ancienne (mode C) car l'API est une API en C (à l'ancienne).
    log_parser.onLogReceived(NULL, SbgEComClass(), SBG_ECOM_LOG_EKF_QUAT,
                             (SbgBinaryLogData*)(&quat_data));
    log_parser.onLogReceived(NULL, SbgEComClass(), SBG_ECOM_LOG_EKF_NAV,
                             (SbgBinaryLogData*)(&nav_data));
    log_parser.onLogReceived(NULL, SbgEComClass(), SBG_ECOM_LOG_SHIP_MOTION,
                             (SbgBinaryLogData*)(&ship_motion_data));

    // Exception Boost: bad call function
    //    log_parser.onLogReceived(NULL, SbgEComClass(), SBG_ECOM_LOG_SHIP_MOTION_HP, NULL);
}

void test_wrapper(WrapperSBG2ROS& wrapper)
{
    //
    const SbgLogEkfQuatData quat_data = {0, {1, 2, 3, 4}, {5, 6, 7}, 8};
    const SbgLogEkfNavData nav_data = {0, {1, 2, 3}, {5, 6, 7}, {8, 9, 10}, 11,
                                       {12, 13, 14}, 15};
    const SbgLogShipMotionData ship_motion_data = {0, 11, 12, {1, 2, 3},
                                                   {5, 6, 7}, {8, 9, 10}};

    SbgBinaryLogData genericLogData;
    //
    //    wrapper.publish<SBG_ECOM_LOG_EKF_QUAT>((const SbgBinaryLogData*)(&quat_data));
    //    wrapper.publish<SBG_ECOM_LOG_EKF_NAV>((const SbgBinaryLogData*)(&nav_data));
    //    wrapper.publish<SBG_ECOM_LOG_SHIP_MOTION>((const SbgBinaryLogData*)(&ship_motion_data));

    try {
        memcpy(&genericLogData.ekfQuatData, &quat_data, sizeof(SbgLogEkfQuatData));
        wrapper.onLogReceived(NULL, SbgEComClass(), SBG_ECOM_LOG_EKF_QUAT,
                              &genericLogData);
    }
    catch(SbgEComMsgIdException& err) {
        std::cout << "Erreur dans 'onLogReceived(...)': " << err.what() << std::endl;
    }

    try {
        memcpy(&genericLogData.ekfNavData, &nav_data, sizeof(SbgLogEkfNavData));
        wrapper.onLogReceived(NULL, SbgEComClass(), SBG_ECOM_LOG_EKF_NAV,
                              &genericLogData);
    }
    catch(SbgEComMsgIdException& err) {
        std::cout << "Erreur dans 'onLogReceived(...)': " << err.what() << std::endl;
    }

    try {
        memcpy(&genericLogData.shipMotionData, &ship_motion_data, sizeof(SbgLogShipMotionData));
        wrapper.onLogReceived(NULL, SbgEComClass(), SBG_ECOM_LOG_SHIP_MOTION,
                              &genericLogData);
    }
    catch(SbgEComMsgIdException& err) {
        std::cout << "Erreur dans 'onLogReceived(...)': " << err.what() << std::endl;
    }
}

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

    try {
        sbgwrapper.initialize();
    } catch (const sbgExceptions& e) {
        ROS_ERROR_STREAM("initialize -> Exception: " << e.what());
    }

    ROS_INFO("CONNEXTION SET-UP");

    // ****************************** SBG Config ******************************
    //
    try {
        sbgwrapper.set_configuration(SBGConfiguration::build_configuration_for_log_efk_nav());
        sbgwrapper.set_configuration(SBGConfiguration::build_configuration_for_log_efk_quat());
        sbgwrapper.set_configuration(SBGConfiguration::build_configuration_for_log_ship_motion());
    } catch (const sbgExceptions& e) {
        ROS_ERROR_STREAM("set configuration -> Exception: " << e.what());
    }
    //
    try {
        sbgwrapper.save_settings();
    } catch (const sbgExceptions& e) {
        ROS_ERROR_STREAM("save settings-> Exception: " << e.what());
    }
    ROS_INFO("CONFIGURATION DONE");

    // ************************** SBG Callback for data ************************
    try {
        sbgwrapper.set_callback_for_logs();
    } catch (const sbgExceptions& e) {
        ROS_ERROR_STREAM("set callback for logs -> Exception: " << e.what());
    }
    ROS_INFO("START RECEIVING DATA");

//    SBGLogtoROSMsg sbglogparser_to_ros(n, private_nh);
    WrapperSBG2ROS wrapper(n);

    ros::Rate loop_rate(25);
    while (ros::ok())
    {
        try {
            sbgwrapper.handle_logs();
        } catch (const sbgExceptions& e) {
            ROS_ERROR_STREAM("handle logs -> Exception: " << e.what());
        }

        //        test_sbglogparser(sbglogparser_to_ros);
        test_wrapper(wrapper);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
