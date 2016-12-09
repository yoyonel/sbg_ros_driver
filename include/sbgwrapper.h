#ifndef SBGWRAPPER_H
#define SBGWRAPPER_H

#include <sbgEComLib.h>
#include <sbgEComIds.h>
//
#include <string>
#include <iostream>
#include <exception>
//
//#include "enum.h"
//
#include <boost/shared_ptr.hpp>
//
#include "ros/ros.h"
//
#include "sbglogparser.h"
#include "sbgexceptions.h"
#include "sbgconfiguration.h"

//// ------------------
//// BETTER ENUMERATIONS
//// ------------------
//BETTER_ENUM( _SbgWrapperEComOutputPort, char,
//             SBGWRAPPER_ECOM_OUTPUT_PORT_A = 0,				/*!< Main output port. */
//             SBGWRAPPER_ECOM_OUTPUT_PORT_C = 2,				/*!< Secondary output port only available on Ellipse-E devices */
//             SBGWRAPPER_ECOM_OUTPUT_PORT_E = 4				/*!< Secondary output port only available on B1 devices */
//        )


class SBGWrapper
{
public:
    SBGWrapper(const std::string& _uart_port,
               const int& _uart_baud_rate,
               ros::NodeHandle _n=ros::NodeHandle(),
               ros::NodeHandle _private_nh=ros::NodeHandle("~")) :
        uart_port(_uart_port),
        uart_baud_rate(_uart_baud_rate),
        n(_n),
        private_nh(_private_nh)
    {}

    void initialize();

    void save_settings();

    void set_configuration(const SBGConfiguration &_config);

    void handle_logs();

    void set_callback_for_logs();

protected:
    SbgErrorCode createSerialInterface();
    SbgErrorCode initInterface();

    /**
      Get device info
     * @brief getDeviceInfo
     * @return
     */
    SbgErrorCode getDeviceInfo();

    /**
      Save the settings to non-volatile memory and then reboot the device
     * @brief save_and_reboot
     * @return
     */
    SbgErrorCode save_and_reboot();

    SbgErrorCode set_configuration_for_cmd_output(const SBGConfiguration &_config);

    SbgErrorCode handle_sbgECom();

    static void _set_callback_for_logs(SbgEComHandle &_comHandle, SBGLogParser *_this);

private:
    SbgEComHandle       comHandle;
    SbgInterface        sbgInterface;
    SbgEComDeviceInfo   deviceInfo;
    SbgErrorCode        errorCode;

    std::string uart_port;
    int uart_baud_rate;

    ros::NodeHandle n;
    ros::NodeHandle private_nh;

    SBGLogParserPtr log_parser;
};

typedef boost::shared_ptr< SBGWrapper > SBGWrapperPtr;
typedef boost::shared_ptr< SBGWrapper const> SBGSBGWrapperConstPtr;

#endif // SBGWRAPPER_H
