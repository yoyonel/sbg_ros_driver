#ifndef SBGWRAPPER_H
#define SBGWRAPPER_H

#include <sbgEComLib.h>
#include <sbgEComIds.h>
//
#include <string>
#include <iostream>
#include <exception>
//
#include <boost/shared_ptr.hpp>
//
#include "ros/ros.h"
//
#include "sbglogparser.h"
#include "sbglogparser2.h"  // test de spécialisation d'un LogParser
#include "wrapper.h"  // test de spécialisation d'un LogParser
//
#include "sbgexceptions.h"
#include "sbgconfiguration.h"

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
        private_nh(_private_nh),
        serialinterface_is_init(false),
        ecom_is_init(false)
    {}

    void initialize();

    void save_settings();

    void set_configuration(const SBGConfiguration &_config);

    void handle_logs();

    SbgErrorCode set_callback_for_logs();

protected:
    SbgErrorCode createSerialInterface();

    SbgErrorCode initECom();

    SbgErrorCode getDeviceInfo();

    SbgErrorCode save_and_reboot();

    SbgErrorCode set_configuration_for_cmd_output(const SBGConfiguration &_config);

    SbgErrorCode handle_sbgECom();

    void _set_callback_for_logs(SbgEComHandle &_comHandle, SBGLogParser *_this);
    void _set_callback_for_logs(SbgEComHandle &_comHandle, SBGLogtoROSMsg *_this);
    void _set_callback_for_logs(SbgEComHandle &_comHandle, WrapperSBG2ROS *_this);


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

    bool serialinterface_is_init;
    bool ecom_is_init;
};

typedef boost::shared_ptr< SBGWrapper > SBGWrapperPtr;
typedef boost::shared_ptr< SBGWrapper const> SBGSBGWrapperConstPtr;

#endif // SBGWRAPPER_H
