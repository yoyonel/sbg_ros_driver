#include "sbgwrapper/sbgwrapper.h"
#include "ros/ros.h"


//void SBGWrapper::initialize()
//{
//    THROW_EXCEPTION(createSerialInterface, sbgInterfaceSerialCreateException);
//    serialinterface_is_init = true;

//    THROW_EXCEPTION(initECom, sbgEComInitException);
//    ecom_is_init = true;

//    THROW_EXCEPTION(getDeviceInfo, sbgEComCmdGetInfoException);

////    log_parser.reset(new SBGLogParser(n, private_nh));
//    log_parser.reset(new WrapperSBG2ROS(n, private_nh));
//    log_parser->init();
//}

SbgErrorCode SBGWrapper::createSerialInterface() {
    return sbgInterfaceSerialCreate(&sbgInterface,
                                    uart_port.c_str(),
                                    uart_baud_rate);
}

SbgErrorCode SBGWrapper::initECom() {
    SbgErrorCode errCode = SBG_ERROR;
    if (serialinterface_is_init)
        errCode = sbgEComInit(&comHandle, &sbgInterface);
    return errCode;
}

SbgErrorCode SBGWrapper::getDeviceInfo() {
    SbgErrorCode errCode = SBG_ERROR;
    if (ecom_is_init)
        errCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo);
    return errCode;
}

SbgErrorCode SBGWrapper::save_and_reboot() {
    SbgErrorCode errCode = SBG_ERROR;
    if (ecom_is_init)
        errCode = sbgEComCmdSettingsAction(&comHandle, SBG_ECOM_SAVE_SETTINGS);
    return errCode;
}

void SBGWrapper::save_settings() {
    THROW_EXCEPTION(save_and_reboot, sbgEComCmdSettingsActionException);
}

SbgErrorCode SBGWrapper::set_configuration_for_cmd_output(const SBGConfiguration &_config) {
    SbgErrorCode errCode = SBG_ERROR;
    if (ecom_is_init)
        errCode = sbgEComCmdOutputSetConf(&comHandle,
                                          _config.outputPort,
                                          _config.classId,
                                          _config.msgId,
                                          _config.conf);
    return errCode;
}

void SBGWrapper::set_configuration(const SBGConfiguration &_config) {
    THROW_EXCEPTION(boost::bind(&SBGWrapper::set_configuration_for_cmd_output, this, _config),
                    sbgEComCmdSettingsActionException);
}


SbgErrorCode SBGWrapper::handle_sbgECom() {
    SbgErrorCode errCode = SBG_ERROR;
    if (ecom_is_init) {
        errCode = sbgEComHandle(&comHandle);
    }
    return errCode;
}

void SBGWrapper::handle_logs() {
    //    const SbgErrorCode& errorCode = handle_sbgECom();
    // errorCode == 'SBG_NOT_READY' ... => SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
    //    if(errorCode != 10)
    //        ROS_INFO_STREAM(sbgErrorCodeToString(errorCode));
    //        THROW_EXCEPTION(handle_sbgECom, sbgEComHandleException);

    //
    handle_sbgECom();

    if (log_parser)
        log_parser->publish();
}

SbgErrorCode SBGWrapper::set_callback_for_logs() {
    SbgErrorCode errCode = SBG_ERROR;
    if (ecom_is_init) {
        _set_callback_for_logs(comHandle, log_parser.get());
        errCode = SBG_NO_ERROR;
    }
    return errCode;
}



//#include <sbgEComLib.h>
//#include <sbgEComIds.h>
//
//#include <sbg_driver/ELLIPSE_N_settingsConfig.h>
//#include "sbg_driver/SbgEComDeviceInfo.h"
//
////----------------------------------------------------
//// Publisher
////----------------------------------------------------
//// D�claration (� ROS) d'un PUBLISHER
//std::string _name = "SbgEComDeviceInfo";
//std::string topic_name_pub_ = _name + "_pub";
//ros::Publisher SbgEComDeviceInfo_pub_ = n.advertise<sbg_driver::SbgEComDeviceInfo>(topic_name_pub_, 10);
////
//ROS_INFO("Publish %s messages.\t[PUBLISHER]", _name.c_str());
////----------------------------------------------------
///**
// * @brief buildMsgSbgEComDeviceInfo
// * @param _deviceInfo
// * @return
// */
//const sbg_driver::SbgEComDeviceInfoConstPtr buildMsgSbgEComDeviceInfo(
//        const SbgEComDeviceInfo&   _deviceInfo)
//{
//    sbg_driver::SbgEComDeviceInfoPtr msg_SbgEComDeviceInfo(new sbg_driver::SbgEComDeviceInfo());

//    // url: http://stackoverflow.com/questions/16137953/is-there-a-function-to-copy-an-array-in-c-c
//    // ps: il faut activer c++11 pour std::begin & std::end
//    std::copy(std::begin(_deviceInfo.productCode),
//              std::end(_deviceInfo.productCode),
//              std::begin(msg_SbgEComDeviceInfo->productCode));
//    //
//    msg_SbgEComDeviceInfo->serialNumber = _deviceInfo.serialNumber;
//    msg_SbgEComDeviceInfo->calibationRev = _deviceInfo.calibationRev;
//    msg_SbgEComDeviceInfo->calibrationYear = _deviceInfo.calibrationYear;
//    msg_SbgEComDeviceInfo->calibrationMonth = _deviceInfo.calibrationMonth;
//    msg_SbgEComDeviceInfo->calibrationDay = _deviceInfo.calibrationDay;
//    msg_SbgEComDeviceInfo->hardwareRev = _deviceInfo.hardwareRev;
//    msg_SbgEComDeviceInfo->firmwareRev = _deviceInfo.firmwareRev;
//    //
//    return msg_SbgEComDeviceInfo;
//}
