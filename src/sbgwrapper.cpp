#include "sbgwrapper.h"
#include "ros/ros.h"


void SBGWrapper::initialize()
{
    THROW_EXCEPTION(createSerialInterface, sbgInterfaceSerialCreateException);
    THROW_EXCEPTION(initInterface, sbgEComInitException);
    THROW_EXCEPTION(getDeviceInfo, sbgEComCmdGetInfoException);

    log_parser.reset(new SBGLogParser(n, private_nh));
    log_parser->init();
}

SbgErrorCode SBGWrapper::createSerialInterface()
{
    return sbgInterfaceSerialCreate(&sbgInterface,
                                    uart_port.c_str(),
                                    uart_baud_rate);
}

SbgErrorCode SBGWrapper::initInterface()
{
    return sbgEComInit(&comHandle, &sbgInterface);
}

SbgErrorCode SBGWrapper::getDeviceInfo()
{
    return sbgEComCmdGetInfo(&comHandle, &deviceInfo);
}

SbgErrorCode SBGWrapper::save_and_reboot()
{
    return sbgEComCmdSettingsAction(&comHandle, SBG_ECOM_SAVE_SETTINGS);
}

void SBGWrapper::save_settings()
{
    THROW_EXCEPTION(save_and_reboot, sbgEComCmdSettingsActionException);
}

SbgErrorCode SBGWrapper::set_configuration_for_cmd_output(const SBGConfiguration &_config)
{
    return sbgEComCmdOutputSetConfException(&comHandle,
                                            _config.outputPort,
                                            _config.classId,
                                            _config.msgId,
                                            _config.conf);
}

void SBGWrapper::set_configuration(const SBGConfiguration &_config) {
    THROW_EXCEPTION(boost::bind(&SBGWrapper::set_configuration_for_cmd_output, this, _config),
                    sbgEComCmdSettingsActionException);
}


SbgErrorCode SBGWrapper::handle_sbgECom()
{
    return sbgEComHandle(&comHandle);
}

void SBGWrapper::handle_logs()
{
    //    const SbgErrorCode& errorCode = handle_sbgECom();
    // errorCode == 'SBG_NOT_READY' ... => SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
    //    if(errorCode != 10)
    //        ROS_INFO_STREAM(sbgErrorCodeToString(errorCode));
    //        THROW_EXCEPTION(handle_sbgECom, sbgEComHandleException);
    handle_sbgECom();

    log_parser->publish();
}

// STATIC
void SBGWrapper::_set_callback_for_logs(SbgEComHandle &_comHandle,
                                        SBGLogParser* _this)
{
    // On set le callback sur une fonction locale (statique)
    // On utilise 'pUserArg' pour transmettre le pointeur sur l'instance de la
    // classe SBGWrapper (this).
    // Ainsi dans le callback, on doit pouvoir reprendre la main sur notre classe
    // wrapper.
    //    sbgEComSetReceiveLogCallback(&_comHandle, onLogReceived_, _this);

    sbgEComSetReceiveLogCallback(&_comHandle,
                                 SBGLogParser::onLogReceived,
                                 _this);
}

void SBGWrapper::set_callback_for_logs()
{
    _set_callback_for_logs(comHandle, log_parser.get());
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
//// Déclaration (à ROS) d'un PUBLISHER
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
