#ifndef SBGCONFIGURATION_H
#define SBGCONFIGURATION_H

// ------------------
// CONFIGURATIONS
// ------------------
struct SBGConfiguration{
    SBGConfiguration(SbgEComMsgId _msgId,
                     SbgEComOutputMode _conf=SBG_ECOM_OUTPUT_MODE_DIV_8,
                     SbgEComOutputPort _outputPort=SBG_ECOM_OUTPUT_PORT_A,
                     SbgEComClass _classId=SBG_ECOM_CLASS_LOG_ECOM_0
            ) :
        msgId(_msgId),
        conf(_conf),
        outputPort(_outputPort),
        classId(_classId)
    {}
    //
    SbgEComMsgId msgId;
    SbgEComOutputMode conf;
    SbgEComOutputPort outputPort;
    SbgEComClass classId;

    static SBGConfiguration build_configuration_for_log_efk_quat() { return SBGConfiguration(SBG_ECOM_LOG_EKF_QUAT); }
    static SBGConfiguration build_configuration_for_log_efk_nav() { return SBGConfiguration(SBG_ECOM_LOG_EKF_NAV); }
    static SBGConfiguration build_configuration_for_log_ship_motion() { return SBGConfiguration(SBG_ECOM_LOG_SHIP_MOTION); }
};

typedef boost::shared_ptr< SBGConfiguration > SBGConfigurationPtr;
typedef boost::shared_ptr< SBGConfiguration const> SBGConfigurationConstPtr;
// ------------------


#endif // SBGCONFIGURATION_H
