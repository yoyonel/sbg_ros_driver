#include "SBGLogtoROSPublisher_Visitor.h"
#include "SBGLogtoROSPublisher.h"
#include "ros/ros.h"

#define _IMP_OPERATOR_VISITOR(ros_msg_type) \    bool visitor_sbglog_to_ros::operator()(const ros_msg_type& ros_msg, SBGLogtoROSPublisher* _sbglog_publisher) const { \        return _operator<ros_msg_type>(ros_msg, _sbglog_publisher); \    }

_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogImuData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogEvent)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogMagCalib)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogEkfQuatData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogDvlData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogUsblData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogPressureData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogFastImuData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogOdometerData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogStatusData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogEkfEulerData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogGpsHdt)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogGpsVel)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogShipMotionData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogEkfNavData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogUtcData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogMag)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogGpsRaw)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogGpsPos)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogDebug0Data)

template <typename TROSMsg>
bool visitor_sbglog_to_ros::_operator(
        const TROSMsg& ros_msg_with_SBGData,
        SBGLogtoROSPublisher* _sbglog_publisher) const
{
    try {
        // on récupère le ROS publisher associé au type du message
        const ros::Publisher& pub = _sbglog_publisher->get_pub<TROSMsg>();
        // et on publie le message
        pub.publish(ros_msg_with_SBGData);
        return true;
    }
    catch(std::out_of_range) {
        // Si le publisher ROS n'existe pas encore
        // On le créé et on le rajoute à la liste des ROS publisher dispo.
        const ros::Publisher& pub = _sbglog_publisher->add_pub<TROSMsg>();
        // et on publie le message
        pub.publish(ros_msg_with_SBGData);
        // flag to false to indicate a creation of new publisher/topic
        return false;
    }
}
