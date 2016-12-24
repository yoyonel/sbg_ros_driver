#include "sbglogparser/boost/SBGLogtoROSPublisher_Visitor.h"
#include "sbglogparser/boost/SBGLogtoROSPublisher.h"

#define _IMP_OPERATOR_VISITOR(ros_msg_type) \
    bool visitor_sbglog_to_ros::operator() (const ros_msg_type& ros_msg, SBGLogtoROSPublisher* _sbglog_publisher) const  { \
        return _operator<ros_msg_type>(ros_msg, _sbglog_publisher); \
    }

_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogImuData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogMagCalib)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogEkfQuatData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogDvlData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogUsblData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogPressureData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogFastImuData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogOdometerData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogStatusData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogEkfEulerData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogShipMotionData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogEkfNavData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogUtcData)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogMag)
_IMP_OPERATOR_VISITOR(sbg_driver::SbgLogDebug0Data)

template <typename T>
bool visitor_sbglog_to_ros::_operator(const T& ros_msg_with_SBGData, SBGLogtoROSPublisher* _sbglog_publisher) const
{
    try {
        // on récupère le ROS publisher associé au type du message
        const ros::Publisher& pub = _sbglog_publisher->get_pub<T>();
        // et on publie le message
        pub.publish(ros_msg_with_SBGData);
        return true;
    }
    catch(std::out_of_range) {
//        // Si le publisher ROS n'existe pas encore
//        // On le créé et on le rajoute à la liste des ROS publisher dispo.
//        const ros::Publisher& pub = sbglog_parser->add_pub<T>();
//        // et on publie le message
//        pub.publish(ros_msg_with_SBGData);
        return false;
    }
}
