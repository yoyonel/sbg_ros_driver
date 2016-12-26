#include <string>
//#include <boost/variant/apply_visitor.hpp>
#include "../isbglogparser.h"

//#include <cxxabi.h>    // for namespace abi::
//
// static inline std::string
// demangled_type_info_name(const std::type_info&ti)
// {
//     int status = 0;
//     return abi::__cxa_demangle(ti.name(),0,0,&status);
// }

template<typename TRosMsg>
ros::Publisher SBGLogtoROSPublisher::add_pub()
{
    // $  rostopic list
    // /SbgLogEkfEuler
    // /SbgLogEkfNav
    // /SbgLogShipMotion
    // /rosout
    // /rosout_agg
//    auto ros_pub = n.advertise<TRosMsg>( demangled_type_info_name(typeid(TRosMsg)), 10 );
    auto ros_pub = n.advertise<TRosMsg>( build_topic_name<TRosMsg>(), 10 );
    map_sbglog_pub[ typeid(TRosMsg) ] = ros_pub;
    return ros_pub;
}

//template<typename TMsg, typename TVisitor>
//void SBGLogtoROSPublisher::publish(TMsg& _msg, const TVisitor& _visitor)
//{
//    boost::apply_visitor(_visitor, _msg);
//}

//template<typename TMsg>
//void SBGLogtoROSPublisher::publish(TMsg& _msg)
//{
//    boost::apply_visitor(*pVisitor.get(), _msg);
//}
