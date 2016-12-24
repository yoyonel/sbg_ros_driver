#include "boost/variant.hpp"
#include "boost/function.hpp"
#include <boost/exception/all.hpp>
//
#include "ros/ros.h"
//
#include <iostream>
//
#include "sbg_driver/SbgLogImuData.h"
//
#include <sbgEComLib.h>
#include <sbgEComIds.h>
//
#include <typeindex>
#include <boost/variant/apply_visitor.hpp>
//
#include <boost/variant/static_visitor.hpp>


#define _DECL_OPERATOR_VISITOR(ros_msg_type)     bool operator()(const ros_msg_type& ros_msg)


class SBGLogtoROSPublisher {

class visitor_sbglog_to_ros : public boost::static_visitor<bool>
{
public:
    template <typename TSBGLogParser>
    visitor_sbglog_to_ros(TSBGLogParser* _log_parser):
        boost::static_visitor<bool>()
    {
        // url: http://stackoverflow.com/questions/122316/template-constraints-c
        // Compile-time check
        static_assert(std::is_base_of<SBGLogtoROSPublisher, TSBGLogParser>::value,
                      "type parameter of this class must derive from SBGLogtoROSPublisher");
        sbglog_parser = dynamic_cast<SBGLogtoROSPublisher*>(_log_parser);
    }

    bool operator()(const sbg_driver::SbgLogImuData& ros_msg) const {       
        return _operator<sbg_driver::SbgLogImuData>(ros_msg);
    }

protected:
    template <typename T>
    bool _operator(const T& ros_msg_with_SBGData) const
    {
        try {
            // on récupère le ROS publisher associé au type du message
            const ros::Publisher& pub = sbglog_parser->get_pub<T>();
            // et on publie le message
            pub.publish(ros_msg_with_SBGData);
            return true;
        }
        catch(std::out_of_range) {
            return false;
        }
    }

private:

    SBGLogtoROSPublisher* sbglog_parser;
};  // visitor_sbglog_to_ros

public:
    SBGLogtoROSPublisher(ros::NodeHandle _n, ros::NodeHandle _private_nh): 
        n(_n),
        private_nh(_private_nh)
    {
        pVisitor.reset(new visitor_sbglog_to_ros(this));
    }

    // -------------------------------------------------------------------------
    // ROS PUBLISHER MANAGER: ADD and GET
    // -------------------------------------------------------------------------
    template<typename T>
    inline
    ros::Publisher get_pub() const {
        return map_sbglog_pub.at(typeid(T));
    }

    template<typename T>
    std::string build_rostopic_name(
            const std::string& _prefix_pattern="SbgLog",
            const std::string& _suffix_pattern="Data",
            const std::string& _suffix_for_rostopic_name="") const
    {
        // url: http://stackoverflow.com/questions/1055452/c-get-name-of-type-in-template
        char const * name = typeid(T).name();
        //
        // url: http://www.boost.org/doc/libs/master/libs/core/doc/html/core/demangle.html
        const std::string& demangled_name = boost::core::demangle( name );
        //
        // url: http://www.cplusplus.com/reference/string/string/find/
        const std::size_t& found_prefix = demangled_name.find(_prefix_pattern);
        const std::size_t& found_suffix = demangled_name.find(_suffix_pattern);
        assert(found_prefix!=std::string::npos);
        assert(found_suffix!=std::string::npos);
        //
        // url: http://www.cplusplus.com/reference/string/string/substr/
        return demangled_name.substr(found_prefix, found_suffix - found_prefix) +
                _suffix_for_rostopic_name;
    }

    template<typename TRosMsg>
    ros::Publisher add_pub()
    {
        // $  rostopic list
        // /SbgLogEkfEuler
        // /SbgLogEkfNav
        // /SbgLogShipMotion
        // /rosout
        // /rosout_agg
        /*ros::Publisher ros_pub = n.advertise<TRosMsg>(
            build_rostopic_name<TRosMsg>(), 10 );*/
        ros::Publisher ros_pub = n.advertise<TRosMsg>(
            build_rostopic_name<TRosMsg>(), 10 );
        map_sbglog_pub[ typeid(TRosMsg) ] = ros_pub;
        return ros_pub;
    }

    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    // ROS PUBLISHER OPERATION: PUBLISH
    // -------------------------------------------------------------------------
    template<typename TRosMsg, typename TVisitor>
    // inline
    bool publish(TRosMsg& _msg, const TVisitor& _visitor) {
        if(boost::apply_visitor(_visitor, _msg))
            return true;
        else {
            // Si le publisher ROS n'existe pas encore
            // On le créé et on le rajoute à la liste des ROS publisher dispo.
            const ros::Publisher& pub = add_pub<TRosMsg>();
            // et on publie le message
            // pub.publish(_msg);
            //
            return false;
        }
    }

    template<typename TMsg>
    inline
    bool publish(TMsg& _msg) {
        return publish(_msg, visitor());
    }
    // -------------------------------------------------------------------------

protected:
    inline
    const visitor_sbglog_to_ros& visitor() const {
        return *pVisitor.get();
    }

private:
    ros::NodeHandle n;
    ros::NodeHandle private_nh;

    // -------------------------------------------------------------------------
    // Map polymorphique dont les clés sont les type_id (d'un bv_sbglog_data instancié)
    // et les valeurs un ros publisher
    // ps: Pas encore réussi à contraindre à la compilation le type des clés de
    // la map (pour que le compilateur n'accepte que des types contenus dans bv_sbglog_data)
    // ps2: pas sure que ca ait du sens ... (dynamique versus statique)
    std::map<std::type_index, ros::Publisher> map_sbglog_pub;
    // -------------------------------------------------------------------------
    
    boost::shared_ptr<visitor_sbglog_to_ros> pVisitor;
};

class SBGLogParser {
public:
    SBGLogParser() {}

    SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
                               SbgEComClass msgClass,
                               SbgEComMsgId msg,
                               const SbgBinaryLogData *pLogData);

private:
    typedef boost::variant<
        sbg_driver::SbgLogImuData
    > bv_sbglog_data;

    bv_sbglog_data parser_for_SbgLogImuData (const SbgBinaryLogData *pLogData);

    typedef boost::function<bv_sbglog_data(const SBGLogParser*, const SbgBinaryLogData*)> fun_t;

    typedef std::map<const SbgEComLog, const fun_t> funs_t;

    funs_t f = {
        { SBG_ECOM_LOG_IMU_DATA,    &SBGLogParser::parser_for_SbgLogImuData }
        };

    boost::shared_ptr<SBGLogtoROSPublisher> sbglog_rospublisher;
};

SBGLogParser::bv_sbglog_data SBGLogParser::parser_for_SbgLogImuData (const SbgBinaryLogData *pLogData) {
    sbg_driver::SbgLogImuData ros_msg;       
    if (pLogData) 
        std::memcpy(&ros_msg, &pLogData->imuData, sizeof(SbgLogImuData));
    return ros_msg;     
}

SbgErrorCode SBGLogParser::onLogReceived(SbgEComHandle *pHandle,
                                           SbgEComClass msgClass,
                                           SbgEComMsgId msg,
                                           const SbgBinaryLogData *pLogData)
{
    SbgErrorCode errCode = SBG_NO_ERROR;
    bv_sbglog_data msg_ros;
    try {
        msg_ros = f[static_cast<SbgEComLog>(msg)](this, pLogData);
    }
    catch(const boost::exception& e) {
        ROS_ERROR_STREAM("Bad function call: " << boost::diagnostic_information(e));
        ROS_ERROR_STREAM("msg: " << int(msg));
        errCode = SBG_ERROR;
    }

    sbglog_rospublisher->publish(msg_ros);

    return errCode;
}

// url: http://www.boost.org/doc/libs/1_58_0/doc/html/variant.html
class my_visitor : public boost::static_visitor<int>
{
public:
    int operator()(int i) const
    {
        return i;
    }
    
    int operator()(const std::string & str) const
    {
        return str.length();
    }

    int operator()() const {
        return 1;
    }
};

int main()
{
    boost::variant< int, std::string > u("hello world");
    std::cout << u; // output: hello world

    int result = boost::apply_visitor( my_visitor(), u );
    std::cout << result; // output: 11 (i.e., length of "hello world")
}
