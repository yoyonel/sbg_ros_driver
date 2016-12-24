#ifndef SBGLOGTOROSPUBLISHER_H
#define SBGLOGTOROSPUBLISHER_H

#include "ros/ros.h"
#include <typeindex>
#include <boost/variant/apply_visitor.hpp>

#include "SBGLogtoROSPublisher_Visitor.h"

// Bridge SBG Logs Datas to Ros Message Publishers
class SBGLogtoROSPublisher {
public:
    SBGLogtoROSPublisher(ros::NodeHandle _n=ros::NodeHandle(),
                         ros::NodeHandle _private_nh=ros::NodeHandle("~"));

    // -------------------------------------------------------------------------
    // ROS PUBLISHER MANAGER: ADD and GET
    // -------------------------------------------------------------------------
    template<typename TRosMsg>
    inline
    ros::Publisher get_pub() const {
        return map_sbglog_pub.at(typeid(TRosMsg));
    }

    template<typename T>
    ros::Publisher add_pub();
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    // ROS PUBLISHER OPERATION: PUBLISH
    // -------------------------------------------------------------------------
    template<typename TRosMsg, typename TVisitor>
    inline
    void publish(TRosMsg& _msg, const TVisitor& _visitor) {
        // url: http://stackoverflow.com/questions/29618636/boost-variant-visitor-with-an-extra-parameter
        auto bound_visitor = std::bind(_visitor, std::placeholders::_1, this);
        boost::apply_visitor(bound_visitor, _msg);
    }

    template<typename TMsg>
    inline
    void publish(TMsg& _msg) {
        publish(_msg, visitor());
    }
    // -------------------------------------------------------------------------

protected:
    template<typename T>
    std::string build_rostopic_name(
            const std::string& _prefix_pattern="SbgLog",
            const std::string& _suffix_pattern="Data",
            const std::string& _suffix_for_rostopic_name="") const;

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

#include "SBGLogtoROSPublisher.inl"

#endif // SBGLOGTOROSPUBLISHER_H
