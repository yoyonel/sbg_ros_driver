#ifndef SBGLOGTOROSPUBLISHER_VISITOR_H
#define SBGLOGTOROSPUBLISHER_VISITOR_H

// include des headers des messages ROS
#include "sbg_driver/SbgLogImuData.h"
#include "sbg_driver/SbgLogMagCalib.h"
#include "sbg_driver/SbgLogEkfQuatData.h"
#include "sbg_driver/SbgLogDvlData.h"
#include "sbg_driver/SbgLogUsblData.h"
#include "sbg_driver/SbgLogPressureData.h"
#include "sbg_driver/SbgLogFastImuData.h"
#include "sbg_driver/SbgLogOdometerData.h"
#include "sbg_driver/SbgLogStatusData.h"
#include "sbg_driver/SbgLogEkfEulerData.h"
#include "sbg_driver/SbgLogShipMotionData.h"
#include "sbg_driver/SbgLogEkfNavData.h"
#include "sbg_driver/SbgLogUtcData.h"
#include "sbg_driver/SbgLogMag.h"
#include "sbg_driver/SbgLogDebug0Data.h"

//
#include <boost/variant/static_visitor.hpp>


#define _DECL_OPERATOR_VISITOR(ros_msg_type)     bool operator()(const ros_msg_type& ros_msg, SBGLogtoROSPublisher* _sbglog_publisher) const 


class SBGLogtoROSPublisher;

class visitor_sbglog_to_ros : public boost::static_visitor<bool>
{
public:
    visitor_sbglog_to_ros() : boost::static_visitor<bool>()
    {
        /*// url: http://stackoverflow.com/questions/122316/template-constraints-c
        // Compile-time check
        static_assert(std::is_base_of<SBGLogtoROSPublisher, TSBGLogParser>::value,
                      "type parameter of this class must derive from SBGLogtoROSPublisher");
        sbglog_parser = dynamic_cast<SBGLogtoROSPublisher*>(_log_parser);*/
    }

    // TODO: ca serait pas mal de pouvoir itérer (compile time) sur la liste
    // des types de la boost::variant ... mais pas encore trouvé comment faire
    // ça :-/
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogImuData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogMagCalib);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogEkfQuatData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogDvlData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogUsblData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogPressureData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogFastImuData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogOdometerData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogStatusData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogEkfEulerData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogShipMotionData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogEkfNavData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogUtcData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogMag);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogDebug0Data);

private:
    // utilisation de la forme templatisée de l'operator de visitor
    // car l'opération (modulo le type instancé dans boost::variant) reste
    // la même (récupération d'un publisher et publication du message). Le seul
    // paramètre dynamique est le type du ROS message (type du message lié au
    // type du log récupéré via l'API SBG).
    template<typename TROSMsg> 
    bool _operator( const TROSMsg& ros_msg_with_SBGData,
                    SBGLogtoROSPublisher* _sbglog_publisher) const;
};

#endif // SBGLOGTOROSPUBLISHER_VISITOR_H
