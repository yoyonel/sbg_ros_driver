#ifndef SBGLOGPARSER2_VISITOR_H
#define SBGLOGPARSER2_VISITOR_H

#include "sbg_driver/SbgLogEkfNavData.h"
#include "sbg_driver/SbgLogEkfEulerData.h"
#include "sbg_driver/SbgLogShipMotionData.h"
#include "sbg_driver/SbgLogUtcData.h"
#include "sbg_driver/SbgLogStatusData.h"
#include "sbg_driver/SbgLogImuData.h"
//
#include <boost/variant/static_visitor.hpp>


#define _DECL_OPERATOR_VISITOR(ros_msg_type) \
    bool operator()(const ros_msg_type& ros_msg)


class SBGLogtoROSPublisher;

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

    // TODO: ca serait pas mal de pouvoir itérer (compile time) sur la liste
    // des types de la boost::variant ... mais pas encore trouvé comment faire
    // ça :-/
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogShipMotionData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogEkfNavData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogEkfEulerData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogUtcData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogStatusData);
    _DECL_OPERATOR_VISITOR(sbg_driver::SbgLogImuData);

private:
    // utilisation de la forme templatisée de l'operator de visitor
    // car l'opération (modulo le type instancé dans boost::variant) reste
    // la même (récupération d'un publisher et publication du message). Le seul
    // paramètre dynamique est le type du ROS message (type du message lié au
    // type du log récupéré via l'API SBG).
    template<typename T> bool _operator(const T& ros_msg_with_SBGData);

private:
    // R/W car on peut créer de nouveaux publishers
    SBGLogtoROSPublisher* sbglog_parser;
};

#endif // SBGLOGPARSER2_VISITOR_H
