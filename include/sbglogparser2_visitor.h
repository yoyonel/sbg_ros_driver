#ifndef SBGLOGPARSER2_VISITOR_H
#define SBGLOGPARSER2_VISITOR_H

#include "sbglogparser2.h"

#define _OPERATOR_VISITOR(ros_msg_type) \
    inline void operator()(const ros_msg_type& ros_msg) {  \
        _operator<ros_msg_type>(ros_msg);   \
    }

class visitor_sbglog_data : public boost::static_visitor<>
{
public:
    template <typename TSBGLogParser>
    visitor_sbglog_data(TSBGLogParser* _log_parser):
        boost::static_visitor<>()
    {
        // url: http://stackoverflow.com/questions/122316/template-constraints-c
        // Compile-time check
        static_assert(std::is_base_of<ISBGLogParser, TSBGLogParser>::value,
                      "type parameter of this class must derive from ISBGLogParser");
        sbglog_parser = dynamic_cast<ISBGLogParser*>(_log_parser);
    }

    // TODO: ca serait pas mal de pouvoir itérer (compile time) sur la liste
    // des types de la boost::variant ... mais pas encore trouvé comment faire
    // ça (lors de la compilation)
    _OPERATOR_VISITOR(sbg_driver::SbgLogShipMotionData)
    _OPERATOR_VISITOR(sbg_driver::SbgLogEkfNavData)
    _OPERATOR_VISITOR(sbg_driver::SbgLogEkfEulerData)
    _OPERATOR_VISITOR(sbg_driver::SbgLogUtcData)

protected:
    // utilisation de la forme templatisée de l'operator de visitor
    // car l'opération (modulo le type instancé dans boost::variant) reste
    // la même (récupération d'un publisher et publication du message). Le seul
    // paramètre dynamique est le type du ROS message (type du message lié au
    // type du log récupéré via l'API SBG).
    template <typename T>
    inline
    void _operator(const T& ros_msg_with_SBGData)
    {
        try {
            const ros::Publisher& pub = sbglog_parser->get_pub<T>();
            //
            pub.publish(ros_msg_with_SBGData);
        }
        catch(std::out_of_range) {
            //
            sbglog_parser->add_pub<T>();
        }
    }

private:
    ISBGLogParser* sbglog_parser;
};

//    inline
//    void operator()(sbg_driver::SbgLogShipMotionData& ros_msg_with_SBGData) const
//    {
//        _operator<sbg_driver::SbgLogShipMotionData>(ros_msg_with_SBGData);
//    }
#endif // SBGLOGPARSER2_VISITOR_H
