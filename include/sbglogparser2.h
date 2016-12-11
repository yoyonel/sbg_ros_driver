#ifndef SBGLOGPARSER2_H
#define SBGLOGPARSER2_H

#include "sbglogparser.h"
//
#include <boost/function.hpp>
#include <boost/variant.hpp>
//
#include "sbg_driver/SbgLogEkfNavData.h"
#include "sbg_driver/SbgLogEkfEulerData.h"
#include "sbg_driver/SbgLogShipMotionData.h"
#include "sbg_driver/SbgLogUtcData.h"
//
#include <typeindex>
#include <typeinfo>

// Test de spécialisation d'un LogParser pour SBG

// -----------------------------------------------------------------------------

class ISBGLogParser {
public:
    ISBGLogParser(ros::NodeHandle _n=ros::NodeHandle(),
                  ros::NodeHandle _private_nh=ros::NodeHandle("~")):
        n(_n),
        private_nh(_private_nh)
    { }

    virtual SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
                                       SbgEComClass msgClass,
                                       SbgEComMsgId msg,
                                       const SbgBinaryLogData *pLogData) = 0;

    // ps: On place get_pub dans l'interface, car cette function est template
    // et on ne peut pas virtualiser (pure) une function membre ...
    template<typename T>
    ros::Publisher get_pub() const
    {
        return map_sbglog_pub.at(typeid(T));
    }

    template<typename T>
    void add_pub()
    {
        map_sbglog_pub[typeid(T)] =  n.advertise<T>(typeid(T).name(), 10);
    }

protected:
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
};

class SBGLogParserImp : public ISBGLogParser {
public:
    SBGLogParserImp(ros::NodeHandle _n=ros::NodeHandle(),
                    ros::NodeHandle _private_nh=ros::NodeHandle("~")):
        ISBGLogParser(_n, _private_nh)
    { }

    // url: en.cppreference.com/w/cpp/language/override
    SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
                               SbgEComClass msgClass,
                               SbgEComMsgId msg,
                               const SbgBinaryLogData *pLogData);

    void init_ros_publishers();

private:
    // -----------------------------------------------------------------------------
    // Boost Variant
    // -----------------------------------------------------------------------------
    // Boost variant sur les types des datas des logs de la lib SBG.
    // C'est la version C++ du 'union' de C. Ca permet d'avoir un conteneur
    // générique mais qui limite les types possibles (controle à la compilation).
    typedef
    boost::variant<
        sbg_driver::SbgLogEkfEulerData,
        sbg_driver::SbgLogEkfNavData,
        sbg_driver::SbgLogShipMotionData,
        sbg_driver::SbgLogUtcData
    > bv_sbglog_data;

    bv_sbglog_data parser_for_SBG_ECOM_LOG_EKF_QUAT     (const SbgBinaryLogData *pLogData) const;
    bv_sbglog_data parser_for_SBG_ECOM_LOG_EKF_NAV      (const SbgBinaryLogData *pLogData) const;
    bv_sbglog_data parser_for_SBG_ECOM_LOG_SHIP_MOTION  (const SbgBinaryLogData *pLogData) const;
    bv_sbglog_data parser_for_SBG_ECOM_LOG_UTC_DATA     (const SbgBinaryLogData *pLogData) const;

    // -------------------------------------------------------------------------
    // urls:
    // - http://www.boost.org/doc/libs/1_55_0/doc/html/function/tutorial.html#idp95767816
    // - http://stackoverflow.com/questions/2136998/using-a-stl-map-of-function-pointers
    typedef boost::function<bv_sbglog_data(const SBGLogParserImp*, const SbgBinaryLogData*)> fun_t;
    typedef std::map<const SbgEComLog, const fun_t> funs_t;

    // urls:
    // - http://stackoverflow.com/questions/138600/initializing-a-static-stdmapint-int-in-c
    funs_t f = {
        { SBG_ECOM_LOG_EKF_QUAT,    &SBGLogParserImp::parser_for_SBG_ECOM_LOG_EKF_QUAT    },
        { SBG_ECOM_LOG_EKF_NAV,     &SBGLogParserImp::parser_for_SBG_ECOM_LOG_EKF_NAV     },
        { SBG_ECOM_LOG_SHIP_MOTION, &SBGLogParserImp::parser_for_SBG_ECOM_LOG_SHIP_MOTION },
    };
    // -------------------------------------------------------------------------
};
#endif // SBGLOGPARSER2_H
