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
//
#include <typeindex>
#include <typeinfo>

// Test de spécialisation d'un LogParser pour SBG

// -----------------------------------------------------------------------------
// Boost Variant
// -----------------------------------------------------------------------------
// Boost variant sur les types des datas des logs de la lib SBG.
// C'est la version C++ du 'union' de C. Ca permet d'avoir un conteneur
// générique mais qui limite les types possibles (controle à la compilation).
typedef boost::variant<
sbg_driver::SbgLogEkfEulerData,
sbg_driver::SbgLogEkfNavData,
sbg_driver::SbgLogShipMotionData> bv_sbglog_data;
// -----------------------------------------------------------------------------

class SBGLogParser2 : SBGLogParser {
public:
    SBGLogParser2(ros::NodeHandle _n=ros::NodeHandle(),
                  ros::NodeHandle _private_nh=ros::NodeHandle("~")):
        SBGLogParser(_n, _private_nh)
    { }

    // url: en.cppreference.com/w/cpp/language/override
    SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
                               SbgEComClass msgClass,
                               SbgEComMsgId msg,
                               const SbgBinaryLogData *pLogData) override;

    template<typename T>
    ros::Publisher get_pub() const
    {
        return map_sbglog_pub.at(typeid(T));
    }

    void init_ros_publishers();

private:
    bv_sbglog_data parser_for_SBG_ECOM_LOG_EKF_QUAT(const SbgBinaryLogData *pLogData) const;
    bv_sbglog_data parser_for_SBG_ECOM_LOG_EKF_NAV(const SbgBinaryLogData *pLogData) const;
    bv_sbglog_data parser_for_SBG_ECOM_LOG_SHIP_MOTION(const SbgBinaryLogData *pLogData) const;

    // -------------------------------------------------------------------------
    // urls:
    // - http://www.boost.org/doc/libs/1_55_0/doc/html/function/tutorial.html#idp95767816
    // - http://stackoverflow.com/questions/2136998/using-a-stl-map-of-function-pointers
    typedef boost::function<bv_sbglog_data(const SBGLogParser2*, const SbgBinaryLogData*)> fun_t;
    typedef std::map<const SbgEComLog, const fun_t> funs_t;

    // urls:
    // - http://stackoverflow.com/questions/138600/initializing-a-static-stdmapint-int-in-c
    funs_t f = {
        { SBG_ECOM_LOG_EKF_QUAT,    &SBGLogParser2::parser_for_SBG_ECOM_LOG_EKF_QUAT    },
        { SBG_ECOM_LOG_EKF_NAV,     &SBGLogParser2::parser_for_SBG_ECOM_LOG_EKF_NAV     },
        { SBG_ECOM_LOG_SHIP_MOTION, &SBGLogParser2::parser_for_SBG_ECOM_LOG_SHIP_MOTION },
    };
    // -------------------------------------------------------------------------

    //    ros::Publisher pub_for_SbgLogEkfEulerData;
    std::map<std::type_index, ros::Publisher> map_sbglog_pub;
    //    const std::map<std::type_index, ros::Publisher> map_sbglog_pub = {
    //        { typeid(sbg_driver::SbgLogEkfEulerData), ros::Publisher() }
    //    };
};
#endif // SBGLOGPARSER2_H
