#ifndef SBGLOGTOROSMSG_H
#define SBGLOGTOROSMSG_H

#include "ros/ros.h"
//
#include "SBGLogtoROSPublisher.h"
//
#include <sbgEComLib.h>
#include <sbgEComIds.h>
//
#include "sbg_driver/SbgLogEkfNavData.h"
#include "sbg_driver/SbgLogEkfEulerData.h"
#include "sbg_driver/SbgLogShipMotionData.h"
#include "sbg_driver/SbgLogUtcData.h"
#include "sbg_driver/SbgLogStatusData.h"
#include "sbg_driver/SbgLogImuData.h"
//
#include <boost/variant.hpp>

// Bridge SBG Logs Datas to ROS Messages
class SBGLogtoROSMsg {
public:
    SBGLogtoROSMsg(ros::NodeHandle _n=ros::NodeHandle(),
                   ros::NodeHandle _private_nh=ros::NodeHandle("~"))
    {
        sbglog_rospublisher.reset(new SBGLogtoROSPublisher(_n, _private_nh));
    }

    // url: en.cppreference.com/w/cpp/language/override
    SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
                               SbgEComClass msgClass,
                               SbgEComMsgId msg,
                               const SbgBinaryLogData *pLogData);

private:    
    // -------------------------------------------------------------------------
    // Construction d'un map qui fait le pont entre les types (enum) des données
    // renvoyées par log de l'API SBG vers des fonctions de parsing pour construire
    // les messages ROS correspondant. Les types de retour des fonctions de parsing
    // sont boost::variant sur les types des datas des logs.
    // L'utilisation des boost::variant peut etre vu comme une version C++ des 'unions'
    // utilisées par l'API C SBG. Ca permet d'avoir un conteneur
    // générique mais qui limite les types possibles (controle à la compilation).
    //
    // ps: Une grande partie des codes pourraient être méta-générées (meta-prog).
    // C'est partiellement le cas par l'utilisation de MACROs dans l'implémentation.
    // Mais tout (déclaration/implémentation) pourrait être généré à la compile.
    // Ca rendrait surement le code moins lisible ... à voir (peut être une autre
    // branche pour faire ces tests de refactoring).
    //
    // urls:
    // - http://www.boost.org/doc/libs/1_55_0/doc/html/function/tutorial.html#idp95767816
    // - http://stackoverflow.com/questions/2136998/using-a-stl-map-of-function-pointers
    // - http://stackoverflow.com/questions/138600/initializing-a-static-stdmapint-int-in-c

    // boost::variant sur les messages ROS correspondants à leurs équivalents des
    // types enum dans la lib SBG.
    typedef
    boost::variant<
    sbg_driver::SbgLogEkfEulerData,
    sbg_driver::SbgLogEkfNavData,
    sbg_driver::SbgLogShipMotionData,
    sbg_driver::SbgLogUtcData,
    sbg_driver::SbgLogStatusData,
    sbg_driver::SbgLogImuData
    > bv_sbglog_data;

    // Prototypes des fonctions de parsing:
    // SBGLogData -> BOOST::VARIANT(ROS_MSG)
    bv_sbglog_data parser_for_SBG_ECOM_LOG_EKF_QUAT     (const SbgBinaryLogData *pLogData) const;
    bv_sbglog_data parser_for_SBG_ECOM_LOG_EKF_NAV      (const SbgBinaryLogData *pLogData) const;
    bv_sbglog_data parser_for_SBG_ECOM_LOG_SHIP_MOTION  (const SbgBinaryLogData *pLogData) const;
    bv_sbglog_data parser_for_SBG_ECOM_LOG_UTC_DATA     (const SbgBinaryLogData *pLogData) const;
    bv_sbglog_data parser_for_SBG_ECOM_LOG_STATUS       (const SbgBinaryLogData *pLogData) const;
    bv_sbglog_data parser_for_SBG_ECOM_LOG_IMU          (const SbgBinaryLogData *pLogData) const;

    typedef boost::function<bv_sbglog_data(const SBGLogtoROSMsg*, const SbgBinaryLogData*)> fun_t;
    typedef std::map<const SbgEComLog, const fun_t> funs_t;
    funs_t f = {
        { SBG_ECOM_LOG_EKF_QUAT,    &SBGLogtoROSMsg::parser_for_SBG_ECOM_LOG_EKF_QUAT   },
        { SBG_ECOM_LOG_EKF_NAV,     &SBGLogtoROSMsg::parser_for_SBG_ECOM_LOG_EKF_NAV    },
        { SBG_ECOM_LOG_SHIP_MOTION, &SBGLogtoROSMsg::parser_for_SBG_ECOM_LOG_SHIP_MOTION},
        { SBG_ECOM_LOG_UTC_TIME,    &SBGLogtoROSMsg::parser_for_SBG_ECOM_LOG_UTC_DATA   },
        { SBG_ECOM_LOG_STATUS,      &SBGLogtoROSMsg::parser_for_SBG_ECOM_LOG_STATUS     },
        { SBG_ECOM_LOG_IMU_DATA,    &SBGLogtoROSMsg::parser_for_SBG_ECOM_LOG_IMU        }
    };
    // -------------------------------------------------------------------------

    boost::shared_ptr<SBGLogtoROSPublisher> sbglog_rospublisher;
};

#endif // SBGLOGTOROSMSG_H
