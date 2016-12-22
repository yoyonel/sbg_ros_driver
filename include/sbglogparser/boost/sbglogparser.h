#ifndef SBGLOGTOROSMSG_H
#define SBGLOGTOROSMSG_H

#include "ros/ros.h"
#include "sbglogparser/isbglogparser.h"
//
#include "SBGLogtoROSPublisher.h"

//
#include <sbgEComLib.h>
#include <sbgEComIds.h>

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
#include <boost/variant.hpp>

// Bridge SBG Logs Datas to ROS Messages
namespace sbglogparser_boost {
class SBGLogParser : public ISBGLogParser {
public:
    SBGLogParser(ros::NodeHandle _n=ros::NodeHandle(),
                   ros::NodeHandle _private_nh=ros::NodeHandle("~")) :
        ISBGLogParser(_n, _private_nh)
    {
        sbglog_rospublisher.reset(new SBGLogtoROSPublisher(_n, _private_nh));
    }

    // url: en.cppreference.com/w/cpp/language/override
    SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
                               SbgEComClass msgClass,
                               SbgEComMsgId msg,
                               const SbgBinaryLogData *pLogData) override;

	virtual void init() override {}
	virtual void publish() override {}

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
	typedef boost::variant<
        sbg_driver::SbgLogStatusData
	    ,sbg_driver::SbgLogImuData
	    ,sbg_driver::SbgLogMagCalib
	    ,sbg_driver::SbgLogEkfQuatData
	    ,sbg_driver::SbgLogDvlData
	    ,sbg_driver::SbgLogUsblData
	    ,sbg_driver::SbgLogPressureData
	    ,sbg_driver::SbgLogFastImuData
	    ,sbg_driver::SbgLogOdometerData
	    ,sbg_driver::SbgLogStatusData
	    ,sbg_driver::SbgLogEkfEulerData
	    ,sbg_driver::SbgLogShipMotionData
	    ,sbg_driver::SbgLogEkfNavData
	    ,sbg_driver::SbgLogUtcData
	    ,sbg_driver::SbgLogMag
	    ,sbg_driver::SbgLogDebug0Data
	> bv_sbglog_data;

    // Prototypes des fonctions de parsing:
    // SBGLogData -> BOOST::VARIANT(ROS_MSG)
	bv_sbglog_data parser_for_SbgLogImuData	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogMagCalib	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogEkfQuatData	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogDvlData	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogUsblData	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogPressureData	(const SbgBinaryLogData *pLogData) const;
//	bv_sbglog_data parser_for_SbgLogFastImuData	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogOdometerData	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogStatusData	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogEkfEulerData	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogShipMotionData	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogEkfNavData	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogUtcData	(const SbgBinaryLogData *pLogData) const;
	bv_sbglog_data parser_for_SbgLogMag	(const SbgBinaryLogData *pLogData) const;
//	bv_sbglog_data parser_for_SbgLogDebug0Data	(const SbgBinaryLogData *pLogData) const;
    
    typedef boost::function<bv_sbglog_data(const SBGLogParser*, const SbgBinaryLogData*)> fun_t;
	typedef std::map<const SbgEComLog, const fun_t> funs_t;
	funs_t f = {
        { SBG_ECOM_LOG_STATUS,    &SBGLogParser::parser_for_SbgLogStatusData  }
        ,{ SBG_ECOM_LOG_IMU_DATA,    &SBGLogParser::parser_for_SbgLogImuData	}
        ,{ SBG_ECOM_LOG_EKF_EULER,    &SBGLogParser::parser_for_SbgLogEkfEulerData	}
        ,{ SBG_ECOM_LOG_EKF_QUAT,    &SBGLogParser::parser_for_SbgLogEkfQuatData	}
        ,{ SBG_ECOM_LOG_EKF_NAV,    &SBGLogParser::parser_for_SbgLogEkfNavData	}
        ,{ SBG_ECOM_LOG_SHIP_MOTION,    &SBGLogParser::parser_for_SbgLogShipMotionData	}
        ,{ SBG_ECOM_LOG_SHIP_MOTION_HP,    &SBGLogParser::parser_for_SbgLogShipMotionData	}
        ,{ SBG_ECOM_LOG_ODO_VEL,    &SBGLogParser::parser_for_SbgLogOdometerData	}
        ,{ SBG_ECOM_LOG_UTC_TIME,    &SBGLogParser::parser_for_SbgLogUtcData	}
        ,{ SBG_ECOM_LOG_MAG,    &SBGLogParser::parser_for_SbgLogMag	}
        ,{ SBG_ECOM_LOG_MAG_CALIB,    &SBGLogParser::parser_for_SbgLogMagCalib	}
        ,{ SBG_ECOM_LOG_DVL_BOTTOM_TRACK,    &SBGLogParser::parser_for_SbgLogDvlData	}
        ,{ SBG_ECOM_LOG_PRESSURE,    &SBGLogParser::parser_for_SbgLogPressureData	}
        ,{ SBG_ECOM_LOG_USBL,    &SBGLogParser::parser_for_SbgLogUsblData	}
	};
    // -------------------------------------------------------------------------

    boost::shared_ptr<SBGLogtoROSPublisher> sbglog_rospublisher;
};
}

#endif // SBGLOGTOROSMSG_H

