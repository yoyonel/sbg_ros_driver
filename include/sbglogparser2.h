#ifndef SBGLOGPARSER2_H
#define SBGLOGPARSER2_H

#include "sbglogparser.h"
//
#include <boost/function.hpp>

// Test de spécialisation d'un LogParser pour SBG

class SBGLogParser2 : SBGLogParser {
public:
    // url: en.cppreference.com/w/cpp/language/override
    SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
                               SbgEComClass msgClass,
                               SbgEComMsgId msg,
                               const SbgBinaryLogData *pLogData) override;
private:
    void parser_for_SBG_ECOM_LOG_EKF_QUAT(const SbgBinaryLogData*);
    void parser_for_SBG_ECOM_LOG_EKF_NAV(const SbgBinaryLogData*);
    void parser_for_SBG_ECOM_LOG_SHIP_MOTION(const SbgBinaryLogData*);

    // urls:
    // - http://www.boost.org/doc/libs/1_55_0/doc/html/function/tutorial.html#idp95767816
    // - http://stackoverflow.com/questions/2136998/using-a-stl-map-of-function-pointers
    typedef boost::function<void (SBGLogParser2*, const SbgBinaryLogData*)> fun_t;
    typedef std::map<SbgEComLog, fun_t> funs_t;

    // urls:
    // - http://stackoverflow.com/questions/138600/initializing-a-static-stdmapint-int-in-c
    funs_t f = {
        { SBG_ECOM_LOG_EKF_QUAT,    &SBGLogParser2::parser_for_SBG_ECOM_LOG_EKF_QUAT    },
        { SBG_ECOM_LOG_EKF_NAV,     &SBGLogParser2::parser_for_SBG_ECOM_LOG_EKF_NAV     },
        { SBG_ECOM_LOG_SHIP_MOTION, &SBGLogParser2::parser_for_SBG_ECOM_LOG_SHIP_MOTION },
    };
};
#endif // SBGLOGPARSER2_H
