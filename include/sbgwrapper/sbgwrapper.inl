#ifndef SBGWRAPPER_INL
#define SBGWRAPPER_INL

#include "sbgwrapper.h"

template<typename TParser>
void SBGWrapper::initialize()
{
    THROW_EXCEPTION(createSerialInterface, sbgInterfaceSerialCreateException);
    serialinterface_is_init = true;

    THROW_EXCEPTION(initECom, sbgEComInitException);
    ecom_is_init = true;

    THROW_EXCEPTION(getDeviceInfo, sbgEComCmdGetInfoException);

    // Generic log_parser (used with template typename argument)
    log_parser.reset(new TParser(n, private_nh));

    log_parser->init();
}

template<typename TParser>
void SBGWrapper::_set_callback_for_logs(SbgEComHandle &_comHandle,
                                        TParser *_this)
{
    // On set le callback sur une fonction statique.
    // On utilise 'pUserArg' pour transmettre le pointeur sur l'instance de la
    // classe SBGWrapper (this).
    // Ainsi dans le callback, on doit pouvoir reprendre la main sur cette classe
    // wrapper (et par la suite accéder au Parser (des logs)).
    sbgEComSetReceiveLogCallback(&_comHandle,
                                 SBGWrapper::static_onLogReceived<TParser>,
                                 _this);
}

template< typename TParser >
SbgErrorCode SBGWrapper::static_onLogReceived(SbgEComHandle *pHandle,
                                              SbgEComClass msgClass,
                                              SbgEComMsgId msg,
                                              const SbgBinaryLogData *pLogData,
                                              void *pUserArg)
{
    TParser* ptr_sbgwrapper = static_cast<TParser*>(pUserArg);
    return ptr_sbgwrapper->onLogReceived(pHandle, msgClass, msg, pLogData);
}

#endif
