template< typename T >
SbgErrorCode SBGLogParser::static_onLogReceived(SbgEComHandle *pHandle,
                                                SbgEComClass msgClass,
                                                SbgEComMsgId msg,
                                                const SbgBinaryLogData *pLogData,
                                                void *pUserArg)
{
    T* ptr_sbgwrapper = static_cast<T*>(pUserArg);
    return ptr_sbgwrapper->onLogReceived(pHandle, msgClass, msg, pLogData);
}
