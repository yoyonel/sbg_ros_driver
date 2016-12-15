# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  SbgLogStatusData.msg
  SbgLogImuData.msg
  SbgLogEkfEulerData.msg
  SbgLogEkfQuatData.msg
  SbgLogEkfNavData.msg
  SbgLogShipMotionData.msg
  SbgLogOdometerData.msg
  SbgLogUtcData.msg
  SbgLogGpsPos.msg
  SbgLogMag.msg
  SbgLogMagCalib.msg
  SbgLogDvlData.msg
  SbgLogPressureData.msg
  SbgLogUsblData.msg
  SbgLogDebug0Data.msg
  SbgLogFastImuData.msg
)