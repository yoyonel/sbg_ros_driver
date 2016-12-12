# /*!
#  * Structure that stores data for the SBG_ECOM_LOG_IMU_DATA message.
#  */
# typedef struct _SbgLogImuData
# {
#   uint32  timeStamp;                  /*!< Time in us since the sensor power up. */
#   uint16  status;                     /*!< IMU status bitmask. */
#   float   accelerometers[3];          /*!< X, Y, Z accelerometers in m.s^-2. */
#   float   gyroscopes[3];              /*!< X, Y, Z gyroscopes in rad.s^-1. */
#   float   temperature;                /*!< Internal temperature in �C. */ 
#   float   deltaVelocity[3];           /*!< X, Y, Z delta velocity in m.s^-2. */
#   float   deltaAngle[3];              /*!< X, Y, Z delta angle in rad.s^-1. */
# } SbgLogImuData;

uint32      timeStamp
uint16      status
float32[3]  accelerometers
float32[3]  gyroscopes
float32     temperature
float32[3]  deltaVelocity
float32[3]  deltaAngle
