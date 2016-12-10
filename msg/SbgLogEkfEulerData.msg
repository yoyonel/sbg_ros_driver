# typedef struct _SbgLogEkfEulerData
# {
#         uint32  timeStamp;              /*!< Time in us since the sensor power up. */
#         float   euler[3];               /*!< Roll, Pitch and Yaw angles in rad. */
#         float   eulerStdDev[3];         /*!< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad. */
#         uint32  status;                 /*!< EKF solution status bitmask and enum. */
# } SbgLogEkfEulerData;

uint32  timeStamp
float32[3]   euler
float32[3]   eulerStdDev
uint32  status
