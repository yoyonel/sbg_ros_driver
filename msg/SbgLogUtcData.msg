# /*!
#  * Structure that stores data for the SBG_ECOM_LOG_UTC_TIME message.
#  */
# typedef struct _SbgLogUtcData
# {
#   uint32  timeStamp;                  /*!< Time in us since the sensor power up. */
#   uint16  status;                     /*!< UTC time and clock status information */
#   uint16  year;                       /*!< Year for example: 2013. */
#   int8    month;                      /*!< Month in year [1 .. 12]. */
#   int8    day;                        /*!< Day in month [1 .. 31]. */
#   int8    hour;                       /*!< Hour in day [0 .. 23]. */
#   int8    minute;                     /*!< Minute in hour [0 .. 59]. */
#   int8    second;                     /*!< Second in minute [0 .. 60]. (60 is used only when a leap second is added) */
#   int32   nanoSecond;                 /*!< Nanosecond of current second in ns. */
#   uint32  gpsTimeOfWeek;              /*!< GPS time of week in ms. */
# } SbgLogUtcData;

uint32  timeStamp
uint16  status
uint16  year
int8    month
int8    day
int8    hour
int8    minute
int8    second
int32   nanoSecond
uint32  gpsTimeOfWeek
