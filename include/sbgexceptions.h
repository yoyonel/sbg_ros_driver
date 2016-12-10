#ifndef SBGEXCEPTIONS_H
#define SBGEXCEPTIONS_H


#define THROW_EXCEPTION(func_name, exception_name)   \
    if (func_name() != SBG_NO_ERROR) \
    throw exception_name();

//#define THROW_EXCEPTION(func_name, exception_name)   \
//    if (func_name() != SBG_NO_ERROR) \
//    ROS_ERROR_STREAM("Exception: " << #exception_name);


// ------------------
// EXCEPTIONS
// ------------------
// urls:
// - http://www.cplusplus.com/doc/tutorial/exceptions/
// - https://www.youtube.com/watch?v=Ix05fozWn0A
// - http://stackoverflow.com/questions/32890192/error-noexcept-does-not-name-a-type
// - http://geosoft.no/development/cppstyle.html
// - https://www.quora.com/How-does-one-write-a-custom-exception-class-in-C++

struct sbgExceptions:std::exception{
    char const *what() const noexcept{
        return "sbg Error";
    }
};

struct sbgInterfaceSerialCreateException:sbgExceptions{
//class sbgInterfaceSerialCreateException : public std::exception {
    char const *what() const noexcept{
        return "sbgInterfaceSerialCreate Error";
    }
};
//} sbgInterfaceSerialCreateException;

struct sbgEComInitException:sbgExceptions{
    char const *what() const noexcept{
        return "sbgEComInit Error";
    }
};

struct sbgEComCmdGetInfoException:sbgExceptions{
    char const *what() const noexcept{
        return "sbgEComCmdGetInfo Error";
    }
};

struct sbgEComCmdOutputSetConfException:sbgExceptions{
    char const *what() const noexcept{
        return "sbgEComCmdOutputSetConf Error";
    }
};

struct sbgEComCmdSettingsActionException:sbgExceptions{
    char const *what() const noexcept{
        return "sbgEComCmdSettingsAction Error";
    }
};

struct sbgEComHandleException:sbgExceptions{
    char const *what() const noexcept{
        return "sbgEComHandle Error";
    }
};

// ------------------

#endif // SBGEXCEPTIONS_H
