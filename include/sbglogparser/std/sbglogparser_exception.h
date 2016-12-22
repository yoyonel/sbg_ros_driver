#ifndef WRAPPER_EXCEPTION_H
#define WRAPPER_EXCEPTION_H

#include <string>   // for std::string
#include "sbglogparser_specialization.h"

struct WrapperExceptions:std::exception{
    char const *what() const noexcept{
        return "wrapper Error";
    }
};

class SbgEComMsgIdException : public WrapperExceptions{
public:
    SbgEComMsgIdException(const SbgEComMsgId& _msg) : msg(_msg) {}

    char const *what() const noexcept{
        // url: http://www.cplusplus.com/reference/sstream/stringstream/
        std::stringstream ss;
        ss << "msg=" << std::to_string(msg);
        if(map_msg_to_string.count(msg))
              ss << " -> " << std::string(map_msg_to_string.at(msg));
        ss << " is not (yet) managed !";
        // convert ss -> string -> to const char *
        return ss.str().c_str();
    }
private:
    SbgEComMsgId msg;
};

#endif // WRAPPER_EXCEPTION_H
