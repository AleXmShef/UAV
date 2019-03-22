#ifndef UAV_LOGGABLE_H
#define UAV_LOGGABLE_H

#include <vector>
#include <string>
#include <map>
#include "Logger.h"



namespace UAV {
    class Loggable {
    public:
        Loggable(){};
        Loggable(std::string name): mName(name){};
        virtual std::map<std::string, double>* GetLogInfo() { return nullptr;};
        std::string mName = "NameIsNotSet";
    };
}



#endif //UAV_LOGGABLE_H
