#ifndef UAV_LOGGABLE_H
#define UAV_LOGGABLE_H

#include <vector>
#include <string>
#include <map>
#include "Logger.h"

namespace UAV {
    class Loggable {
    public:
        Loggable();
        virtual std::map<std::vector<std::string>*, std::vector<double>*>* GetLogInfo() { return nullptr;};
        std::string mName = "NameIsNotSet";
    };
}


#endif //UAV_LOGGABLE_H
