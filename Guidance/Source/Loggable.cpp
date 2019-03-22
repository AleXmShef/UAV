//
// Created by Sasha on 22.03.2019.
//

#include "Loggable.h"
#include "Logger.h"

using namespace UAV;

Loggable::Loggable() {
    Logger::GetInstance()->registerLoggable(this);
}