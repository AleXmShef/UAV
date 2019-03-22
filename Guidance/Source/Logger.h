#ifndef UAV_LOGGER_H
#define UAV_LOGGER_H

#include <iostream>
#include <windows.h>
#include "Loggable.h"

namespace UAV {
    class Logger {
    public:
        static Logger* GetInstance();
        void registerLoggable(Loggable* loggable);
        void logIntoFile(std::string logString);
        void logConsole();
    protected:
        Logger();
        std::vector<Loggable*> mLoggableVec;
        static Logger* mInstance;

        CONSOLE_SCREEN_BUFFER_INFO mScreenBufferInfo;
        HANDLE mConsole;
        DWORD mWritten;
        DWORD mCells;
    };
}

#endif //UAV_LOGGER_H
