#ifndef UAV_LOGGER_H
#define UAV_LOGGER_H

#include <iostream>
#include <fstream>
#include <windows.h>
#include <time.h>
#include <thread>
#include "Loggable.h"

namespace UAV {
#define LOG_FILES_PATH "C:/Users/Sasha/Desktop/UAV/Bin/"
#define MAIN_LOG_FILE_NAME "UavLog.txt"
    class Loggable;
    enum LogType {ConsoleLog, FileLog, CombinedLog};
    class Logger {
    public:
        static Logger* GetInstance();
        void registerLoggable(Loggable* loggable);
        void registerLoggable(Loggable* loggable, LogType logtype);
        void logIntoMainLogFile(Loggable* loggable, std::string logString);
        void logConsole();
        void logFile(int i);

        ~Logger();
    protected:
        Logger();
        std::string mMainLogFilePath;
        std::ofstream mMainLogFileStream;
        std::vector<Loggable*> mLoggableVec;
        std::vector<LogType> mLoggableVecType;
        static Logger* mInstance;
        clock_t t = 0;
        clock_t tRefill = 0;
        int i = 0;

        CONSOLE_SCREEN_BUFFER_INFO mScreenBufferInfo;
        HANDLE mConsole;
        DWORD mWritten;
        DWORD mCells;
    };
}

#endif //UAV_LOGGER_H
