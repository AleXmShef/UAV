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
    enum CustomFileLogType {TimeStamp, ClockStamp, TimeDelta, Clear};
    class Logger {
    public:
        static Logger* GetInstance();
        void registerLoggable(Loggable* loggable);
        void registerLoggable(Loggable* loggable, LogType logtype);
        void logIntoMainLogFile(Loggable* loggable, std::string logString);
        void logConsole();
        void logCustomFiles();
        void registerCustomFileLogging(std::string fileName, CustomFileLogType logType, std::string(*logger)(void), int timeInterval);

        ~Logger();
    protected:
        Logger();

        static Logger* mInstance;

        std::string mMainLogFilePath;
        std::ofstream mMainLogFileStream;

        std::map<std::string, CustomFileLogType> mCustomLogFileLogTypes;
        std::map<std::string, std::string(*)(void)> mCustomLogFileLoggerFuncs;
        std::map<std::string, int> mCustomLogFileTimeIntervals;


        std::vector<Loggable*> mLoggableVec;
        std::vector<LogType> mLoggableVecType;

        clock_t t = 0;
        clock_t tRefill = 0;
        int i = 0;
        int yCoord = 0;
        int xCoord = 0;

        CONSOLE_SCREEN_BUFFER_INFO mScreenBufferInfo;
        HANDLE mConsole;
        DWORD mWritten;
        DWORD mCells;
    };
}

#endif //UAV_LOGGER_H
