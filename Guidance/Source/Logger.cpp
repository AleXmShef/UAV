#include "Logger.h"

using namespace UAV;

Logger* Logger::mInstance = nullptr;

Logger* Logger::GetInstance() {
    if (mInstance == nullptr)
        mInstance = new Logger();
    return mInstance;
}

Logger::Logger() {
    COORD tl = {0, 0};
    mConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    GetConsoleScreenBufferInfo(mConsole, &mScreenBufferInfo);
    mCells = mScreenBufferInfo.dwSize.X * mScreenBufferInfo.dwSize.Y;
    FillConsoleOutputAttribute(mConsole, mScreenBufferInfo.wAttributes, mCells, tl, &mWritten);
    system("mode 80,200");   //Set mode to ensure window does not exceed buffer size
    SMALL_RECT WinRect = {0, 0, 80, 200};   //New dimensions for window in 8x12 pixel chars
    SMALL_RECT* WinSize = &WinRect;
    SetConsoleWindowInfo(mConsole, true, WinSize);   //Set new size for windo
    GetConsoleScreenBufferInfo(mConsole, &mScreenBufferInfo);
    mCells = mScreenBufferInfo.dwSize.X * mScreenBufferInfo.dwSize.Y;
    FillConsoleOutputAttribute(mConsole, mScreenBufferInfo.wAttributes, mCells, tl, &mWritten);

    mMainLogFilePath = LOG_FILES_PATH;
    mMainLogFilePath += MAIN_LOG_FILE_NAME;
    mMainLogFileStream.open(mMainLogFilePath, std::ios::trunc);
    time_t rawtime;
    time(&rawtime);
    mMainLogFileStream << "Program started at " << ctime(&rawtime);
    mMainLogFileStream.close();
}

void Logger::registerLoggable(UAV::Loggable *loggable) {
    mLoggableVec.push_back(loggable);
    mLoggableVecType.push_back(ConsoleLog);
}

void Logger::registerLoggable(UAV::Loggable *loggable, UAV::LogType logtype) {
    mLoggableVec.push_back(loggable);
    mLoggableVecType.push_back(logtype);
}

void Logger::logConsole() {
    clock_t temp;
    temp = clock();
    if (temp > tRefill + 5000) {
        COORD tl = {0, 0};
        FillConsoleOutputCharacter(mConsole, ' ', mCells, tl, &mWritten);
        tRefill = temp;
    }
    if (temp > t) {
        if (i > mLoggableVec.size() - 1) {
            i = 0;
            COORD tl = {0, 0};
            SetConsoleCursorPosition(mConsole, tl);

        }

        if (!mLoggableVec.empty()) {
            if (mLoggableVecType[i] == ConsoleLog || mLoggableVecType[i] == CombinedLog) {
                if (mLoggableVec[i]->getLogInfo() != nullptr) {
                    std::cout << mLoggableVec[i]->mName << std::endl;
                    auto tMap = mLoggableVec[i]->getLogInfo();
                    std::map<std::string, double>::iterator j;
                    for (j = tMap->begin(); j != tMap->end(); j++) {
                        std::cout << j->first << ": " << j->second << std::endl;
                    }
                    delete (tMap);
                }
            }
            if (mLoggableVecType[i] == FileLog || mLoggableVecType[i] == CombinedLog) {
                logFile(i);
            }
        }
        std::cout << std::endl;
        i++;
    }

    t = temp;
}

void Logger::logFile(int i) {

}

void Logger::logIntoMainLogFile(Loggable* loggable, std::string logString) {
    if(loggable == NULL && logString == "Terminate") {
        time_t rawtime;
        time(&rawtime);
        mMainLogFileStream.open(mMainLogFilePath, std::ios::out | std::ios::app);
        mMainLogFileStream << "Program ended at " << ctime(&rawtime);
        mMainLogFileStream.close();
    }
    mMainLogFileStream.open(mMainLogFilePath, std::ios::out | std::ios::app);
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime (buffer,80,"%X",timeinfo);
    puts (buffer);
    mMainLogFileStream << buffer << " " << loggable->mName << ": " << logString << "\n";
    mMainLogFileStream.close();
}

Logger::~Logger() {
    //mMainLogFileStream.close();
}
