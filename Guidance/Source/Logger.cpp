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
}

void Logger::registerLoggable(UAV::Loggable *loggable) {
    mLoggableVec.push_back(loggable);
}

void Logger::logConsole() {
    clock_t temp;
    temp = clock();
    if (temp > t) {
        if (i > mLoggableVec.size() - 1) {
            i = 0;
            COORD tl = {0, 0};
            SetConsoleCursorPosition(mConsole, tl);
        }
        //FillConsoleOutputCharacter(mConsole, ' ', mCells, tl, &mWritten);
        if (!mLoggableVec.empty()) {
            if (mLoggableVec[i]->getLogInfo() != nullptr) {
                std::cout << mLoggableVec[i]->mName << std::endl;
                auto tMap = mLoggableVec[i]->getLogInfo();
                std::map<std::string, double>::iterator j;
                for (j = tMap->begin(); j != tMap->end(); j++) {
                    std::cout << j->first << ": " << j->second<< std::endl;
                }
                delete (tMap);
            }
        }
        std::cout << std::endl;
        i++;
    }

    t = temp;
}
