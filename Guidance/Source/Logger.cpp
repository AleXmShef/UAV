//
// Created by Sasha on 22.03.2019.
//

#include "Logger.h"

using namespace UAV;

Logger* Logger::mInstance = nullptr;

Logger* Logger::GetInstance() {
    if (mInstance == nullptr)
        Logger();
    return mInstance;
}

Logger::Logger() {
    COORD tl = {0, 0};
    mConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    GetConsoleScreenBufferInfo(mConsole, &mScreenBufferInfo);
    mCells = mScreenBufferInfo.dwSize.X * mScreenBufferInfo.dwSize.Y;
    FillConsoleOutputAttribute(mConsole, mScreenBufferInfo.wAttributes, mCells, tl, &mWritten);
    mInstance = this;
}

void Logger::registerLoggable(UAV::Loggable *loggable) {
    mLoggableVec.push_back(loggable);
}

void Logger::logConsole() {
    COORD tl = {0, 0};
    FillConsoleOutputCharacter(mConsole, ' ', mCells, tl, &mWritten);

    for(int i = 0; i < mLoggableVec.size(); i++) {
        if(mLoggableVec[i]->GetLogInfo() != nullptr) {
            auto tMap = mLoggableVec[i]->GetLogInfo();
            std::map<std::vector<std::string>*, std::vector<double>*>::iterator j;
            for(j = tMap->begin(); j != tMap->end(); j++) {
                std::cout << j->first << ": " << j->second << std::endl;
            }
            delete(tMap);
        }
    }
}