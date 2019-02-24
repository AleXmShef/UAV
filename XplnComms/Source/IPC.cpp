#include "IPC.h"

using namespace UAV;

IPC* IPC::mInstance = nullptr;

IPC::IPC() {
    //create shared memory object & mapped region
    _segment = new managed_shared_memory(create_only, SHRDMEM_NAME, 65335);
    _alloc_inst = new ShmemAllocator(_segment->get_segment_manager());
    //log maybe?
}

IPC* IPC::GetInstance() {
    if (mInstance == nullptr) {
        mInstance = new IPC();
        return mInstance;
    }
    return mInstance;
}

IPCSharedMap* IPC::registerData(UAV::IPCSharedMap *Dptr, char* Dname) {
    IPCSharedMap* ptr =
            _segment->construct<IPCSharedMap>(Dname)(std::less<std::string>(), _alloc_inst);
    return ptr;
}


