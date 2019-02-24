#include "IPC.h"

using namespace UAV;

IPC* IPC::mInstance = nullptr;
managed_shared_memory* IPC::_segment;
ShmemAllocator* IPC::_alloc_inst;

IPC::IPC() {
    //create shared memory object & mapped region
    _segment = new managed_shared_memory(create_only, SHRDMEM_NAME, 16355);
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
    Dptr = _segment->construct<IPCSharedMap>(Dname)(std::less<int>(), (*_alloc_inst));
    return Dptr;
}

IPCSharedMap* IPC::findData(UAV::IPCSharedMap *Dptr, char *Dname) {
    Dptr = _segment->find<IPCSharedMap>(Dname).first;
    return Dptr;
}


