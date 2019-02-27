#ifndef UAV_IPC_H
#define UAV_IPC_H

#include "boost/interprocess/managed_shared_memory.hpp"
#include "boost/interprocess/sync/interprocess_mutex.hpp"
#include "boost/interprocess/containers/map.hpp"
#include "boost/interprocess/allocators/allocator.hpp"

using namespace boost::interprocess;

namespace IPCns {
    //Typedefing STL-like map container-----------------------------------------------------------
    typedef int KeyType;
    typedef float MappedType;
    typedef std::pair<const int, float> ValueType;
    typedef allocator<ValueType, managed_shared_memory::segment_manager>ShmemAllocator;
    typedef map<KeyType, MappedType, std::less<KeyType>, ShmemAllocator> IPCSharedMap;

    //--------------------------------------------------------------------------------------------

    class IPC {
    public:
        static IPC* GetInstance();
        ~IPC();
        static IPCSharedMap* registerData(IPCSharedMap* Dptr, char* Dname);
        static IPCSharedMap* findData(IPCSharedMap* Dptr, char* Dname);
        static void lock();
        static void unlock();
        //static IPCSharedMap* unregisterData(IPCSharedMap* Dptr);
        //struct 1 - outgoing data
        //struct 2 - incoming data
    private:
        IPC();
        static managed_shared_memory* _segment;
        static interprocess_mutex* _mutex;
        static ShmemAllocator* _alloc_inst;
        static IPC* mInstance;
    };
}
#endif //UAV_IPC_H
