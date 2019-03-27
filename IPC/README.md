h1 InterProcess Communication Library
***
A small library written on C++ using <boost> to allow autopilot testing within X-Plane application
Basically contains 1 class with 2 public static methods:
    IPC::lock() to lock process shared memory for data editing
    IPC::unlock() to unlock shared memory to allow another process to gather acces
Has its own defined namespace IPCns