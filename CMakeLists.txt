cmake_minimum_required(VERSION 3.27)
project(KingEngine)

set(CMAKE_CXX_STANDARD 14)

include_directories(src/stubs)
include_directories(src/subsystems)

add_executable(KingEngine
        src/stubs/destination_sys_test/listener.cpp
        src/stubs/destination_sys_test/speaker.cpp
        src/stubs/fix_ipc_utility/main.cpp
        src/stubs/notif_sys_test/notifier.cpp
        src/stubs/notif_sys_test/waiter.cpp
        src/stubs/python_test/reader.cpp
        src/stubs/destination.cpp
        src/stubs/destination.hpp
        src/stubs/location.cpp
        src/stubs/location.hpp
        src/stubs/notif_sys.cpp
        src/stubs/notif_sys.hpp
        src/stubs/transfer_types.hpp
        src/stubs/util.hpp
        src/subsystems/Collection.cpp
        src/subsystems/Hopper.cpp
        src/subsystems/Hopper.h
        src/main.cpp)
