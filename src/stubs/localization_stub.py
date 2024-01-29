from multiprocessing.shared_memory import SharedMemory
import sys
import struct

class LocationSystem:
    __sh_mem__ : SharedMemory

    def __init__(self) -> None:
        self.__sh_mem__ = SharedMemory(name="LOCATION_SHM")
    
    def __del__(self) -> None:
        self.__sh_mem__.close()

    def set_robot_location(self, x_meters: float, y_meters: float):
        x : int =int(x_meters * 1000)
        y : int =int(y_meters * 1000)
        locationI_t_struct_bytes = struct.pack("@hh",x,y)
        self.__sh_mem__[0:8] = locationI_t_struct_bytes 