from multiprocessing.shared_memory import SharedMemory
import os
import struct

class LocationSystem:
    __sh_mem__ : SharedMemory

    def __init__(self) -> None:
        self.__sh_mem__ = SharedMemory(name="/LOCATION_SHM", create=False, size=8)
    
    def __del__(self) -> None:
        self.__sh_mem__.close()

    def set_robot_location(self, x_meters: float, y_meters: float):
        x : int =int(x_meters * 1000)
        y : int =int(y_meters * 1000)
        struct.pack_into("@ii", self.__sh_mem__.buf, 0 ,x,y)