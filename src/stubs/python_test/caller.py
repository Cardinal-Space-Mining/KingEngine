import sys 
import os
sys.path.append(os.path.abspath("./.."))

from localization_stub import LocationSystem


if __name__ == "__main__":
    lsys = LocationSystem()
    while True:
        x = float(input("\nGive X: "))
        y = float(input("\nGive Y: "))
        if x == 0 and y == 0:
            del lsys
            exit()
        lsys.set_robot_location(x,y)
