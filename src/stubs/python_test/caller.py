import sys 
import os
sys.path.append(os.path.abspath("./.."))

from localization_stub import LocationSystem


if __name__ == "__main__":
    sys = LocationSystem()
    while True:
        x = float(input("\nGive X: "))
        y = float(input("\nGive X: "))
        sys.set_robot_location(x,y)
