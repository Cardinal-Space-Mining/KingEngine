from typing import Callable
import time
from custom_types.msg import Location   

def localization_main(publish_location_cb: Callable[Location, None]):
    time.sleep(2)
    publish_location_cb(2,23)

