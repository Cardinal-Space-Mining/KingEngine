from typing import Callable
import time
def localization_main(publish_location_cb: Callable[[float, float], None]):
    time.sleep(2)
    publish_location_cb(2,23)

