import subprocess
import os
from datetime import datetime
import threading
import psutil

TOPICS="/cloud_all_fields_fullframe /filtered_imu /filtered_cloud /ImageRight /ImageLeft /ImageCenter"
ROS_BAG_PROC_INIT = ["ros2", "wtf", "bag", "record", TOPICS, "--compression-mode", "file", "--compression-format", "zstd"]

SECONDS_PER_BAG = 10
OVERLAP_SECONDS = 5

recorders = []
running = True

def create_next_bag(bag_num, last_bag):
    global running
    if not running:
        return
    BagRecorder(bag_num, SECONDS_PER_BAG)
    print(f"Bag number {bag_num} started, press enter to cancel")

    killer_timer = threading.Timer(OVERLAP_SECONDS, last_bag.end_proc)
    killer_timer.start()


class BagRecorder:
    def __init__(self, num, slice_time):
        recorders.append(self)
        self.num = num

        # self.sp = subprocess.Popen["ros2", "wtf", "bag", "record", TOPICS, "--compression-mode", "file", "--compression-format", "zstd", "-o", f"ros-bag-slice-{num}"], shell=True)
        self.proc = subprocess.Popen(["ros2", "bag", "record"] + TOPICS.split(' ') + ["--compression-mode", "file", "--compression-format", "zstd", "-o", f"ros-bag-slice-{num}"], stdout=subprocess.PIPE)

        self.timer = threading.Timer(slice_time, create_next_bag, (num+1, self))
        self.timer.start()

    def end_proc(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            print(f"Bag number {self.num} killed successfully")

def cleanup():
    for br in recorders:
        br.end_proc()


def main():
    global running
    # folder is created of the format "rosbag-collection-ddmmyy-HHMMSS"
    folder_name: str = f"rosbag-collection-" + datetime.now().strftime("%d%m%y-%H%M%S")

    subprocess.call(["mkdir", folder_name])
    os.chdir(folder_name)

    BagRecorder(0, 1)

    input("Bag recording started, press enter to exit program\n")
    running = False
    cleanup()
    print("Exited successfully")

if __name__ == "__main__":
    main()