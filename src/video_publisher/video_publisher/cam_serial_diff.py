import glob
import os
import subprocess

names = glob.glob("/dev/video*")

class CameraInfo:
    __slots__ = ["camera_name", "video_stream", "serial_number"]
    camera_name : str
    video_stream : str
    serial_number : str

    def __str__(self) -> str:
        return f"{{camera_name: {self.camera_name}, video_stream: {self.video_stream}, serial_number: {self.serial_number}}}"

def get_video_info(name: str) -> any:
    string = subprocess.run(["udevadm", "info", "--query=all", "--name=*".replace("*", name)], capture_output=True).stdout.decode()
    # print(string)
    lines = string.split('\n')
    info = CameraInfo()

    info.camera_name = ''
    info.serial_number = ''
    info.video_stream = ''

    for line in lines:
        if "ID_SERIAL_SHORT" in line:
            info.serial_number = line.split('=')[1]
    
        if "ID_V4L_PRODUCT" in line:
            info.camera_name = line.split('=')[1]

        if "DEVNAME" in line:
            info.video_stream = line.split('=')[1]
    return info

for name in names:
    print(get_video_info(name))