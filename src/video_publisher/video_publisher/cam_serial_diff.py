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

def get_camera_info(self):
        endpoints = subprocess.run(("v4l2-ctl", "--list-devices"), capture_output=True).stdout.decode().splitlines()
        cameras = []
        for line in endpoints:
            if line != '' and line[0] == '\t':
                cameras.append(line.replace('\t', ''))

        cams = []
        for camera in cameras:
            lines = subprocess.run(("v4l2-ctl", "-d", camera, "--info"), capture_output=True).stdout.decode().splitlines()
            name = None
            serial = None
            for line in lines:
                if name is not None and serial is not None:
                    break
                if 'Serial' in line:
                    _, serial = line.replace('\t', '').replace(' ', '').split(':')
                if 'Name' in line:
                    _, name = line.replace('\t', '').replace(' ', '').split(':')
            cams.append()
        return tuple(cams)

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