import subprocess
import re

def get_camera_info():
    endpoints = subprocess.run(("v4l2-ctl", "--list-devices"), capture_output=True).stdout.decode().splitlines()
    cameras = []
    for line in endpoints:
        if line != '' and line[0] == '\t':
            cameras.append(line.replace('\t', ''))

    print(cameras)

    cams = []
    for camera in cameras:
        lines = subprocess.run(("v4l2-ctl", "-d", camera, "--info"), capture_output=True).stdout.decode().splitlines()
        for line in lines:
            if 'Serial' in line:
                _, serial = line.replace('\t', '').replace(' ', '').split(':')
                cams.append((serial, camera))
                break
    return tuple(cams)

print(get_camera_info())