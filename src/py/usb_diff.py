import subprocess, re

def get_video_streams() -> tuple[str]:
    files = subprocess.run(("ls", "/dev"), capture_output=True).stdout.decode().split('\n')
    pattern = re.compile('video\d+')
    return tuple(f"/dev/{x}" for x in files if pattern.match(x))

def get_MJPG_video_stream_from_serial_number(num: str) -> None | str:
    video_streams: tuple[str] = get_video_streams()
    results: tuple[str] = tuple(subprocess.run(("v4l2-ctl", f"--device={stream}", "--all"), capture_output=True).stdout.decode() for stream in video_streams)
    for idx, res in enumerate(results):
        if num in res and 'MJPG' in res:
            return video_streams[idx]
        
from linuxpy.video.device import Device

with Device.from_id(0) as cam:
    for i, frame in enumerate(cam):
        print(f"frame #{i}: {len(frame)} bytes")
        if i > 9:
            break
