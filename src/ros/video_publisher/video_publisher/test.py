import cv2
import time

import contextlib
IMX179_MJPG_settings = [
{
"resolution":(3264,2448),
"fps":15
},
{
"resolution":(2592,1944),
"fps":15
},{
"resolution":(1920,1080),
"fps":30
},{
"resolution":(1600,1200),
"fps":30
},{
"resolution":(1280,720),
"fps":30
},{
"resolution":(960,540),
"fps":30
},{
"resolution":(848,480),
"fps":30
},{
"resolution":(640,480),
"fps":30
},{
"resolution":(640,360),
"fps":30
},{
"resolution":(424,240),
"fps":30
},{
"resolution":(320,240),
"fps":30
},{
"resolution":(320,180),
"fps":30
}
]


@contextlib.contextmanager
def escapable():
    class Escape(RuntimeError): pass
    class Unblock(object):
        def escape(self):
            raise Escape()

    try:
        yield Unblock()
    except Escape:
        pass


def apply_mjpg_setting(cap, setting):
    cap.set(cv2.CAP_PROP_FPS, setting["fps"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, setting["resolution"][0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, setting["resolution"][1])
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

def main():
      

    
    for setting in IMX179_MJPG_settings:

        while True:
            # Open the USB camera by path
            cap = cv2.VideoCapture("/dev/video4")
            # cap = cv2.VideoCapture(1, cv2.CAP_V4L2)

            time.sleep(1)

            # Check if camera opened successfully
            if not cap.isOpened():
                print("Error: Unable to open camera.")
                cap.release()
                continue
            else:
                break
    
        print(f"Applying {setting}")
        apply_mjpg_setting(cap, setting)

        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()

            # If frame is read correctly, ret is True
            if ret:
                # Display the captured frame
                cv2.imshow('USB Camera', frame)
                print(f"Frame Size: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)} x {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
                print(f"fps: {cap.get(cv2.CAP_PROP_FPS)}")
                print(f"encoding: {cap.get(cv2.CAP_PROP_FOURCC)}")
            else:
                print("No frame :(")
                break
            
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release the capture
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
