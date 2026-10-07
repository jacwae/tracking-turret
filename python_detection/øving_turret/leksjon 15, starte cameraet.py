
import cv2 
from picamera2 import Picamera2
#skaper framen
piCam = Picamera2()
W = 1280
H = 720
RES = (W,H)

#configurerer kameraet
piCam.preview_configuration.main.size = RES
piCam.preview_configuration.main.format = "RGB888"
piCam.preview_configuration.align()
piCam.configure("preview")
piCam.start()
#hwile loop som starter video
while True:
    frame = piCam.capture_array()
    #frame = cv2.flip(frame, -1)
    cv2.imshow("photo", frame)
    cv2.moveWindow("photo", 0,60)
    if cv2.waitKey(1) == ord("q"):
        break
    
cv2.destroyAllWindows(())
print('Program terminated')
