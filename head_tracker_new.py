import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata2 as pyfirmata
import numpy as np
import time

cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

port = "COM3"
board = pyfirmata.Arduino(port)
time.sleep(3)
board.servo_config(9)
board.servo_config(10)
servo_pinX = board.get_pin('d:9:s')
servo_pinY = board.get_pin('d:10:s')

detector = FaceDetector()

# Set initial position to your requested values
servo_pinX.write(70)
servo_pinY.write(165)
time.sleep(2)

servoPos = [70.0, 165.0]  # Match the initial position - use floats for smoothness

# Create window and make it fullscreen
cv2.namedWindow("Image", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("Image", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        #get the coordinate
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        pos = [fx, fy]
        #convert coordinat to servo degree - WIDER ranges for more movement
        servoX = np.interp(fx, [0, ws], [0, 180])  # Full X range 0-180
        servoY = np.interp(fy, [0, hs], [100, 180])  # Wide Y range 100-180

        if servoX < 0:
            servoX = 0
        elif servoX > 180:
            servoX = 180
        if servoY < 0:
            servoY = 0
        elif servoY > 180:
            servoY = 180

        # Smooth movement - gradually move towards target position
        servoPos[0] = servoPos[0] + (servoX - servoPos[0]) * 0.3  # 30% of the way each frame
        servoPos[1] = servoPos[1] + (servoY - servoPos[1]) * 0.3  # Adjust 0.3 for more/less smoothness

        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    else:
        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
        cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
        cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)
        cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)

    cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, "Press 'F' for fullscreen, 'ESC' to exit", (50, 150), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

    servo_pinX.write(servoPos[0])
    servo_pinY.write(servoPos[1])

    cv2.imshow("Image", img)
    
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC key to exit
        break
    elif key == ord('f') or key == ord('F'):  # F key to toggle fullscreen
        prop = cv2.getWindowProperty("Image", cv2.WND_PROP_FULLSCREEN)
        if prop == cv2.WINDOW_FULLSCREEN:
            cv2.setWindowProperty("Image", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        else:
            cv2.setWindowProperty("Image", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

cap.release()
cv2.destroyAllWindows()
board.exit()