# Project: 零用錢發放機
透過鏡頭偵測，控制馬達旋轉方向移動阻片，從上方盒子掉出零用錢。

<img src="https://github.com/111453023/-Project/blob/main/PIC/PIC1.jpg" height="480px" width="320px" />
<img src="https://github.com/111453023/-Project/blob/main/PIC/PIC2.jpg" height="480px" width="320px" />

## Demo

Full Project Demo： https://youtu.be/3XLDWTJobOg

SG90 Test： https://youtu.be/KIroEsAjAVU 

## 所需的組件列表

 1. Raspberry pi

 2. 攝影鏡頭

 3. 伺服馬達SG90

 4. 杜邦線


## Getting Started
1. Setting up your Raspberry Pi
   
  https://www.raspberrypi.com/documentation/computers/getting-started.html

2. OpenVino with NCS2 on Raspberry Pi
   
  https://hackmd.io/HV6hQ2PHSiWlrRsfxC10SA


3. SG90 Servomotor Controlling section
   
  https://www.instructables.com/Controlling-Servo-Motor-Sg90-With-Raspberry-Pi-4/
  
  ![接線圖](https://github.com/111453023/-Project/blob/main/PIC/Completed_Schematic_No_External_Power.jpg)

```python

# We imports the GPIO module
import RPi.GPIO as GPIO
# We import the command sleep from time
from time import sleep

# Stops all warnings from appearing
GPIO.setwarnings(False)

# We name all the pins on BOARD mode
GPIO.setmode(GPIO.BOARD)
# Set an output for the PWM Signal
GPIO.setup(16, GPIO.OUT)

# Set up the  PWM on pin #16 at 50Hz
pwm = GPIO.PWM(16, 50)
pwm.start(0)  # Start the servo with 0 duty cycle ( at 0 deg position )
pwm.ChangeDutyCycle(5)  # Tells the servo to turn to the left ( -90 deg position )
sleep(1)  # Tells the servo to Delay for 1sec
pwm.ChangeDutyCycle(10)  # Tells the servo to turn to the right ( +90 deg position )
sleep(1)  # Tells the servo to Delay for 1sec
pwm.stop(0)  # Stop the servo with 0 duty cycle ( at 0 deg position )
GPIO.cleanup()  # Clean up all the ports we've used.

```

4. Motion detection section

```python

import cv2
import time

def detect_motion_sensitivity_adjustable():
    # Initialize the camera
    cap = cv2.VideoCapture(0)

    # Sensitivity settings
    threshold_value = 25  # Adjust this value to change sensitivity, lower is more sensitive
    min_contour_area = 500  # Adjust this value to change sensitivity, lower is more sensitive

    # Initialize frames
    _, frame1 = cap.read()
    time.sleep(1)
    _, frame2 = cap.read()

    while True:
        # Calculate the absolute difference between the current frame and the next frame
        diff = cv2.absdiff(cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY), cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY))
        _, thresh = cv2.threshold(diff, threshold_value, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Check for movement in the contours
        for contour in contours:
            if cv2.contourArea(contour) > min_contour_area:
                print("yes") #<-這邊是先測試鏡頭偵測到移動後是否會列印yes，成功的話再改成上述步驟3的SG90移動指令
                break  # Exit the loop after the first detected motion

        # Wait for 1 second
        time.sleep(1)

        # Read the next frame
        frame1 = frame2
        _, frame2 = cap.read()

        # Press 'q' to exit the loop (if the video window is focused)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the VideoCapture object and close windows
    cap.release()
    cv2.destroyAllWindows()

detect_motion_sensitivity_adjustable()

```

5. Final program
   
移動偵測結合SG90馬達旋轉的程式請參閱 `Main_Project.py`.


## Reference
https://github.com/thomashuang2017/NCU_IOT_project

https://github.com/weberlu88/2019-Fall-MIS-IoT-Project

https://github.com/WenHeChang/IoT

感謝已修過這門課的曾同學指導移動偵測部分

## 可以改進的地方
* 原本預計是想使用人臉偵測來啟動馬達旋轉，但礙於沒太多時間研究與實作這部分，所以改成移動偵測
* 使用人臉偵測後，可以進一步確認是誰來領取零用金並照相存檔(領取記錄)
* 可以串接Line bot作更進一步的運用
