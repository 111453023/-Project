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