import cv2
import numpy as np
import time
import serial


def initialize_serial(port, baudrate):
    ser = serial.Serial(port, baudrate)
    return ser


def calculate_speed(contour, roi_width):
    (x, y, w, h) = cv2.boundingRect(contour)
    speed = (x + w/2) / roi_width * 100
    return speed


def detect_fish(frame, roi, previous_frame):
    speeds = []
    contour_list = []
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    previous_frame_resized = cv2.resize(previous_frame, (gray.shape[1], gray.shape[0]))
    thresh = cv2.threshold(cv2.absdiff(previous_frame_resized, gray), 50, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) < 500:
            continue
        speed = calculate_speed(contour, roi[2])
        speeds.append(speed)
        # Reduce the size of the contour
        epsilon = 0.1 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        # Add the approximated contour to the list of contours
        contour_list.append(approx)
        # Draw a rectangle around the contour
        (x, y, w, h) = cv2.boundingRect(contour)
        cv2.rectangle(frame, (roi[0]+x, roi[1]+y), (roi[0]+x+w, roi[1]+y+h), (0, 255, 0), 2)
    return speeds, contour_list


def move_servo(ser):
    ser.write(b'o')
    #time.sleep(2)
    #ser.write(b'0')


def display_frame(frame, average_speed, font):
    cv2.putText(frame, f"Speed: {average_speed:.2f}", (10, 50), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
    cv2.imshow("Fish Tank", frame)
    if cv2.waitKey(1) & 0xFF == ord('f'):
        return True


def display_video(frame, speeds, roi, average_speed, font):
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Threshold the image
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # Find contours in the image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw ROI rectangle
    cv2.rectangle(frame, (roi[0], roi[1]), (roi[0] + roi[2], roi[1] + roi[3]), (0, 255, 0), 2)

    # Draw contours and speeds for detected fish
    X=len(contours)
    for i, speed in enumerate(speeds):
            if X <= 0:
                print("No contours detected")
            else:
                try:
                    cv2.drawContours(frame, [contours[i]], 0, (0, 0, 255), 2)
                    (x, y, w, h) = cv2.boundingRect(contours[i])
                    cv2.putText(frame, f"{speed:.2f}", (x + w + 10, y + h//2), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                except:
                    pass
    # Display average speed
    cv2.putText(frame, f"Average speed: {average_speed:.2f}", (10, 50), font, 1, (255, 0, 0), 2, cv2.LINE_AA)

    # Display video
    cv2.imshow("Fish Tank", frame)
    if cv2.waitKey(1) & 0xFF == ord('f'):
        return True


def main():
    print("Press 'F' to pay respect...\n\n")
    ser = initialize_serial('COM10', 9600)
    threshold_speed = 79
    font = cv2.FONT_HERSHEY_SIMPLEX
    camera_index = 1
    cap = cv2.VideoCapture(camera_index)
    previous_frame = None
    roi = (0, 0, 500, 500)
    
    while True:
        ret, frame = cap.read()
        #frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        if not ret:
            break
        if previous_frame is None:
            previous_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            continue
        speeds, contours = detect_fish(frame[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]], roi, previous_frame)

        if len(speeds) > 0:
            average_speed = sum(speeds) / len(speeds)
            print(f"Average speed: {average_speed:.2f}")
        else:
            average_speed = 0
        if average_speed > threshold_speed:
            move_servo(ser)
            print("Servo triggered: Fish is HUNGRY!!!!")
            # Print the average speed to Arduino
            serial_output = str(average_speed).encode()
            ser.write(serial_output)
            print(f"Serial output sent to Arduino: {serial_output}")
        quit_program = display_video(frame, speeds, roi, average_speed, font)
        if quit_program:
            break
        previous_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cap.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
