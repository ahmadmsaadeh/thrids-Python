import RPi.GPIO as GPIO
import time
import serial
import threading
import depthai as dai
import numpy as np
import cv2

arduino_port = '/dev/ttyACM0'
baud_rate = 9600
arduino = serial.Serial(arduino_port, baud_rate, timeout=1)

SENSOR_1_PIN = 17
SENSOR_2_PIN = 27
SENSOR_3_PIN = 22

data_queue = []
queue_lock = threading.Lock()

pipeline = dai.Pipeline()
colorCam = pipeline.createColorCamera()
colorCam.setBoardSocket(dai.CameraBoardSocket.RGB)
colorCam.setPreviewSize(640, 480)
xout = pipeline.createXLinkOut()
xout.setStreamName("video")
colorCam.preview.link(xout.input)


def get_rgb_color_name(b, g, r, threshold=40):
    if r > g + threshold and r > b + threshold:
        return "R"
    elif g > r + threshold and g > b + threshold:
        return "M"
    elif b > r + threshold and b > g + threshold:
        return "L"
    return "Unknown"


def send_to_arduino(data):
    if arduino.is_open:
        arduino.write(f"{data}\n".encode())


def sensor_interrupt(sensor_id):
    global data_queue
    with queue_lock:
        for item in data_queue:
            if not item["Flag"] and item["Value"] == sensor_id:
                time.sleep(3)
                send_to_arduino(sensor_id)
                item["Flag"] = True
                break


def sensor_1_callback(channel):
    sensor_interrupt("R")


def sensor_2_callback(channel):
    sensor_interrupt("M")


def sensor_3_callback(channel):
    sensor_interrupt("L")


def process_camera_feed():
    global data_queue
    with dai.Device(pipeline) as device:
        videoQueue = device.getOutputQueue(name="video", maxSize=8, blocking=False)
        item_id = 1

        while True:
            inFrame = videoQueue.get()
            frame = inFrame.getCvFrame()
            height, width, _ = frame.shape
            cx, cy = width // 2, height // 2
            roi_size = 75
            roi = frame[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]
            avg_color_per_row = np.average(roi, axis=0)
            avg_color = np.average(avg_color_per_row, axis=0)
            b, g, r = int(avg_color[0]), int(avg_color[1]), int(avg_color[2])
            color_name = get_rgb_color_name(b, g, r)

            if color_name in ["R", "M", "L"]:
                with queue_lock:
                    data_queue.append({"Value": color_name, "ID": item_id, "Flag": False})
                    item_id += 1

            cv2.rectangle(frame, (cx - roi_size, cy - roi_size), (cx + roi_size, cy + roi_size), (0, 255, 0), 2)
            cv2.putText(frame, color_name, (cx - 100, cy - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imshow("Color Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()


GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SENSOR_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SENSOR_3_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.add_event_detect(SENSOR_1_PIN, GPIO.RISING, callback=sensor_1_callback, bouncetime=200)
GPIO.add_event_detect(SENSOR_2_PIN, GPIO.RISING, callback=sensor_2_callback, bouncetime=200)
GPIO.add_event_detect(SENSOR_3_PIN, GPIO.RISING, callback=sensor_3_callback, bouncetime=200)

camera_thread = threading.Thread(target=process_camera_feed)
camera_thread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
    arduino.close()
