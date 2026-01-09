import cv2
import time
import math
import threading
import board
import busio
import Jetson.GPIO as GPIO
from adafruit_servokit import ServoKit
from ultralytics import YOLO

# --- CONFIGURATION (User Provided) ---
CAMERA_INDEX = 0
CONF_THRESH = 0.5
PERSON_CLASS = 0
PIXEL_TOLERANCE = 150
SERVO_CHANNELS = [0, 1, 2, 3, 4]

# Hardware Constants
BUZZER_PIN = 32
DROP_ANGLE = 180
RESET_SERVO_AFTER_DROP = False   # optional reset after drop

class HardwareManager:
    def __init__(self):
        self.kit = None
        self.active = True

        try:
            GPIO.cleanup()
        except:
            pass

        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(BUZZER_PIN, GPIO.OUT)

            GPIO.output(BUZZER_PIN, GPIO.LOW)
            print("[HW] Buzzer ready (NC relay) on Pin 15.")
        except Exception as e:
            print(f"[HW] GPIO ERROR: {e}")

        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.kit = ServoKit(channels=16, i2c=self.i2c)
            self.kit.frequency = 50
        except Exception as e:
            print(f"[HW] SERVO ERROR: {e}")

    def buzzer_on(self):
        GPIO.output(BUZZER_PIN, GPIO.HIGH)

    def buzzer_off(self):
        GPIO.output(BUZZER_PIN, GPIO.LOW)

    def _drop_sequence(self, idx):
        try:
            if not self.active:
                return

            print(f"[HW] Slow DROP on servo {idx}")

            if self.kit:
                start = 0
                end = DROP_ANGLE
                steps = 60
                delay = 0.5

                for s in range(steps + 1):
                    if not self.active:
                        break
                    angle = start + (end - start) * (s / steps)
                    self.kit.servo[idx].angle = angle
                    time.sleep(delay)

                self.kit.servo[idx].angle = None

                if RESET_SERVO_AFTER_DROP:
                    time.sleep(1)
                    self.kit.servo[idx].angle = 0

            print(f"--- DROP {idx} FINISHED (30s move) ---")
        except Exception as e:
            print(f"[HW] Thread Error: {e}")

    def trigger_drop(self, idx):
        threading.Thread(target=self._drop_sequence, args=(idx,), daemon=True).start()

    def cleanup(self):
        self.active = False
        try:
            self.buzzer_off()
            GPIO.cleanup()
        except:
            pass


class MissionControl:
    def __init__(self, hw):
        self.hw = hw
        self.trigger_count = 0
        self.processed_locs = []

        self.confirm_buffer = {}
        self.FRAME_CONFIRM = 4

    def is_ghost(self, x, y):
        for (ox, oy) in self.processed_locs:
            if math.dist((x, y), (ox, oy)) < PIXEL_TOLERANCE:
                return True
        return False

    def evaluate(self, x, y):
        if self.trigger_count >= len(SERVO_CHANNELS):
            return False
        if self.is_ghost(x, y):
            return False

        key = f"{x//20}_{y//20}"
        self.confirm_buffer[key] = self.confirm_buffer.get(key, 0) + 1

        if self.confirm_buffer[key] < self.FRAME_CONFIRM:
            return False

        print(f"[LOGIC] DROP CONFIRMED at ({x},{y})")
        self.processed_locs.append((x, y))

        if len(self.processed_locs) > 50:
            self.processed_locs.pop(0)

        self.hw.trigger_drop(SERVO_CHANNELS[self.trigger_count])
        self.trigger_count += 1
        return True


class CameraThread:
    def __init__(self, index):
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            raise RuntimeError("Camera Error")

        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()

            if not ret:
                self.cap.release()
                time.sleep(0.5)
                self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
                continue

            with self.lock:
                self.frame = frame

    def read(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.cap.release()


class USBDetector:
    def __init__(self, show_display=False):
        self.show_display = show_display
        self.hw = HardwareManager()
        self.mission = MissionControl(self.hw)

        print("[VISION] Loading Model...")

        try:
            self.model = YOLO("yolov8n.engine", task="detect")
            print("[VISION] TensorRT engine loaded.")
        except Exception as e:
            print(f"[VISION] Engine failed -> Falling back to PT. Reason: {e}")
            self.model = YOLO("yolov8n.pt")

        self.cam = CameraThread(CAMERA_INDEX)
        time.sleep(2)
        print("[VISION] System Armed.")

    def detect(self):
        frame = self.cam.read()
        if frame is None:
            return False, 0, None, False

        results = self.model.predict(
            frame,
            imgsz=(480, 640),
            conf=CONF_THRESH,
            classes=[PERSON_CLASS],
            verbose=False
        )[0]

        human_detected = False
        human_count = 0
        drop_triggered = False

        if results.boxes is not None:
            boxes = results.boxes.xyxy.cpu().numpy().astype(int)
            human_count = len(boxes)

            if human_count > 0:
                human_detected = True

            for (x1, y1, x2, y2) in boxes:
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                if self.mission.evaluate(cx, cy):
                    drop_triggered = True

        if human_detected:
            self.hw.buzzer_on()
        else:
            self.hw.buzzer_off()

        cv2.putText(frame, f"Drops: {self.mission.trigger_count}/5", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if self.show_display:
            cv2.imshow("Drone", frame)
            cv2.waitKey(1)

        return human_detected, human_count, frame, drop_triggered

    def release(self):
        self.cam.stop()
        self.hw.cleanup()
        if self.show_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    det = USBDetector(show_display=True)
    try:
        while True:
            det.detect()
    except KeyboardInterrupt:
        det.release()
