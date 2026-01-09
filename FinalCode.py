import time
import sys
import threading
import cv2
import uuid
import subprocess
import re
import os
import gc
import firebase_admin
from firebase_admin import credentials, firestore, storage
from pymavlink import mavutil
from flask import Flask, Response
from flask_cors import CORS
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor

# --- IMPORT MODULE ---
from USB_humanDetection import USBDetector

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyTHS1'
CRED_PATH = "serviceAccountKey.json"
APP_ID = "default-app-id"
DOC_ID = "live_status_orin"
STORAGE_BUCKET = "nidar-2d21f.firebasestorage.app"

# --- AUTO-KILL ZOMBIE PROCESSES ---
try:
    subprocess.run(["fuser", "-k", "5000/tcp"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
except: pass

detector = None
output_frame = None
video_lock = threading.Lock()
telemetry_data = {"altitude": 0, "speed": 0, "battery": 0, "mode": "INIT", "lat": 0, "lng": 0}
telemetry_lock = threading.Lock()
shared_state = {"stream_url": "WAITING...", "url_found": False}

upload_executor = ThreadPoolExecutor(max_workers=2)
app = Flask(__name__)
CORS(app)

print("\n[STEP 1] Database Init...")
try:
    cred = credentials.Certificate(CRED_PATH)
    firebase_admin.initialize_app(cred, {'storageBucket': STORAGE_BUCKET})
    db = firestore.client()
    bucket = storage.bucket()
    print("✅ Firebase Connected.")
except Exception as e:
    sys.exit(f"Firebase Error: {e}")

def generate_frames():
    global output_frame
    while True:
        with video_lock:
            if output_frame is None:
                time.sleep(0.1)
                continue
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')
        time.sleep(0.04)

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

threading.Thread(
    target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False),
    daemon=True
).start()

def start_cloudflare():
    while not shared_state["url_found"]:
        try:
            process = subprocess.Popen(
                ['cloudflared', 'tunnel', '--url', 'http://localhost:5000', '--no-tls-verify'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            url_pattern = re.compile(r'https://[a-zA-Z0-9-]+\.trycloudflare\.com')
            start = time.time()
            while time.time() - start < 60 and not shared_state["url_found"]:
                line = process.stdout.readline()
                if not line: break
                match = url_pattern.search(line)
                if match:
                    final_url = match.group(0) + "/video_feed"
                    shared_state["stream_url"] = final_url
                    shared_state["url_found"] = True
                    print(f"✅ LINK LIVE: {final_url}")
                    db.collection('artifacts').document(APP_ID)\
                        .collection('public').document('data')\
                        .collection('telemetry').document(DOC_ID)\
                        .update({"stream_url": final_url})
            process.kill()
        except:
            time.sleep(2)

threading.Thread(target=start_cloudflare, daemon=True).start()

print("\n[STEP 3] Hardware Startup...")
TODAY_DOC = f"Day-{datetime.now().strftime('%Y-%m-%d')}"
MISSION_ID = f"Mission-{datetime.now().strftime('%H:%M:%S')}"

# --- FLIGHT CONTROLLER CONNECTION (FROM YOUR WORKING CODE) ---
master = None
def connect_flight_controller():
    # Exactly matching your trial code scan order
    BAUD_RATES = [115200,57600]
    for baud in BAUD_RATES:
        print(f"   -> Trying to connect on {SERIAL_PORT} @ {baud}...")
        try:
            conn = mavutil.mavlink_connection(SERIAL_PORT, baud=baud)
            # Wait 2 seconds for heartbeat
            conn.wait_heartbeat(timeout=2)
            print(f"✅ Flight Controller Connected @ {baud}!")
            return conn
        except:
            pass 
    return None

master = connect_flight_controller()

if master:
    try:
        master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    except: pass
else:
    print("⚠️ Flight Controller Missing or Port Busy (Sim Mode).")

try:
    detector = USBDetector(show_display=False)
    print("✅ System Armed & Ready.")
except Exception as e:
    sys.exit(f"Vision Error: {e}")

def upload_worker(image, lat, lng, alt, is_drop):
    try:
        label = "PAYLOAD DROP" if is_drop else "Human Detected"
        unique_id = uuid.uuid4().hex[:6]
        filename = f"evid_{int(time.time())}_{unique_id}.jpg"
        cv2.imwrite(filename, image)
        
        blob = bucket.blob(f"detections/{filename}")
        blob.upload_from_filename(filename)
        blob.make_public()
        
        hid = f"event-{unique_id}"
        db.collection('History of Mission').document(TODAY_DOC).set({
            MISSION_ID: {
                hid: {
                    "image": blob.public_url, 
                    "location": f"{lat},{lng}", 
                    "lat": float(lat), "lng": float(lng), "altitude": alt, 
                    "label": label, 
                    "timestamp": datetime.now().strftime("%H:%M:%S")
                }
            }
        }, merge=True)
        print(f"✅ Uploaded: {label}")
        if os.path.exists(filename): os.remove(filename)
    except Exception as e: 
        print(f"Upload Fail: {e}")

def telemetry_loop():
    while True:
        if master:
            try:
                msg = master.recv_match(blocking=False)
                if msg:
                    if msg.get_type() == 'VFR_HUD': 
                        with telemetry_lock: telemetry_data['altitude'] = round(msg.alt, 2)
                    if msg.get_type() == 'GPS_RAW_INT': 
                        with telemetry_lock: 
                            telemetry_data['lat'] = msg.lat/1e7
                            telemetry_data['lng'] = msg.lon/1e7
            except: pass
        else:
            with telemetry_lock: telemetry_data['altitude'] = 15.0
            
        try:
            with telemetry_lock: payload = telemetry_data.copy()
            if shared_state["url_found"]: payload["stream_url"] = shared_state["stream_url"]
            db.collection('artifacts').document(APP_ID).collection('public').document('data').collection('telemetry').document(DOC_ID).set(payload)
        except: pass
        time.sleep(0.5)

threading.Thread(target=telemetry_loop, daemon=True).start()

last_upload_time = 0

try:
    while True:
        found, count, frame, drop_triggered = detector.detect()
        
        if frame is None:
            time.sleep(0.01)
            continue

        should_upload = False
        if drop_triggered:
            print("🚀 DROP TRIGGERED UPLOAD")
            should_upload = True
            gc.collect() 
        elif found and (time.time() - last_upload_time > 10.0):
            should_upload = True
            last_upload_time = time.time()

        if should_upload:
            cv2.putText(frame, "UPLOADING...", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            with telemetry_lock: 
                u_lat, u_lng, u_alt = telemetry_data['lat'], telemetry_data['lng'], telemetry_data['altitude']
            upload_executor.submit(upload_worker, frame.copy(), u_lat, u_lng, u_alt, drop_triggered)

        with video_lock: 
            output_frame = frame.copy()

except KeyboardInterrupt:
    detector.release()
    sys.exit(0)
