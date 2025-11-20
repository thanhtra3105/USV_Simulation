from flask import Flask, request, jsonify, render_template, Response
from pymavlink import mavutil
import os, time
from flask_cors import CORS
from threading import Lock
import csv
from datetime import datetime

import statistics

mavlink_lock = Lock()
polling_enabled = True

app = Flask(__name__)
CORS(app)

# =============================
# Config
# =============================
VEHICLE_PORT = '/dev/ttyAMA0'
VEHICLE_BAUD = 57600
ACK_TIMEOUT = 10

master = None

# =========================================
# Logging Functions (ghi log CSV phân tích)
# =========================================

import os
import csv
from datetime import datetime
import threading
import time

MAX_LOG_LINES = 100
log_buffer = {
    "vehicle-position": [],
    "vehicle-info": [],
    "mission-progress": []
}

HEADERS = {
    "vehicle-position": ["timestamp", "endpoint", "latency_ms", "status", "loss_ratio"],
    "vehicle-info": ["timestamp", "endpoint", "latency_ms", "status", "loss_ratio"],
    "mission-progress": ["timestamp", "endpoint", "latency_ms", "status", "loss_ratio"],
}


packet_stats = {
    "vehicle-position": {"sent": 0, "received": 0},
    "vehicle-info": {"sent": 0, "received": 0},
    "mission-progress": {"sent": 0, "received": 0}
}


def add_latency_sample(endpoint, latency, status="success"):
    global log_buffer, packet_stats
    packet_stats[endpoint]["sent"] += 1
    if status == "success":
        packet_stats[endpoint]["received"] += 1

    loss_ratio = 0 if packet_stats[endpoint]["sent"] == 0 else (1 - packet_stats[endpoint]["received"]/packet_stats[endpoint]["sent"])
    log_buffer[endpoint].append([datetime.now(), endpoint, latency, status, loss_ratio])

    if len(log_buffer[endpoint]) > MAX_LOG_LINES:
        log_buffer[endpoint] = log_buffer[endpoint][-MAX_LOG_LINES:]


# def flush_log_buffer():
#     global log_buffer
#     while True:
#         time.sleep(2)

#         # ----- VEHICLE POSITION -----
#         if log_buffer["vehicle-position"]:
#             filename = "logs/api_latency/position_latency.csv"
#             write_with_header(filename, HEADERS["vehicle-position"], log_buffer["vehicle-position"])

#         # ----- VEHICLE INFO -----
#         if log_buffer["vehicle-info"]:
#             filename = "logs/api_latency/vehicle_info_latency.csv"
#             write_with_header(filename, HEADERS["vehicle-info"], log_buffer["vehicle-info"])

#         # ----- MISSION PROGRESS -----
#         if log_buffer["mission-progress"]:
#             filename = "logs/api_latency/mission_progress_latency.csv"
#             write_with_header(filename, HEADERS["mission-progress"], log_buffer["mission-progress"])

def flush_log_buffer():
    global log_buffer
    while True:
        time.sleep(2)
        for endpoint in log_buffer:
            if log_buffer[endpoint]:
                filename = f"logs/latency_and_loss/{endpoint}_latency.csv"
                os.makedirs(os.path.dirname(filename), exist_ok=True)
                write_with_header(filename, HEADERS[endpoint], log_buffer[endpoint])


def write_with_header(filename, header, rows):
    file_exists = os.path.exists(filename)
    file_empty = (not file_exists) or os.path.getsize(filename) == 0

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def log_rtt(command, rtt, success):
    with open("logs/command_rtt/command_rtt.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([datetime.now(), command, rtt, success])

def log_packet_loss(endpoint):
    stats = packet_stats[endpoint]
    received = stats['received']
    total = stats['sent']
    loss_ratio = 0 if total == 0 else (1 - received / total)

    filename = f"logs/packet_loss/{endpoint}_loss.csv"
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    file_exists = os.path.exists(filename)
    file_empty = (not file_exists) or os.path.getsize(filename) == 0

    with open(filename, "a", newline="") as f:
        writer = csv.writer(f)
        if file_empty:
            writer.writerow(["timestamp", "received", "total", "loss_ratio"])
        writer.writerow([datetime.now(), received, total, loss_ratio])


# =============================
# Hàm kết nối MAVLink (giữ 1 kết nối global)
# =============================
def get_master(timeout=10):
    global master
    if master is None or not master.port:  # nếu chưa có hoặc mất kết nối
        # print(f"Connecting to serial: {VEHICLE_PORT} @ {VEHICLE_BAUD}")
        master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
        try:
            hb = master.wait_heartbeat(timeout=timeout)
            if not hb:
                raise Exception(" Không nhận được HEARTBEAT")
            print(f"Heartbeat from system: {master.target_system}, component: {master.target_component}")
        except Exception as e:
            raise Exception(f"Kết nối thất bại: {e}")
    return master

# =============================
# Upload mission
# =============================
def send_mission_via_mavlink(master, mission):
    wp_count = len(mission)
    if wp_count == 0:
        return {"success": False, "message": "Empty mission"}
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    # time.sleep(1)
    master.mav.mission_count_send(master.target_system, master.target_component, wp_count)
    print(f"Sending MISSION_COUNT={wp_count}")
    start = time.perf_counter() # do tu khi send waypoint cho den khi nhan ack
    for seq, wp in enumerate(mission):
        lat = int(wp["lat"] * 1e7)
        lon = int(wp["lng"] * 1e7)
        alt = 0

        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,
            5, 0, 0, 0,
            lat, lon, alt
        )
        print(f"Sent WP {seq+1}: lat={wp['lat']}, lon={wp['lng']}")

    # ======================
    # MEASURE RTT: SEND → ACK
    # ======================
    
    msg = master.recv_match(type="MISSION_ACK", blocking=True)
    rtt = (time.perf_counter() - start) * 1000

    if msg:
        log_rtt("MISSION_ACK", rtt, True)
        return {"success": True, "message": "Mission upload complete", "ack": msg.to_dict()}
    else:
        log_rtt("MISSION_ACK", rtt, False)
        return {"success": False, "message": "No MISSION_ACK received"}

def get_mission(master):
    master.mav.mission_request_list_send(master.target_system, master.target_component)
    mission = []
    count = None

    while True:
        msg = master.recv_match(blocking=True)
        t = msg.get_type()

        if t == "MISSION_COUNT":
            count = msg.count
            master.mav.mission_request_int_send(master.target_system, master.target_component, 0)

        elif t == "MISSION_ITEM_INT":
            mission.append(msg)
            if msg.seq + 1 < count:
                master.mav.mission_request_int_send(master.target_system, master.target_component, msg.seq + 1)
            else:
                break

    return mission
# =============================
# API endpoints
# =============================
@app.route("/", methods=["GET"])
def home():
    return render_template("index.html")

@app.route("/telemetry", methods=["GET"])
def telemetry():
    return render_template("dashboard.html")


sensor_data = {
    "ph" : None,
    "do" : None,
    "cod" : None,
    "tss" : None
}

@app.route("/api/telemetry", methods=["GET"])
def api_telemetry():
    global sensor_data, polling_enabled
    try:
        master = get_master()
        msg = master.recv_match(type="NAMED_VALUE_FLOAT", blocking=True, timeout=2)
        with mavlink_lock:
            if not polling_enabled:
                return jsonify({"success": False, "message": "Busy uploading mission"}), 503
        
        if not msg:
            return jsonify({"success": False, "message": "No Sensor Data"}), 500

        if msg.name == "pH":
            sensor_data["ph"] = msg.value
        elif msg.name == "do":
            sensor_data["do"] = msg.value
        elif msg.name == "cod":
            sensor_data["cod"] = msg.value
        elif msg.name == "tss":
            sensor_data["tss"] = msg.value

        telemetry = {
        "battery": 75,
        "speed": 2.5,
        "heading": 90,
        "ph": sensor_data["ph"],
        "do": sensor_data["do"],
        "cod": sensor_data["cod"],
        "tss": sensor_data["tss"]
        }
        

        return jsonify({"success": True, "telemetry": telemetry})
    except Exception as e:
        return jsonify({"success": False, "message": f"Position failed: {e}"}), 500   

    

@app.route("/upload-mission", methods=["POST"])
def upload_mission():
    global polling_enabled

    data = request.get_json()
    if not data or "mission" not in data:
        return jsonify({"success": False, "message": "Missing mission in payload"}), 400

    mission = data["mission"]
    try:
        master = get_master()
        # ===== STOP POLLING =====
        polling_enabled = False
        time.sleep(0.2)    # đợi các API đang chạy dừng hẳn

        # ===== UPLOAD MISSION =====
        with mavlink_lock:    
            res = send_mission_via_mavlink(master, mission)
            polling_enabled = True   
            return jsonify(res), (200 if res["success"] else 500)
    except Exception as e:
        return jsonify({"success": False, "message": f"Exception: {e}"}), 503
    

@app.route("/start-mission", methods=["POST"])
def start_mission():
    try:
        master = get_master()

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("Mission started")
        time.sleep(1)
        return jsonify({"success": True, "message": "Mission started"})
    except Exception as e:
        return jsonify({"success": False, "message": f"Start mission failed: {e}"}), 500

@app.route("/vehicle-position", methods=["GET"])
def vehicle_position():
    try:
        master = get_master()
        with mavlink_lock:
            if not polling_enabled:
                return jsonify({"success": False, "message": "Busy uploading mission"}), 503

            # ===== Xóa message GLOBAL_POSITION_INT cũ trong buffer =====
            while master.recv_match(type="GLOBAL_POSITION_INT", blocking=False):
                pass

            # ===== Bắt đầu đo từ khi gửi request tới PX4 =====
            start = time.perf_counter()
            msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
            latency_ms = (time.perf_counter() - start) * 1000  # tính latency

            if not msg:
                add_latency_sample("vehicle-position", latency_ms, "error")
                return jsonify({"success": False, "message": "No GPS data"}), 500

            # ===== Parse dữ liệu =====
            pos = msg.to_dict()
            lat = pos["lat"] / 1e7
            lon = pos["lon"] / 1e7
            alt = pos["alt"] / 1000.0
            
            add_latency_sample("vehicle-position", latency_ms)
            return jsonify({
                "success": True,
                "lat": lat,
                "lon": lon,
                "alt": alt
            })
    except Exception as e:
        add_latency_sample("vehicle-position", 0, "error")
        return jsonify({"success": False, "message": f"Position failed: {e}"}), 500

@app.route("/vehicle-info", methods=["GET"])
def vehicle_info():
    try:
        master = get_master()
        with mavlink_lock:
            if not polling_enabled:
                return jsonify({"success": False, "message": "Busy uploading mission"}), 503
            
            # ===== Xóa message BATTERY and VFR_HUD cũ trong buffer =====
            while master.recv_match(type="BATTERY_STATUS", blocking=False):
                pass
            while master.recv_match(type="VFR_HUD", blocking=False):
                pass

            start = time.perf_counter()
            msg_bat = master.recv_match(type="BATTERY_STATUS", blocking=True)
            msg_speed_heading = master.recv_match(type="VFR_HUD", blocking=True)
            latency_ms = (time.perf_counter() - start) * 1000  # tính latency

            if not msg_bat or not msg_speed_heading:
                add_latency_sample("vehicle-info", latency_ms, "error")
                return jsonify({"success": False, "message": "No vehicle data"}), 500

            battery_remaining = msg_bat.battery_remaining
            speed = msg_speed_heading.groundspeed
            heading = msg_speed_heading.heading

            add_latency_sample("vehicle-info", latency_ms)

            return jsonify({
                "success": True,
                "battery": battery_remaining,
                "speed": speed,
                "heading": heading,
            })
    except Exception as e:
        add_latency_sample("vehicle-info", 0, "error")
        return jsonify({"success": False, "message": f"Battery failed: {e}"}), 500

mission_current = None
mission_total = None
mission_state = None

@app.route("/mission-progress", methods=["GET"])
def misson_progress():
    global mission_current, mission_state, mission_total
    try:
        master = get_master()
        with mavlink_lock:
            if not polling_enabled:
                return jsonify({"success": False, "message": "Busy uploading mission"}), 503
            
            # ===== Xóa message MISSION_CURRENT cũ trong buffer =====
            while master.recv_match(type="MISSION_CURRENT", blocking=False):
                pass

            start = time.perf_counter()
            msg = master.recv_match(type="MISSION_CURRENT", blocking=True)
            latency_ms = (time.perf_counter() - start) * 1000

            if msg:
                add_latency_sample("mission-progress", latency_ms)
                mission_current = msg.seq + 1
                mission_total = msg.total
                mission_state = msg.mission_state
                return jsonify({
                    "success": True,
                    "mission_current": mission_current,
                    "mission_total": mission_total,
                    "mission_state": mission_state,
                })
            else:
                add_latency_sample("mission-progress", 0, 'error')
                return jsonify({
                    "success": False,
                    "mission_current": mission_current,
                    "mission_total": mission_total,
                    "mission_state": mission_state,
                })
    except Exception as e:
        add_latency_sample("mission-progress", latency_ms, 'error')
        return jsonify({"success": False, "message": f"Get mission failed: {e}"}), 500

@app.route("/get-mission", methods=["GET"])
def api_get_mission():
    try:
        master = get_master()
        with mavlink_lock:
            mission_items = get_mission(master)

        mission_list = []
        for wp in mission_items:
            mission_list.append({
                "seq": wp.seq,
                "lat": wp.x / 1e7,
                "lng": wp.y / 1e7,
                "alt": wp.z
            })

        return jsonify({"success": True, "mission": mission_list})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})

# =============================
# Main
# =============================
if __name__ == "__main__":
    threading.Thread(target=flush_log_buffer, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, debug=False)

