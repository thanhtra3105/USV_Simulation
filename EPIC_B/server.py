from flask import Flask, request, jsonify, render_template, Response
from pymavlink import mavutil
import os, time
from flask_cors import CORS
from threading import Lock
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
    time.sleep(1)

    master.mav.mission_count_send(master.target_system, master.target_component, wp_count)
    print(f"Sending MISSION_COUNT={wp_count}")
    for seq, wp in enumerate(mission):
        lat = int(wp["lat"] * 1e7)
        lon = int(wp["lng"] * 1e7)
        alt = 0
        # hold_time = wp.get("hold_time", 0)

        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,  # current, autocontinue
            5, 0, 0, 0,
            lat, lon, alt
        )
        print(f"Sent WP {seq+1}: lat={wp['lat']}, lon={wp['lng']}")
    
    msg = master.recv_match(type="MISSION_ACK", blocking=True, timeout=1)
    time.sleep(1)
    if msg:
        print(f"Got final MISSION_ACK: {msg}")
        return {"success": True, "message": "Mission upload complete", "ack": msg.to_dict()}
    else:
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
            msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
            if not msg:
                return jsonify({"success": False, "message": "No GPS data"}), 500

            pos = msg.to_dict()
            lat = pos["lat"] / 1e7
            lon = pos["lon"] / 1e7
            alt = pos["alt"] / 1000.0
            # print(f"lat: {lat}, lon:{lon}");
            return jsonify({
                "success": True,
                "lat": lat,
                "lon": lon,
                "alt": alt
            })
    except Exception as e:
        return jsonify({"success": False, "message": f"Position failed: {e}"}), 500


@app.route("/vehicle-info", methods=["GET"])
def vehicle_info():
    try:
        master = get_master()
        with mavlink_lock:
            if not polling_enabled:
                return jsonify({"success": False, "message": "Busy uploading mission"}), 503
        
            msg_bat = master.recv_match(type="BATTERY_STATUS", blocking=True)
            msg_speed_heading = master.recv_match(type="VFR_HUD", blocking= True)
            if not msg_bat or not msg_speed_heading:
                return jsonify({"success": False, "message": "No vehicel data"}), 500
            battery_remaining  = msg_bat.battery_remaining
            speed = msg_speed_heading.groundspeed
            heading = msg_speed_heading.heading

            print(battery_remaining)
            return jsonify({
                "success": True,
                "battery": battery_remaining,
                "speed": speed,
                "heading": heading,
            })
    except Exception as e:
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
            msg = master.recv_match(type="MISSION_CURRENT", blocking=True, timeout=2)
        
            if msg:
                print(msg)
                mission_current = msg.seq+1
                mission_total = msg.total
                mission_state = msg.mission_state
                return jsonify({
                "success": True,
                "mission_current": mission_current,
                "mission_total": mission_total,
                "mission_state": mission_state,
                })
            else:
                print("Lỗi: Không nhận được MISSION_CURRENT")
                return jsonify({
                "success": False,
                "mission_current": mission_current,
                "mission_total": mission_total,
                "mission_state": mission_state,
                })
    except Exception as e:
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

@app.route("/set-speed", methods=["POST"])
def set_speed():
    try:
        data = request.get_json()
        if not data:
            return jsonify({"success": False, "message": "Missing payload"}), 400

        vx = data.get("vx", 0.0)
        vy = data.get("vy", 0.0)
        vz = data.get("vz", 0.0)
        yaw_rate = data.get("yaw_rate", 0.0)

        master = get_master()

        # ===== SEND VELOCITY SETPOINT =====
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000),         # time_boot_ms
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000110111000111,             # type_mask
            0, 0, 0,                         # position ignored
            vx, vy, vz,                      # velocity in m/s
            0, 0, 0,                         # acceleration ignored
            0, yaw_rate                      # yaw, yaw_rate
        )

        return jsonify({
            "success": True,
            "message": "Velocity command sent",
            "vx": vx, "vy": vy, "vz": vz, "yaw_rate": yaw_rate
        })

    except Exception as e:
        return jsonify({"success": False, "message": f"Send speed failed: {e}"}), 500

@app.route("/set-offboard", methods=["POST"])
def set_offboard():
    try:
        master = get_master()

        # ===============================
        # 1) Gửi 1 velocity setpoint dummy
        #    PX4 yêu cầu phải gửi BEFORE vào offboard
        # ===============================

        time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF
        master.mav.set_position_target_local_ned_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000110111000111,   # velocity only
            0, 0, 0,
            0.0, 0.0, 0.0,         # dummy vx, vy, vz
            0, 0, 0,
            0, 0
        )

        # ===============================
        # 2) Chuyển mode sang OFFBOARD
        # ===============================
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1
            6, 0, 0, 0, 0, 0     # param2 = PX4 offboard mode = 6
        )

        # ===============================
        # 3) Đợi PX4 confirm
        # ===============================
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if not ack:
            return jsonify({"success": False, "message": "No COMMAND_ACK received"})

        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return jsonify({
                "success": False,
                "message": f"OFFBOARD rejected: {ack.result}"
            })

        print(">>> PX4 switched to OFFBOARD")

        return jsonify({"success": True, "message": "Switched to OFFBOARD mode"})

    except Exception as e:
        return jsonify({"success": False, "message": f"Failed: {e}"}), 500

# =============================
# Main
# =============================
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
