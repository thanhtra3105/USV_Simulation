import time
import csv
import os
from pymavlink import mavutil

def start_mavlink_logger(master=None, output_file="mavlog.csv", log_rate=0.8):
    """
    Log GPS, speed, heading, battery, mission từ MAVLink ra file CSV.
    Có thể dùng chung kết nối MAVLink với server.

    master: mavutil.mavlink_connection hoặc None (tự tạo connection)
    output_file: tên file CSV
    log_rate: thời gian giữa 2 lần ghi (seconds)
    """

    # -------------------------------------------------------
    # Tự tạo kết nối nếu không truyền từ server
    # -------------------------------------------------------
    created_master = False
    if master is None:
        master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
        master.wait_heartbeat()
        created_master = True
        print("[Logger] Connected to MAVLink")

    # -------------------------------------------------------
    # Chuẩn bị folder + file CSV
    # -------------------------------------------------------
    os.makedirs(os.path.dirname(output_file) or ".", exist_ok=True)

    csv_file = open(output_file, "w", newline="")
    writer = csv.writer(csv_file)

    writer.writerow([
        "timestamp",
        "lat", "lon", "alt",
        "speed_ground",
        "heading",
        "battery_remaining",
        "mission_current", "mission_total"
    ])

    # -------------------------------------------------------
    # Biến lưu dữ liệu mới nhất
    # -------------------------------------------------------
    latest = {
        "lat": None,
        "lon": None,
        "alt": None,
        "speed": None,
        "heading": None,
        "volt": None,
        "curr": None,
        "remain": None,
        "mission_curr": None,
        "mission_total": None
    }

    print("[Logger] MAVLink logging started ...")

    # -------------------------------------------------------
    # Vòng lặp chính
    # -------------------------------------------------------
    try:
        while True:
            msg = master.recv_match(blocking=False)

            if msg:
                msg_type = msg.get_type()

                # ---------------- GPS ----------------
                if msg_type == "GPS_RAW_INT":
                    latest["lat"] = msg.lat / 1e7
                    latest["lon"] = msg.lon / 1e7
                    latest["alt"] = msg.alt / 1000.0

                # ------------ SPEED + HEADING ------------
                elif msg_type == "VFR_HUD":
                    latest["speed"] = msg.groundspeed
                    latest["heading"] = msg.heading

                # ---------------- BATTERY ----------------
                elif msg_type == "BATTERY_STATUS":
                    latest["remain"] = msg.battery_remaining

                # ---------------- MISSION ----------------
                elif msg_type == "MISSION_CURRENT":
                    latest["mission_curr"] = msg.seq

                elif msg_type == "MISSION_COUNT":
                    latest["mission_total"] = msg.count

            # ---------------- WRITE CSV ----------------
            writer.writerow([
                time.time(),
                latest["lat"], latest["lon"], latest["alt"],
                latest["speed"],
                latest["heading"],
                latest["remain"],
                latest["mission_curr"], latest["mission_total"]
            ])
            csv_file.flush()

            time.sleep(log_rate)

    except KeyboardInterrupt:
        print("[Logger] Stopped by user.")
    except Exception as e:
        print("[Logger] ERROR:", e)
    finally:
        csv_file.close()
        if created_master:
            master.close()
        print("[Logger] Logging ended.")
