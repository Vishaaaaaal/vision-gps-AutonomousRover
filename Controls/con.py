#!/usr/bin/env python3
import serial
import pynmea2
import requests
import polyline
import numpy as np
import matplotlib.pyplot as plt
from geopy.distance import geodesic
import time
import rospy
import threading
import socket
from std_msgs.msg import String

# ==== CONFIG ====
API_KEY = "AIzaSyDN6Txao6CqdD-txYakU8Q6YS4iqzuSO08"
destination = {"latitude": 12.820753535208507, "longitude": 80.03986456084907}
GPS_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
DISTANCE_THRESHOLD = 5  # meters to stop

# ==== STATE ====
obstacle_detected = False  # updated by socket listener

# ==== ROS SETUP ====
rospy.init_node("gps_throttle_driver_live", anonymous=True)
cmd_pub = rospy.Publisher("nano_control", String, queue_size=10)
rospy.loginfo("🧭 Live GPS + Throttle Control Initialized")

# ==== SOCKET LISTENER THREAD ====
def socket_listener():
    global obstacle_detected
    HOST = '0.0.0.0'
    PORT = 5050
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        rospy.loginfo("📡 Listening on port 5050 for obstacle signal...")
        conn, addr = s.accept()
        rospy.loginfo(f"✅ Connected to: {addr}")
        with conn:
            while not rospy.is_shutdown():
                data = conn.recv(1024)
                if not data:
                    continue
                msg = data.decode().strip()
                if msg == "stop":
                    obstacle_detected = True
                    rospy.logwarn("🛑 Obstacle detected! Throttle paused.")
                elif msg == "start":
                    obstacle_detected = False
                    rospy.loginfo("✅ Clear path. Throttle enabled.")

# ==== HELPERS ====
def get_gps_data():
    with serial.Serial(GPS_PORT, BAUD_RATE, timeout=1) as ser:
        while True:
            line = ser.readline().decode('ascii', errors='replace')
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                try:
                    msg = pynmea2.parse(line)
                    return (msg.latitude, msg.longitude)
                except pynmea2.ParseError:
                    continue

def fetch_route_from_google(start_lat, start_lon):
    url = "https://routes.googleapis.com/directions/v2:computeRoutes"
    headers = {
        "Content-Type": "application/json",
        "X-Goog-Api-Key": API_KEY,
        "X-Goog-FieldMask": "routes.polyline"
    }
    data = {
        "origin": {"location": {"latLng": {"latitude": start_lat, "longitude": start_lon}}},
        "destination": {"location": {"latLng": destination}},
        "travelMode": "DRIVE",
        "polylineQuality": "HIGH_QUALITY"
    }
    response = requests.post(url, headers=headers, json=data)
    route_data = response.json()

    if "routes" in route_data:
        encoded_polyline = route_data["routes"][0]["polyline"]["encodedPolyline"]
        return np.array(polyline.decode(encoded_polyline))
    else:
        print("Error: Could not fetch route.")
        print("Google response:", route_data)
        return None

def estimate_throttle_duration(distance_m):
    if distance_m <= 1:
        return 2.45
    elif distance_m <= 5:
        return 2.45 + (distance_m - 1) * ((12.4 - 2.45) / 4)
    elif distance_m <= 10:
        return 12.4 + (distance_m - 5) * ((24.6 - 12.4) / 5)
    else:
        return 24.6  # cap max duration

def send_command(cmd):
    rospy.loginfo(f"📤 Sending command: {cmd}")
    cmd_pub.publish(String(data=cmd))
    time.sleep(0.2)

# ==== MAIN ====
listener_thread = threading.Thread(target=socket_listener, daemon=True)
listener_thread.start()

print("Getting GPS position...")
start_lat, start_lon = get_gps_data()
print(f"Start: {start_lat}, {start_lon}")

print("Fetching route...")
refined_waypoints = fetch_route_from_google(start_lat, start_lon)
if refined_waypoints is None:
    print("Route fetch failed. Exiting.")
    exit()

# Plotting
plt.ion()
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_title("Live GPS Navigation")
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
ax.grid(True)
ax.axis('equal')

ax.plot(refined_waypoints[:, 1], refined_waypoints[:, 0], 'g-', label="Planned Route")
ax.plot(start_lon, start_lat, 'bo', markersize=8, label="Start")
ax.plot(destination["longitude"], destination["latitude"], 'ks', markersize=8, label="Destination")
position_dot, = ax.plot([], [], 'ro', markersize=6, label="Current")
trail_plot, = ax.plot([], [], 'b--', alpha=0.7, label="Trail")
ax.legend()

trail = []
timestamps = []
arrived = False

lat, lon = get_gps_data()
dist_to_dest = geodesic((lat, lon), (destination["latitude"], destination["longitude"])).meters
throttle_time = estimate_throttle_duration(dist_to_dest)

# 🔁 First throttle (if no obstacle)
if not obstacle_detected:
    send_command("throttle")
else:
    rospy.logwarn("Initial obstacle present. Waiting...")

start_time = time.time()

while not rospy.is_shutdown() and not arrived:
    lat, lon = get_gps_data()
    current_time = time.time()

    trail.append((lat, lon))
    timestamps.append(current_time)
    position_dot.set_data([lon], [lat])

    if len(trail) > 1:
        try:
            trail_np = np.array(trail)
            trail_plot.set_data(trail_np[:, 1], trail_np[:, 0])
        except Exception as e:
            print(f"Trail plotting error: {e}")

    # Live distance tracking
    dist_to_dest = geodesic((lat, lon), (destination["latitude"], destination["longitude"])).meters
    speed = (geodesic(trail[-2], trail[-1]).meters / (timestamps[-1] - timestamps[-2])) if len(trail) > 1 else 0
    eta = dist_to_dest / speed if speed > 0.5 else None

    print("\033c", end="")
    print("📍 Current Location:")
    print(f"   Lat: {lat:.6f}, Lon: {lon:.6f}")
    print(f"📏 Distance to Destination: {dist_to_dest:.2f} m")
    print(f"🚀 Speed: {speed:.2f} m/s")
    if eta:
        print(f"🕒 ETA: {eta:.1f} sec ({eta/60:.1f} min)")
    else:
        print("🕒 Speed too low to estimate ETA.")

    # === STOP CHECK ===
    if dist_to_dest < DISTANCE_THRESHOLD:
        rospy.loginfo("🎯 Destination Reached!")
        send_command("reverse_throttle")
        send_command("stop")
        arrived = True
        break

    # === Re-check every timeout ===
    if current_time - start_time > throttle_time:
        rospy.loginfo("⏱️ Rechecking conditions after throttle duration...")
        send_command("reverse_throttle")
        time.sleep(1)
        throttle_time = estimate_throttle_duration(dist_to_dest)

        if not obstacle_detected:
            send_command("throttle")
        else:
            rospy.logwarn("🛑 Obstacle still present. Throttle paused.")
        start_time = time.time()

    plt.draw()
    plt.pause(0.5)

print("✅ Navigation complete.")

