import serial
import pynmea2
import requests
import polyline
import numpy as np
import matplotlib.pyplot as plt
from geopy.distance import geodesic
import time

API_KEY = "AIzaSyDN6Txao6CqdD-txYakU8Q6YS4iqzuSO08"

destination = {"latitude": 12.820753535208507, "longitude": 80.03986456084907}
GPS_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

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
        return None

def find_nearest_waypoint(current_pos, waypoints):
    distances = [geodesic(current_pos, wp).meters for wp in waypoints]
    min_idx = distances.index(min(distances))
    return min_idx, distances[min_idx]

def estimate_speed(trail, timestamps):
    if len(trail) < 2:
        return 0
    dist = geodesic(trail[-2], trail[-1]).meters
    time_elapsed = timestamps[-1] - timestamps[-2]
    return dist / time_elapsed if time_elapsed > 0 else 0

# Get live start location
print("Getting GPS position...")
start_lat, start_lon = get_gps_data()
print(f"Start: {start_lat}, {start_lon}")

# Get route from Google
print("Fetching route...")
refined_waypoints = fetch_route_from_google(start_lat, start_lon)
if refined_waypoints is None:
    print("Route fetch failed. Exiting.")
    exit()

# Setup plot
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

try:
    while True:
        lat, lon = get_gps_data()
        current_time = time.time()

        trail.append((lat, lon))
        timestamps.append(current_time)

        position_dot.set_data([lon], [lat])

        if len(trail) > 1:
            try:
                trail_np = np.array(trail)
                if trail_np.shape[0] > 1:
                    trail_plot.set_data(trail_np[:, 1], trail_np[:, 0])
            except Exception as e:
                print(f"Trail plotting error: {e}")

        # Distance to destination
        dist_to_dest = geodesic((lat, lon), (destination["latitude"], destination["longitude"])).meters

        # ETA estimation
        speed = estimate_speed(trail, timestamps)  # m/s
        eta = dist_to_dest / speed if speed > 0.5 else None

        # Clear console output
        print("\033c", end="")
        print("📍 Current Location:")
        print(f"   Lat: {lat:.6f}, Lon: {lon:.6f}")
        print(f"📏 Distance to Destination: {dist_to_dest:.2f} m")
        if eta:
            print(f"🕒 Estimated Time to Arrival: {eta:.1f} sec ({eta/60:.1f} min)")
        else:
            print("🕒 Speed too low to estimate ETA.")

        if dist_to_dest < 5 and not arrived:
            print("🎉🚩 You’ve ARRIVED at your destination!")
            arrived = True

        plt.draw()
        plt.pause(0.5)

except KeyboardInterrupt:
    print("Navigation stopped.")

