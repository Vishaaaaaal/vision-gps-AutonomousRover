import requests
import json
import polyline
import numpy as np
import matplotlib.pyplot as plt
from geopy.distance import geodesic
from scipy.interpolate import CubicSpline

# Google Maps API Key (Replace with your actual key)
API_KEY = "AIzaSyDN6Txao6CqdD-txYakU8Q6YS4iqzuSO08"

# Define Start and End Points
origin = {"latitude": 12.820903282839026, "longitude": 80.03926411601142}
destination = {"latitude": 12.820566557822302, "longitude": 80.03938146243097}

# API Request URL
url = "https://routes.googleapis.com/directions/v2:computeRoutes"

# Define Headers
headers = {
    "Content-Type": "application/json",
    "X-Goog-Api-Key": API_KEY,
    "X-Goog-FieldMask": "routes.duration,routes.distanceMeters,routes.polyline,routes.legs.steps"
}

# Define API Request Body with Route Modifiers
data = {
    "origin": {"location": {"latLng": origin}},
    "destination": {"location": {"latLng": destination}},
    "travelMode": "DRIVE",
    "polylineQuality": "HIGH_QUALITY"
}

# Send API Request
response = requests.post(url, headers=headers, json=data)
route_data = response.json()

# Extract Route Information
if "routes" in route_data:
    route = route_data["routes"][0]
    encoded_polyline = route["polyline"]["encodedPolyline"]

    # Decode the polyline into GPS coordinates
    waypoints = np.array(polyline.decode(encoded_polyline))

    # Compute Haversine Distances Along the Path
    distances = [0]
    for i in range(1, len(waypoints)):
        dist = geodesic(waypoints[i - 1], waypoints[i]).meters
        distances.append(distances[-1] + dist)

    total_distance = distances[-1]
    print(f"Total Route Distance: {total_distance:.2f} meters")
    print(f"Total Original API Waypoints: {len(waypoints)}")

    # Define new equal-distance waypoints (every 2 meters)
    spacing = 2  
    num_waypoints = int(total_distance / spacing)
    new_distances = np.linspace(0, total_distance, num_waypoints)

    # Interpolate Latitude & Longitude
    lat_interp = np.interp(new_distances, distances, waypoints[:, 0])
    lon_interp = np.interp(new_distances, distances, waypoints[:, 1])
    refined_waypoints = np.column_stack((lat_interp, lon_interp))

    # Apply Pure Pursuit-Like Smoothing on Refined Waypoints
    smoothed_path = refined_waypoints.copy()
    lookahead_distance = 6  # Adjust this for smoothing effect

    for i in range(1, len(smoothed_path) - 1):
        # Compute a circular arc from previous to next point
        prev_point = smoothed_path[i - 1]
        curr_point = smoothed_path[i]
        next_point = smoothed_path[i + 1]

        # Calculate mid-point and curvature
        mid_point = (prev_point + next_point) / 2
        radius = geodesic(prev_point, next_point).meters / 2
        curvature = 1 / radius if radius > 0 else 0

        # Modify current point position to follow a smooth transition
        if curvature > 0:
            smoothed_path[i] = 0.6 * curr_point + 0.2 * prev_point + 0.2 * next_point

    # Plot Paths
    plt.figure(figsize=(8, 6))
    plt.axis("equal")  

    # Original API waypoints
    plt.plot(waypoints[:, 1], waypoints[:, 0], 'ro', markersize=5, label="Original API Waypoints")
    plt.plot(waypoints[0, 1], waypoints[0, 0], 'go', markersize=7, label="Start Point")
    plt.plot(waypoints[-1, 1], waypoints[-1, 0], 'go', markersize=7, label="End Point")

    # Refined waypoints (2m spacing) in Orange
    plt.plot(refined_waypoints[:, 1], refined_waypoints[:, 0], 'o', color="orange", markersize=4, label="Refined 2m Waypoints")

    # Unsmoothed path (grey)
    plt.plot(refined_waypoints[:, 1], refined_waypoints[:, 0], '--', color="grey", linewidth=1, label="Unsmoothed Path")

    # Smoothed Pure Pursuit Path (Blue)
    plt.plot(smoothed_path[:, 1], smoothed_path[:, 0], 'b-', linewidth=2, alpha=0.75, label="Pure Pursuit Smooth Path")

    # Labels & Titles
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title(f"Generated Path with Pure Pursuit-Like Smoothing (2m Spacing, Total Distance: {total_distance:.2f}m)")
    plt.legend()
    plt.grid()
    plt.show()

else:
    print("Error: No valid route found in API response.")

