import requests
import polyline
import numpy as np
import pandas as pd
from geopy.distance import geodesic
import matplotlib.pyplot as plt


API_KEY = "AIzaSyDN6Txao6CqdD-txYakU8Q6YS4iqzuSO08"  # Replace with your actual key

# Replace with start/end from your .bag
start_lat, start_lon = 12.820049907593226, 80.03978853561273
end_lat, end_lon = 12.820669054378678, 80.04001014351456

def fetch_route_from_google(start_lat, start_lon, end_lat, end_lon):
    url = "https://routes.googleapis.com/directions/v2:computeRoutes"
    headers = {
        "Content-Type": "application/json",
        "X-Goog-Api-Key": API_KEY,
        "X-Goog-FieldMask": "routes.polyline"
    }
    data = {
        "origin": {"location": {"latLng": {"latitude": start_lat, "longitude": start_lon}}},
        "destination": {"location": {"latLng": {"latitude": end_lat, "longitude": end_lon}}},
        "travelMode": "DRIVE",
        "polylineQuality": "HIGH_QUALITY"
    }

    response = requests.post(url, headers=headers, json=data)
    route_data = response.json()

    if "routes" in route_data:
        encoded_polyline = route_data["routes"][0]["polyline"]["encodedPolyline"]
        return np.array(polyline.decode(encoded_polyline))
    else:
        print("❌ Error: Could not fetch route.")
        print(route_data)
        return None

# Fetch path
route = fetch_route_from_google(start_lat, start_lon, end_lat, end_lon)

if route is not None:
    df = pd.DataFrame(route, columns=["latitude", "longitude"])

    # Calculate distances between waypoints
    segment_distances = []
    total_distance = 0.0

    for i in range(len(df) - 1):
        point_a = (df.loc[i, "latitude"], df.loc[i, "longitude"])
        point_b = (df.loc[i+1, "latitude"], df.loc[i+1, "longitude"])
        dist = geodesic(point_a, point_b).meters
        segment_distances.append(dist)
        total_distance += dist

    segment_distances.append(0)  # last point has no next segment
    df["segment_distance_m"] = segment_distances
    df["cumulative_distance_m"] = np.cumsum(segment_distances)

    df.to_csv("google_route_detailed.csv", index=False)
    print("✅ Route with distances saved as google_route_detailed.csv")

    # Plot
    plt.figure(figsize=(6, 6))
    plt.plot(df["longitude"].values, df["latitude"].values, 'g-', label="Google Route")
    plt.scatter([start_lon], [start_lat], color='blue', label='Start')
    plt.scatter([end_lon], [end_lat], color='red', label='End')
    plt.title("Google Route Path with Segments")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    print(f"📏 Total Route Distance: {total_distance:.2f} meters")
else:
    print("❌ Route not retrieved.")


