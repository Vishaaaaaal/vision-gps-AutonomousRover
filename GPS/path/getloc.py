import requests
import json

# Google Maps API Key (Replace with your actual key)
API_KEY = "AIzaSyDN6Txao6CqdD-txYakU8Q6YS4iqzuSO08"

# Define Origin and Destination
origin = {"latitude": 12.8208087, "longitude": 80.0388578}
destination = {"latitude": 12.8215000, "longitude": 80.0392000}

# API Request URL
url = "https://routes.googleapis.com/directions/v2:computeRoutes"

# Define Headers
headers = {
    "Content-Type": "application/json",
    "X-Goog-Api-Key": "AIzaSyDN6Txao6CqdD-txYakU8Q6YS4iqzuSO08",
    "X-Goog-FieldMask": "routes.duration,routes.distanceMeters,routes.polyline"
}

# Define API Request Body
data = {
    "origin": {"location": {"latLng": origin}},
    "destination": {"location": {"latLng": destination}},
    "travelMode": "DRIVE"
}

# Send API Request
response = requests.post(url, headers=headers, json=data)
route_data = response.json()

# Extract Route Information
if "routes" in route_data:
    route = route_data["routes"][0]
    distance = route["distanceMeters"]
    duration = route["duration"]
    polyline = route["polyline"]["encodedPolyline"]

    print(f"Distance: {distance} meters")
    print(f"Duration: {duration}")
    print(f"Encoded Polyline: {polyline}")

else:
    print("Error fetching route data.")

