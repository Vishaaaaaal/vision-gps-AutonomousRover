import sys
import time
import os
import polyline
import folium
import requests
import csv
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QWidget, QMessageBox, QSizePolicy, QSpinBox
)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QTimer, QUrl
from geopy.distance import geodesic

API_KEY = "AIzaSyDN6Txao6CqdD-txYakU8Q6YS4iqzuSO08"
MAP_FILE = "route_map.html"
WAYPOINT_SPACING_METERS = 1  # Changed to 1 meter spacing

# Hardcoded coordinates
START_LAT = 12.820157252931018
START_LON = 80.03925968752883
DEST_LAT = 12.820674775677514
DEST_LON = 80.04001391725382

class GPSWebMapApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🌍 VisionFirst Navigator - Real Map")
        self.setGeometry(100, 100, 1400, 700)

        self.fetch_button = QPushButton("🧭 Get Route")
        self.fetch_button.clicked.connect(self.generate_route_map)

        self.start_button = QPushButton("▶️ Start")
        self.start_button.clicked.connect(self.start_simulation)
        self.start_button.setEnabled(False)

        self.pause_button = QPushButton("⏸ Pause")
        self.pause_button.clicked.connect(self.pause_simulation)
        self.pause_button.setEnabled(False)

        self.export_button = QPushButton("💾 Export Route")
        self.export_button.clicked.connect(self.export_route)
        self.export_button.setEnabled(False)

        self.speed_label = QLabel("Speed (ms):")
        self.speed_spinbox = QSpinBox()
        self.speed_spinbox.setRange(1000, 10000)
        self.speed_spinbox.setValue(5000)
        self.speed_spinbox.setSingleStep(500)

        self.web_view = QWebEngineView()
        self.web_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.status_label = QLabel("🔍 Click 'Get Route' to generate route between hardcoded locations")

        control_layout = QHBoxLayout()
        control_layout.addWidget(self.fetch_button)
        control_layout.addWidget(self.start_button)
        control_layout.addWidget(self.pause_button)
        control_layout.addWidget(self.export_button)
        control_layout.addWidget(self.speed_label)
        control_layout.addWidget(self.speed_spinbox)

        layout = QVBoxLayout()
        layout.setSpacing(5)
        layout.addLayout(control_layout)
        layout.addWidget(self.status_label)
        layout.addWidget(self.web_view)  # Only the web view remains

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.route = []
        self.timestamps = []
        self.sim_index = 0
        self.destination = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_marker)
        self.total_distance = 0
        self.last_update_time = 0

    def generate_route_map(self):
        self.status_label.setText("📡 Fetching route...")
        QApplication.processEvents()

        url = "https://routes.googleapis.com/directions/v2:computeRoutes"
        headers = {
            "Content-Type": "application/json",
            "X-Goog-Api-Key": API_KEY,
            "X-Goog-FieldMask": "routes.polyline"
        }
        data = {
            "origin": {"location": {"latLng": {"latitude": START_LAT, "longitude": START_LON}}},
            "destination": {"location": {"latLng": {"latitude": DEST_LAT, "longitude": DEST_LON}}},
            "travelMode": "DRIVE",
            "polylineQuality": "HIGH_QUALITY"
        }
        
        try:
            response = requests.post(url, headers=headers, json=data)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            QMessageBox.critical(self, "API Error", f"Failed to fetch route: {str(e)}")
            return

        route_data = response.json()
        if "routes" not in route_data:
            QMessageBox.critical(self, "Route Error", "No routes returned from Google.")
            return

        encoded = route_data["routes"][0]["polyline"]["encodedPolyline"]
        self.route = polyline.decode(encoded)
        self.sim_index = 0
        self.timestamps = []
        self.total_distance = 0
        self.destination = self.route[-1]
        self.last_update_time = 0

        self.save_map_with_marker(self.route[0])
        self.start_button.setEnabled(True)
        self.pause_button.setEnabled(True)
        self.export_button.setEnabled(True)
        self.status_label.setText(f"✅ Route ready from ({START_LAT}, {START_LON}) to ({DEST_LAT}, {DEST_LON}). Click ▶️ Start.")

    def generate_waypoints(self):
        waypoints = [self.route[0]]
        accumulated = 0
        for i in range(1, len(self.route)):
            segment = geodesic(self.route[i-1], self.route[i]).meters
            accumulated += segment
            if accumulated >= WAYPOINT_SPACING_METERS:
                waypoints.append(self.route[i])
                accumulated = 0
        return waypoints

    def save_map_with_marker(self, current_pos):
        fmap = folium.Map(location=current_pos, zoom_start=18, control_scale=True)
        folium.Marker(location=self.route[0], tooltip="Start", icon=folium.Icon(color='blue')).add_to(fmap)
        folium.Marker(location=self.destination, tooltip="Destination", icon=folium.Icon(color='red')).add_to(fmap)
        folium.PolyLine(self.route, color="green", weight=5).add_to(fmap)
        folium.Marker(location=current_pos, tooltip="Current", icon=folium.Icon(color='orange')).add_to(fmap)

        # Add waypoints every 1 meter (smaller and more transparent)
        waypoints = self.generate_waypoints()
        for wp in waypoints:
            folium.CircleMarker(
                location=wp,
                radius=2,  # Smaller radius for more frequent waypoints
                color='purple',
                fill=True,
                fill_opacity=0.4,  # More transparent
                tooltip=f"Waypoint {waypoints.index(wp)+1}"
            ).add_to(fmap)

        fmap.save(MAP_FILE)
        self.web_view.load(QUrl.fromLocalFile(os.path.abspath(MAP_FILE)))

    def start_simulation(self):
        speed = self.speed_spinbox.value()
        self.timer.start(speed)
        self.last_update_time = time.time()

    def pause_simulation(self):
        if self.timer.isActive():
            self.timer.stop()
            self.pause_button.setText("▶️ Resume")
        else:
            self.start_simulation()
            self.pause_button.setText("⏸ Pause")

    def update_marker(self):
        if self.sim_index >= len(self.route):
            self.timer.stop()
            
            # Create a custom message box with large text
            msg = QMessageBox()
            msg.setWindowTitle("Arrived!")
            msg.setText("🎉 YOU'VE ARRIVED AT YOUR DESTINATION! 🎉")
            
            # Make the text huge
            msg.setStyleSheet("""
                QLabel {
                    font-size: 24px;
                    font-weight: bold;
                }
                QPushButton {
                    font-size: 18px;
                    min-width: 100px;
                    min-height: 40px;
                }
            """)
            
            # Make the dialog larger
            msg.resize(600, 300)
            
            # Add a celebration emoji
            msg.setIcon(QMessageBox.Information)
            
            # Show the message
            msg.exec_()
            return

        current_time = time.time()
        if current_time - self.last_update_time < (self.speed_spinbox.value() / 1000):
            return

        current_pos = self.route[self.sim_index]
        now = time.time()
        self.timestamps.append(now)
        self.last_update_time = now

        if self.sim_index > 0:
            prev = self.route[self.sim_index-1]
            self.total_distance += geodesic(prev, current_pos).meters

        self.sim_index += 1
        self.save_map_with_marker(current_pos)
        dist = geodesic(current_pos, self.destination).meters
        self.status_label.setText(f"📍 Moving... Distance left: {dist:.1f} meters")

    def export_route(self):
        if not self.route:
            QMessageBox.warning(self, "No Route", "No route to export.")
            return
        with open("exported_route.csv", "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Latitude", "Longitude", "Timestamp"])
            for i, coord in enumerate(self.route[:len(self.timestamps)]):
                ts = self.timestamps[i] if i < len(self.timestamps) else ''
                writer.writerow([coord[0], coord[1], ts])
        QMessageBox.information(self, "Exported", "Route and timestamps saved to exported_route.csv")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GPSWebMapApp()
    window.show()
    sys.exit(app.exec_())
