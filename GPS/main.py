import sys
import os
import requests
import polyline
import folium
import time
import math
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, 
                            QWidget, QLabel, QPushButton, QHBoxLayout,
                            QMessageBox, QSizePolicy)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QTimer, QUrl, Qt
from geopy.distance import geodesic

# Configuration
API_KEY = "AIzaSyDN6Txao6CqdD-txYakU8Q6YS4iqzuSO08"
GPS_UPDATE_INTERVAL = 1000  # ms
WAYPOINT_INTERVAL = 5  # meters
MAP_REFRESH_INTERVAL = 5000  # ms (refresh map every 5 seconds)

# FIXED STARTING AND DESTINATION POSITION
START_LATITUDE = 12.820176536812353
START_LONGITUDE = 80.03980131757676
DEST_LATITUDE = 12.820679990213058
DEST_LONGITUDE =  80.04001723540208

class GPSNavigator(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live GPS Navigation")
        self.setGeometry(100, 100, 1200, 800)

        self.start_lat, self.start_lon = START_LATITUDE, START_LONGITUDE
        self.dest_lat, self.dest_lon = DEST_LATITUDE, DEST_LONGITUDE

        self.central_widget = QWidget()
        self.main_layout = QVBoxLayout()
        self.browser = QWebEngineView()
        self.main_layout.addWidget(self.browser)
        self.central_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.central_widget)

        self.simulated_position = None
        self.simulation_step = 0
        self.route = []
        self.waypoints = []

        self.update_counter = 0

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_position)

        self.fetch_route()
        self.generate_waypoints()
        self.generate_map()
        self.timer.start(GPS_UPDATE_INTERVAL)
        self.show()

    def fetch_route(self):
        origin = f"{self.start_lat},{self.start_lon}"
        destination = f"{self.dest_lat},{self.dest_lon}"
        url = f"https://maps.googleapis.com/maps/api/directions/json?origin={origin}&destination={destination}&key={API_KEY}"

        try:
            response = requests.get(url)
            if response.status_code == 200:
                data = response.json()
                if data['status'] == 'OK':
                    points = data['routes'][0]['overview_polyline']['points']
                    decoded = polyline.decode(points)
                    self.route = [(lat, lon) for lat, lon in decoded]
        except Exception as e:
            print(f"Error fetching route: {e}")

    def generate_waypoints(self):
        self.waypoints = []
        for i in range(len(self.route)-1):
            start = self.route[i]
            end = self.route[i+1]
            distance = geodesic(start, end).meters
            steps = int(distance / WAYPOINT_INTERVAL)
            if steps > 0:
                lat_step = (end[0] - start[0]) / steps
                lon_step = (end[1] - start[1]) / steps
                for j in range(steps):
                    lat = start[0] + j * lat_step
                    lon = start[1] + j * lon_step
                    self.waypoints.append((lat, lon))
        self.waypoints.append(self.route[-1])

    def update_position(self):
        if self.simulation_step < len(self.waypoints):
            self.simulated_position = self.waypoints[self.simulation_step]
            self.simulation_step += 1

        self.update_counter += GPS_UPDATE_INTERVAL
        if self.update_counter >= MAP_REFRESH_INTERVAL:
            self.update_counter = 0
            self.generate_map()

    def generate_map(self):
        if self.simulated_position:
            lat, lon = self.simulated_position
        else:
            lat, lon = self.start_lat, self.start_lon

        m = folium.Map(location=[lat, lon], zoom_start=20, max_zoom=20, min_zoom=20)

        folium.Marker(
            [self.start_lat, self.start_lon],
            popup="Start",
            icon=folium.Icon(color="green")
        ).add_to(m)

        folium.Marker(
            [self.dest_lat, self.dest_lon],
            popup="Destination",
            icon=folium.Icon(color="red")
        ).add_to(m)

        if self.route:
            folium.PolyLine(
                self.route,
                color="green",
                weight=5,
                opacity=0.8
            ).add_to(m)

        if self.simulated_position:
            folium.Marker(
                self.simulated_position,
                popup="You",
                icon=folium.Icon(color="blue", icon="car", prefix="fa")
            ).add_to(m)

        data = m._repr_html_()
        self.browser.setHtml(data)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GPSNavigator()
    sys.exit(app.exec_())

