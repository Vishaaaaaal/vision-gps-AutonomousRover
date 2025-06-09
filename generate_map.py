import folium

start_coords = [12.8199, 80.0386]
m = folium.Map(location=start_coords, zoom_start=18)

map_id = m.get_name()

# JavaScript: Send clicked lat/lon to Flask and display popup
click_js = f"""
<script>
window.addEventListener('load', function () {{
    var map = window.{map_id};
    window.map = map;

    map.on('click', function(e) {{
        var lat = e.latlng.lat;
        var lng = e.latlng.lng;

        // Popup
        L.popup()
            .setLatLng(e.latlng)
            .setContent("✅ Destination Set:<br>Lat: " + lat.toFixed(5) + "<br>Lon: " + lng.toFixed(5))
            .openOn(map);

        // Send to Flask
        fetch('http://localhost:5000/set_destination', {{
            method: 'POST',
            headers: {{
                'Content-Type': 'application/json'
            }},
            body: JSON.stringify({{ lat: lat, lon: lng }})
        }})
        .then(response => response.json())
        .then(data => console.log("✅ Flask response:", data))
        .catch(error => console.error("❌ Error sending destination:", error));
    }});
}});
</script>
"""

m.get_root().html.add_child(folium.Element(click_js))
m.save("map.html")
print("✅ Destination-enabled map generated.")

