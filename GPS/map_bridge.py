# map_bridge.py
from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

destination = {"lat": None, "lon": None}

@app.route('/set_destination', methods=['POST'])
def set_destination():
    data = request.get_json()
    print("📬 Received destination:", data)
    destination['lat'] = data['lat']
    destination['lon'] = data['lon']
    return jsonify(status="OK")

@app.route('/get_destination')
def get_destination():
    return jsonify(destination)

if __name__ == "__main__":
    app.run(debug=False, port=5000)
