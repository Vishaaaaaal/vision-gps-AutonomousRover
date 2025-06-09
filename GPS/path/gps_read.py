import serial
import pynmea2

# Set the correct port and baud rate
port = "/dev/ttyACM0"  # Updated to match your GPS module
baudrate = 115200  # Default baud rate for NEO-M9N

try:
    # Open serial connection
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"✅ Connected to GPS module on {port} at {baudrate} baud")
except serial.SerialException as e:
    print(f"❌ Could not open port {port}: {e}")
    exit()

# Read and parse GPS data
while True:
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith("$G"):
            try:
                msg = pynmea2.parse(line)
                print(f"📍 GPS Data - Lat: {msg.latitude}, Lon: {msg.longitude}, Alt: {msg.altitude}")
            except pynmea2.ParseError as e:
                print(f"⚠️ Parse Error: {e}")
        else:
            print(f"🔹 Raw Data: {line}")  # Print other NMEA sentences
    except KeyboardInterrupt:
        print("\n🛑 Stopping GPS data reading.")
        ser.close()
        break
    except Exception as e:
        print(f"❌ Error: {e}")

