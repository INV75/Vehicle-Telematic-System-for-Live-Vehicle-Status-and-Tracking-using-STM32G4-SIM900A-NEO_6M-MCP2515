import os
import time
import requests

# Automatically find the folder this script is saved in to prevent FileNotFoundError
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------
# This is the API Endpoint (The "Back Door"), NOT the browser dashboard URL!
NODE_RED_URL = "http://localhost:1880/sim-data" 

CAN_FILE = os.path.join(BASE_DIR, "simulated_can_acceleration.txt")
GPS_FILE = os.path.join(BASE_DIR, "test_coordinates.txt")

# ---------------------------------------------------------
# STATE VARIABLES (The "Virtual Car")
# ---------------------------------------------------------
current_state = {
    "speed": 0.0,
    "rpm": 0.0,
    "temp": 0.0,
    "lat": 0.0,
    "lon": 0.0
}

def load_gps_data(filepath):
    """Reads the GPS CSV and returns a list of (lat, lon) tuples."""
    coords = []
    try:
        with open(filepath, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) == 2:
                    coords.append((float(parts[0]), float(parts[1])))
    except FileNotFoundError:
        print(f"❌ ERROR: Could not find {filepath}. Ensure it is in the same folder as this script.")
        exit(1)
    return coords

def main():
    print("🚗 Starting Vehicle Simulation Engine...")
    
    gps_coords = load_gps_data(GPS_FILE)
    gps_index = 0
    gps_length = len(gps_coords)

    try:
        with open(CAN_FILE, 'r') as can_file:
            for line in can_file:
                if line.startswith(';'):
                    continue
                
                parts = line.split()
                if len(parts) < 11:
                    continue
                
                try:
                    timestamp_ms = float(parts[1])
                    can_id = parts[3]
                    dlc = int(parts[4])
                    data_bytes = [int(b, 16) for b in parts[5:5+dlc]]
                    
                    # Decode CAN IDs
                    if can_id == "0106":
                        current_state["rpm"] = ((data_bytes[0] * 256) + data_bytes[1]) / 4.0
                        current_state["speed"] = data_bytes[2] 
                    elif can_id == "0197":
                        current_state["temp"] = data_bytes[0] - 40

                    # Sync GPS
                    if gps_index < gps_length:
                        current_state["lat"] = gps_coords[gps_index][0]
                        current_state["lon"] = gps_coords[gps_index][1]
                        gps_index += 1
                    
                    # ---------------------------------------------------------
                    # TRANSMIT TO NODE-RED
                    # ---------------------------------------------------------
                    try:
                        # Send the POST request to the HTTP IN node
                        response = requests.post(NODE_RED_URL, json=current_state, timeout=2)
                        
                        # Check if Node-RED actually accepted the data
                        if response.status_code == 200:
                            print(f"✅ Sent -> RPM: {current_state['rpm']:.0f} | Speed: {current_state['speed']} | Temp: {current_state['temp']}")
                        elif response.status_code == 404:
                            print(f"❌ ERROR 404: Node-RED is running, but the '/sim-data' endpoint is missing. Check your 'http in' node!")
                        else:
                            print(f"⚠️ Warning: Node-RED returned status code {response.status_code}")

                    except requests.exceptions.ConnectionError:
                        print("❌ ERROR: Could not connect to Node-RED. Is the Node-RED server running on port 1880?")
                        break

                    # 100ms delay to simulate real-time
                    time.sleep(0.1)

                except Exception as e:
                    pass # Ignore malformed trace lines silently
                    
    except FileNotFoundError:
        print(f"❌ ERROR: Could not find {CAN_FILE}. Ensure it is in the same folder as this script.")

    print("🏁 Simulation Complete.")

if __name__ == "__main__":
    main()