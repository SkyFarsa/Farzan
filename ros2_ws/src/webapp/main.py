from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse
from pydantic import BaseModel
import json, os, subprocess

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

DESTINATIONS_FILE = "destinations.json"

# Load destinations from file
def load_destinations():
    if os.path.exists(DESTINATIONS_FILE):
        with open(DESTINATIONS_FILE) as f:
            return json.load(f)
    return {}

# Save destinations to file
def save_destinations(data):
    with open(DESTINATIONS_FILE, "w") as f:
        json.dump(data, f, indent=2)

# --- Models ---
class Destination(BaseModel):
    name: str
    x: float
    y: float
    yaw: float  # robot orientation when it arrives

# --- Admin: add a destination ---
@app.post("/destinations")
def add_destination(dest: Destination):
    data = load_destinations()
    data[dest.name] = {"x": dest.x, "y": dest.y, "yaw": dest.yaw}
    save_destinations(data)
    return {"status": "saved", "destination": dest.name}

# --- Admin: delete a destination ---
@app.delete("/destinations/{name}")
def delete_destination(name: str):
    data = load_destinations()
    if name in data:
        del data[name]
        save_destinations(data)
    return {"status": "deleted"}

# --- Client: list all destinations ---
@app.get("/destinations")
def get_destinations():
    return load_destinations()

# --- Client: send robot to a destination ---
@app.post("/go/{name}")
def go_to_destination(name: str):
    data = load_destinations()
    if name not in data:
        return JSONResponse(status_code=404, content={"error": "Unknown destination"})
    
    dest = data[name]
    # Use ros2 action CLI to send Nav2 goal
    cmd = [
        "ros2", "action", "send_goal",
        "/navigate_to_pose",
        "nav2_msgs/action/NavigateToPose",
        f"{{pose: {{header: {{frame_id: 'map'}}, pose: {{position: {{x: {dest['x']}, y: {dest['y']}, z: 0.0}}, orientation: {{w: 1.0}}}}}}}}"
    ]
    subprocess.Popen(cmd)  # fire and forget
    return {"status": "going", "destination": name}

# --- Robot status (client polls this) ---
@app.get("/status")
def get_status():
    # Simple: just return "active" for now
    # You can expand this later with actual Nav2 feedback
    return {"status": "active"}
