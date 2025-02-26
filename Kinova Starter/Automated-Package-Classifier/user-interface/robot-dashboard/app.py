from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import random
import json
from datetime import datetime

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

active_connections = []
status_data = {"state": "Idle", "position": "0,0", "cpu_usage": 10, "memory_usage": 20}
logs = []

def generate_fake_logs():
    levels = ["INFO", "WARNING", "ERROR"]
    messages = [
        "Routine maintenance check performed",
        "Low battery warning",
        "Obstacle detected, rerouting",
        "System reboot required",
    ]
    return {"level": random.choice(levels), "message": random.choice(messages), "timestamp": datetime.utcnow().isoformat()}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.append(websocket)
    try:
        while True:
            status_data["cpu_usage"] = random.randint(10, 80)
            status_data["memory_usage"] = random.randint(20, 90)
            status_data["position"] = f"{random.randint(0, 100)},{random.randint(0, 100)}"
            await websocket.send_json(status_data)
            await asyncio.sleep(2)
    except Exception as e:
        print(f"WebSocket Error: {e}")
    finally:
        active_connections.remove(websocket)

@app.get("/api/stats")
async def get_stats():
    return status_data

@app.get("/api/logs")
async def get_logs(limit: int = 10):
    global logs
    logs.append(generate_fake_logs())
    return logs[-limit:]

@app.post("/api/control")
async def control_robot(command: str):
    if command.upper() == "STOP":
        status_data["state"] = "Stopped"
        return {"message": "Robot stopped"}
    raise HTTPException(status_code=400, detail="Invalid command")
