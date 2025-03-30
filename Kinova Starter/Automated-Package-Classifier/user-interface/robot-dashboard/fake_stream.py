from fastapi import FastAPI, WebSocket
import asyncio
import json
import random

app = FastAPI()

async def fake_status_stream(websocket: WebSocket):
    await websocket.accept()
    while True:
        status_data = {
            "state": "Running",
            "position": f"X:{random.randint(0, 100)}, Y:{random.randint(0, 100)}, Z:{random.randint(0,360)}",  # Random X, Y positions
            "rollpitchyaw": f"Roll:{random.randint(0, 100)}, Pitch:{random.randint(0, 100)}, Raw:{random.randint(0,360)}",  # Random X, Y positions
            "cpu_usage": round(random.uniform(10, 90), 2),  # Random CPU usage between 10% and 90%
            "memory_usage": round(random.uniform(30, 80), 2),  # Random Memory usage between 30% and 80%
            "timestamp": "2025-03-08T12:00:00",
            "ip_addr": "10.0.0.7"
        }
        await websocket.send_text(json.dumps(status_data))
        await asyncio.sleep(1)  # Simulate new data every 2 seconds

async def fake_stats_stream(websocket: WebSocket):
    await websocket.accept()
    while True:
        status_data = {
            "state": "Running",
            "position": f"X:{random.randint(0, 100)}, Y:{random.randint(0, 100)}, Z:{random.randint(0,360)}",  # Random X, Y positions
            "rollpitchyaw": f"Roll:{random.randint(0, 100)}, Pitch:{random.randint(0, 100)}, Raw:{random.randint(0,360)}",  # Random X, Y positions
            "cpu_usage": round(random.uniform(10, 90), 2),  # Random CPU usage between 10% and 90%
            "memory_usage": round(random.uniform(30, 80), 2),  # Random Memory usage between 30% and 80%
            "timestamp": "2025-03-08T12:00:00",
            "ip_addr": "10.0.0.7"
        }
        await websocket.send_text(json.dumps(status_data))
        await asyncio.sleep(1)  # Simulate new data every 2 seconds

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await fake_status_stream(websocket)


@app.websocket("/api/stats")
async def websocket_endpoint(websocket: WebSocket):
    await fake_stats_stream(websocket)
