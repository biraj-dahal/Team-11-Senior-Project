import React, { useRef, useState, useEffect } from "react";
import "./RobotDashboard.css";
import "./RemoteControl.css";
import Navbar from "../components/Navbar";
// HEllow ord
const RemoteControl = () => {
  const [sliders, setSliders] = useState([
    245.84, 39.66, 100.21, 250.67, 345.7, 331.63, 97.46,
  ]);
  const [fingersValue, setFingersValue] = useState(100);
  const ws = useRef(null);

  const handleSliderChange = (index, value) => {
    const updated = [...sliders];
    updated[index] = parseFloat(value);
    setSliders(updated);
  };

  const [mode, setMode] = useState("automatic");
  const isManual = mode === "automatic";

  useEffect(() => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      const control_type = isManual ? "manual_control" : "automatic_control";
      const control = sliders;
      const gripper = fingersValue === 100 ? "open" : "closed";

      const message = {
        type: "controls",
        control_type,
        control,
        gripper,
      };

      ws.current.send(JSON.stringify(message));
    }
  }, [fingersValue, sliders, isManual]);

  useEffect(() => {
    ws.current = new WebSocket("ws://localhost:8000/robot_ws");

    ws.current.onopen = () => {
      console.log("ws opened");
      if (ws.current.readyState === WebSocket.OPEN) {
        const message = {
          type: "identification",
          identity: "frontend",
        };
        ws.current.send(message);
        console.log("identification message sent");
      }
    };
    ws.current.onclose = () => console.log("ws closed");

    return () => {
      ws.current.close();
    };
  }, []);

  return (
    <div className="app-container">
      <Navbar />
      <div className="dashboard-container">
        <h1 className="dashboard-title">Remote Control</h1>
        <div className="dashboard-grid">
          <div className="card status-card">
            <div className="container">
              <div className="status-section">
                <h2>Arm Controls</h2>
                <div className="slider-panel">
                  {sliders.map((val, i) => (
                    <div key={i} className="slider-group">
                      <input
                        type="number"
                        className="slider-value"
                        value={val.toFixed(2)}
                        onChange={(e) => handleSliderChange(i, e.target.value)}
                        disabled={isManual}
                      />
                      <input
                        type="range"
                        min="0"
                        max="360"
                        step="0.1"
                        value={val}
                        onChange={(e) => handleSliderChange(i, e.target.value)}
                        className="vertical-slider"
                        disabled={isManual}
                      />
                    </div>
                  ))}
                </div>

                <div className="fingers-control">
                  <label className="switch-label">Grippers</label>
                  <label className="switch">
                    <input
                      type="checkbox"
                      checked={fingersValue === 100}
                      onChange={(e) =>
                        setFingersValue(e.target.checked ? 100 : 0)
                      }
                      disabled={isManual}
                    />
                    <span className="slider-switch"></span>
                  </label>
                  <div className="grip-indicator">
                    {fingersValue === 100 ? "Closed" : "Open"}
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
        <div className="control-bar">
          <div className="mode-toggle">
            <label className="mode-label">Manual</label>
            <label className="switch">
              <input
                type="checkbox"
                checked={mode === "automatic"}
                onChange={() =>
                  setMode(mode === "manual" ? "automatic" : "manual")
                }
              />
              <span className="slider-switch"></span>
            </label>
            <label className="mode-label">Automatic</label>
          </div>

          <button className="emergency-stop-button">Emergency Stop</button>
        </div>
      </div>
    </div>
  );
};

export default RemoteControl;
