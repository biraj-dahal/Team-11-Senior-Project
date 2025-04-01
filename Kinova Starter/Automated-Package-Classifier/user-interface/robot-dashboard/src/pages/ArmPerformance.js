import React, { useState, useEffect } from "react";
import "./RobotDashboard.css";
import Navbar from "../components/Navbar";
import logo from '../assets/kinova_arm.jpeg'

const ArmPerf = () => {
  const [ws, setWs] = useState(null);
  const [status, setStatus] = useState("Offline");
  const [gripperStatus, setGripperStatus] = useState("Unknown");
  const [statistics, setStatistics] = useState(null);

  useEffect(() => {
    const socket = new WebSocket("ws://127.0.0.1:8000/dashboard_ws");

    socket.onopen = () => {
      console.log("WebSocket connected!");
      socket.send(JSON.stringify({ type: "arm_performance" }));
    };

    socket.onmessage = (event) => {
      console.log(event.data)
      const message = JSON.parse(JSON.parse(event.data));
      setStatus(message.operational_status);
      setGripperStatus(message.gripper_status);
      setStatistics(message.live_statistics);
      console.log(statistics);
    };

    socket.onclose = () => {
      console.log("WebSocket disconnected!");
    };

    socket.onerror = (error) => {
      console.error("WebSocket Error:", error);
    };

    console.log(status)

    setWs(socket);

    return () => {
      if (socket) {
        socket.close();
      }
    };
  }, []);

  return (
    <div className="app-container">
      <Navbar />
      <div className="dashboard-container">
        <h1 className="dashboard-title">Arm Performance</h1>
        <div className="dashboard-grid">
          <div className="card status-card">
            <div className="container">
              <div className="status-section">
                <div>
                  <h2>
                    Operational Status: {" "}
                    <span className="status online">{status ?? "Loading..."}</span>
                  </h2>
                  <h2>
                    Gripper Status: {" "}
                    <span className="status closed">{gripperStatus ?? "Loading..."}</span>
                  </h2>
                  <div className="statistics">
                    <h3>Live Statistics</h3>
                    <div className="stats-table">
                      <p>
                        Power Consumption: {" "}
                        <span className="stats-value">
                          {statistics?.power_consumption ?? "Loading..."}
                        </span>
                      </p>
                      <p>
                        Current Load: {" "}
                        <span className="stats-value">
                          {statistics?.current_load ?? "Loading..."}
                        </span>
                      </p>
                      <p>
                        X alignment: {" "}
                        <span className="stats-value">
                          {statistics?.x_alignment ?? "Loading..."}
                        </span>
                      </p>
                      <p>
                        Y alignment: {" "}
                        <span className="stats-value">
                          {statistics?.y_alignment ?? "Loading..."}
                        </span>
                      </p>
                    </div>
                  </div>
                </div>
              </div>
  
              <div className="model-specs">
                <h3>Model Specifications</h3>
                <img src={logo} alt="Logo" className="logo" />
                <h3>Kinova Gen3</h3>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ArmPerf;
