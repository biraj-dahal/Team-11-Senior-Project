import React, { useState, useEffect } from "react";
import "./RobotDashboard.css";
import Navbar from "../components/Navbar";
import logo from "../assets/kinova_arm.jpeg";
import { useWebSocket } from "../components/WebSocketProvider";

const ArmPerf = () => {
  const socket = useWebSocket(); // Get WebSocket instance from context
  const [status, setStatus] = useState("Offline");
  const [gripperStatus, setGripperStatus] = useState("Unknown");
  const [statistics, setStatistics] = useState(null);

  useEffect(() => {
    if (!socket) {
      console.log("Error socket is nil!");
      return;
    }

    // const sendIdentification = () => {
    //   if (socket.readyState === WebSocket.OPEN) {
    //     socket.send(JSON.stringify({ type: "arm_performance" }));
    //     console.log("Request sent for performance data. Awaiting response.");
    //   }
    // };

    const handleMessage = (event) => {
      console.log("WebSocket Message:", event.data);
      if (event.data !== "Identity set") {
        try {
          const message = JSON.parse(JSON.parse(event.data));
          setStatus(message.operational_status);
          setGripperStatus(message.gripper_status);
          setStatistics(message.live_statistics);
        } catch (error) {
          console.error("Error parsing WebSocket message:", error);
        }
      }
    };

    // socket.addEventListener("open", sendIdentification);
    // socket.addEventListener("message", handleMessage);

    return () => {
      // socket.removeEventListener("open", sendIdentification);
      // socket.removeEventListener("message", handleMessage);
    };
  }, [socket]);

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
                    Operational Status:{" "}
                    <span className="status online">
                      {status ?? "Loading..."}
                    </span>
                  </h2>
                  <h2>
                    Gripper Status:{" "}
                    <span className="status closed">
                      {gripperStatus ?? "Loading..."}
                    </span>
                  </h2>
                  <div className="statistics">
                    <h3>Live Statistics</h3>
                    <div className="stats-table">
                      <p>
                        Power Consumption:{" "}
                        <span className="stats-value">
                          {statistics?.power_consumption ?? "Loading..."}
                        </span>
                      </p>
                      <p>
                        Current Load:{" "}
                        <span className="stats-value">
                          {statistics?.current_load ?? "Loading..."}
                        </span>
                      </p>
                      <p>
                        X alignment:{" "}
                        <span className="stats-value">
                          {statistics?.x_alignment ?? "Loading..."}
                        </span>
                      </p>
                      <p>
                        Y alignment:{" "}
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
