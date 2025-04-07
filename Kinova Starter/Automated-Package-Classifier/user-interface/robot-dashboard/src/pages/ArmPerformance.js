import React, { useState, useEffect } from "react";
import "./RobotDashboard.css";
import Navbar from "../components/Navbar";
import logo from "../assets/kinova_arm.jpeg";

const ArmPerf = () => {
  const [status, setStatus] = useState("Offline");
  const [gripperStatus, setGripperStatus] = useState("Unknown");
  const [statistics, setStatistics] = useState(null);

    return (
    <div className="app-container">
      <Navbar />
      <div className="dashboard-container">
        <h1 className="dashboard-title-vert">Arm Performance</h1>
        <div className="dashboard-grid">
          <div className="card status-card">
            <div className="container">
              <div className="status-section">
                <div>
                  <h2>
                    Operational Status:{""}
                    <span className="status online">{status ?? "Online"}</span>
                  </h2>
                  <h2>
                    Gripper Status:{" "}
                    <span className="status closed">
                      {gripperStatus ?? "Open"}
                    </span>
                  </h2>
                  <div className="statistics">
                    <h3>Live Statistics</h3>
                    <div className="stats-table">
                      <p>
                        Power Consumption:{" "}
                        <span className="stats-value">
                          {statistics?.power_consumption ?? "63W"}
                        </span>
                      </p>
                      <p>
                        Current Load:{" "}
                        <span className="stats-value">
                          {statistics?.current_load ?? "570g"}
                        </span>
                      </p>
                      <p>
                        X alignment:{" "}
                        <span className="stats-value">
                          {statistics?.x_alignment ?? "350"}
                        </span>
                      </p>
                      <p>
                        Y alignment:{" "}
                        <span className="stats-value">
                          {statistics?.y_alignment ?? "275"}
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
