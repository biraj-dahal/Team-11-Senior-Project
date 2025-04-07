import React from "react";
import "./RobotDashboard.css";
import Navbar from "../components/Navbar";
import { AlertTriangle, Info, XCircle } from "lucide-react"; // Optional icons

const logs = [
  {
    type: "info",
    message: "System check complete. No anomalies detected.",
    timestamp: "2025-04-07 14:00:01",
  },
  {
    type: "warning",
    message: "Slight delay in actuator response on Joint 2.",
    timestamp: "2025-04-07 14:02:45",
  },
  {
    type: "info",
    message: "Path planning initiated for object pickup.",
    timestamp: "2025-04-07 14:03:30",
  },
  {
    type: "error",
    message: "Collision detected: Obstacle within safety zone.",
    timestamp: "2025-04-07 14:04:12",
  },
  {
    type: "info",
    message: "Recalibration completed successfully.",
    timestamp: "2025-04-07 14:05:59",
  },
  {
    type: "warning",
    message: "Gripper position unstable, attempting to realign.",
    timestamp: "2025-04-07 14:07:23",
  },
  {
    type: "info",
    message: "Joint angles synced with command input.",
    timestamp: "2025-04-07 14:08:50",
  },
  {
    type: "error",
    message: "Motor overheat detected on Joint 5.",
    timestamp: "2025-04-07 14:10:01",
  },
  {
    type: "warning",
    message: "High torque usage on Joint 1.",
    timestamp: "2025-04-07 14:12:34",
  },
  {
    type: "info",
    message: "Emergency stop released. Resuming operation.",
    timestamp: "2025-04-07 14:13:22",
  },
];

const ErrorLogs = () => {
  return (
    <div className="app-container">
      <Navbar />
      <div className="dashboard-container">
        <h1 className="dashboard-title-vert">Errors and Logs</h1>
        <div className="card">
          <div className="logs-container">
            {logs.map((log, index) => (
              <div key={index} className={`log-item log-${log.type}`}>
                <div className="log-icon">
                  {log.type === "error" && <XCircle />}
                  {log.type === "warning" && <AlertTriangle />}
                  {log.type === "info" && <Info />}
                </div>
                <div>
                  <div>{log.message}</div>
                  <div className="log-timestamp">{log.timestamp}</div>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};

export default ErrorLogs;
