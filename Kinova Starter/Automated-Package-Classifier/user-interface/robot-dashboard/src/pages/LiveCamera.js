import "./RobotDashboard.css";
import Navbar from "../components/Navbar";
import sampleVideo from "../assets/live_view.mp4";
import React from "react";

const LiveCamera = () => {
  const mp4Url = "http://localhost:8096/sample.mp4"; // Replace with your MP4 file path

  return (
    <div className="app-container">
      <Navbar />
      <div className="dashboard-container">
        <h1 className="dashboard-title">Live Camera</h1>

        <div className="dashboard-grid">
          <video src={sampleVideo} autoPlay muted width="100%" height="100%" />
        </div>
      </div>
    </div>
  );
};

export default LiveCamera;
