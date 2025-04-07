import "./RobotDashboard.css";
import Navbar from "../components/Navbar";

import React, { useEffect, useRef } from "react";
import Hls from "hls.js"; // Import hls.js

const LiveCamera = () => {
  const videoRef = useRef(null); // Reference to the video element

  useEffect(() => {
    const video = videoRef.current;
    const hlsUrl = "http://localhost:8096/stream.m3u8"; // Ensure this is correct

    if (Hls.isSupported()) {
      const hls = new Hls();
      hls.loadSource(hlsUrl);
      hls.attachMedia(video);

      return () => {
        hls.destroy(); // Cleanup
      };
    } else if (video.canPlayType("application/vnd.apple.mpegurl")) {
      // Directly set the source for Safari and other HLS-supported browsers
      video.src = hlsUrl;
    } else {
      console.error("HLS is not supported in this browser.");
    }
  }, []);

  return (
    <div className="app-container">
      <Navbar />
      <div className="dashboard-container">
        <h1 className="dashboard-title">Live Camera</h1>

        <div className="dashboard-grid">
          <video
            ref={videoRef}
            controls
            autoPlay
            muted
            width="100%"
            height="100%"
          />
        </div>
      </div>
    </div>
  );
};

export default LiveCamera;
