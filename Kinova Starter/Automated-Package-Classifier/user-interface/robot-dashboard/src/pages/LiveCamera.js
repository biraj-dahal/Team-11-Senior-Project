
import './RobotDashboard.css';
import Navbar from '../components/Navbar';

import React, { useEffect, useRef } from "react";
import Hls from "hls.js";  // Import hls.js

const LiveCamera = () => {
  const videoRef = useRef(null); // Reference to the video element

  useEffect(() => {
    const video = videoRef.current;

    // Check if the browser supports HLS.js
    if (Hls.isSupported()) {
      const hls = new Hls();

      // Bind the HLS stream to the video element
      hls.loadSource("http://localhost:8080/stream.m3u8"); // The HLS stream URL
      hls.attachMedia(video);

      // Clean up when the component unmounts
      return () => {
        hls.destroy();
      };
    } else {
      console.error("HLS.js is not supported in your browser.");
    }
  }, []);


return (
    <div className='app-container'>
      <Navbar/>
    <div className="dashboard-container">
    
      <h1 className="dashboard-title">Live Camera</h1>
      
      <div className="dashboard-grid">
        <video ref={videoRef} controls autoPlay muted width= "100%" height="100%" />
      </div>

      
    </div>
    </div>
  );
};

export default LiveCamera;