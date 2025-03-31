import React from "react";
import './RobotDashboard.css';
import Navbar from '../components/Navbar';

const RemoteControl = () => {
    return (
        <div className='app-container'>
          <Navbar/>
            <div className="dashboard-container">
                <h1 className="dashboard-title">Remote Control</h1>
            </div>
        </div>
      );
};

export default RemoteControl;