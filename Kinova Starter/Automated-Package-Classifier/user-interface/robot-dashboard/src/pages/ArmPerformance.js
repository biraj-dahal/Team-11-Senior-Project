import React from "react";
import './RobotDashboard.css';
import Navbar from '../components/Navbar';

const ArmPerf = () => {
    return (
        <div className='app-container'>
          <Navbar/>
            <div className="dashboard-container">
                <h1 className="dashboard-title">Arm Performance</h1>
            </div>
        </div>
      );
};

export default ArmPerf;