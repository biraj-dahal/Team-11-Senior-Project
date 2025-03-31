import React from "react";
import './RobotDashboard.css';
import Navbar from '../components/Navbar';

const PackageStats = () => {
    return (
        <div className='app-container'>
          <Navbar/>
            <div className="dashboard-container">
                <h1 className="dashboard-title">Package Statistics</h1>
            </div>
        </div>
      );
};

export default PackageStats;