import React from "react";
import './RobotDashboard.css';
import Navbar from '../components/Navbar';

const ErrorLogs = () => {
    return (
        <div className='app-container'>
          <Navbar/>
            <div className="dashboard-container">
                <h1 className="dashboard-title">Errors and Logs</h1>
            </div>
        </div>
      );
};

export default ErrorLogs;