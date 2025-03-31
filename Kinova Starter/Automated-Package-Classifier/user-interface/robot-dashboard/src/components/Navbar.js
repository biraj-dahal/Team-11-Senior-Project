import React from 'react';
import { Link, useLocation } from 'react-router-dom'; // Import Link from react-router-dom
import './navbar.css';
import logo from '../assets/arm_logo.jpg'

const Navbar = () => {
  const location = useLocation(); // Get current route path

  return (
    <div className="navbar flex flex-col items-center p-4">
      <img src={logo} alt="Logo" className="logo" />
      <h3 className="title">APC RAMESH</h3>

      <nav className="flex flex-col items-center">
        <Link className={`menu-item mb-2 ${location.pathname === "/" ? "active" : ""}`} to="/">Dashboard</Link>
        <Link className={`menu-item mb-2 ${location.pathname === "/packagestats" ? "active" : ""}`} to="/packagestats">Package Stats</Link>
        <Link className={`menu-item mb-2 ${location.pathname === "/armperformance" ? "active" : ""}`} to="/armperformance">Arm Performance</Link>
        <Link className={`menu-item mb-2 ${location.pathname === "/livecamera" ? "active" : ""}`} to="/livecamera">Live Camera</Link>
        <Link className={`menu-item mb-2 ${location.pathname === "/remotecontrol" ? "active" : ""}`} to="/remotecontrol">Remote Control</Link>
        <Link className={`menu-item ${location.pathname === "/errorlogs" ? "active" : ""}`} to="/errorlogs">Errors and Logs</Link>
      </nav>
    </div>
  );
};

export default Navbar;

