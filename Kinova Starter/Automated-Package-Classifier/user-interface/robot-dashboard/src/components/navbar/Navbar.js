import React from 'react';
import { Link } from 'react-router-dom'; // Import Link from react-router-dom
import './navbar.css';

const Navbar = () => {
  return (
    <div className="navbar">
      <Link className="menu-item" to="/">Dashboard</Link>
      <Link className="menu-item" to="/salads">Package Stats</Link>
      <Link className="menu-item" to="/pizzas">Arm Performance</Link>
      <Link className="menu-item" to="/desserts">Live Camera</Link>
      <Link className="menu-item" to="/remote-control">Remote Control</Link>
      <Link className="menu-item" to="/errors-logs">Errors and Logs</Link>
    </div>
  );
};

export default Navbar;
