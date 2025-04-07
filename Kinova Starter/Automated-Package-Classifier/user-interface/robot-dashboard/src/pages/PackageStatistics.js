import React from "react";
import "./RobotDashboard.css";
import Navbar from "../components/Navbar";
import {
  BarChart,
  Bar,
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from "recharts";

const barData = [
  { name: "December 29", value: 30 },
  { name: "December 30", value: 50 },
  { name: "December 31", value: 60 },
  { name: "January 1", value: 40 },
  { name: "January 2", value: 70 },
];

const lineData = [
  {
    index: 10,
    processed: 20,
    errorFree: 15,
    errors: 5,
    hazardous: 8,
    perishable: 7,
  },
  {
    index: 20,
    processed: 40,
    errorFree: 35,
    errors: 10,
    hazardous: 16,
    perishable: 14,
  },
  {
    index: 30,
    processed: 55,
    errorFree: 50,
    errors: 12,
    hazardous: 22,
    perishable: 19,
  },
  {
    index: 40,
    processed: 70,
    errorFree: 60,
    errors: 15,
    hazardous: 30,
    perishable: 25,
  },
  {
    index: 50,
    processed: 85,
    errorFree: 75,
    errors: 18,
    hazardous: 38,
    perishable: 30,
  },
  {
    index: 60,
    processed: 90,
    errorFree: 85,
    errors: 20,
    hazardous: 45,
    perishable: 35,
  },
];

const PackageStats = () => {
  return (
    <div className="app-container">
      <Navbar />
      <div className="dashboard-container">
        <h1 className="dashboard-title">Package Statistics</h1>
        <div className="chart-container">
          <div className="bar-chart">
            <h3>Packages Processed</h3>
            <ResponsiveContainer width="100%" height={300}>
              <BarChart data={barData}>
                <XAxis dataKey="name" />
                <YAxis />
                <Tooltip />
                <Bar dataKey="value" fill="#4A90E2" />
              </BarChart>
            </ResponsiveContainer>
          </div>

          <div className="line-chart">
            <h3>Packages Processed</h3>
            <ResponsiveContainer width="100%" height={300}>
              <LineChart data={lineData}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="index" />
                <YAxis />
                <Tooltip />
                <Legend />
                <Line type="monotone" dataKey="processed" stroke="#4285F4" />
                <Line type="monotone" dataKey="errorFree" stroke="#34A853" />
                <Line type="monotone" dataKey="errors" stroke="#EA4335" />
                <Line type="monotone" dataKey="hazardous" stroke="#FBBC05" />
                <Line type="monotone" dataKey="perishable" stroke="#FF6D00" />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>
      </div>
    </div>
  );
};

export default PackageStats;
