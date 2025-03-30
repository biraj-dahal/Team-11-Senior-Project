import React, { useState, useEffect } from 'react';
import { LineChart, XAxis, YAxis, Tooltip, Legend, Line, ResponsiveContainer } from 'recharts';
import { AlertCircle, Power, Move3D, Rotate3D} from 'lucide-react';
import Alert, { AlertDescription, AlertTitle } from './Alert';
import './RobotDashboard.css';

const RobotDashboard = () => {
  const [status, setStatus] = useState(null);
  const [logs, setLogs] = useState([]);
  const [stats, setStats] = useState(null);
  const [statusHistory, setStatusHistory] = useState([]);
  const [error, setError] = useState(null);
  
  useEffect(() => {
    const ws = new WebSocket('ws://127.0.0.1:8000/ws');

    ws.onopen = () => console.log("Connected to WebSocket!");
    ws.onmessage = (event) => console.log("Received:", event.data);
    ws.onerror = (error) => console.log("WebSocket Error:", error);
    ws.onclose = () => console.log("WebSocket closed.");

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      setStatus(data);
      setStatusHistory(prev => [...prev, { ...data, timestamp: new Date().toLocaleTimeString() }].slice(-20));
    };
    ws.onerror = () => setError('WebSocket connection failed');
    return () => ws.close();
  }, []);
  
  useEffect(() => {
    const fetchData = async () => {
      try {
        const [logsRes, statsRes] = await Promise.all([
          fetch('http://localhost:8000/api/logs?limit=10'), 
          fetch('http://localhost:8000/api/stats')
        ]);
        setLogs(await logsRes.json());
        setStats(await statsRes.json());
      } catch (err) {
        setError('Failed to fetch data');
      }
    };
    fetchData();
    const interval = setInterval(fetchData, 5000);
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="dashboard-container">
      <h1 className="dashboard-title">ROBITS Dashboard</h1>
      
      <div className="dashboard-grid">
        <div className="card status-card">
          <h2 className="card-title">Current Status</h2>
          {status ? (
            <div className="status-content">
              <p className="status-item">
                <Power className="status-icon activity-icon" />
                State: <span className="status-value">{status.state}</span>
              </p>
              <p className="status-item">
                <Move3D className="status-icon position-icon" />
                Position: <span className="status-value">{status.position}</span>
              </p>
              <p className="status-item">
                <Rotate3D className="status-icon position-icon" />
                Attitude: <span className="status-value">{status.rollpitchyaw}</span>
              </p>
            </div>
          ) : <p className="loading-text">Loading...</p>}
        </div>

        <div className="card stats-card">
          <h2 className="card-title">System Stats</h2>
          {stats ? (
            <div className="stats-content">
              <p>CPU Usage: <span className="stats-value">{stats.position}%</span></p>
              <p>Memory Usage: <span className="stats-value">{stats.position}%</span></p>
              <p>Active Connections: <span className="stats-value">{stats.position}</span></p>
            </div>
          ) : <p className="loading-text">Loading...</p>}
        </div>
      </div>

      <div className="card chart-card">
        <h2 className="card-title">Status History</h2>
        <div className="chart-container">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={statusHistory}>
              <XAxis dataKey="timestamp" tick={{ fontSize: 12 }} />
              <YAxis tick={{ fontSize: 12 }} />
              <Tooltip contentStyle={{ backgroundColor: '#fff', borderRadius: '8px' }} />
              <Legend />
              <Line 
                type="monotone" 
                dataKey="cpu_usage" 
                stroke="#3b82f6" 
                strokeWidth={2}
                dot={false}
                activeDot={{ r: 6 }}
                isAnimationActive={false}
              />
              <Line 
                type="monotone" 
                dataKey="memory_usage" 
                stroke="#10b981" 
                strokeWidth={2}
                dot={false}
                activeDot={{ r: 6 }}
                isAnimationActive={false}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>

      <div className="card logs-card">
        <h2 className="card-title">Recent Logs</h2>
        <div className="logs-container">
          {logs.map((log, index) => (
            <Alert 
              key={index} 
              className={`log-item log-${log.level.toLowerCase()}`}
            >
              <AlertCircle className="log-icon" />
              <div>
                <AlertTitle>{log.level}</AlertTitle>
                <AlertDescription>
                  {log.message}
                  <span className="log-timestamp">{new Date(log.timestamp).toLocaleTimeString()}</span>
                </AlertDescription>
              </div>
            </Alert>
          ))}
        </div>
      </div>

      <button className="stop-button">
        <Power className="button-icon" />
        STOP
      </button>

      {error && (
        <Alert className="error-alert">
          <AlertCircle className="error-icon" />
          <AlertTitle>Error</AlertTitle>
          <AlertDescription>{error}</AlertDescription>
        </Alert>
      )}
    </div>
  );
};

export default RobotDashboard;