import React, { useState, useEffect } from 'react';
import RobotDashboardView from './RobotDashboardView';

const RobotDashboard = () => {
  const [status, setStatus] = useState(null);
  const [logs, setLogs] = useState([]);
  const [stats, setStats] = useState(null);
  const [statusHistory, setStatusHistory] = useState([]);
  const [error, setError] = useState(null);
  
  useEffect(() => {
    const ws = new WebSocket('http://127.0.0.1:8000');
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

  const stopRobot = async () => {
    try {
      await fetch('http://localhost:8000/api/control', {
        method: 'POST',
        body: JSON.stringify({ command: 'STOP' }),
        headers: { 'Content-Type': 'application/json' },
      });
    } catch (err) {
      setError('Failed to send STOP command');
    }
  };

  return (
    <RobotDashboardView 
      status={status} 
      stats={stats} 
      logs={logs} 
      statusHistory={statusHistory} 
      error={error} 
      stopRobot={stopRobot} 
    />
  );
};

export default RobotDashboard;