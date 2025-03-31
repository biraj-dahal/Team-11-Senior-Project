import { Routes, Route } from "react-router-dom";
import ArmPerf from "./pages/ArmPerformance"
import ErrorLogs from "./pages/ErrorLogs";
import LiveCamera from "./pages/LiveCamera";
import PackageStats from "./pages/PackageStatistics";
import RemoteControl from "./pages/RemoteControl";
import Dash from "./pages/RobotDashboard";

function App() {
  return (
   <Routes>
      <Route path="/" element={<Dash/>}/>
      <Route path="/armperformance" element={<ArmPerf/>}/>
      <Route path="/errorlogs" element={<ErrorLogs/>}/>
      <Route path="/livecamera" element={<LiveCamera/>}/>
      <Route path="/packagestats" element={<PackageStats/>}/>
      <Route path="/remotecontrol" element={<RemoteControl/>}/>
   </Routes>
  );
}

export default App;
