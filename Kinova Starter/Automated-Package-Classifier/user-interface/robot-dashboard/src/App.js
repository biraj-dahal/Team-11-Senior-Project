import { Routes, Route } from "react-router-dom";
import Home from "./pages/Home"
import Dash from "./pages/RobotDashboard";

function App() {
  return (
   <Routes>
      <Route path="/" element={<Dash/>}/>
      <Route path="/dash" element={<Home/>}/>
   </Routes>
  );
}

export default App;
