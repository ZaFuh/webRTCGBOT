import "./App.css";
import WebRTCConnection from "./WebRTC";

function App() {
  return (
    <div className="wrapper">
      <div className="visualisation"></div>
      <div className="content">
        <WebRTCConnection />
      </div>
    </div>
  );
}

export default App;
