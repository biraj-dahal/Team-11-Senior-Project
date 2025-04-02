import React, {
  createContext,
  useContext,
  useEffect,
  useRef,
  useState,
} from "react";

const WebSocketContext = createContext(null);

export const WebSocketProvider = ({ children }) => {
  const socketRef = useRef(null);
  const [isConnected, setIsConnected] = useState(false);
  const reconnectAttempts = useRef(0);
  const maxReconnectAttempts = 5;

  const connectWebSocket = () => {
    if (socketRef.current) {
      socketRef.current.close();
    }

    console.log("Attempting to connect to WebSocket...");
    const socket = new WebSocket("ws://localhost:8000/robot_ws");

    socket.onopen = () => {
      console.log("Connected to WebSocket");
      setIsConnected(true);
      reconnectAttempts.current = 0; // Reset reconnect attempts

      const authMessage = JSON.stringify({
        type: "identification",
        identity: "frontend",
      });
      socket.send(authMessage);
    };

    socket.onmessage = (event) => {
      if (event.data !== "Identity set") {
        const data = JSON.parse(event.data);
        console.log("Received WebSocket data:", data);
      }
    };

    socket.onclose = () => {
      console.log("WebSocket disconnected");
      //   setIsConnected(false);

      //   if (reconnectAttempts.current < maxReconnectAttempts) {
      //     const retryDelay = Math.min(
      //       1000 * 2 ** reconnectAttempts.current,
      //       30000
      //     ); // Exponential backoff (max 30s)
      //     reconnectAttempts.current += 1;

      //     console.log(`Reconnecting in ${retryDelay / 1000} seconds...`);
      //     setTimeout(connectWebSocket, retryDelay);
      //   } else {
      //     console.log("Max reconnection attempts reached.");
      //   }
    };

    socket.onerror = (error) => {
      console.error("WebSocket Error:", error);
    };

    socketRef.current = socket;
  };

  useEffect(() => {
    connectWebSocket();

    return () => {
      if (socketRef.current) {
        socketRef.current.close();
      }
    };
  }, []);

  return (
    <WebSocketContext.Provider value={socketRef.current}>
      {children}
    </WebSocketContext.Provider>
  );
};

export const useWebSocket = () => {
  return useContext(WebSocketContext);
};
