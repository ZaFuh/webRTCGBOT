import { useEffect, useRef, useState } from "react";

interface SignalingMessage {
  type: "offer" | "answer" | "candidate";
  sdp?: string;
  candidate?: string;
  mid?: string;
}

const WebRTCConnection = () => {
  const [connectionStatus, setConnectionStatus] = useState("Disconnected");
  const [isConnected, setIsConnected] = useState(false);

  //interface MessagePayload {
  //  message: string;
  //}
  interface ControlMessage {
    x: number;
    y: number;
  }

  type MessagePayload = string | ControlMessage;

  interface WebRTCMessage {
    type: "text" | "control";
    payload: MessagePayload;
    timestamp: number;
  }

  const [message, setMessage] = useState("");
  const [controls, setControls] = useState<ControlMessage>({ x: 0, y: 0 });
  const pressedKeys = useRef<Set<string>>(new Set());
  const [receivedMessages, setReceivedMessages] = useState<WebRTCMessage[]>([]);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const peerConnection = useRef<RTCPeerConnection | null>(null);
  const ws = useRef<WebSocket | null>(null);
  const dataChannel = useRef<RTCDataChannel | null>(null);
  const manuallyDisconnected = useRef<boolean>(false);

  const connectWebSocket = () => {
    if (ws.current?.readyState === WebSocket.OPEN) return;

    manuallyDisconnected.current = false;
    ws.current = new WebSocket("ws://localhost:8080");

    ws.current.onopen = () => {
      console.log("WebSocket Connected");
      setConnectionStatus("WebSocket Connected");
      setIsConnected(true);
    };

    ws.current.onclose = () => {
      console.log("WebSocket Disconnected");
      setConnectionStatus("WebSocket Disconnected");
      // Only attempt to reconnect if not manually disconnected
      if (!manuallyDisconnected.current) {
        setTimeout(connectWebSocket, 2000);
      }
    };

    ws.current.onerror = (error) => {
      console.error("WebSocket Error:", error);
      setConnectionStatus("WebSocket Error");
    };

    ws.current.onmessage = async (event) => {
      try {
        const msg = JSON.parse(event.data);
        handleSignalingMessage(msg);
      } catch (error) {
        console.error("Error handling message:", error);
      }
    };
  };

  const initializePeerConnection = () => {
    peerConnection.current = new RTCPeerConnection({
      iceServers: [], // Empty array since we're operating on LAN only - no STUN/TURN needed
    });

    // Create data channel for text communication
    dataChannel.current = peerConnection.current.createDataChannel("text");
    setupDataChannel(dataChannel.current);

    // Handle incoming data channels
    peerConnection.current.ondatachannel = (event) => {
      const channel = event.channel;
      if (channel.label === "video") {
        channel.onmessage = (e) => {
          // Convert received data to image
          const blob = new Blob([e.data], { type: "image/jpeg" });
          const img = new Image();
          img.onload = () => {
            const canvas = canvasRef.current;
            if (!canvas) return;
            const ctx = canvas.getContext("2d");
            if (!ctx) return;

            // Calculate aspect ratio preserving dimensions
            const ratio = Math.min(
              canvas.width / img.width,
              canvas.height / img.height,
            );
            const centerX = (canvas.width - img.width * ratio) / 2;
            const centerY = (canvas.height - img.height * ratio) / 2;

            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.drawImage(
              img,
              centerX,
              centerY,
              img.width * ratio,
              img.height * ratio,
            );
          };
          img.src = URL.createObjectURL(blob);
        };
      } else if (channel.label === "text") {
        setupDataChannel(channel);
      }
    };

    peerConnection.current.onconnectionstatechange = () => {
      const state = peerConnection.current?.connectionState;
      setConnectionStatus(`Connection: ${state}`);
    };

    peerConnection.current.onicecandidate = (event) => {
      if (event.candidate) {
        ws.current?.send(
          JSON.stringify({
            type: "candidate",
            candidate: event.candidate.candidate,
            mid: event.candidate.sdpMid,
          }),
        );
      }
    };
  };

  const handleSignalingMessage = async (msg: SignalingMessage) => {
    if (!peerConnection.current) {
      initializePeerConnection();
    }

    try {
      switch (msg.type) {
        case "offer": {
          await peerConnection.current?.setRemoteDescription({
            type: "offer",
            sdp: msg.sdp,
          });
          const answer = await peerConnection.current?.createAnswer();
          await peerConnection.current?.setLocalDescription(answer);
          ws.current?.send(
            JSON.stringify({
              type: "answer",
              sdp: answer?.sdp,
            }),
          );
          break;
        }

        case "candidate":
          if (peerConnection.current?.remoteDescription) {
            await peerConnection.current.addIceCandidate({
              candidate: msg.candidate,
              sdpMid: msg.mid,
              sdpMLineIndex: 0,
            });
          }
          break;
      }
    } catch (error) {
      console.error("Error handling signaling message:", error);
    }
  };

  const resetConnection = () => {
    if (peerConnection.current) {
      peerConnection.current.close();
      peerConnection.current = null;
    }

    // Send reset message to server
    if (ws.current?.readyState === WebSocket.OPEN) {
      ws.current.send(JSON.stringify({ type: "reset" }));
    }

    initializePeerConnection();
  };

  const handleDisconnect = () => {
    // Update status and flag immediately
    setConnectionStatus("Disconnected");
    setIsConnected(false);
    manuallyDisconnected.current = true;

    // Then clean up connections
    if (peerConnection.current) {
      // Close the peer connection (this will also close associated data channels)
      peerConnection.current.close();
      peerConnection.current = null;
    }

    if (ws.current) {
      if (ws.current.readyState === WebSocket.OPEN) {
        ws.current.close();
      }
      ws.current = null;
    }

    // Clear the canvas
    const canvas = canvasRef.current;
    if (canvas) {
      const ctx = canvas.getContext("2d");
      if (ctx) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
      }
    }

    // Clear data channel
    if (dataChannel.current) {
      dataChannel.current.close();
      dataChannel.current = null;
    }

    // Clear messages
    setReceivedMessages([]);
  };

  const setupDataChannel = (channel: RTCDataChannel) => {
    channel.onopen = () => {
      console.log(`Data channel '${channel.label}' opened`);
    };

    channel.onclose = () => {
      console.log(`Data channel '${channel.label}' closed`);
    };

    channel.onmessage = (event) => {
      try {
        const message: WebRTCMessage = JSON.parse(event.data);
        setReceivedMessages((prev) => [...prev, message]);
      } catch (error) {
        console.error("Error parsing received message:", error);
      }
    };
  };

  const sendMessage = () => {
    if (dataChannel.current?.readyState === "open" && message) {
      const messageObj: WebRTCMessage = {
        type: "text",
        payload: message,
        timestamp: Date.now(),
      };

      try {
        const messageString = JSON.stringify(messageObj);
        console.log("Attempting to send message:", messageString);
        dataChannel.current.send(messageString);
        console.log("Message sent successfully");
        setReceivedMessages((prev) => [...prev, messageObj]);
        setMessage("");
      } catch (error) {
        console.error("Error sending message:", error);
      }
    } else {
      console.log("Data channel not open or message is empty");
    }
  };

  useEffect(() => {
    if (canvasRef.current) {
      canvasRef.current.width = 640;
      canvasRef.current.height = 480;
    }

    const checkConnectionStatus = () => {
      if (!manuallyDisconnected.current) {
        if (
          peerConnection.current?.connectionState === "disconnected" ||
          peerConnection.current?.connectionState === "failed"
        ) {
          console.log("Peer connection lost, reinitializing...");
          resetConnection();
        } else if (!ws.current || ws.current.readyState === WebSocket.CLOSED) {
          console.log("WebSocket disconnected, reconnecting...");
          connectWebSocket();
        }
      }
    };

    const connectionMonitor = setInterval(checkConnectionStatus, 2000);

    return () => {
      clearInterval(connectionMonitor);
      handleDisconnect();
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, []);
  const updateMovementVector = () => {
    let x = 0;
    let y = 0;

    // Only calculate movement if there are pressed keys
    if (pressedKeys.current.size > 0) {
      if (pressedKeys.current.has("a")) x -= 1;
      if (pressedKeys.current.has("d")) x += 1;
      if (pressedKeys.current.has("w")) y += 1;
      if (pressedKeys.current.has("s")) y -= 1;

      // Normalize the vector if moving diagonally
      if (x !== 0 && y !== 0) {
        const length = Math.sqrt(x * x + y * y);
        x /= length;
        y /= length;
      }
    }

    // Always send an update when the vector changes or when all keys are released
    if (
      x !== controls.x ||
      y !== controls.y ||
      pressedKeys.current.size === 0
    ) {
      setControls({ x, y });

      if (dataChannel.current?.readyState === "open") {
        const controlMsg: WebRTCMessage = {
          type: "control",
          payload: { x, y },
          timestamp: Date.now(),
        };
        dataChannel.current.send(JSON.stringify(controlMsg));
      }
    }
  };

  const handleKeyDown = (e: KeyboardEvent) => {
    if (["w", "a", "s", "d"].includes(e.key.toLowerCase())) {
      e.preventDefault();
      pressedKeys.current.add(e.key.toLowerCase());
      updateMovementVector();
    }
  };

  const handleKeyUp = (e: KeyboardEvent) => {
    if (["w", "a", "s", "d"].includes(e.key.toLowerCase())) {
      e.preventDefault();
      pressedKeys.current.delete(e.key.toLowerCase());
      updateMovementVector();
    }
  };

  useEffect(() => {
    // Set initial canvas size
    if (canvasRef.current) {
      canvasRef.current.width = 640;
      canvasRef.current.height = 480;
    }

    // Add keyboard event listeners
    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    // Set up WebSocket status monitoring
    const checkConnectionStatus = () => {
      if (
        !manuallyDisconnected.current &&
        (peerConnection.current?.connectionState === "disconnected" ||
          peerConnection.current?.connectionState === "failed")
      ) {
        console.log("Peer connection lost, reinitializing...");
        handleDisconnect();
        connectWebSocket();
      }
    };

    const connectionMonitor = setInterval(checkConnectionStatus, 2000);

    // Only setup cleanup on unmount
    return () => {
      clearInterval(connectionMonitor);
      handleDisconnect();
    };
  }, []);

  // Add connection state monitoring
  useEffect(() => {
    if (peerConnection.current) {
      const pc = peerConnection.current;
      pc.oniceconnectionstatechange = () => {
        console.log("ICE Connection State:", pc.iceConnectionState);
        setConnectionStatus(`ICE: ${pc.iceConnectionState}`);
      };

      pc.onconnectionstatechange = () => {
        console.log("Connection State:", pc.connectionState);
        setConnectionStatus(`Connection: ${pc.connectionState}`);

        if (pc.connectionState === "failed") {
          console.log("Connection failed");
          handleDisconnect();
        }
      };
    }
  }, [peerConnection.current]);

  return (
    <div className="p-4">
      <div className="max-w-2xl mx-auto">
        <h1 className="text-2xl mb-4">ROS2 Camera Stream</h1>
        <div className="mb-4 flex items-center gap-4">
          <div className="p-2 rounded bg-gray-100 flex-grow">
            Status: {connectionStatus}
          </div>
          <button
            onClick={isConnected ? handleDisconnect : connectWebSocket}
            className="px-4 py-2 rounded bg-blue-500 text-white hover:bg-blue-600 transition-colors"
          >
            {isConnected ? "Disconnect" : "Connect"}
          </button>
        </div>
        <canvas
          ref={canvasRef}
          className="w-full border-2 border-gray-300 bg-black"
        />
        <div className="mt-4">
          <div className="mb-4 p-4 bg-gray-100 rounded">
            <h3 className="font-semibold mb-2">Movement Controls (WASD)</h3>
            <div>
              Current Vector: X: {controls.x.toFixed(2)}, Y:{" "}
              {controls.y.toFixed(2)}
            </div>
          </div>
          <div className="mb-2 flex gap-2">
            <input
              type="text"
              value={message}
              onChange={(e) => setMessage(e.target.value)}
              onKeyPress={(e) => e.key === "Enter" && sendMessage()}
              placeholder="Type a message..."
              className="flex-grow p-2 border rounded"
              disabled={!isConnected}
            />
            <button
              onClick={sendMessage}
              disabled={!isConnected || !message}
              className="px-4 py-2 rounded bg-blue-500 text-white hover:bg-blue-600 transition-colors disabled:bg-gray-400"
            >
              Send
            </button>
          </div>
          <div className="h-40 overflow-y-auto border rounded p-2">
            {receivedMessages.map((msg, index) => (
              <div
                key={index}
                className="text-sm py-1 border-b last:border-b-0"
              >
                <span className="font-semibold">
                  {msg.type === "text" && msg.payload === message
                    ? "Sent"
                    : "Received"}
                  :
                </span>{" "}
                <span className="text-gray-600 text-xs">
                  {new Date(msg.timestamp).toLocaleTimeString()}
                </span>
                <br />
                <span className="font-mono text-sm">
                  {JSON.stringify(msg.payload)}
                </span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};

export default WebRTCConnection;
