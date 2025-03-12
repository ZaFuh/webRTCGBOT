import WebSocket, { WebSocketServer } from "ws";


let connections = new Set();

wss.on("connection", (ws) => {
  const clientId = Math.random().toString(36).substr(2, 9);
  ws.clientId = clientId;
  connections.add(ws);
  console.log(
    `Client ${clientId} connected. Total connections: ${connections.size}`,
  );

  ws.on("message", (message) => {
    try {
      const data = JSON.parse(message);
      console.log(`Client ${clientId} sent message type: ${data.type}`);

      // Broadcast to all other clients
      connections.forEach((client) => {
        if (client !== ws && client.readyState === WebSocket.OPEN) {
          console.log(`Broadcasting ${data.type} to client ${client.clientId}`);
          client.send(message.toString());
        }
      });
    } catch (error) {
      console.error(`Error handling message from client ${clientId}:`, error);
    }
  });

  ws.on("close", () => {
    connections.delete(ws);
    console.log(
      `Client ${clientId} disconnected. Remaining connections: ${connections.size}`,
    );
  });

  ws.on("error", (error) => {
    console.error(`WebSocket error for client ${clientId}:`, error);
  });
});

console.log("WebSocket server running on port 8080");
