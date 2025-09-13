let communicationEnabled = false;  

function toggleCommunication() {
  communicationEnabled = !communicationEnabled; // Toggle the state
  const button = document.getElementById("start-communication");
  button.textContent = communicationEnabled ? "Stop Communication" : "Start Communication";
}

async function init() {

  const responseKeymap = await fetch("keymap.json");
  const keyMap = await responseKeymap.json();

  //WebSocket connection is created
  //ip asignada por el router al dispositivo
  const wsUrl = `ws://${'0.0.0.0'}:${8082}`;
  const socket = new WebSocket(wsUrl);

  const messagesDiv = document.getElementById("info-messages");

  function addMessage(text) {
    messagesDiv.textContent += text + "\n";
    messagesDiv.scrollTop = messagesDiv.scrollHeight; // autoscroll
  }

  socket.onopen = () => {
    addMessage("Connecting to server...");
  };

  socket.onmessage = (event) => {
    console.log("Received message:", event.data);
    addMessage(`Server : ${event.data}`);
  };

  socket.onerror = (error) => {
    addMessage(`Error: ${error}`);
  };

  socket.onclose = () => {
    addMessage("Closed connection");
  };

  // Gatter pressed keys and send them 
  document.addEventListener("keydown", (event) => {
    if (!communicationEnabled) return; 
    const key = event.key.toLowerCase(); 
    if (key in keyMap) { 
      const message = keyMap[key];
      socket.send(message);
      addMessage(`Sent (${key}): ${message}`);
    }
  });
}

init();

