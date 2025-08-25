
async function init() {
  // Gather connection info and key mapping 
  const responseConfig = await fetch("com_vars.json");
  const config = await responseConfig.json();
  const responseKeymap = await fetch("keymap.json");
  const keyMap = await responseKeymap.json();

  //WebSocket connection is created
  const wsUrl = `ws://${config.server_ip}:${config.communication_port}`;
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
    const key = event.key.toLowerCase(); 
    if (key in keyMap) { 
      const message = keyMap[key];
      socket.send(message);
      addMessage(`Sent (${key}): ${message}`);
    }
  });
}

init();

