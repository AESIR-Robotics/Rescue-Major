let communicationEnabled = false;  
let inputMode = "keyboard"; 

function toggleCommunication() {
  communicationEnabled = !communicationEnabled; 
  const button = document.getElementById("start-communication");
  button.textContent = communicationEnabled ? "Stop Communication" : "Start Communication";
}

function toggleInputMode() {
  const button = document.getElementById("toggle-input");
  inputMode = (inputMode === "keyboard") ? "controller" : "keyboard";
  button.textContent = (inputMode === "controller") ? "Use Keyboard" : "Use Controller";
}

async function init() {
  // keymaps
  const keyboard_keymap = await (await fetch("/static/keyboard_keymap.json")).json();
  const controller_keymap = await (await fetch("/static/controller_keymap.json")).json();
  
  const messagesDiv = document.getElementById("info-messages");
  let socket = null;
  let isConnected = false;

  function addMessage(text) { 
    messagesDiv.textContent += text + "\n";
    messagesDiv.scrollTop = messagesDiv.scrollHeight;
  }

  // Function to attempt WebSocket connection with retry logic
  function connectToServer() {
    if (isConnected) return;

    const wsUrl = `ws://${location.hostname}:${8082}`;
    addMessage("Looking for server...");
    
    socket = new WebSocket(wsUrl);

    socket.onopen = () => {
      isConnected = true;
      addMessage("Connected to server!");
    };

    socket.onmessage = (event) => {
      addMessage(`Server: ${event.data}`);
    };

    socket.onerror = (error) => {
      addMessage("Connection error - retrying in 5 seconds...");
    };

    socket.onclose = () => {
      isConnected = false;
      if (socket) {
        addMessage("Connection lost - retrying in 5 seconds...");
      }
      
      setTimeout(() => {
        if (!isConnected) {
          connectToServer();
        }
      }, 5000);
    };
  }

  // Start initial connection attempt
  connectToServer();

  // Helper function to send message safely
  function sendMessage(message) {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(message);
      return true;
    } else {
      addMessage("Not connected to server - message not sent");
      return false;
    }
  }

  // Keyboard
  document.addEventListener("keydown", (event) => {
    if (!communicationEnabled || inputMode !== "keyboard") return; 
    const key = event.key.toLowerCase(); 
    if (key in keyboard_keymap) { 
      const message = keyboard_keymap[key];
      sendMessage(message);
    }
  });

  pollController.lastspeed = null;
  let visionmode = 0;
  let current_camera = 0;
  let prevButtons = [];

  // CONTROLLER
  function pollController() {
    if (!communicationEnabled || inputMode !== "controller") {
      requestAnimationFrame(pollController);
      return;
    }

    const controllers = navigator.getGamepads();
    if (controllers[0]) {
      const ctrl = controllers[0];

      // Left joystick, for dc motors 
      const speedX = ctrl.axes[0];
      const speedY = ctrl.axes[1];
      const message = "dc_motors:" + speedX.toString() + "," + speedY.toString();
      sendMessage(message);

      // Botones con edge detection
      ctrl.buttons.forEach((button, index) => {
        const prev = prevButtons[index] || { pressed: false };
        if (button.pressed && !prev.pressed) {
          if (index in controller_keymap) { 
            const msg = controller_keymap[index];
            sendMessage(msg);
          }
          else if (index === 4) { // change vision mode -
            if (visionmode === 0) visionmode = 3;
            else visionmode -= 1;
            const msg = "vision:mode," + current_camera.toString() + "," + visionmode.toString();
            sendMessage(msg);
          }
          else if (index === 5) { // change vision mode +
            if (visionmode === 3) visionmode = 0;
            else visionmode += 1;
            const msg = "vision:mode," + current_camera.toString() + "," + visionmode.toString();
            sendMessage(msg);
          }
          else if (index === 9) { // select camera
            if (current_camera === 3) current_camera = 0;
            else current_camera += 1;
            addMessage(`Camera selected: ${current_camera}`);
          }
        }
        prevButtons[index] = { pressed: button.pressed };
      });
    }

    requestAnimationFrame(pollController);
  }

  window.addEventListener("gamepadconnected", (e) => {
    pollController();
  });

  window.addEventListener("gamepaddisconnected", (e) => {
  });
}

init();
