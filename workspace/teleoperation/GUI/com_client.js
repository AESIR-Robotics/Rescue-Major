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
  
  // WebSocket, location hostname is the ip of the server
  const wsUrl = `ws://${location.hostname}:${8082}`;
  const socket = new WebSocket(wsUrl);

  const messagesDiv = document.getElementById("info-messages");

  function addMessage(text) { 
    messagesDiv.textContent += text + "\n";
    messagesDiv.scrollTop = messagesDiv.scrollHeight;
  }

  socket.onopen = () => addMessage("Connecting to server...");
  socket.onmessage = (event) => addMessage(`Server : ${event.data}`);
  socket.onerror = (error) => addMessage(`Error: ${error}`);
  socket.onclose = () => addMessage("Closed connection");

  // Keyboard
  document.addEventListener("keydown", (event) => {
    if (!communicationEnabled || inputMode !== "keyboard") return; 
    const key = event.key.toLowerCase(); 
    if (key in keyboard_keymap) { 
      const message = keyboard_keymap[key];
      socket.send(message);
      console.log("Sent message with keyboard: "  + message);
    }
  });
  

  pollController.lastspeed = null;
  let visionmode = 0;
  let current_camera = 0;
  let prevButtons = []; // buttons state for edge detection

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
      const speedX = ctrl.axes[0]; // Left stick X
      const speedY = ctrl.axes[1]; // Left stick Y
      const message = "dc_motors:" +  speedX.toString() +"," + speedY.toString();
      socket.send(message);
      console.log(`Sent from controller: ${message}`);


      // Botones con edge detection
      ctrl.buttons.forEach((button, index) => {
        const prev = prevButtons[index] || { pressed: false };
        //edge detection
        if (button.pressed && !prev.pressed) {
          if (index in controller_keymap) { 
            const msg = controller_keymap[index];
            socket.send(msg);
            console.log(`Sent from controller ${msg}`);
          }
          else if (index === 4) { // change vision mode -
            if (visionmode === 0) visionmode = 3;
            else visionmode -= 1;
            socket.send("vision:mode," + current_camera.toString() + "," + visionmode.toString());
            console.log(`Sent (change vision mode): for ${current_camera}, ${visionmode}`);
          }
          else if (index === 5) { // change vision mode +
            if (visionmode === 3) visionmode = 0;
            else visionmode += 1;
            socket.send("vision:mode," + current_camera.toString() + "," + visionmode.toString());
            console.log(`Sent (change vision mode): for ${current_camera}, ${visionmode}`);
          }
          else if (index === 9) { // select camera
            if (current_camera === 3) current_camera = 0;
            else current_camera += 1;
            addMessage(`Camera selected: ${current_camera}`);
          }
        }
        // Get button state 
        prevButtons[index] = { pressed: button.pressed };
      });
    }

    requestAnimationFrame(pollController);
  }

  window.addEventListener("gamepadconnected", (e) => {
    console.log("Controller connected: " + e.gamepad.id);
    pollController();
  });

  window.addEventListener("gamepaddisconnected", (e) => {
    console.log("Controller disconnected: " + e.gamepad.id);
  });
}

init();
