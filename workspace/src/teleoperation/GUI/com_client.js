
//______________________ Input Handler ____________________
class InputHandler {
  constructor({ deadzone = 0.12, robotAPI = null } = {}) {
    this.deadzone = deadzone;
    this.keys = new Set();
    this.gamepadIndex = null;
    this._rafId = null;
    this.prevButtonsByPad = {};
    this.listeners = Object.create(null);
    
    this.robotAPI = robotAPI;
    this.actionMap = {};
    this.localFunctions = {};
    
    this._setupKeyboard();
    
    // Descomentar para reactivar control)
    // this._setupGamepadEvents();
    // this._startGamepadPoll();
  }

  // Set RobotAPI reference
  setRobotAPI(robotAPI) {
    this.robotAPI = robotAPI;
  }

  // Load action configuration from JSON
  loadActions(configJson) {
    if (!configJson || typeof configJson !== 'object') {
      console.error('[InputHandler] Invalid config: must be an object');
      return false;
    }

    try {
      Object.entries(configJson).forEach(([keyId, action]) => {
        const { type, data_type, topics, payload, actions } = action;

        if (!type) {
          console.warn(`[InputHandler] Skipping action for ${keyId}: missing type`);
          return;
        }

        // Validate action type
        if (!['send_topic', 'send_service', 'local_function'].includes(type)) {
          console.warn(`[InputHandler] Unknown action type "${type}" for key ${keyId}`);
          return;
        }

        // For send actions, ensure topics array and data_type exist
        if (type === 'send_topic' || type === 'send_service') {
          if (!topics || !Array.isArray(topics) || topics.length === 0) {
            console.warn(`[InputHandler] Action ${keyId} missing topics array`);
            return;
          }
          if (!data_type) {
            console.warn(`[InputHandler] Action ${keyId} missing data_type`);
            return;
          }
        }

        this.actionMap[keyId] = { type, data_type, topics, payload };
        console.log(`[InputHandler] Mapped ${keyId} -> ${type} (${topics ? topics.length : 0} targets)`);
      });

      console.log(`[InputHandler] Loaded ${Object.keys(this.actionMap).length} actions`);
      return true;
    } catch (error) {
      console.error('[InputHandler] Failed to load actions:', error.message);
      return false;
    }
  }

  // Register a local function that can be called by actions
  registerLocalFunction(name, callback) {
    this.localFunctions[name] = callback;
    console.log(`[InputHandler] Registered local function: ${name}`);
  }

  // Execute an action for a given key
  executeAction(keyId, keyState) {
    const action = this.actionMap[keyId];
    if (!action) {
      return { success: false, error: `Action "${keyId}" not mapped` };
    }

    if (keyState === 'released' && !action.release_payload) {
      return { success: true, type: 'ignored_release' };
    }

    const basePayload = (keyState === 'released' && action.release_payload) ? action.release_payload : action.payload;
    const payloadWithState = basePayload ? { ...basePayload, state: keyState } : { state: keyState };

    try {
      switch (action.type) {
        case 'send_topic':
          if (!this.robotAPI) {
            return { success: false, error: 'RobotAPI not set' };
          }
          return {
            success: true,
            type: 'send_topic',
            count: action.topics.length,
            results: this.robotAPI.sendTopic(action.topics, action.data_type, payloadWithState)
          };

        case 'send_service':
          if (!this.robotAPI) {
            return { success: false, error: 'RobotAPI not set' };
          }
          return {
            success: true,
            type: 'send_service',
            count: action.topics.length,
            results: this.robotAPI.sendService(action.topics, action.data_type, payloadWithState)
          };

        case 'local_function':
          const funcName = payloadWithState.function_name;
          if (!funcName || !this.localFunctions[funcName]) {
            return { success: false, error: `Local function "${funcName}" not found` };
          }
          this.localFunctions[funcName](payloadWithState);
          return { success: true, type: 'local_function', function: funcName };

        default:
          return { success: false, error: `Unknown action type: ${action.type}` };
      }
    } catch (error) {
      console.error(`[InputHandler] Execute failed for ${keyId}:`, error.message);
      return { success: false, error: error.message };
    }
  }

  _setupKeyboard() {
    this._onKeyDown = (e) => { const k = (e.key || '').toLowerCase(); this.keys.add(k); };
    this._onKeyUp = (e) => { const k = (e.key || '').toLowerCase(); this.keys.delete(k); };
    window.addEventListener('keydown', this._onKeyDown);
    window.addEventListener('keyup', this._onKeyUp);
  }

  _setupGamepadEvents() {
    this._onGamepadConnected = (e) => {
      this.gamepadIndex = e.gamepad.index;
      this._emit('gamepadConnected', e.gamepad.index);
    };
    this._onGamepadDisconnected = (e) => {
      if (this.gamepadIndex === e.gamepad.index) this.gamepadIndex = null;
      this._emit('gamepadDisconnected', e.gamepad.index);
    };
    window.addEventListener('gamepadconnected', this._onGamepadConnected);
    window.addEventListener('gamepaddisconnected', this._onGamepadDisconnected);
  }

  _applyDeadzone(v) {
    if (Math.abs(v) < this.deadzone) return 0;
    return Math.max(-1, Math.min(1, v));
  }

  _readGamepad() {
    if (this.gamepadIndex == null) return { x: 0, y: 0, active: false };
    const gp = navigator.getGamepads()[this.gamepadIndex];
    if (!gp) return { x: 0, y: 0, active: false };
    // Use left stick by default (axes[0], axes[1])
    let x = this._applyDeadzone(gp.axes[0] || 0);
    let y = this._applyDeadzone(gp.axes[1] || 0);
    const active = x !== 0 || y !== 0;
    return { x, y, active };
  }

  // Simple event emitter API
  on(name, cb) {
    if (!this.listeners[name]) this.listeners[name] = [];
    this.listeners[name].push(cb);
  }
  off(name, cb) {
    if (!this.listeners[name]) return;
    const i = this.listeners[name].indexOf(cb);
    if (i >= 0) this.listeners[name].splice(i, 1);
  }
  _emit(name, ...args) {
    const ls = this.listeners[name];
    if (!ls) return;
    for (let i = 0; i < ls.length; i++) {
      try { ls[i](...args); } catch (e) { console.error('InputHandler listener error', e); }
    }
  }

  _startGamepadPoll() {
    if (this._rafId != null) return;
    const poll = () => {
      this._pollGamepads();
      this._rafId = requestAnimationFrame(poll);
    };
    this._rafId = requestAnimationFrame(poll);
  }
  _stopGamepadPoll() {
    if (this._rafId != null) {
      cancelAnimationFrame(this._rafId);
      this._rafId = null;
    }
  }

  _pollGamepads() {
    const gps = navigator.getGamepads ? navigator.getGamepads() : [];
    for (let i = 0; i < gps.length; i++) {
      const gp = gps[i];
      if (!gp) continue;
      const id = String(i);
      if (!this.prevButtonsByPad[id]) this.prevButtonsByPad[id] = [];
      gp.buttons.forEach((b, idx) => {
        const was = !!this.prevButtonsByPad[id][idx];
        const now = !!b.pressed;
        if (now && !was) {
          this._emit('buttonDown', i, idx);
        } else if (!now && was) {
          this._emit('buttonUp', i, idx);
        }
        this.prevButtonsByPad[id][idx] = now;
      });
      // emit axis update per pad (left stick only)
      const x = this._applyDeadzone(gp.axes[0] || 0);
      const y = this._applyDeadzone(gp.axes[1] || 0);
      this._emit('axis', { pad: i, x, y, active: x !== 0 || y !== 0 });
    }
  }

  dispose() {
    try {
      window.removeEventListener('keydown', this._onKeyDown);
      window.removeEventListener('keyup', this._onKeyUp);
    } catch (e) { console.error('Error removing keyboard listeners', e); }
    try {
      window.removeEventListener('gamepadconnected', this._onGamepadConnected);
      window.removeEventListener('gamepaddisconnected', this._onGamepadDisconnected);
    } catch (e) { console.error('Error removing gamepad listeners', e); }
    this._stopGamepadPoll();
    this.listeners = Object.create(null);
  }

  // Public method to read gamepad joystick values
  readJoystick() {
    return this._readGamepad();
  }

  // Read trigger values (LT and RT)
  // Returns: { leftTrigger: 0-1, rightTrigger: 0-1 }
  readTriggers() {
    if (this.gamepadIndex == null) return { leftTrigger: 0, rightTrigger: 0 };
    const gp = navigator.getGamepads()[this.gamepadIndex];
    if (!gp) return { leftTrigger: 0, rightTrigger: 0 };
    
    let leftTrigger = 0;
    let rightTrigger = 0;

    if (gp.axes.length > 5) {
      leftTrigger = Math.max(0, gp.axes[4] || 0);
      rightTrigger = Math.max(0, gp.axes[5] || 0);
    }
    
    if (leftTrigger < 0.05 && rightTrigger < 0.05) {
      if (gp.buttons && gp.buttons.length > 7) {
        leftTrigger = typeof gp.buttons[6].value !== 'undefined' ? gp.buttons[6].value : (gp.buttons[6].pressed ? 1 : 0);
        rightTrigger = typeof gp.buttons[7].value !== 'undefined' ? gp.buttons[7].value : (gp.buttons[7].pressed ? 1 : 0);
      }
    }
    
    return { leftTrigger, rightTrigger };
  }
}

//______________________ Main Application ____________________
(function initializeApp() {
  const infoPanel = document.getElementById('info-messages');
  
  function log(message) {
    if (!infoPanel) return console.log(message);
    infoPanel.textContent += message + '\n';
    infoPanel.scrollTop = infoPanel.scrollHeight;
  }

  let robotAPI = null;
  let inputHandler = null;
  let cmdVelTopic = null;
  let jointCommandTopic = null;
  let isControlEnabled = false;
  let controlInterval = null;
  
  const UPDATE_INTERVAL = 50;  // 20Hz 
  
  // Flipper joint names (standard ROS naming)
  const FLIPPER_JOINTS = [
    'flipper_0',
    'flipper_1',
    'flipper_2',
    'flipper_3'
  ];

  // Flipper state
  let flipperPositions = [0.0, 0.0, 0.0, 0.0];
  let flipperDirection = 1;  // 1 = add, 0 = subtract
  const FLIPPER_STEP = 0.1;  // Incrementos pequeños en radianes
  let flipperVelocity = 1.0;
  let flipperAcceleration = 0.5;
  let dcMotorVelocity = 1.0;

  let allKeymapProfiles = {};
  let currentKeymapProfile = "default";
  let activeSlot = 1;

  window.handleSlotChange = function(slotNum, selectElem) {
    if (activeSlot === slotNum) {
       currentKeymapProfile = selectElem.value;
       inputHandler.actionMap = {};
       inputHandler.loadActions(allKeymapProfiles[currentKeymapProfile]);
       const profileLabel = document.getElementById("current-profile-label");
       if (profileLabel) { profileLabel.textContent = "Activo: " + currentKeymapProfile; }
       log(`Controles actualizados al perfil: ${currentKeymapProfile}`);
    }
  };

  window.toggleKeymapProfile = function() {
    const slot1 = document.getElementById("keymap-slot-1");
    const slot2 = document.getElementById("keymap-slot-2");
    if (!slot1 || !slot2) return;
    
    activeSlot = activeSlot === 1 ? 2 : 1;
    const newProfile = activeSlot === 1 ? slot1.value : slot2.value;
    
    currentKeymapProfile = newProfile;
    inputHandler.actionMap = {};
    inputHandler.loadActions(allKeymapProfiles[currentKeymapProfile]);
    
    slot1.style.borderColor = activeSlot === 1 ? "#0dcaf0" : "#495057";
    slot2.style.borderColor = activeSlot === 2 ? "#0dcaf0" : "#495057";
    const profileLabel = document.getElementById("current-profile-label");
    if (profileLabel) { profileLabel.textContent = "Activo: " + currentKeymapProfile; }
    
    log(`Se cambió al Slot ${activeSlot} (${currentKeymapProfile})`);
  };


  function initializeROS(rosbridgeHost = location.hostname, rosbridgePort = 9090) {
    if (typeof ROSLIB === 'undefined') {
      log('ERROR: ROSLIB not loaded');
      return;
    }

    // Use wss:// if page is loaded over https://, otherwise ws://
    const protocol = location.protocol === 'https:' ? 'wss://' : 'ws://';
    const rosbridgeUrl = `${protocol}${rosbridgeHost}:${rosbridgePort}`;
    
    log(`Connecting to ROS bridge at ${rosbridgeUrl}`);

    const ros = new ROSLIB.Ros({
      url: rosbridgeUrl
    });

    ros.on('connection', () => {
      log('Connected to ROS');
      
      // Create RobotAPI and InputHandler
      robotAPI = new RobotAPI(ros);
      inputHandler = new InputHandler({ robotAPI });
      
      // Create topics for motor control
      cmdVelTopic = robotAPI.getOrCreateTopic(
        '/hardware_node/cmd_vel',
        'geometry_msgs/msg/Twist'
      );
      jointCommandTopic = robotAPI.getOrCreateTopic(
        '/hardware_node/joint_command',
        'hardware/msg/JointControl'
      );
      
      log('Motor control topics ready');
      log('  - /hardware_node/cmd_vel (Twist)');
      log('  - /hardware_node/joint_command (JointControl)');
      
      registerFlipperFunctions();
      setupKeyboardBindings();
      setupSliders();
      
      // Load keyboard keymaps for other functions
      fetch('static/keymaps.json')
        .then(r => r.json())
        .then(profiles => {
          allKeymapProfiles = profiles;
          inputHandler.loadActions(allKeymapProfiles[currentKeymapProfile]);
          log(`Keyboard profiles loaded (${Object.keys(profiles).length}). Start active: ${currentKeymapProfile}`);
          const select1 = document.getElementById("keymap-slot-1");
          const select2 = document.getElementById("keymap-slot-2");
          if (select1 && select2) {
            select1.innerHTML = "";
            select2.innerHTML = "";
            Object.keys(profiles).forEach(key => {
              const opt1 = document.createElement("option");
              opt1.value = key;
              opt1.textContent = key;
              select1.appendChild(opt1);
              const opt2 = document.createElement("option");
              opt2.value = key;
              opt2.textContent = key;
              select2.appendChild(opt2);
            });
            // Default select values if available
            if (profiles["default"]) select1.value = "default";
            if (profiles["custom"]) select2.value = "custom";
          }
          return fetch('static/keys_map_controller.json');
        })
        .then(r => r.json())
        .then(controllerConfig => {
          // Store controller mappings separately
          inputHandler.controllerActionMap = controllerConfig;
          log(`Controller actions loaded: ${Object.keys(controllerConfig).length} mappings`);
          
          inputHandler.on('buttonDown', (padIndex, buttonIndex) => {
            const buttonId = String(buttonIndex);
            const action = inputHandler.controllerActionMap[buttonId];
            
            if (!action) return;
            
            if (action.type === 'local_function') {
              const funcName = action.payload?.function_name;
              if (funcName && inputHandler.localFunctions[funcName]) {
                inputHandler.localFunctions[funcName](action.payload);
                log(`Button ${buttonId} -> ${funcName}()`);
              }
            } else if (action.type === 'send_topic' && robotAPI) {
              robotAPI.sendTopic(action.topics, action.data_type, action.payload);
              log(`Button ${buttonId} -> send_topic`);
            } else if (action.type === 'send_service' && robotAPI) {
              robotAPI.sendService(action.topics, action.data_type, action.payload);
              log(`Button ${buttonId} -> send_service`);
            }
          });
        })
        .catch(e => log(`Failed to load keymaps: ${e.message}`));
    });

    ros.on('close', () => {
      log('Disconnected from ROS');
      if (controlInterval) {
        clearInterval(controlInterval);
        controlInterval = null;
      }
    });

    ros.on('error', (err) => {
      log(`ROS Error: ${err}`);
    });
  }

  function setupKeyboardBindings() {
    
    window.addEventListener('keydown', (e) => {
      // prevent multiple calls 
      if (e.repeat) return; 

      if (!inputHandler) return;
      const key = e.key;
      
      const result = inputHandler.executeAction(key.toLowerCase(), 'pressed');
      
      if (result.success) {
        log(`Key '${key}' (Pressed) -> ${result.type}`);
      }
    });

    // Keyup
    window.addEventListener('keyup', (e) => {
      if (!inputHandler) return;
      const key = e.key;
      // Realeased 
      const result = inputHandler.executeAction(key.toLowerCase(), 'released');
      
      if (result.success && result.type !== 'ignored_release') {
        log(`Key '${key}' (Released) -> ${result.type}`);
      }
    });
  }
  
  function modifyFlipperPosition(index) {
    const delta = flipperDirection === 1 ? FLIPPER_STEP : -FLIPPER_STEP;
    let newPos = flipperPositions[index] + delta;
    if (newPos < 0.0) newPos = 0.0;
    if (newPos > 2 * Math.PI) newPos = 2 * Math.PI;
    
    flipperPositions[index] = newPos;
    log(`${FLIPPER_JOINTS[index]}: ${flipperPositions[index].toFixed(2)}`);
  }
  
  function sendCmdVel(linear, angular) {
    if (!cmdVelTopic) return;
    const msg = new ROSLIB.Message({
      linear: { x: linear, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angular }
    });
    try {
      cmdVelTopic.publish(msg);
    } catch (e) {
      console.error('cmd_vel publish error:', e);
    }
  }
  
  function registerFlipperFunctions() {
    inputHandler.registerLocalFunction('toggleFlipperDirection', () => {
      flipperDirection = flipperDirection === 1 ? 0 : 1;
      log(`Flipper direction: ${flipperDirection === 1 ? 'ADD' : 'SUBTRACT'}`);
    });
    
    inputHandler.registerLocalFunction('modifyFlipper', (payload) => {
      if (typeof payload.index === 'number') {
        modifyFlipperPosition(payload.index);
      }
    });
    
    inputHandler.registerLocalFunction('resetFlipperPositions', () => {
      flipperPositions = [0, 0, 0, 0];
      log('Flipper positions reset to 0');
    });
    
    inputHandler.registerLocalFunction('sendCmdVel', (payload) => {
      const linearDir = payload.linear_dir ?? 0;
      const angularDir = payload.angular_dir ?? 0;
      sendCmdVel(linearDir * dcMotorVelocity, angularDir * dcMotorVelocity);
    });
  }

  function setupSliders() {
    const flipperVel = document.getElementById('flipper-vel');
    const flipperVelVal = document.getElementById('flipper-vel-val');
    if (flipperVel && flipperVelVal) {
      flipperVel.value = flipperVelocity;
      flipperVelVal.textContent = flipperVelocity.toFixed(2);
      flipperVel.addEventListener('input', (e) => {
        flipperVelocity = parseFloat(e.target.value);
        flipperVelVal.textContent = flipperVelocity.toFixed(2);
      });
    }

    const flipperAccel = document.getElementById('flipper-accel');
    const flipperAccelVal = document.getElementById('flipper-accel-val');
    if (flipperAccel && flipperAccelVal) {
      flipperAccel.value = flipperAcceleration;
      flipperAccelVal.textContent = flipperAcceleration.toFixed(2);
      flipperAccel.addEventListener('input', (e) => {
        flipperAcceleration = parseFloat(e.target.value);
        flipperAccelVal.textContent = flipperAcceleration.toFixed(2);
      });
    }

    const dcMotorVel = document.getElementById('dc-motor-vel');
    const dcMotorVelVal = document.getElementById('dc-motor-vel-val');
    if (dcMotorVel && dcMotorVelVal) {
      dcMotorVel.value = dcMotorVelocity;
      dcMotorVelVal.textContent = dcMotorVelocity.toFixed(2);
      dcMotorVel.addEventListener('input', (e) => {
        dcMotorVelocity = parseFloat(e.target.value);
        dcMotorVelVal.textContent = dcMotorVelocity.toFixed(2);
      });
    }
  }

  function publishControlMessages() {
    if (!jointCommandTopic || !inputHandler) return;
    
  
    
    // Publish joint_command (JointControl) 
    const jointMsg = new ROSLIB.Message({
      header: {
        stamp: { sec: 0, nanosec: 0 },
        frame_id: ''
      },
      joint_names: FLIPPER_JOINTS,
      position: flipperPositions.slice(),
      velocity: [flipperVelocity, flipperVelocity, flipperVelocity, flipperVelocity],
      acceleration: [flipperAcceleration, flipperAcceleration, flipperAcceleration, flipperAcceleration],
      effort: []
    });
    
    try {
      jointCommandTopic.publish(jointMsg);
    } catch (e) {
      console.error('joint_command publish error:', e);
    }
  }

  const startButton = document.getElementById('toggle-teleoperation');
  if (startButton) {
    startButton.addEventListener('click', () => {
      isControlEnabled = !isControlEnabled;
      
      const btnText = document.getElementById('teleoperation-btn-text');
      if (btnText) {
        btnText.textContent = isControlEnabled ? 'Stop Teleoperation' : 'Start Teleoperation';
      }
      
      log('Teleoperation ' + (isControlEnabled ? 'ENABLED' : 'DISABLED'));
      
      if (!isControlEnabled) {
        if (controlInterval) {
          clearInterval(controlInterval);
          controlInterval = null;
        }
        // Send stop commands
        if (cmdVelTopic) {
          const stopTwist = new ROSLIB.Message({
            linear: { x: 0.0, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: 0.0 }
          });
          cmdVelTopic.publish(stopTwist);
        }
      } else {
        if (!controlInterval) {
          controlInterval = setInterval(publishControlMessages, UPDATE_INTERVAL);
        }
      }
    });
  } else {
    log('Warning: Start button not found (id: start-communication)');
  }

  window.executeAction = (actionId) => {
    if (!inputHandler) {
      console.warn('InputHandler not initialized');
      return { success: false, error: 'InputHandler not initialized' };
    }
    return inputHandler.executeAction(actionId);
  };

  window.toggleServerMic = () => {
    const button = document.getElementById('toggle-speaker');
    if (!button) return;
    
    const isMuted = button.textContent === 'Unmute Server';
    const actionId = isMuted ? 'button_unmute_server' : 'button_mute_server';
    
    const result = window.executeAction(actionId);
    
    if (result.success) {
      button.textContent = isMuted ? 'Mute Server' : 'Unmute Server';
      button.style.backgroundColor = isMuted ? '#dc3545' : '#6c757d';
      log(`Server microphone ${isMuted ? 'unmuted' : 'muted'}`);
    } else {
      log(`Failed to toggle server mic: ${result.error || 'unknown error'}`);
    }
  };
  
  window.getFlipperState = () => ({
    positions: flipperPositions.slice(),
    direction: flipperDirection,
    joints: FLIPPER_JOINTS
  });

  window.addEventListener('beforeunload', () => {
    if (cmdVelTopic) {
      try {
        cmdVelTopic.publish(new ROSLIB.Message({
          linear: { x: 0.0, y: 0.0, z: 0.0 },
          angular: { x: 0.0, y: 0.0, z: 0.0 }
        }));
      } catch (e) {
        console.error('Error publishing emergency stop:', e);
      }
    }
    if (robotAPI) robotAPI.dispose();
    if (inputHandler) inputHandler.dispose();
  });

  log('Initializing application...');
  initializeROS();
})();
