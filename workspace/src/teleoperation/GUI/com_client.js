
//______________________ Input Handler ____________________
class InputHandler {
  constructor({ deadzone = 0.12, robotAPI = null } = {}) {
    this.deadzone = deadzone;
    this.keys = new Set();
    this.gamepadIndex = null;
    this._rafId = null;
    this.prevButtonsByPad = {};
    this.listeners = Object.create(null);
    
    this.robotAPI = robotAPI;  // Reference to RobotAPI for ROS communication
    this.actionMap = {};       // { 'key': { type, data_type, topics, payload } }
    this.localFunctions = {};  // { 'function_name': callback }
    
    this._setupKeyboard();
    this._setupGamepadEvents();
    this._startGamepadPoll();
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
  executeAction(keyId) {
    const action = this.actionMap[keyId];
    if (!action) {
      return { success: false, error: `Action "${keyId}" not mapped` };
    }

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
            results: this.robotAPI.sendTopic(action.topics, action.data_type, action.payload)
          };

        case 'send_service':
          if (!this.robotAPI) {
            return { success: false, error: 'RobotAPI not set' };
          }
          return {
            success: true,
            type: 'send_service',
            count: action.topics.length,
            results: this.robotAPI.sendService(action.topics, action.data_type, action.payload)
          };

        case 'local_function':
          const funcName = action.payload?.function_name;
          if (!funcName || !this.localFunctions[funcName]) {
            return { success: false, error: `Local function "${funcName}" not found` };
          }
          this.localFunctions[funcName](action.payload);
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
    
    // Axes: izquierdo = axes[4], derecho = axes[5]
    if (gp.axes.length > 5) {
      // Asegurar que el valor esté en rango [0, 1]
      leftTrigger = Math.max(0, gp.axes[4] || 0);
      rightTrigger = Math.max(0, gp.axes[5] || 0);
    }
    
    // Fallback to button values (buttons 6 and 7) si los axes no funcionan
    if (leftTrigger < 0.05 && rightTrigger < 0.05) {
      if (gp.buttons.length > 7) {
        leftTrigger = gp.buttons[6]?.value || (gp.buttons[6]?.pressed ? 1 : 0);
        rightTrigger = gp.buttons[7]?.value || (gp.buttons[7]?.pressed ? 1 : 0);
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

  // Setup ROS connection and API
  let robotAPI = null;
  let inputHandler = null;
  let motorTopic = null;
  let isControlEnabled = false;
  let velocityInterval = null;
  
  // Variables locales para control adicional (a,b,c,d)
  let motorVars = { a: 0, b: 0, c: 0, d: 0 };
  
  // Variables para control de velocidad con gatillos y aceleración
  let currentVelocity = { x: 0, y: 0 };  
  const ACCELERATION_TIME = 0.5;  
  const MAX_VELOCITY = 100;
  const UPDATE_INTERVAL = 100;  // 100ms = 10Hz
  const ACCELERATION_STEP = (MAX_VELOCITY / ACCELERATION_TIME) * (UPDATE_INTERVAL / 1000);
  
  // Variables para giro cerrado (bumpers B4 y B5)
  let closedTurnState = { left: false, right: false };  // B4 = left, B5 = right  
  
  // Función para cambiar las 4 variables simultáneamente
  window.setMotorVars = function(values) {
    if (Array.isArray(values) && values.length === 4) {
      motorVars.a = values[0];
      motorVars.b = values[1];
      motorVars.c = values[2];
      motorVars.d = values[3];
      log(`Motor vars: a=${motorVars.a}, b=${motorVars.b}, c=${motorVars.c}, d=${motorVars.d}`);
    }
  };
  
  // Función para resetear variables (tecla 's')
  window.resetMotorVars = function() {
    motorVars.a = 0;
    motorVars.b = 0;
    motorVars.c = 0;
    motorVars.d = 0;
    log('Motor vars reset to 0');
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
      
      // Load keyboard keymaps
      fetch('static/keys_map_keyboard.json')
        .then(r => r.json())
        .then(config => {
          inputHandler.loadActions(config);
          log(`Keyboard actions loaded: ${Object.keys(inputHandler.actionMap).length} mappings`);
          setupKeyboardBindings();
          
          // Get motor topic for joystick control (formato: "x,y,a,b,c,d")
          motorTopic = robotAPI.getOrCreateTopic('/all_motors', 'std_msgs/String');
          log('Motor control topic ready (/all_motors)');
          
          // Registrar funciones locales para InputHandler
          inputHandler.registerLocalFunction('setMotorVars', (payload) => {
            if (payload.data) window.setMotorVars(payload.data);
          });
          inputHandler.registerLocalFunction('resetMotorVars', () => {
            window.resetMotorVars();
          });
          
          // Load controller keymaps
          return fetch('static/keys_map_controller.json');
        })
        .then(r => r.json())
        .then(controllerConfig => {
          // Store controller mappings separately
          inputHandler.controllerActionMap = controllerConfig;
          log(`Controller actions loaded: ${Object.keys(controllerConfig).length} mappings`);
          
          // Setup gamepad button handler
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
      if (velocityInterval) {
        clearInterval(velocityInterval);
        velocityInterval = null;
      }
    });

    ros.on('error', (err) => {
      log(`ROS Error: ${err}`);
    });
  }

  function setupKeyboardBindings() {
    window.addEventListener('keydown', (e) => {
      if (!inputHandler) return;
      
      const key = e.key.toLowerCase();
      const result = inputHandler.executeAction(key);
      
      if (result.success) {
        if (result.count > 1) {
          log(`Key '${key}' -> ${result.type} (${result.count} targets)`);
        } else if (result.type === 'local_function') {
          log(`Key '${key}' -> ${result.function}()`);
        } else {
          log(`Key '${key}' -> ${result.type}`);
        }
      } else if (result.error !== `Action "${key}" not mapped`) {
        log(`Key '${key}' failed: ${result.error}`);
      }
    });
  }

  function getTargetVelocityFromTriggers() {
    if (!inputHandler) return { x: 0, y: 0 };
    
    const { leftTrigger, rightTrigger } = inputHandler.readTriggers();
    const { x: joystickX } = inputHandler.readJoystick();
    
    // Determinar velocidad base según gatillos
    let baseVelocity = 0;
    
    if (rightTrigger > 0.1) {
      // Gatillo derecho: velocidad positiva
      baseVelocity = MAX_VELOCITY;
    } else if (leftTrigger > 0.1) {
      // Gatillo izquierdo: velocidad negativa
      baseVelocity = -MAX_VELOCITY;
    }
    
    if (baseVelocity === 0) {
      return { x: 0, y: 0 };
    }
    
    // direction  joystick X
    // rigth (75%, 0%)
    // leftizquierda (0%, 75%)
    const absX = Math.abs(joystickX);
    let multiplierX, multiplierY;
    
    if (joystickX >= 0) {
      multiplierX = 1 - absX * 0.25;  
      multiplierY = 1 - absX;         
    } else {
      multiplierX = 1 - absX;         
      multiplierY = 1 - absX * 0.25;  
    }
    
    return { 
      x: baseVelocity * multiplierX, 
      y: baseVelocity * multiplierY 
    };
  }

  // Aplica aceleración gradual hacia la velocidad objetivo
  function applyAcceleration(current, target) {
    if (current < target) {
      // Acelerando hacia adelante
      return Math.min(current + ACCELERATION_STEP, target);
    } else if (current > target) {
      // Desacelerando o acelerando hacia atrás
      return Math.max(current - ACCELERATION_STEP, target);
    }
    return current;
  }

  function readBumperState() {
    if (!inputHandler || inputHandler.gamepadIndex == null) return;
    const gp = navigator.getGamepads()[inputHandler.gamepadIndex];
    if (!gp || gp.buttons.length < 6) return;
    
    // Button 4 = Left Bumper (LB), Button 5 = Right Bumper (RB)
    closedTurnState.left = gp.buttons[4]?.pressed || false;
    closedTurnState.right = gp.buttons[5]?.pressed || false;
  }

  function publishTriggerVelocities() {
    if (!motorTopic || !inputHandler) return;
    
    // Leer estado de bumpers para giro cerrado
    readBumperState();
    
    // Obtener velocidad objetivo basada en gatillos
    const target = getTargetVelocityFromTriggers();
    
    // Aplicar aceleración gradual
    currentVelocity.x = applyAcceleration(currentVelocity.x, target.x);
    currentVelocity.y = applyAcceleration(currentVelocity.y, target.y);
    
    // Velocidades finales a enviar
    let finalX = Math.round(currentVelocity.x);
    let finalY = Math.round(currentVelocity.y);
    
    // Override con giro cerrado si los bumpers están presionados
    if (closedTurnState.left) {
      finalX = -MAX_VELOCITY;
      finalY = MAX_VELOCITY;
    } else if (closedTurnState.right) {
      finalX = MAX_VELOCITY;
      finalY = -MAX_VELOCITY;
    }
    
    // Formato: "vx,vy,a,b,c,d" - velocidades de -100 a 100
    const dataString = [
      finalX,
      finalY,
      motorVars.a,
      motorVars.b,
      motorVars.c,
      motorVars.d
    ].join(',');
    
    const message = new ROSLIB.Message({ data: dataString });
    
    try {
      motorTopic.publish(message);
    } catch (e) {
      console.error('Trigger velocity publish error:', e);
    }
  }

  // Control button to enable/disable joystick velocity publishing
  const startButton = document.getElementById('start-communication');
  if (startButton) {
    startButton.addEventListener('click', () => {
      isControlEnabled = !isControlEnabled;
      startButton.textContent = isControlEnabled ? 'Stop Control' : 'Start Control';
      log('Control ' + (isControlEnabled ? 'ENABLED' : 'DISABLED'));
      
      if (!isControlEnabled) {
        if (velocityInterval) {
          clearInterval(velocityInterval);
          velocityInterval = null;
        }
        // Reset current velocity and send stop command
        currentVelocity.x = 0;
        currentVelocity.y = 0;
        if (motorTopic) {
          const stopMsg = new ROSLIB.Message({ data: '0,0,0,0,0,0' });
          motorTopic.publish(stopMsg);
        }
      } else {
        // Start publishing trigger velocities at 10Hz
        if (!velocityInterval) {
          velocityInterval = setInterval(publishTriggerVelocities, UPDATE_INTERVAL);
        }
      }
    });
  } else {
    log('Warning: Start button not found (id: start-btn or start-communication)');
  }

  // Expose executeAction globally 
  window.executeAction = (actionId) => {
    if (!inputHandler) {
      console.warn('InputHandler not initialized');
      return { success: false, error: 'InputHandler not initialized' };
    }
    return inputHandler.executeAction(actionId);
  };

  // Toggle server microphone with dynamic button update
  window.toggleServerMic = () => {
    const button = document.getElementById('toggle-speaker');
    if (!button) return;
    
    const isMuted = button.textContent === 'Unmute Server';
    const actionId = isMuted ? 'button_unmute_server' : 'button_mute_server';
    
    const result = window.executeAction(actionId);
    
    if (result.success) {
      // Update button appearance
      button.textContent = isMuted ? 'Mute Server' : 'Unmute Server';
      button.style.backgroundColor = isMuted ? '#dc3545' : '#6c757d';
      log(`Server microphone ${isMuted ? 'unmuted' : 'muted'}`);
    } else {
      log(`Failed to toggle server mic: ${result.error || 'unknown error'}`);
    }
  };

  // Emergency stop on page unload
  window.addEventListener('beforeunload', () => {
    currentVelocity.x = 0;
    currentVelocity.y = 0;
    if (motorTopic) {
      try {
        motorTopic.publish(new ROSLIB.Message({ data: '0,0,0,0,0,0' }));
      } catch (e) {
        console.error('Error publishing emergency stop:', e);
      }
    }
    if (robotAPI) robotAPI.dispose();
    if (inputHandler) inputHandler.dispose();
  });

  // Initialize on load
  log('Initializing application...');
  initializeROS();
})();
