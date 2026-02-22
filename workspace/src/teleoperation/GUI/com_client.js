
//______________________ Rosbridge ____________________
class RobotAPI {
  constructor(rosInstance) {
    if (!rosInstance) {
      throw new Error('RobotAPI requires a valid ROSLIB.Ros instance');
    }
    
    this.ros = rosInstance;
    this.topicCache = {};      // { '/target_name': ROSLIB.Topic }
    this.serviceCache = {};    // { '/target_name': ROSLIB.Service }
  }

  // Get or create a topic publisher
  getOrCreateTopic(topicName, messageType) {
    if (this.topicCache[topicName]) {
      return this.topicCache[topicName];
    }

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
      queue_size: 1
    });

    this.topicCache[topicName] = topic;
    console.log(`[RobotAPI] Created topic: ${topicName}`);
    return topic;
  }

  // Get or create a service client
  getOrCreateService(serviceName, serviceType) {
    if (this.serviceCache[serviceName]) {
      return this.serviceCache[serviceName];
    }

    const service = new ROSLIB.Service({
      ros: this.ros,
      name: serviceName,
      serviceType: serviceType
    });

    this.serviceCache[serviceName] = service;
    console.log(`[RobotAPI] Created service: ${serviceName}`);
    return service;
  }

  // Send message to topic(s)
  sendTopic(topicNames, messageType, payload) {
    if (!Array.isArray(topicNames)) {
      topicNames = [topicNames];
    }

    const results = [];
    topicNames.forEach(topicName => {
      try {
        const topic = this.getOrCreateTopic(topicName, messageType);
        const message = new ROSLIB.Message(payload);
        topic.publish(message);
        console.log(`[RobotAPI] Published to ${topicName}:`, payload);
        results.push({ success: true, target: topicName });
      } catch (error) {
        console.error(`[RobotAPI] Failed to publish to ${topicName}:`, error.message);
        results.push({ success: false, target: topicName, error: error.message });
      }
    });

    return results;
  }

  // Call service(s)
  sendService(serviceNames, serviceType, payload) {
    if (!Array.isArray(serviceNames)) {
      serviceNames = [serviceNames];
    }

    const results = [];
    serviceNames.forEach(serviceName => {
      try {
        const service = this.getOrCreateService(serviceName, serviceType);
        const request = new ROSLIB.ServiceRequest(payload);
        service.callService(request, (response) => {
          console.log(`[RobotAPI] Service ${serviceName} response:`, response);
        });
        console.log(`[RobotAPI] Called service ${serviceName} with:`, payload);
        results.push({ success: true, target: serviceName });
      } catch (error) {
        console.error(`[RobotAPI] Failed to call service ${serviceName}:`, error.message);
        results.push({ success: false, target: serviceName, error: error.message });
      }
    });

    return results;
  }

  getStats() {
    return {
      topicsCached: Object.keys(this.topicCache).length,
      servicesCached: Object.keys(this.serviceCache).length
    };
  }

  dispose() {
    Object.values(this.topicCache).forEach(topic => {
      try {
        topic.unsubscribe && topic.unsubscribe();
      } catch (e) {
        console.error('[RobotAPI] Error unsubscribing topic:', e);
      }
    });

    this.topicCache = {};
    this.serviceCache = {};
    console.log('[RobotAPI] Disposed');
  }
}

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

  // Mapea posición del joystick (-1 a 1) a velocidad (-100 a 100) con curva cuadrática
  function mapToVelocity(pos) {
    // Curva cuadrática: más precisión cerca del centro, más potencia en extremos
    const sign = pos >= 0 ? 1 : -1;
    return Math.round(sign * Math.pow(Math.abs(pos), 1.5) * 100);
  }

  function publishJoystickVelocities() {
    if (!motorTopic || !inputHandler) return;
    
    const { x, y } = inputHandler.readJoystick();
    // Formato: "vx,vy,a,b,c,d" - velocidades de -100 a 100
    const dataString = [
      mapToVelocity(x),
      mapToVelocity(y),
      motorVars.a,
      motorVars.b,
      motorVars.c,
      motorVars.d
    ].join(',');
    
    const message = new ROSLIB.Message({ data: dataString });
    
    try {
      motorTopic.publish(message);
    } catch (e) {
      console.error('Joystick velocity publish error:', e);
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
        // Send stop command
        if (motorTopic) {
          const stopMsg = new ROSLIB.Message({ data: '0.0,0.0,0,0,0,0' });
          motorTopic.publish(stopMsg);
        }
      } else {
        // Start publishing joystick velocities at 10Hz
        if (!velocityInterval) {
          velocityInterval = setInterval(publishJoystickVelocities, 100);
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
    if (motorTopic) {
      try {
        motorTopic.publish(new ROSLIB.Message({ data: '0.0,0.0,0,0,0,0' }));
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
