
//______________________ Rosbridge ____________________
class RobotAPI {
  constructor(rosInstance) {
    if (!rosInstance) {
      throw new Error('RobotAPI requires a valid ROSLIB.Ros instance');
    }
    
    this.ros = rosInstance;
    
    this.topicCache = {};      // { '/target_name': ROSLIB.Topic }
    this.serviceCache = {};    // { '/target_name': ROSLIB.Service }
    
    this.actionMap = {};       // { 'key_id': { type, ros_object, payload } }
    
    this.isInitialized = false;
  }

  init(configJson) {
    if (!configJson || typeof configJson !== 'object') {
      console.error('[RobotAPI] Invalid config: must be an object');
      return false;
    }

    try {
      Object.entries(configJson).forEach(([keyId, action]) => {
        const { type, target, msg_type, srv_type, payload } = action;

        if (!type || !target) {
          console.warn(`[RobotAPI] Skipping invalid action for ${keyId}: missing type or target`);
          return;
        }

        let rosObject = null;

        // LÓGICA DE PRE-CACHING: Verifica y reutiliza conexiones existentes
        if (type === 'topic') {
          rosObject = this._getOrCreateTopic(target, msg_type || 'std_msgs/String');
        } else if (type === 'service') {
          rosObject = this._getOrCreateService(target, srv_type || 'std_srvs/Trigger');
        } else {
          console.warn(`[RobotAPI] Unknown action type "${type}" for key ${keyId}`);
          return;
        }

        // MAPEO: Asocia la tecla con el objeto ROS y su payload
        this.actionMap[keyId] = {
          type,
          target,
          rosObject,
          payload: payload || {}
        };

        console.log(`[RobotAPI] Mapped ${keyId} -> ${type}:${target}`);
      });

      this.isInitialized = true;
      console.log(`[RobotAPI] Initialized with ${Object.keys(this.actionMap).length} actions`);
      return true;
    } catch (error) {
      console.error('[RobotAPI] Init failed:', error.message);
      return false;
    }
  }

  _getOrCreateTopic(target, messageType) {

    if (this.topicCache[target]) {
      console.log(`[RobotAPI] Topic cache hit for ${target}`);
      return this.topicCache[target];
    }

    console.log(`[RobotAPI] Creating new Topic for ${target}`);
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: target,
      messageType: messageType,
      queue_size: 1
    });

    this.topicCache[target] = topic;
    return topic;
  }

  _getOrCreateService(target, serviceType) {
    if (this.serviceCache[target]) {
      console.log(`[RobotAPI] Service cache hit for ${target}`);
      return this.serviceCache[target];
    }

    console.log(`[RobotAPI] Creating new Service for ${target}`);
    const service = new ROSLIB.Service({
      ros: this.ros,
      name: target,
      serviceType: serviceType
    });

    this.serviceCache[target] = service;
    return service;
  }

  // Executes an action mapped to a key
  executeAction(keyId) {
    // Validar que la acción está mapeada
    const action = this.actionMap[keyId];
    if (!action) {
      return { success: false, error: `Action "${keyId}" not found in map` };
    }

    try {
      if (action.type === 'topic') {
        return this._executeTopic(action);
      } else if (action.type === 'service') {
        return this._executeService(action);
      } else {
        return { success: false, error: `Unknown action type: ${action.type}` };
      }
    } catch (error) {
      console.error(`[RobotAPI] Execute failed for ${keyId}:`, error.message);
      return { success: false, error: error.message };
    }
  }

  _executeTopic(action) {
    const { target, rosObject, payload } = action;

    const message = new ROSLIB.Message(payload);
    rosObject.publish(message);

    console.log(`[RobotAPI] Published to ${target}:`, payload);
    return { success: true, type: 'topic', target, payload };
  }

  _executeService(action) {
    const { target, rosObject, payload } = action;

    // Crear request del servicio
    const request = new ROSLIB.ServiceRequest(payload);

    // Llamar al servicio con manejo de respuesta
    rosObject.callService(request, (response) => {
      console.log(`[RobotAPI] Service ${target} response:`, response);
    });

    console.log(`[RobotAPI] Called service ${target} with:`, payload);
    return { success: true, type: 'service', target, payload };
  }


  getStats() {
    return {
      topicsCached: Object.keys(this.topicCache).length,
      servicesCached: Object.keys(this.serviceCache).length,
      actionsMapped: Object.keys(this.actionMap).length,
      isInitialized: this.isInitialized
    };
  }

  dispose() {
    Object.values(this.topicCache).forEach(topic => {
      try {
        topic.unsubscribe && topic.unsubscribe();
      } catch (e) {
        console.error('[RobotAPI] Error unsubscribing topic:', e.message);
      }
    });

    this.topicCache = {};
    this.serviceCache = {};
    this.actionMap = {};
    this.isInitialized = false;
    console.log('[RobotAPI] Disposed');
  }

  // Get cached topic directly without executing an action
  getTopicPublisher(topicName) {
    return this.topicCache[topicName] || null;
  }
}

//______________________ Input Handler ____________________
class InputHandler {
  constructor({ deadzone = 0.12 } = {}) {
    this.deadzone = deadzone;
    this.keys = new Set();
    this.gamepadIndex = null;
    this._rafId = null;
    this.prevButtonsByPad = {};
    this.listeners = Object.create(null); // simple event emitter: buttonDown, buttonUp, axis, gamepadConnected, gamepadDisconnected
    this._setupKeyboard();
    this._setupGamepadEvents();
    this._startGamepadPoll();
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
    // Some controllers use inverted y; keep raw values as requested
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

  // Initialize input handler
  const inputHandler = new InputHandler();

  // Setup ROS connection and RobotAPI
  let robotAPI = null;
  let motorPublisher = null; // Direct publisher for joystick velocities
  let isControlEnabled = false;
  let velocityInterval = null;

  function initializeROS(rosbridgeHost = location.hostname, rosbridgePort = 9090) {
    if (typeof ROSLIB === 'undefined') {
      log('ERROR: ROSLIB not loaded');
      return;
    }

    const ros = new ROSLIB.Ros({
      url: `ws://${rosbridgeHost}:${rosbridgePort}`
    });

    ros.on('connection', () => {
      log('Connected to ROS');
      
      // Initialize RobotAPI with keymaps
      robotAPI = new RobotAPI(ros);
      
      // Load keyboard keymaps
      fetch('static/keys_map_keyboard.json')
        .then(r => r.json())
        .then(config => {
          robotAPI.init(config);
          log(`RobotAPI initialized: ${robotAPI.getStats().actionsMapped} actions mapped`);
          setupKeyboardBindings();
          
          // Get motor publisher from RobotAPI cache (reuse connection)
          motorPublisher = robotAPI.getTopicPublisher('/dc_motors');
          if (motorPublisher) {
            log('Motor publisher obtained from RobotAPI cache');
          }
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
      if (!robotAPI || !robotAPI.isInitialized) return;
      
      const key = e.key.toLowerCase();
      const result = robotAPI.executeAction(key);
      
      if (result.success) {
        log(`Key '${key}' executed: ${result.type} -> ${result.target}`);
      } else if (result.error !== `Action "${key}" not found in map`) {
        log(`Key '${key}' failed: ${result.error}`);
      }
    });
  }

  function publishJoystickVelocities() {
    if (!motorPublisher) return;
    
    const { x, y } = inputHandler.readJoystick();
    const message = new ROSLIB.Message({
      data: [Number(x) || 0.0, Number(y) || 0.0]
    });
    
    try {
      motorPublisher.publish(message);
    } catch (e) {
      console.error('Joystick velocity publish error:', e);
    }
  }

  // Control button to enable/disable joystick velocity publishing
  const startButton = document.getElementById('start-btn') || document.getElementById('start-communication');
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
        if (motorPublisher) {
          const stopMsg = new ROSLIB.Message({ data: [0.0, 0.0] });
          motorPublisher.publish(stopMsg);
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

  // Emergency stop on page unload
  window.addEventListener('beforeunload', () => {
    if (motorPublisher) {
      try {
        motorPublisher.publish(new ROSLIB.Message({ data: [0.0, 0.0] }));
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
