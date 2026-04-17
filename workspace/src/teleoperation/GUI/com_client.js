const TeleopState = {
  active: false,
  speeds: { arm: 0.2, arm_joint: 0.5, arm_joint_gains: [1.0, 1.0, 1.0, 1.0], base: 1.0, flipper: 1.0, flipperAccel: 0.5 },
  base: { linear: 0, angular: 0 },
  arm: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
  arm_joints: {
    velocities: [0, 0, 0, 0, 0, 0],
    directions: [1, 1, 1, 1, 1, 1],
    globalDirection: 1,
    currentDir: 0,
    lastPressTime: [0, 0, 0, 0, 0, 0]
  },
  flippers: {
    velocities: [0, 0, 0, 0, 0, 0, 0, 0],
    active: [false, false, false, false, false, false, false, false],
    currentDir: 0,
  },
  resetAll: function() {
    this.base = { linear: 0, angular: 0 };
    this.arm = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
    this.arm_joints.velocities.fill(0);
    this.flippers.velocities.fill(0);
    this.flippers.active.fill(false);
    this.flippers.currentDir = 0;
  }
};

// Sync speeds across different windows (e.g. index.html and feedback.html)
window.addEventListener('storage', (e) => {
  if (e.key === 'teleop_speeds' && e.newValue) {
    try {
      const newSpeeds = JSON.parse(e.newValue);
      Object.assign(TeleopState.speeds, newSpeeds);
      console.log("[TeleopState] Speeds synced from localStorage:", TeleopState.speeds);
    } catch(err) { console.error('Error syncing speeds', err) }
  }
});


const FLIPPER_JOINTS = ['flipper_0', 'flipper_1', 'flipper_2', 'flipper_3', 'joint_1', 'joint_2', 'joint_3', 'joint_4'];

const MAX_RAD_PER_SEC = Math.PI;

const TeleopActions = {
  publish_topic: (action, payload, handler) => {
    if (!handler.robotAPI) return { success: false, error: 'RobotAPI not set' };
    return {
      success: true,
      type: 'publish_topic',
      count: action.topics.length,
      results: handler.robotAPI.publishTopic(action.topics, action.data_type, payload)
    };
  },
  send_service: (action, payload, handler) => {
    if (!handler.robotAPI) return { success: false, error: 'RobotAPI not set' };
    return {
      success: true,
      type: 'send_service',
      count: action.topics.length,
      results: handler.robotAPI.sendService(action.topics, action.data_type, payload)
    };
  },
  local_function: (action, payload, handler) => {
    const funcName = payload.function_name;
    if (funcName === 'setGlobalDirection') {
      const newDir = payload.dir; 
      TeleopState.flippers.currentDir = newDir;
      
      return { success: true, type: 'local_function', function: funcName };
    }
    if (funcName === 'resetFlipperPositions') {
      TeleopActions.teleop_flipper({ type: 'teleop_flipper' }, { action: 'reset' }, handler);
      return { success: true, type: 'local_function', function: funcName };
    }

    if (!funcName || !handler.localFunctions[funcName]) {
      return { success: false, error: `Local function "${funcName}" not found` };
    }
    handler.localFunctions[funcName](payload);
    return { success: true, type: 'local_function', function: funcName };
  },
  teleop_cmd_vel: (action, payload, handler) => {
    if (payload.linear_dir !== undefined) TeleopState.base.linear = payload.linear_dir;
    if (payload.angular_dir !== undefined) TeleopState.base.angular = payload.angular_dir;

    const lin = TeleopState.base.linear;
    const ang = TeleopState.base.angular;
    const isStop = lin === 0 && ang === 0;

    if (payload.linear_dir === 0 && payload.angular_dir === 0 && payload.force_stop) {
        TeleopState.base.linear = 0;
        TeleopState.base.angular = 0;
    }
    
    if (!handler.robotAPI || (!TeleopState.active && !isStop)) {
        return { success: false, error: 'Control disabled' };
    }
    
    handler.robotAPI.getOrCreatePublisher('/hardware_node/cmd_vel', 'geometry_msgs/msg/Twist').publish(
      new ROSLIB.Message({
        linear: { x: lin * TeleopState.speeds.base, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: ang * TeleopState.speeds.base }
      })
    );
    return { success: true, type: 'teleop_cmd_vel' };
  },
  // Inverse Kinematics via MoveIt Servo
  teleop_cmd_arm: (action, payload, handler) => {
    const isStop = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'].every(axis => (payload[`${axis}_dir`] || 0) === 0);
    if (!handler.robotAPI || (!TeleopState.active && !isStop)) return { success: false, error: 'Control disabled' };
    
    ['x', 'y', 'z', 'roll', 'pitch', 'yaw'].forEach(axis => {
      if (payload[`${axis}_dir`] !== undefined) {
        TeleopState.arm[axis] = payload[`${axis}_dir`];
      }
    });

    // Forced zeroing via payload for absolute stops
    if (isStop) {
      TeleopState.arm = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
    }
    
    const now = new Date();
    handler.robotAPI.getOrCreatePublisher('/servo_node/delta_twist_cmds', 'geometry_msgs/msg/TwistStamped').publish(
      new ROSLIB.Message({
        header: {
          stamp: { sec: Math.floor(now.getTime()/1000), nanosec: (now.getTime()%1000)*1000000 },
          frame_id: 'base_link'
        },
        twist: {
          linear: { x: TeleopState.arm.x * TeleopState.speeds.arm, y: TeleopState.arm.y * TeleopState.speeds.arm, z: TeleopState.arm.z * TeleopState.speeds.arm },
          angular: { x: TeleopState.arm.roll * TeleopState.speeds.arm, y: TeleopState.arm.pitch * TeleopState.speeds.arm, z: TeleopState.arm.yaw * TeleopState.speeds.arm }
        }
      })
    );
    return { success: true, type: 'teleop_cmd_arm' };
  },
  
  // Joint Velocity Control through moveit
  teleop_cmd_arm_joint_vel: (action, payload, handler) => {
    const actionCmd = payload.action;
    const isStop = actionCmd === 'reset';
    if (!handler.robotAPI || (!TeleopState.active && !isStop)) return { success: false, error: 'Control disabled' };

    if (actionCmd === 'reset') {
      TeleopState.arm_joints.velocities = [0, 0, 0, 0, 0, 0];
    } else if (actionCmd === 'toggle_global_dir') {
      TeleopState.arm_joints.globalDirection = TeleopState.arm_joints.globalDirection === 1 ? 0 : 1;
      TeleopState.arm_joints.directions = Array(6).fill(TeleopState.arm_joints.globalDirection);
    } else if (payload.index !== undefined && payload.index >= 0 && payload.index < 6) {
      const idx = payload.index;
      if (payload.state === 'released') {
        TeleopState.arm_joints.velocities[idx] = 0.0;
      } else {
        const t = Date.now();
        if (t - TeleopState.arm_joints.lastPressTime[idx] < 400) {
          TeleopState.arm_joints.directions[idx] = TeleopState.arm_joints.directions[idx] === 1 ? 0 : 1;
          TeleopState.arm_joints.lastPressTime[idx] = 0;
        } else {
          TeleopState.arm_joints.lastPressTime[idx] = t;
        }
        
        const dir = TeleopState.arm_joints.directions[idx] === 1 ? 1 : -1;
        const baseSpeed = TeleopState.speeds.arm_joint;
        const gain = TeleopState.speeds.arm_joint_gains ? (TeleopState.speeds.arm_joint_gains[idx] || 1.0) : 1.0;
        
        TeleopState.arm_joints.velocities[idx] = dir * baseSpeed * gain * MAX_RAD_PER_SEC;
      }
    }

    const now = new Date();
    handler.robotAPI.getOrCreatePublisher('/servo_node/delta_joint_cmds', 'control_msgs/msg/JointJog').publish(
      new ROSLIB.Message({
        header: {
          stamp: { sec: Math.floor(now.getTime()/1000), nanosec: (now.getTime()%1000)*1000000 },
          frame_id: 'base_link'
        },
        joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
        velocities: TeleopState.arm_joints.velocities.slice(),
        displacements: Array(6).fill(-1.0),
        duration: 0.0
      })
    );
    return { success: true, type: 'teleop_cmd_arm_joint_vel' };
  },

  teleop_flipper: (action, payload, handler) => {
    const actionCmd = payload.action;
    const isStop = actionCmd === 'reset';
    if (!handler.robotAPI || (!TeleopState.active && !isStop)) return { success: false, error: 'Control disabled' };
    
    if (actionCmd === 'reset') {
      TeleopState.flippers.velocities.fill(0);
      TeleopState.flippers.active.fill(false);
      TeleopState.flippers.currentDir = 0;
    } else if (payload.dir !== undefined) {
      TeleopState.flippers.currentDir = payload.dir;
    } else if (payload.index !== undefined) {
      TeleopState.flippers.active[payload.index] = (payload.state !== 'released');
    }

    for (let i = 0; i < 8; i++) {
      if (TeleopState.flippers.active[i] && TeleopState.flippers.currentDir !== 0) {
        const speedMultiplier = i < 4 ? TeleopState.speeds.flipper : TeleopState.speeds.arm;
        TeleopState.flippers.velocities[i] = TeleopState.flippers.currentDir * speedMultiplier * MAX_RAD_PER_SEC;
      } else {
        TeleopState.flippers.velocities[i] = 0.0;
      }
    }
    
    handler.robotAPI.getOrCreatePublisher('/hardware_node/joint_command', 'hardware/msg/JointControl').publish(
      new ROSLIB.Message({
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
        joint_names: FLIPPER_JOINTS,
        position: Array(8).fill(-1.0), 
        effort: [],
        velocity: TeleopState.flippers.velocities.slice(),
        acceleration: Array(8).fill(TeleopState.speeds.flipperAccel)
      })
    );
    return { success: true, type: 'teleop_flipper' };
  }
};

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
    
    // Descomentar para reactivar control
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
        const { type, data_type, topics, payload, release_payload, actions } = action;

        if (!type) {
          console.warn(`[InputHandler] Skipping action for ${keyId}: missing type`);
          return;
        }

        // Validate action type
        if (!['publish_topic', 'send_service', 'local_function', 'teleop_cmd_vel', 'teleop_cmd_arm', 'teleop_cmd_arm_joint_vel', 'teleop_flipper'].includes(type)) {
          console.warn(`[InputHandler] Unknown action type "${type}" for key ${keyId}`);
          return;
        }

        // For send actions, ensure topics array and data_type exist
        if (type === 'publish_topic' || type === 'send_service') {
          if (!topics || !Array.isArray(topics) || topics.length === 0) {
            console.warn(`[InputHandler] Action ${keyId} missing topics array`);
            return;
          }
          if (!data_type) {
            console.warn(`[InputHandler] Action ${keyId} missing data_type`);
            return;
          }
        }

        this.actionMap[keyId] = { type, data_type, topics, payload, release_payload, actions };
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

  // Load specific actions
  loadGuiActions(guiConfigJson) {
    this.guiActionMap = {};
    if (!guiConfigJson || typeof guiConfigJson !== 'object') return;
    Object.entries(guiConfigJson).forEach(([key, action]) => {
      this.guiActionMap[key] = action;
    });
    console.log(`[InputHandler] Loaded ${Object.keys(this.guiActionMap).length} GUI actions`);
  }

  trigger_gui_action(actionName) {
    const action = this.guiActionMap ? this.guiActionMap[actionName] : null;
    if (!action) {
      console.warn(`[InputHandler] GUI action "${actionName}" not found`);
      return { success: false, error: `GUI action "${actionName}" not found` };
    }
    
    try {
      if (TeleopActions[action.type]) {
        return TeleopActions[action.type](action, action.payload, this);
      } else {
        return { success: false, error: `Unknown action type: ${action.type}` };
      }
    } catch (error) {
      console.error(`[InputHandler] trigger_gui_action failed for ${actionName}:`, error.message);
      return { success: false, error: error.message };
    }
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
      if (TeleopActions[action.type]) {
        return TeleopActions[action.type](action, payloadWithState, this);
      } else {
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

  let allKeymapProfiles = {};
  let currentKeymapProfile = "";
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

    // Use ws:// for rosbridge connection (rosbridge is running without SSL)
    const protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
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
      
      setupKeyboardBindings();
      setupSliders();
      
      // Load keyboard keymaps for other functions
      fetch('static/keymaps.json')
        .then(r => r.json())
        .then(profiles => {
          // Extract the gamepad mapping so we don't treat it as a keyboard profile
          let controllerConfig = {};
          if (profiles["gamepad"]) {
            controllerConfig = profiles["gamepad"];
            delete profiles["gamepad"];
          }
          
          // Extract gui_buttons mapping
          let guiButtonsConfig = {};
          if (profiles["gui_buttons"]) {
            guiButtonsConfig = profiles["gui_buttons"];
            delete profiles["gui_buttons"];
          }
          if (inputHandler) {
            inputHandler.loadGuiActions(guiButtonsConfig);
            
            // ========== MODIFICACIÓN: EXPONER inputHandler Y NOTIFICAR ==========
            window.inputHandler = inputHandler;
            window.guiActionsReady = true;
            window.dispatchEvent(new Event('guiActionsLoaded'));
            console.log('[com_client] GUI actions loaded and ready. Total actions:', Object.keys(inputHandler.guiActionMap).length);
            // ========== FIN MODIFICACIÓN ==========
          }
          
          allKeymapProfiles = profiles;
          const profileKeys = Object.keys(profiles);
          if (profileKeys.length === 0) {
            log('Error: No keymap profiles found in keymaps.json.');
          } else {
            currentKeymapProfile = profileKeys[0];
            inputHandler.loadActions(allKeymapProfiles[currentKeymapProfile]);
            log(`Keyboard profiles loaded (${profileKeys.length}). Start active: ${currentKeymapProfile}`);
            
            const select1 = document.getElementById("keymap-slot-1");
            const select2 = document.getElementById("keymap-slot-2");
            const profileLabel = document.getElementById("current-profile-label");
            if (profileLabel) { profileLabel.textContent = "Activo: " + currentKeymapProfile; }
            
            if (select1 && select2) {
              select1.innerHTML = "";
              select2.innerHTML = "";
              profileKeys.forEach(key => {
                const opt1 = document.createElement("option");
                opt1.value = key;
                opt1.textContent = key;
                select1.appendChild(opt1);
                
                const opt2 = document.createElement("option");
                opt2.value = key;
                opt2.textContent = key;
                select2.appendChild(opt2);
              });
              
              select1.value = profileKeys[0];
              if (profileKeys.length > 1) {
                select2.value = profileKeys[1];
              } else {
                select2.value = profileKeys[0];
              }
            }
          }
          
          // Store controller mappings separately
          inputHandler.controllerActionMap = controllerConfig;
          if (Object.keys(controllerConfig).length > 0) {
            log(`Controller actions loaded: ${Object.keys(controllerConfig).length} mappings`);
          } else {
            log(`Warning: 'gamepad' profile not found in keymaps.json. Controller actions not loaded.`);
          }
          
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
            } else if (action.type === 'publish_topic' && robotAPI) {
              robotAPI.publishTopic(action.topics, action.data_type, action.payload);
              log(`Button ${buttonId} -> publish_topic`);
            } else if (action.type === 'send_service' && robotAPI) {
              robotAPI.sendService(action.topics, action.data_type, action.payload);
              log(`Button ${buttonId} -> send_service`);
            }
          });
        })
        .catch(e => log(`Failed to load keymaps: ${e.message}`));

      // temporal handle for move it prepos 
      let servoDeactivations = 0;
      const maxDeactivations = 5;
      
      const deactivateServo = () => {
        servoDeactivations++;
        if (servoDeactivations > maxDeactivations) {
          log('[Servo] Error: No se pudo desactivar el modo servo tras 5 intentos.');
          return;
        }

        log(`[Servo] Intentando desactivar el modo servo (Intento ${servoDeactivations}/${maxDeactivations})...`);
        const pauseSrv = robotAPI.getOrCreateService('/servo_node/pause_servo', 'std_srvs/srv/Trigger');
        const request = new ROSLIB.ServiceRequest({});
        
        pauseSrv.callService(request, (result) => {
          if (result.success) {
            log('[Servo] Modo servo desactivado correctamente.');
          } else {
            log(`[Servo] Respuesta indicando que falló al desactivar servo: ${result.message}`);
            setTimeout(deactivateServo, 5000);
          }
        }, (err) => {
          log(`[Servo] Excepción al intentar llamar al servicio pause_servo.`);
          setTimeout(deactivateServo, 5000);
        });
      };
      setTimeout(deactivateServo, 1000);
    });

    ros.on('close', () => {
      log('Disconnected from ROS');
      if (typeof controlInterval !== 'undefined' && controlInterval) {
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
  
  function setupSliders() {
    const flipperVel = document.getElementById('flipper-vel');
    const flipperVelVal = document.getElementById('flipper-vel-val');
    if (flipperVel && flipperVelVal) {
      flipperVel.value = TeleopState.speeds.flipper;
      flipperVelVal.textContent = TeleopState.speeds.flipper.toFixed(2);
      flipperVel.addEventListener('input', (e) => {
        TeleopState.speeds.flipper = parseFloat(e.target.value);
        flipperVelVal.textContent = TeleopState.speeds.flipper.toFixed(2);
        localStorage.setItem('teleop_speeds', JSON.stringify(TeleopState.speeds));
      });
    }

    const flipperAccel = document.getElementById('flipper-accel');
    const flipperAccelVal = document.getElementById('flipper-accel-val');
    if (flipperAccel && flipperAccelVal) {
      flipperAccel.value = TeleopState.speeds.flipperAccel;
      flipperAccelVal.textContent = TeleopState.speeds.flipperAccel.toFixed(2);
      flipperAccel.addEventListener('input', (e) => {
        TeleopState.speeds.flipperAccel = parseFloat(e.target.value);
        flipperAccelVal.textContent = TeleopState.speeds.flipperAccel.toFixed(2);
        localStorage.setItem('teleop_speeds', JSON.stringify(TeleopState.speeds));
      });
    }

    const dcMotorVel = document.getElementById('dc-motor-vel');
    const dcMotorVelVal = document.getElementById('dc-motor-vel-val');
    if (dcMotorVel && dcMotorVelVal) {
      dcMotorVel.value = TeleopState.speeds.base;
      dcMotorVelVal.textContent = TeleopState.speeds.base.toFixed(2);
      dcMotorVel.addEventListener('input', (e) => {
        TeleopState.speeds.base = parseFloat(e.target.value);
        dcMotorVelVal.textContent = TeleopState.speeds.base.toFixed(2);
        localStorage.setItem('teleop_speeds', JSON.stringify(TeleopState.speeds));
      });
    }

    const armVel = document.getElementById('arm-vel');
    const armVelVal = document.getElementById('arm-vel-val');
    if (armVel && armVelVal) {
      armVel.value = TeleopState.speeds.arm;
      armVelVal.textContent = TeleopState.speeds.arm.toFixed(2);
      armVel.addEventListener('input', (e) => {
        TeleopState.speeds.arm = parseFloat(e.target.value);
        armVelVal.textContent = TeleopState.speeds.arm.toFixed(2);
        localStorage.setItem('teleop_speeds', JSON.stringify(TeleopState.speeds));
      });
    }
  }

  const startButton = document.getElementById('toggle-teleoperation');
  if (startButton) {
    startButton.addEventListener('click', () => {
      TeleopState.active = !TeleopState.active;
      
      const btnText = document.getElementById('teleoperation-btn-text');
      if (btnText) {
        btnText.textContent = TeleopState.active ? 'Stop Teleoperation' : 'Start Teleoperation';
      }
      
      log('Teleoperation ' + (TeleopState.active ? 'ENABLED' : 'DISABLED'));
      
      if (!TeleopState.active && robotAPI) { 
        inputHandler.executeAction('stop_dc', 'pressed');
        inputHandler.executeAction('stop_arm', 'pressed');
        inputHandler.executeAction('stop_flipper', 'pressed');
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
    
    if (inputHandler.guiActionMap && inputHandler.guiActionMap[actionId]) {
      return inputHandler.trigger_gui_action(actionId);
    }
    return inputHandler.executeAction(actionId, 'pressed');
  };

  window.toggleServerMic = () => {
    const button = document.getElementById('toggle-speaker');
    if (!button) return;
    
    // Check if currently unmuted
    // Instead of relying on specific english text like 'Unmute Server', we track the state using a custom attribute or assume based on color.
    const isMuted = button.getAttribute('data-muted') !== 'false'; // Defaults to true initially
    const actionId = isMuted ? 'unmute_server' : 'mute_server';
    
    const result = inputHandler ? inputHandler.trigger_gui_action(actionId) : { success: false, error: 'InputHandler init' };
    
    if (result.success) {
      const willBeMuted = !isMuted;
      button.setAttribute('data-muted', willBeMuted.toString());
      
      // Retain "Servidor" logic but append status if needed, or simply change color and icon
      button.style.backgroundColor = willBeMuted ? '#dc3545' : '#22c55e'; // Red if muted, Green if unmuted
      log(`Server microphone ${willBeMuted ? 'muted' : 'unmuted'}`);
    } else {
      log(`Failed to toggle server mic: ${result.error || 'unknown error'}`);
    }
  };
  
  window.getFlipperState = () => ({
    velocities: TeleopState.flippers.velocities.slice(),
    directions: TeleopState.flippers.directions.slice(), 
    direction: TeleopState.flippers.globalDirection,
    joints: FLIPPER_JOINTS
  });
  window.flipperState = TeleopState.flippers;

  window.addEventListener('beforeunload', () => {
    if (inputHandler) {
      inputHandler.executeAction('stop_dc', 'pressed');
      inputHandler.executeAction('stop_arm', 'pressed');
      inputHandler.executeAction('stop_flipper', 'pressed');
    }
    if (robotAPI) robotAPI.dispose();
    if (inputHandler) inputHandler.dispose();
  });

  log('Initializing application...');
  initializeROS();
})();