class InputHandler {
  constructor({ deadzone = 0.12 } = {}) {
    this.deadzone = deadzone;
    this.keys = new Set();
    this.gamepadIndex = null;
    this._setupKeyboard();
    this._setupGamepadEvents();
  }

  _setupKeyboard() {
    window.addEventListener('keydown', (e) => {
      const k = e.key.toLowerCase();
      this.keys.add(k);
    });
    window.addEventListener('keyup', (e) => {
      const k = e.key.toLowerCase();
      this.keys.delete(k);
    });
  }

  _setupGamepadEvents() {
    window.addEventListener('gamepadconnected', (e) => {
      this.gamepadIndex = e.gamepad.index;
    });
    window.addEventListener('gamepaddisconnected', (e) => {
      if (this.gamepadIndex === e.gamepad.index) this.gamepadIndex = null;
    });
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

  _readKeyboard() {
    // WASD or arrow keys
    let x = 0, y = 0;
    if (this.keys.has('a') || this.keys.has('arrowleft')) x -= 1;
    if (this.keys.has('d') || this.keys.has('arrowright')) x += 1;
    if (this.keys.has('w') || this.keys.has('arrowup')) y -= 1;
    if (this.keys.has('s') || this.keys.has('arrowdown')) y += 1;
    // Normalize diagonal to length 1
    if (x !== 0 && y !== 0) {
      const inv = 1 / Math.sqrt(2);
      x *= inv; y *= inv;
    }
    const active = x !== 0 || y !== 0;
    return { x, y, active };
  }

  read() {
    // Read both, prioritize gamepad when it gives non-zero values
    const gp = this._readGamepad();
    const kb = this._readKeyboard();
    if (gp.active) return { x: gp.x, y: gp.y };
    return { x: kb.x, y: kb.y };
  }
}

class RobotComms {
  constructor({ rosbridgeHost = location.hostname, rosbridgePort = 9090, onInfoMessage = null } = {}) {
    this.rosbridgeHost = rosbridgeHost;
    this.rosbridgePort = rosbridgePort;
    this.onInfoMessage = onInfoMessage;
    this.ros = null;
    this.publisher = null; // /dc_motors
    this.visionParam = null; // /vision_camera_modes
    this.webrtcService = null; // /web_rtc_commands
    this.infoSub = null; // /client_info_dummy
    this.connected = false;
    this._connect();
  }

  _connect() {
    if (typeof ROSLIB === 'undefined') {
      console.error('ROSLIB not found. Please include roslib.js in the page.');
      return;
    }
    const url = `ws://${this.rosbridgeHost}:${this.rosbridgePort}`;
    this.ros = new ROSLIB.Ros({ url });
    this.ros.on('connection', () => {
      console.info('Connected to rosbridge at', url);
      this.connected = true;
      this._setupTopicsAndServices();
    });
    this.ros.on('close', () => {
      console.warn('Connection to rosbridge closed');
      this.connected = false;
      setTimeout(() => this._connect(), 2000);
    });
    this.ros.on('error', (err) => {
      console.error('rosbridge error', err);
    });
  }

  _setupTopicsAndServices() {
    // Publisher for dc motors: std_msgs/Float32MultiArray
    this.publisher = new ROSLIB.Topic({
      ros: this.ros,
      name: '/dc_motors',
      messageType: 'std_msgs/Float32MultiArray',
      queue_size: 1,
    });

    // Vision-specific parameter for camera modes
    this.visionParam = new ROSLIB.Param({ ros: this.ros, name: '/vision_camera_modes' });

    // Publisher for WebRTC commands (server expects a topic String)
    this.webrtcPub = new ROSLIB.Topic({ ros: this.ros, name: '/web_rtc_commands', messageType: 'std_msgs/String' });

    // Subscriber for info
    this.infoSub = new ROSLIB.Topic({ ros: this.ros, name: '/client_info_dummy', messageType: 'std_msgs/String' });
    this.infoSub.subscribe((msg) => {
      if (this.onInfoMessage) this.onInfoMessage(msg.data);
      else console.info('client_info_dummy:', msg.data);
    });

    // Load keyboard and controller keymaps (files must be served alongside GUI)
    this.keymapKeyboard = {};
    this.keymapController = {};
    this.bindingsKeyboard = {}; // precreated bindings for keyboard
    this.bindingsController = {}; // precreated bindings for controller
    this.topicPublishers = {};
    this.servicesSet = new Set();
    this.params = {};
    this.services = {};
    this._loadKeymaps();
  }

  _loadKeymaps() {
    // Only use the canonical static paths for the keymaps
    const keyboardPath = 'static/keys_map_keyboard.json';
    const controllerPath = '/static/keys_map_controller.json';

    const pKeyboard = fetch(keyboardPath).then(r => { if (!r.ok) throw new Error('not found'); return r.json(); }).catch(e => { console.warn('Keyboard keymap not loaded', e); return null; });
    const pController = fetch(controllerPath).then(r => { if (!r.ok) throw new Error('not found'); return r.json(); }).catch(e => { console.warn('Controller keymap not loaded', e); return null; });

    Promise.all([pKeyboard, pController]).then(([kb, ctl]) => {
      // Use keymaps provided by the server/static JSON only.
      // Do NOT fall back to hardcoded mappings here; if no JSON is served,
      // the keymaps will be empty and no mapped-key actions will be available.
      this.keymapKeyboard = kb || {};
      this.keymapController = ctl || {};
      console.info('Keymaps loaded (keyboard/controller)', Object.keys(this.keymapKeyboard).length, Object.keys(this.keymapController).length);
      // Prepare publishers/services/params based on entries (precreate instances)
      this._prepareBindings();
      try { if (this.onInfoMessage) this.onInfoMessage(`Keymaps loaded: keyboard=${Object.keys(this.keymapKeyboard).length}, controller=${Object.keys(this.keymapController).length}`); } catch (e) {}
    }).catch((e) => { console.warn('Error loading keymaps', e); this.keymapKeyboard = {}; this.keymapController = {}; });
  }

  _prepareBindings() {
    // Helper to create instance for an entry: mode,target,value
    const createFor = (key, entry, sourceBindings) => {
      if (!Array.isArray(entry) || entry.length < 2) return;
      const mode = entry[0];
      const target = entry[1];
      const value = entry.length > 2 ? entry[2] : undefined;

      if (mode === 'topic') {
        // create publisher based on known targets or inferred type
        let msgType = 'std_msgs/String';
        if (target === '/dc_motors') msgType = 'std_msgs/Float32MultiArray';
        else if (Array.isArray(value)) msgType = 'std_msgs/Float32MultiArray';
        const pub = new ROSLIB.Topic({ ros: this.ros, name: target, messageType: msgType });
        this.topicPublishers[target] = pub;
        sourceBindings[key] = { mode: 'topic', target, value, instance: pub, msgType };
      } else if (mode === 'service') {
        const svc = new ROSLIB.Service({ ros: this.ros, name: target, serviceType: 'std_srvs/Trigger' });
        this.services[target] = svc;
        sourceBindings[key] = { mode: 'service', target, value, instance: svc };
      } else if (mode === 'param') {
        const param = new ROSLIB.Param({ ros: this.ros, name: target });
        this.params[target] = param;
        sourceBindings[key] = { mode: 'param', target, value, instance: param };
      } else {
        // unknown mode: store raw for fallback
        sourceBindings[key] = { mode: 'unknown', target, value };
      }
    };

    // Prepare keyboard bindings
    Object.keys(this.keymapKeyboard).forEach((k) => createFor(k, this.keymapKeyboard[k], this.bindingsKeyboard));
    // Prepare controller bindings
    Object.keys(this.keymapController).forEach((k) => createFor(k, this.keymapController[k], this.bindingsController));

    console.info('Prepared bindings', Object.keys(this.bindingsKeyboard).length, Object.keys(this.bindingsController).length);
    try {
      if (this.onInfoMessage) this.onInfoMessage(`Prepared bindings: keyboard=[${Object.keys(this.bindingsKeyboard).join(', ')}] controller=[${Object.keys(this.bindingsController).join(', ')}]`);
    } catch (e) {}
  }

  // Send a mapped key. JSON format: key: [ target_string, value ]
  // target_string must be explicit like '/dc_motors', '/web_rtc_commands', '/vision_camera_modes', etc.
  sendMappedKey(key, source = 'keyboard') {
    if (!this.connected) return { ok: false, reason: 'rosbridge not connected' };
    const bindings = source === 'controller' ? this.bindingsController : this.bindingsKeyboard;
    let b = bindings && bindings[key];
    if (!b) {
      // try alternate map
      const alt = source === 'controller' ? this.bindingsKeyboard : this.bindingsController;
      b = alt && alt[key];
    }
    if (!b) return { ok: false, reason: 'no binding' };

    const mode = b.mode;
    const target = b.target;
    const value = b.value;

    try {
      if (mode === 'topic') {
        const pub = b.instance; // ROSLIB.Topic
        if (!pub) return { ok: false, reason: 'publisher missing' };
        let msgObj;
        if (b.msgType === 'std_msgs/Float32MultiArray') {
          msgObj = { data: (Array.isArray(value) ? value : [value]).map(v => Number(v) || 0.0) };
        } else if (b.msgType === 'std_msgs/Float32') {
          msgObj = { data: Number(value) };
        } else if (b.msgType === 'std_msgs/Bool') {
          msgObj = { data: !!value };
        } else {
          msgObj = { data: String(value) };
        }
        pub.publish(new ROSLIB.Message(msgObj));
        return { ok: true, mode: 'topic' };
      }

      if (mode === 'service') {
        const svc = b.instance; // ROSLIB.Service
        if (!svc) return { ok: false, reason: 'service client missing' };
        const req = (value === null || value === undefined) ? {} : (typeof value === 'object' ? value : { data: value });
        svc.callService(new ROSLIB.ServiceRequest(req), (res) => console.info('service result', res));
        return { ok: true, mode: 'service' };
      }

      if (mode === 'param') {
        const param = b.instance; // ROSLIB.Param
        if (!param) return { ok: false, reason: 'param client missing' };
        if (value !== null && value !== undefined) {
          param.set(value);
          return { ok: true, mode: 'param_set' };
        } else {
          param.get((v) => console.info('param value', v));
          return { ok: true, mode: 'param_get' };
        }
      }

      return { ok: false, reason: 'unknown mode' };
    } catch (e) {
      console.error('sendMappedKey error', e);
      return { ok: false, reason: e.message };
    }
  }

  publishMotors(x, y) {
    if (!this.publisher || !this.connected) return false;
    const arr = new Float32Array([Number(x) || 0.0, Number(y) || 0.0]);
    const msg = new ROSLIB.Message({ data: Array.from(arr) });
    try {
      this.publisher.publish(msg);
      return true;
    } catch (e) {
      console.error('publish error', e);
      return false;
    }
  }

  // Example helper to set/get vision camera modes parameter
  getVisionCameraModes(callback) {
    if (!this.visionParam) { callback(null); return; }
    this.visionParam.get((value) => callback(value));
  }

  callWebRTCService(args, callback) {
    // Backwards-compatible: try calling a service if present, otherwise publish on the topic.
    if (this.webrtcService) {
      const request = new ROSLIB.ServiceRequest(args || {});
      this.webrtcService.callService(request, (result) => {
        callback && callback(result);
      });
      return;
    }

    if (this.webrtcPub) {
      try {
        const cmd = (args && args.command) ? String(args.command) : String(args || '');
        const msg = new ROSLIB.Message({ data: cmd });
        this.webrtcPub.publish(msg);
        callback && callback({ success: true, message: 'published' });
      } catch (e) {
        callback && callback({ success: false, message: String(e) });
      }
      return;
    }

    callback && callback({ success: false, message: 'webrtc interface unavailable' });
  }

  // Convenience helper to publish a WebRTC command string
  sendWebRTCCommand(command) {
    if (!this.webrtcPub || !this.connected) return false;
    try {
      const msg = new ROSLIB.Message({ data: String(command) });
      this.webrtcPub.publish(msg);
      return true;
    } catch (e) {
      console.error('webrtc publish error', e);
      return false;
    }
  }

  dispose() {
    try {
      if (this.infoSub) this.infoSub.unsubscribe();
    } catch (e) {}
    try {
      if (this.ros) this.ros.close();
    } catch (e) {}
  }
}

// Main control wiring
(function main() {
  const infoMessages = document.getElementById('info-messages');
  function addMessage(s) {
    if (!infoMessages) return console.log(s);
    infoMessages.textContent += s + '\n';
    infoMessages.scrollTop = infoMessages.scrollHeight;
  }

  const input = new InputHandler();
  const comms = new RobotComms({ onInfoMessage: (d) => addMessage('ROS INFO: ' + d) });

  // Show connection and bindings diagnostics in the frontend log once available
  (function showBindings() {
    const maxAttempts = 25; // ~5 seconds at 200ms
    let attempts = 0;
    const iv = setInterval(() => {
      attempts += 1;
      if (comms.bindingsKeyboard && Object.keys(comms.bindingsKeyboard).length > 0) {
        addMessage(`ROS connected: ${comms.connected}`);
        addMessage(`Keyboard bindings: ${Object.keys(comms.bindingsKeyboard).join(', ')}`);
        addMessage(`Controller bindings: ${Object.keys(comms.bindingsController || {}).join(', ')}`);
        clearInterval(iv);
      } else if (attempts >= maxAttempts) {
        addMessage(`Bindings not ready after ${attempts} attempts. ROS connected: ${comms.connected}`);
        clearInterval(iv);
      }
    }, 200);
  })();

  // Quick debug toggle: add `?input_debug=1` to the GUI URL to log raw key events
  const INPUT_DEBUG = (new URLSearchParams(location.search).get('input_debug') === '1');

  // Keyboard mappings driven by preloaded keymap
  window.addEventListener('keydown', (e) => {
    const raw = (e.key || '');
    const key = raw.toLowerCase();
    if (INPUT_DEBUG) console.info('DEBUG keydown:', { raw, key, bindings: comms.bindingsKeyboard && Object.keys(comms.bindingsKeyboard) });
    // Show detected key in frontend log
    const displayKey = (raw === ' ') ? 'SPACE' : raw;
    addMessage(`Detected key: ${displayKey}`);

    // Accept both explicit single-space and the word 'space' in the JSON
    const spaceKeys = [' ', 'space', 'spacebar'];

    // Only attempt mapped-key send if a binding exists
    const bindings = comms.bindingsKeyboard || {};
    const hasDirect = !!bindings[key];
    const hasSpace = spaceKeys.includes(key) && (bindings[' '] || bindings['space']);
    if (!hasDirect && !hasSpace) return; // nothing to send

    // Determine bindingKey used in bindings map
    let bindingKey = key;
    if (!hasDirect && hasSpace) bindingKey = (bindings[' '] ? ' ' : 'space');

    // Send and log the command sent
    const res = comms.sendMappedKey(bindingKey, 'keyboard');
    if (res && res.ok) {
      const b = (comms.bindingsKeyboard && comms.bindingsKeyboard[bindingKey]) || {};
      addMessage(`Command sent: ${b.mode || 'unknown'} ${b.target || ''} ${JSON.stringify(b.value !== undefined ? b.value : '')}`);
    } else if (res && res.reason) {
      addMessage(`Key ${bindingKey} mapped but failed: ${res.reason}`);
    }
  });

  // Controller polling for button presses (uses controller bindings)
  let prevButtonsByPad = {};
  function pollGamepads() {
    const gps = navigator.getGamepads ? navigator.getGamepads() : [];
    for (let i = 0; i < gps.length; i++) {
      const gp = gps[i];
      if (!gp) continue;
      const id = String(i);
      if (!prevButtonsByPad[id]) prevButtonsByPad[id] = [];
      gp.buttons.forEach((b, idx) => {
        const was = !!prevButtonsByPad[id][idx];
        const now = !!b.pressed;
        if (now && !was) {
          addMessage(`Detected controller button: ${idx}`);
          const res = comms.sendMappedKey(String(idx), 'controller');
          if (res && res.ok) {
            const b = (comms.bindingsController && comms.bindingsController[String(idx)]) || {};
            addMessage(`Command sent (controller): ${b.mode || 'unknown'} ${b.target || ''} ${JSON.stringify(b.value !== undefined ? b.value : '')}`);
          } else if (res && res.reason) addMessage(`Controller button ${idx} not mapped: ${res.reason}`);
        }
        prevButtonsByPad[id][idx] = now;
      });
    }
    requestAnimationFrame(pollGamepads);
  }
  requestAnimationFrame(pollGamepads);

  let isControlEnabled = false;
  let intervalId = null;

  const startButton = document.getElementById('start-btn') || document.getElementById('start-communication');
  if (startButton) {
    startButton.addEventListener('click', () => {
      isControlEnabled = !isControlEnabled;
      startButton.textContent = isControlEnabled ? 'Stop Control' : 'Start Control';
      addMessage('Control ' + (isControlEnabled ? 'ENABLED' : 'DISABLED'));
      if (!isControlEnabled) {
        // send immediate stop
        comms.publishMotors(0.0, 0.0);
        if (intervalId) { clearInterval(intervalId); intervalId = null; }
      } else {
        // start periodic publisher at 10Hz
        if (!intervalId) {
          intervalId = setInterval(() => {
            const { x, y } = input.read();
            // publish raw x,y as requested
            comms.publishMotors(x || 0.0, y || 0.0);
          }, 100);
        }
      }
    });
  } else {
    addMessage('Warning: start button not found (id start-btn or start-communication)');
  }

  // Ensure we always send heartbeat when enabled even if page unloads
  window.addEventListener('beforeunload', () => {
    try { comms.publishMotors(0.0, 0.0); } catch (e) {}
  });

})();

