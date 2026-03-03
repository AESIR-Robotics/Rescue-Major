# Sistema de Transmisión de Inputs - com_client.js

## 📋 Índice
1. [Resumen General](#resumen-general)
2. [Recepción de Inputs](#recepción-de-inputs)
3. [Sistema de Mapeo Input → Mensaje ROS](#sistema-de-mapeo-input--mensaje-ros)
4. [Adaptación para Publishers y Services](#adaptación-para-publishers-y-services)
5. [Guía: Agregar un Nuevo Mensaje](#guía-agregar-un-nuevo-mensaje)

---

## Resumen General

El sistema de transmisión de inputs en `com_client.js` implementa un flujo completo desde la captura de inputs del usuario (teclado y gamepad) hasta la comunicación con nodos ROS a través de ROSBridge. El sistema está dividido en tres componentes principales:

1. **RobotAPI**: Encapsula la comunicación con ROS (topics y services)
2. **InputHandler**: Gestiona la captura de inputs del teclado y gamepad
3. **Configuración externa**: Un archivo JSON (`keys_map_keyboard.json`) que mapea inputs a acciones ROS

---

## Recepción de Inputs

### 1. Teclado

La clase `InputHandler` captura inputs de teclado usando event listeners nativos del DOM:

```javascript
_setupKeyboard() {
  this._onKeyDown = (e) => { 
    const k = (e.key || '').toLowerCase(); 
    this.keys.add(k);  // Almacena tecla presionada
  };
  this._onKeyUp = (e) => { 
    const k = (e.key || '').toLowerCase(); 
    this.keys.delete(k);  // Elimina tecla cuando se suelta
  };
  window.addEventListener('keydown', this._onKeyDown);
  window.addEventListener('keyup', this._onKeyUp);
}
```

**Características:**
- Convierte todas las teclas a minúsculas para normalización
- Mantiene un `Set` de teclas actualmente presionadas en `this.keys`
- El evento `keydown` dispara la ejecución de acciones

### 2. Gamepad (Joystick/Controller)

El sistema detecta y gestiona gamepads conectados:

```javascript
_setupGamepadEvents() {
  this._onGamepadConnected = (e) => {
    this.gamepadIndex = e.gamepad.index;
    this._emit('gamepadConnected', e.gamepad.index);
  };
  this._onGamepadDisconnected = (e) => {
    if (this.gamepadIndex === e.gamepad.index) 
      this.gamepadIndex = null;
    this._emit('gamepadDisconnected', e.gamepad.index);
  };
  window.addEventListener('gamepadconnected', this._onGamepadConnected);
  window.addEventListener('gamepaddisconnected', this._onGamepadDisconnected);
}
```

**Características:**
- Detecta conexión/desconexión automática de gamepads
- Monitorea continuamente los ejes y botones mediante polling
- Aplica deadzone para evitar ruido en analógicos

### 3. Polling del Gamepad

El sistema utiliza `requestAnimationFrame` para leer periódicamente el estado del gamepad:

```javascript
_pollGamepads() {
  const gps = navigator.getGamepads ? navigator.getGamepads() : [];
  for (let i = 0; i < gps.length; i++) {
    const gp = gps[i];
    if (!gp) continue;
    
    // Detecta cambios de botones (presión/liberación)
    gp.buttons.forEach((b, idx) => {
      const was = !!this.prevButtonsByPad[id][idx];
      const now = !!b.pressed;
      if (now && !was) {
        this._emit('buttonDown', i, idx);
      } else if (!now && was) {
        this._emit('buttonUp', i, idx);
      }
    });
    
    // Lee los ejes del stick izquierdo con deadzone
    const x = this._applyDeadzone(gp.axes[0] || 0);
    const y = this._applyDeadzone(gp.axes[1] || 0);
    this._emit('axis', { pad: i, x, y, active: x !== 0 || y !== 0 });
  }
}
```

**Características:**
- Detecta cambios de estado (presión/liberación de botones)
- Lee ejes analógicos con aplicación automática de deadzone
- Emite eventos que pueden ser escuchados por otros componentes

### 4. Deadzone (Rango Muerto)

El sistema aplicaA una zona muerta para analógicos para evitar ruido:

```javascript
_applyDeadzone(v) {
  if (Math.abs(v) < this.deadzone) return 0;  // Valor por defecto: 0.12
  return Math.max(-1, Math.min(1, v));
}
```

Cualquier valor menor a 0.12 se convierte en 0, normalizando el rango de valores.

---

## Sistema de Mapeo Input → Mensaje ROS

### 1. Arquitectura del Mapeo

El mapeo se realiza en dos niveles:

```
┌─────────────────────────────────────────────────────────────┐
│ ENTRADA: Teclado / Gamepad                                  │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
         ┌─────────────────────┐
         │  InputHandler       │
         │  .executeAction()   │
         └──────────┬──────────┘
                    │ Lee configuración del JSON
                    ▼
         ┌─────────────────────────────────────────┐
         │  Mapeo JSON (keys_map_keyboard.json)    │
         │  {                                      │
         │    "r": {                              │
         │      type: "send_service",             │
         │      data_type: "vision/srv/Command",  │
         │      topics: [...],                    │
         │      payload: {...}                    │
         │    }                                   │
         │  }                                     │
         └──────────┬──────────────────────────────┘
                    │
                    ▼
         ┌─────────────────────┐
         │  RobotAPI           │
         │  .sendTopic()       │
         │  .sendService()     │
         └──────────┬──────────┘
                    │
                    ▼
         ┌─────────────────────┐
         │  ROSBridge (ws://)  │
         └──────────┬──────────┘
                    │
                    ▼
         ┌─────────────────────┐
         │  Nodos ROS / Topics │
         │  / Services         │
         └─────────────────────┘
```

### 2. Estructura de Configuración JSON

Cada entrada en `keys_map_keyboard.json` contiene:

```json
{
  "KEY_ID": {
    "type": "send_topic|send_service|local_function",
    "data_type": "std_msgs/Float32MultiArray",
    "topics": ["/topic1", "/topic2"],
    "payload": {
      "field1": value1,
      "field2": value2
    }
  }
}
```

**Campos obligatorios según tipo:**

| Campo | Requerido | Descripción |
|-------|-----------|-------------|
| `type` | Sí | Tipo de acción: `send_topic`, `send_service`, `local_function` |
| `data_type` | Solo send_topic, send_service | Tipo de mensaje ROS (ej: `std_msgs/Float32MultiArray`) |
| `topics` | Solo send_topic, send_service | Array de nombres de topics/services destino |
| `payload` | Sí | Datos a enviar o parámetros de la función |

### 3. Proceso de Mapeo

**Paso 1: Cargar configuración**

```javascript
// En el evento 'connection' de ROSBridge
fetch('static/keys_map_keyboard.json')
  .then(r => r.json())
  .then(config => {
    inputHandler.loadActions(config);
    // Ahora inputHandler.actionMap = config
  });
```

**Paso 2: InputHandler almacena el mapeo**

```javascript
loadActions(configJson) {
  Object.entries(configJson).forEach(([keyId, action]) => {
    // Validación de campos
    // ...
    this.actionMap[keyId] = { type, data_type, topics, payload };
  });
}
```

**Paso 3: Ejecutar acción en cada keydown**

```javascript
window.addEventListener('keydown', (e) => {
  const key = e.key.toLowerCase();
  const result = inputHandler.executeAction(key);
  // El resultado contiene info sobre el éxito/fracaso
});
```

**Paso 4: executeAction distribuye según tipo**

```javascript
executeAction(keyId) {
  const action = this.actionMap[keyId];
  
  switch (action.type) {
    case 'send_topic':
      return this.robotAPI.sendTopic(
        action.topics, 
        action.data_type, 
        action.payload
      );
    case 'send_service':
      return this.robotAPI.sendService(
        action.topics, 
        action.data_type, 
        action.payload
      );
    case 'local_function':
      const func = this.localFunctions[action.payload.function_name];
      func(action.payload);
      break;
  }
}
```

### 4. Ejemplo Real: Tecla "R" (Cambiar Cámara)

Configuración JSON:
```json
"r": {
  "type": "send_service",
  "data_type": "vision/srv/Command",
  "topics": ["/command_vision_videoStream", "/web_rtc_commands"],
  "payload": {"data": "vision:camera,0"}
}
```

Flujo de ejecución:
1. Usuario presiona tecla "R"
2. Evento `keydown` captura la tecla y convierte a minúscula: `"r"`
3. `inputHandler.executeAction("r")` busca en `actionMap`
4. Encuentra que es un `send_service`
5. Llama a `robotAPI.sendService()` con los 2 topics
6. ROSBridge envía la solicitud de servicio a ambos topics
7. Los nodos ROS reciben la solicitud y ejecutan la acción

---

## Adaptación para Publishers y Services

### 1. Publishers (send_topic)

Los publishers envían mensajes a topics de forma **no-bloqueante**:

```javascript
sendTopic(topicNames, messageType, payload) {
  if (!Array.isArray(topicNames)) {
    topicNames = [topicNames];
  }

  const results = [];
  topicNames.forEach(topicName => {
    try {
      // Obtiene o crea el topic
      const topic = this.getOrCreateTopic(topicName, messageType);
      
      // Crea el mensaje con el payload
      const message = new ROSLIB.Message(payload);
      
      // Publica el mensaje
      topic.publish(message);
      
      results.push({ success: true, target: topicName });
    } catch (error) {
      results.push({ success: false, target: topicName, error: error.message });
    }
  });

  return results;
}
```

**Características:**
- Puede enviar a múltiples topics simultáneamente
- Crea topics bajo demanda y los cachea para reutilización
- Retorna información de éxito/fracaso para cada topic

**Ejemplo de payload:**
```javascript
// Para std_msgs/Float32MultiArray
{ data: [0.5, -0.3] }

// Para geometry_msgs/Twist
{ linear: { x: 0.5, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0.2 } }
```

### 2. Services (send_service)

Los services envían solicitudes que **esperan respuesta**:

```javascript
sendService(serviceNames, serviceType, payload) {
  if (!Array.isArray(serviceNames)) {
    serviceNames = [serviceNames];
  }

  const results = [];
  serviceNames.forEach(serviceName => {
    try {
      // Obtiene o crea el servicio
      const service = this.getOrCreateService(serviceName, serviceType);
      
      // Crea la solicitud
      const request = new ROSLIB.ServiceRequest(payload);
      
      // Llama al servicio con callback para la respuesta
      service.callService(request, (response) => {
        console.log(`Service ${serviceName} response:`, response);
      });
      
      results.push({ success: true, target: serviceName });
    } catch (error) {
      results.push({ success: false, target: serviceName, error: error.message });
    }
  });

  return results;
}
```

**Características:**
- Soporta múltiples servicios simultáneamente
- Cachea servicios para reutilización
- Proporciona callbacks para manejar respuestas
- Bloquea hasta recibir respuesta del nodo ROS

**Ejemplo de payload:**
```javascript
// Para vision/srv/Command
{ data: "vision:camera,0" }

// Para custom/srv/Instruction
{ id: 42, command: "move", params: "forward" }
```

### 3. Caching de Recursos

Ambas clases (Topic y Service) se cachean para evitar recrearlas:

```javascript
getOrCreateTopic(topicName, messageType) {
  if (this.topicCache[topicName]) {
    return this.topicCache[topicName];  // Reutiliza existente
  }
  
  const topic = new ROSLIB.Topic({...});
  this.topicCache[topicName] = topic;
  return topic;
}
```

Ventajas:
- Mejora rendimiento
- Reduce latencia en envíos consecutivos
- Mantiene la suscripción activa

---

## Guía: Agregar un Nuevo Mensaje

Esta sección te guía paso a paso para agregar soporte a un nuevo input que dispare un mensaje ROS.

### Escenario: Agregar tecla "L" para activar LED

Supongamos que queremos:
- **Tecla:** `L`
- **Acción:** Enviar comando a service `/control_led` 
- **Tipo de servicio:** `hardware/srv/LedCommand`
- **Payload:** `{action: "on", color: "red"}`

---

### Opción 1: Agregar a Publisher (Topic)

**Paso 1: Actualizar JSON**

Edita `keys_map_keyboard.json` y agrega:

```json
{
  "l": {
    "type": "send_topic",
    "data_type": "std_msgs/String",
    "topics": ["/led_control"],
    "payload": {"data": "led:on:red"}
  }
}
```

**Paso 2: Verificar que el nodo ROS está escuchando**

En tu nodo ROS debe haber un suscriptor:
```python
# En tu nodo ROS (Python ejemplo)
import rospy
from std_msgs.msg import String

def led_callback(msg):
    print(f"Received: {msg.data}")
    if msg.data == "led:on:red":
        activate_led("red")

rospy.Subscriber("/led_control", String, led_callback)
```

---

### Opción 2: Agregar a Service

**Paso 1: Crear el tipo de servicio en ROS**

En tu paquete ROS, crea `srv/LedCommand.srv`:
```
string action
string color
---
bool success
string message
```

**Paso 2: Actualizar JSON**

```json
{
  "l": {
    "type": "send_service",
    "data_type": "hardware/srv/LedCommand",
    "topics": ["/control_led"],
    "payload": {"action": "on", "color": "red"}
  }
}
```

**Paso 3: Nodo ROS con servicio**

```python
import rospy
from hardware.srv import LedCommand, LedCommandResponse

def handle_led_command(req):
    print(f"Action: {req.action}, Color: {req.color}")
    if req.action == "on":
        activate_led(req.color)
    return LedCommandResponse(success=True, message="LED activated")

rospy.Service('/control_led', LedCommand, handle_led_command)
```

---

### Opción 3: Múltiples Targets (Difusión)

Para enviar el mismo mensaje a múltiples nodos:

```json
{
  "m": {
    "type": "send_service",
    "data_type": "vision/srv/Command",
    "topics": ["/command_vision_sensors", "/command_vision_videoStream"],
    "payload": {"data": "mode:thermal"}
  }
}
```

**Resultado:** El mensaje se envía a AMBOS topics/services simultáneamente.

---

### Opción 4: Control Continuo (Joystick)

Para control continuo basado en joystick (ej: motores), NO uses la config JSON. En su lugar, modifica `publishJoystickVelocities()`:

**Actual:**
```javascript
function publishJoystickVelocities() {
  if (!motorTopic || !inputHandler) return;
  
  const { x, y } = inputHandler.readJoystick();
  const message = new ROSLIB.Message({
    data: [Number(x) || 0.0, Number(y) || 0.0]
  });
  
  motorTopic.publish(message);
}
```

**Para múltiples ejes (ejemplo: drone con 4 motores):**
```javascript
function publishDroneVelocities() {
  if (!droneMotorTopic || !inputHandler) return;
  
  const { x, y } = inputHandler.readJoystick();
  // x = roll, y = pitch (proporcionales)
  // Podrías agregar más parámetros aquí
  
  const message = new ROSLIB.Message({
    data: [x, y, 0.0, 0.0]  // 4 motores
  });
  
  droneMotorTopic.publish(message);
}
```

---

### Opción 5: Función Local (Sin ROS)

Para acciones que NO requieren ROS (solo interfaz local):

**JSON:**
```json
{
  "p": {
    "type": "local_function",
    "payload": {"function_name": "togglePanel"}
  }
}
```

**Registro en com_client.js:**
```javascript
// Después de crear inputHandler
inputHandler.registerLocalFunction('togglePanel', (payload) => {
  const panel = document.getElementById('control-panel');
  panel.style.display = panel.style.display === 'none' ? 'block' : 'none';
  log('Panel toggled');
});
```

---

### Checklist para Agregar un Nuevo Mensaje

- [ ] **Identificar el tipo:** ¿Topic o Service?
- [ ] **Crear/verificar el tipo en ROS:** 
  - Para topics: `std_msgs/*` o custom
  - Para services: Crear archivo `.srv`
- [ ] **Definir el payload:** Estructura de datos exacta
- [ ] **Editar `keys_map_keyboard.json`:** Agregar entrada con:
  - Identificador único (tecla o button)
  - `type`: `send_topic`, `send_service`, o `local_function`
  - `data_type`: Nombre completo del tipo ROS
  - `topics`: Array de destinos
  - `payload`: Datos a enviar
- [ ] **Compilar y deployar** el paquete ROS
- [ ] **Recargalr la página** web para cargar nuevamente el JSON
- [ ] **Probar:** Presionar la tecla y verificar logs en ROS

---

### Ejemplo Completo: Agregar tecla "D" para modo Debug

**1. JSON (`keys_map_keyboard.json`):**
```json
{
  "d": {
    "type": "send_service",
    "data_type": "hardware/srv/DebugMode",
    "topics": ["/hardware_controller"],
    "payload": {"enabled": true, "level": 2}
  }
}
```

**2. Tipo ROS (`hardware/srv/DebugMode.srv`):**
```
bool enabled
int32 level
---
bool acknowledged
```

**3. Nodo ROS receptor:**
```python
import rospy
from hardware.srv import DebugMode, DebugModeResponse

def debug_callback(req):
    rospy.loginfo(f"Debug mode: enabled={req.enabled}, level={req.level}")
    # Lógica de debug aquí
    return DebugModeResponse(acknowledged=True)

rospy.Service('/hardware_controller', DebugMode, debug_callback)
rospy.spin()
```

**4. Resultado:**
- Presionar "D" en la interfaz web
- Se envía solicitud de servicio a `/hardware_controller`
- Nodo ROS recibe y procesa
- Respuesta aparece en console del navegador

---

## Referencia Rápida

### Variables Globales Expuestas

```javascript
// Ejecutar acción manualmente
window.executeAction(actionId)  // Retorna {success, ...}

// Toggle micrófono servidor
window.toggleServerMic()

// Leer estado del joystick
inputHandler.readJoystick()  // Retorna {x, y, active}
```

### Estructura de Respuesta de executeAction

```javascript
{
  success: true|false,
  type: "send_topic|send_service|local_function",
  count: N,  // Número de targets alcanzados
  results: [{
    success: true|false,
    target: "nombre",
    error: "mensaje"
  }],
  error: "descripción"  // Si success = false
}
```

### Métodos Útiles de InputHandler

```javascript
// Cargar configuración
inputHandler.loadActions(configObject)

// Registrar función local
inputHandler.registerLocalFunction(name, callback)

// Ejecutar acción
inputHandler.executeAction(keyId)

// Leer joystick
inputHandler.readJoystick()

// Escuchar eventos
inputHandler.on('buttonDown', (padIndex, buttonIndex) => {})
inputHandler.on('buttonUp', (padIndex, buttonIndex) => {})
inputHandler.on('axis', (data) => {})
inputHandler.on('gamepadConnected', (padIndex) => {})
inputHandler.on('gamepadDisconnected', (padIndex) => {})
```

### Métodos Útiles de RobotAPI

```javascript
// Obtener o crear topic
robotAPI.getOrCreateTopic(topicName, messageType)

// Obtener o crear service
robotAPI.getOrCreateService(serviceName, serviceType)

// Enviar a topic(s)
robotAPI.sendTopic(topicNames, messageType, payload)

// Enviar a service(s)
robotAPI.sendService(serviceNames, serviceType, payload)

// Obtener estadísticas
robotAPI.getStats()  // {topicsCached, servicesCached}

// Limpiar recursos
robotAPI.dispose()
```

---

## Resumen

El sistema `com_client.js` proporciona un **pipeline eficiente y flexible** para transmitir inputs desde la interfaz web hasta los nodos ROS:

1. **Captura:** Teclado y Gamepad eventos → `InputHandler`
2. **Mapeo:** JSON config → Acciones específicas
3. **Envío:** `RobotAPI` → ROSBridge → Nodos ROS
4. **Respuesta:** Services retornan datos → Callbacks en JS

Todo es **configurable externamente** mediante JSON, permitiendo agregar nuevos inputs sin modificar código JavaScript. El sistema soporta tanto **control discreto** (servicios) como **control continuo** (topics en bucle).