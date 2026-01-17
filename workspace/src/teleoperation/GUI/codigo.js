// ===========================================
// VARIABLES GLOBALES PARA LAS VENTANAS FLOTANTES
// ===========================================

// Referencias a las ventanas de cámara
const cameraWindow1 = document.getElementById('camera-window-1');
const cameraWindow2 = document.getElementById('camera-window-2');
const cameraWindow3 = document.getElementById('camera-window-3');

// Estado de las ventanas de cámara
let cameraWindowsState = {
    1: { isOpen: false, alwaysOnTop: false, position: { left: 50, top: 50, width: 400, height: 300 } },
    2: { isOpen: false, alwaysOnTop: false, position: { left: 500, top: 50, width: 400, height: 300 } },
    3: { isOpen: false, alwaysOnTop: false, position: { left: 50, top: 400, width: 400, height: 300 } }
};

// Contador de índice Z (para controlar qué ventana está al frente)
let zIndexCounter = 1000;

// Estado "siempre al frente" para todas las cámaras
let allCamerasAlwaysOnTop = false;

// ===========================================
// FUNCIONES PARA LAS VENTANAS DE CÁMARA
// ===========================================

/**
 * Alterna (abre/cierra) una ventana de cámara específica
 * @param {number} cameraNumber - Número de la cámara (1, 2 o 3)
 */
function toggleCameraWindow(cameraNumber) {
    const windowElement = getCameraWindowElement(cameraNumber);
    const state = cameraWindowsState[cameraNumber];
    
    if (state.isOpen) {
        // CERRAR la ventana
        closeCameraWindow(cameraNumber);
    } else {
        // ABRIR la ventana
        openCameraWindow(cameraNumber);
    }
}

/**
 * Abre y configura una ventana de cámara
 * @param {number} cameraNumber - Número de la cámara (1, 2 o 3)
 */
function openCameraWindow(cameraNumber) {
    const windowElement = getCameraWindowElement(cameraNumber);
    const state = cameraWindowsState[cameraNumber];
    
    // Mostrar la ventana añadiendo la clase 'active'
    windowElement.classList.add('active');
    
    // Configurar posición y tamaño
    windowElement.style.left = state.position.left + 'px';
    windowElement.style.top = state.position.top + 'px';
    windowElement.style.width = state.position.width + 'px';
    windowElement.style.height = state.position.height + 'px';
    
    // Aplicar "siempre al frente" si está configurado
    if (state.alwaysOnTop || allCamerasAlwaysOnTop) {
        windowElement.style.zIndex = '9999';
    } else {
        windowElement.style.zIndex = '1000';
    }
    
    // Actualizar estado
    state.isOpen = true;
    updateCameraStatus(cameraNumber, true);
    
    // Hacer la ventana arrastrable
    makeWindowDraggable(windowElement, cameraNumber);
    
    // Traer al frente
    bringWindowToFront(windowElement);
    
    // Simular la carga de video (en una implementación real, esto vendría de WebRTC)
    simulateVideoLoad(cameraNumber);
    
    console.log(`Camera window ${cameraNumber} opened`);
}

/**
 * Cierra una ventana de cámara
 * @param {number} cameraNumber - Número de la cámara (1, 2 o 3)
 */
function closeCameraWindow(cameraNumber) {
    const windowElement = getCameraWindowElement(cameraNumber);
    const state = cameraWindowsState[cameraNumber];
    
    // Ocultar la ventana removiendo la clase 'active'
    windowElement.classList.remove('active');
    
    // Actualizar estado
    state.isOpen = false;
    updateCameraStatus(cameraNumber, false);
    
    console.log(`Camera window ${cameraNumber} closed`);
}

/**
 * Obtiene el elemento de ventana para una cámara específica
 * @param {number} cameraNumber - Número de la cámara (1, 2 o 3)
 * @returns {HTMLElement} Elemento de la ventana
 */
function getCameraWindowElement(cameraNumber) {
    switch(cameraNumber) {
        case 1: return cameraWindow1;
        case 2: return cameraWindow2;
        case 3: return cameraWindow3;
        default: return cameraWindow1;
    }
}

/**
 * Simula la carga de video para una cámara (para demostración)
 * @param {number} cameraNumber - Número de la cámara (1, 2 o 3)
 */
function simulateVideoLoad(cameraNumber) {
    // En una implementación real, esto conectaría con el stream WebRTC
    setTimeout(() => {
        // Ocultar el placeholder después de un breve retraso (simulando carga)
        const windowElement = getCameraWindowElement(cameraNumber);
        const placeholder = windowElement.querySelector('.placeholder');
        if (placeholder) {
            placeholder.style.display = 'none';
        }
    }, 500);
}

/**
 * Minimiza una ventana de cámara (funcionalidad futura)
 * @param {number} cameraNumber - Número de la cámara (1, 2 o 3)
 */
function minimizeCameraWindow(cameraNumber) {
    // Por ahora, simplemente cerramos la ventana
    // En una implementación futura, esto podría minimizarla a la barra de tareas
    closeCameraWindow(cameraNumber);
    console.log(`Camera window ${cameraNumber} minimized`);
}

// ===========================================
// FUNCIONES PARA CONTROLAR TODAS LAS CÁMARAS
// ===========================================

/**
 * Alterna (muestra/oculta) todas las ventanas de cámara
 */
function toggleAllCameras() {
    const allOpen = Object.values(cameraWindowsState).every(state => state.isOpen);
    
    if (allOpen) {
        // Cerrar todas las cámaras
        closeAllCameras();
    } else {
        // Abrir todas las cámaras
        openAllCameras();
    }
}

/**
 * Abre todas las ventanas de cámara
 */
function openAllCameras() {
    for (let cameraNumber = 1; cameraNumber <= 3; cameraNumber++) {
        if (!cameraWindowsState[cameraNumber].isOpen) {
            openCameraWindow(cameraNumber);
        }
    }
    console.log("All camera windows opened");
}

/**
 * Cierra todas las ventanas de cámara
 */
function closeAllCameras() {
    for (let cameraNumber = 1; cameraNumber <= 3; cameraNumber++) {
        if (cameraWindowsState[cameraNumber].isOpen) {
            closeCameraWindow(cameraNumber);
        }
    }
    console.log("All camera windows closed");
}

/**
 * Organiza las ventanas de cámara automáticamente en la pantalla
 */
function arrangeCameras() {
    const screenWidth = window.innerWidth;
    const screenHeight = window.innerHeight;
    
    // Configurar posiciones organizadas
    const positions = [
        { left: 20, top: 20, width: 400, height: 300 },           // Superior izquierda
        { left: screenWidth - 420, top: 20, width: 400, height: 300 }, // Superior derecha
        { left: screenWidth / 2 - 200, top: screenHeight - 320, width: 400, height: 300 } // Centro inferior
    ];
    
    // Aplicar posiciones a las cámaras abiertas
    let positionIndex = 0;
    for (let cameraNumber = 1; cameraNumber <= 3; cameraNumber++) {
        if (cameraWindowsState[cameraNumber].isOpen) {
            const windowElement = getCameraWindowElement(cameraNumber);
            const pos = positions[positionIndex];
            
            windowElement.style.left = pos.left + 'px';
            windowElement.style.top = pos.top + 'px';
            windowElement.style.width = pos.width + 'px';
            windowElement.style.height = pos.height + 'px';
            
            // Actualizar posición en el estado
            cameraWindowsState[cameraNumber].position = { ...pos };
            
            positionIndex++;
            if (positionIndex >= positions.length) positionIndex = 0;
        }
    }
    
    console.log("Camera windows arranged");
}

/**
 * Restablece las posiciones de las cámaras a las posiciones predeterminadas
 */
function resetCameraPositions() {
    // Restablecer posiciones predeterminadas
    cameraWindowsState[1].position = { left: 50, top: 50, width: 400, height: 300 };
    cameraWindowsState[2].position = { left: 500, top: 50, width: 400, height: 300 };
    cameraWindowsState[3].position = { left: 50, top: 400, width: 400, height: 300 };
    
    // Aplicar a las cámaras abiertas
    for (let cameraNumber = 1; cameraNumber <= 3; cameraNumber++) {
        if (cameraWindowsState[cameraNumber].isOpen) {
            const windowElement = getCameraWindowElement(cameraNumber);
            const pos = cameraWindowsState[cameraNumber].position;
            
            windowElement.style.left = pos.left + 'px';
            windowElement.style.top = pos.top + 'px';
            windowElement.style.width = pos.width + 'px';
            windowElement.style.height = pos.height + 'px';
        }
    }
    
    console.log("Camera positions reset to default");
}

/**
 * Alterna el modo "siempre al frente" para todas las cámaras
 */
function toggleAllAlwaysOnTop() {
    allCamerasAlwaysOnTop = !allCamerasAlwaysOnTop;
    
    // Aplicar a todas las cámaras abiertas
    for (let cameraNumber = 1; cameraNumber <= 3; cameraNumber++) {
        const windowElement = getCameraWindowElement(cameraNumber);
        const state = cameraWindowsState[cameraNumber];
        
        if (state.isOpen) {
            if (allCamerasAlwaysOnTop) {
                windowElement.style.zIndex = '9999';
                state.alwaysOnTop = true;
            } else {
                windowElement.style.zIndex = '1000';
                state.alwaysOnTop = false;
            }
        }
    }
    
    console.log(`All cameras always on top: ${allCamerasAlwaysOnTop ? 'ON' : 'OFF'}`);
}

// ===========================================
// FUNCIONALIDAD DE ARRASTRAR VENTANAS
// ===========================================

/**
 * Hace una ventana flotante arrastrable
 * @param {HTMLElement} windowElement - Elemento de la ventana
 * @param {number} windowId - Identificador de la ventana
 */
function makeWindowDraggable(windowElement, windowId) {
    const header = windowElement.querySelector('.floating-window-header');
    let isDragging = false;
    let offsetX = 0;
    let offsetY = 0;
    
    // Evento cuando se presiona el mouse en la cabecera
    header.addEventListener('mousedown', function(e) {
        // Evitar que se active si se hace clic en un botón
        if (e.target.classList.contains('floating-window-btn')) {
            return;
        }
        
        // Activar modo de arrastre
        isDragging = true;
        
        // Traer ventana al frente cuando se empieza a arrastrar
        bringWindowToFront(windowElement);
        
        // Calcular la posición del clic relativa a la ventana
        offsetX = e.clientX - windowElement.offsetLeft;
        offsetY = e.clientY - windowElement.offsetTop;
        
        // Agregar eventos para mover y soltar
        document.addEventListener('mousemove', onMouseMove);
        document.addEventListener('mouseup', onMouseUp);
    });
    
    /**
     * Manejador para el movimiento del mouse durante el arrastre
     */
    function onMouseMove(e) {
        if (!isDragging) return;
        
        // Calcular nueva posición
        let newLeft = e.clientX - offsetX;
        let newTop = e.clientY - offsetY;
        
        // Limitar movimiento para que la ventana no salga de la pantalla
        const maxLeft = window.innerWidth - windowElement.offsetWidth;
        const maxTop = window.innerHeight - windowElement.offsetHeight;
        
        // Aplicar límites
        newLeft = Math.max(0, Math.min(newLeft, maxLeft));
        newTop = Math.max(0, Math.min(newTop, maxTop));
        
        // Actualizar posición de la ventana
        windowElement.style.left = newLeft + 'px';
        windowElement.style.top = newTop + 'px';
        
        // Actualizar posición en el estado
        cameraWindowsState[windowId].position.left = newLeft;
        cameraWindowsState[windowId].position.top = newTop;
    }
    
    /**
     * Manejador para cuando se suelta el mouse
     */
    function onMouseUp() {
        isDragging = false;
        
        // Remover los eventos de movimiento y suelta
        document.removeEventListener('mousemove', onMouseMove);
        document.removeEventListener('mouseup', onMouseUp);
    }
}

// ===========================================
// FUNCIONALIDADES ADICIONALES DE LAS VENTANAS
// ===========================================

/**
 * Trae una ventana al frente
 * Incrementa el índice Z para que esté sobre otros elementos
 * @param {HTMLElement} windowElement - Elemento de la ventana
 */
function bringWindowToFront(windowElement) {
    zIndexCounter++;
    windowElement.style.zIndex = zIndexCounter;
}

/**
 * Actualiza el estado de una cámara en la interfaz
 * @param {number} cameraNumber - Número de la cámara
 * @param {boolean} isOpen - Si la cámara está abierta o cerrada
 */
function updateCameraStatus(cameraNumber, isOpen) {
    const statusElement = document.getElementById(`camera${cameraNumber}-status`);
    if (!statusElement) return;
    
    if (isOpen) {
        statusElement.textContent = "Abierta";
        statusElement.className = "camera-open";
    } else {
        statusElement.textContent = "Cerrada";
        statusElement.className = "camera-closed";
    }
}

// ===========================================
// SISTEMA DE CONFIGURACIÓN DE CONTROLES SIMPLIFICADO
// ===========================================

// Variables para la configuración de controles
let currentControlConfig = {}; // Configuración actual (solo 5 acciones)
let isRecordingKey = false;    // Estado de grabación de tecla
let currentRecordingInput = null; // Input actual en modo de grabación

// Mapeo de códigos de tecla a nombres legibles
const keyCodeMap = {
    // Teclas de letras
    'KeyW': 'W', 'KeyA': 'A', 'KeyS': 'S', 'KeyD': 'D',
    'KeyQ': 'Q', 'KeyE': 'E', 'KeyR': 'R', 'KeyF': 'F',
    'KeyZ': 'Z', 'KeyX': 'X', 'KeyC': 'C', 'KeyV': 'V',
    'KeyB': 'B', 'KeyN': 'N', 'KeyM': 'M',
    
    // Teclas numéricas
    'Digit1': '1', 'Digit2': '2', 'Digit3': '3', 'Digit4': '4',
    'Digit5': '5', 'Digit6': '6', 'Digit7': '7', 'Digit8': '8',
    'Digit9': '9', 'Digit0': '0',
    
    // Teclas de función
    'F1': 'F1', 'F2': 'F2', 'F3': 'F3', 'F4': 'F4',
    'F5': 'F5', 'F6': 'F6', 'F7': 'F7', 'F8': 'F8',
    'F9': 'F9', 'F10': 'F10', 'F11': 'F11', 'F12': 'F12',
    
    // Teclas especiales
    'Space': 'ESPACIO',
    'ShiftLeft': 'SHIFT IZQ', 'ShiftRight': 'SHIFT DER',
    'ControlLeft': 'CTRL IZQ', 'ControlRight': 'CTRL DER',
    'AltLeft': 'ALT IZQ', 'AltRight': 'ALT DER',
    'Tab': 'TAB',
    'CapsLock': 'CAPS LOCK',
    'Enter': 'ENTER',
    'Backspace': 'BACKSPACE',
    'Delete': 'DELETE',
    'Insert': 'INSERT',
    'Home': 'HOME',
    'End': 'END',
    'PageUp': 'PAGE UP',
    'PageDown': 'PAGE DOWN',
    'ArrowUp': 'FLECHA ARRIBA',
    'ArrowDown': 'FLECHA ABAJO',
    'ArrowLeft': 'FLECHA IZQ',
    'ArrowRight': 'FLECHA DER',
    
    // Teclas de mouse
    'MouseLeft': 'CLICK IZQ',
    'MouseRight': 'CLICK DER',
    'MouseMiddle': 'CLICK MEDIO',
    
    // Otras teclas
    'Escape': 'ESC',
    'BracketLeft': '[', 'BracketRight': ']',
    'Semicolon': ';', 'Quote': "'",
    'Comma': ',', 'Period': '.', 'Slash': '/',
    'Backslash': '\\'
};

/**
 * Inicializa el sistema de configuración de controles simplificado
 * Configura eventos y carga la configuración guardada
 */
function initControlConfig() {
    // Cargar configuración guardada (si existe)
    loadControlConfig();
    
    // Configurar eventos para los inputs de teclas
    const keyInputs = document.querySelectorAll('.control-input');
    keyInputs.forEach(input => {
        // Evento de clic para iniciar la grabación
        input.addEventListener('click', function() {
            startKeyRecording(this);
        });
        
        // Evitar que el input sea editable manualmente
        input.addEventListener('keydown', function(e) {
            e.preventDefault();
        });
    });
    
    // Configurar eventos globales de teclado para la grabación
    document.addEventListener('keydown', handleGlobalKeyDown);
    document.addEventListener('keyup', handleGlobalKeyUp);
    
    console.log("Simplified control configuration system initialized (5 actions only)");
}

/**
 * Inicia el modo de grabación para un input específico
 * @param {HTMLElement} input - El elemento input que se va a configurar
 */
function startKeyRecording(input) {
    // Si ya está grabando, detener la grabación anterior
    if (isRecordingKey && currentRecordingInput) {
        stopKeyRecording();
    }
    
    // Configurar el nuevo input para grabación
    isRecordingKey = true;
    currentRecordingInput = input;
    
    // Añadir clase para indicar modo de grabación
    input.classList.add('recording');
    
    // Cambiar el texto del input para indicar que está grabando
    input.value = "GRABANDO...";
    
    // Actualizar mensaje de estado
    showControlStatus("Presiona la tecla que deseas asignar...", false);
    
    console.log(`Started recording for action: ${input.dataset.action}`);
}

/**
 * Detiene el modo de grabación y guarda la tecla asignada
 */
function stopKeyRecording() {
    if (!isRecordingKey || !currentRecordingInput) return;
    
    // Remover clase de grabación
    currentRecordingInput.classList.remove('recording');
    
    // Restaurar el valor anterior si no se asignó nueva tecla
    if (currentRecordingInput.value === "GRABANDO...") {
        const action = currentRecordingInput.dataset.action;
        currentRecordingInput.value = currentControlConfig[action] || getDefaultKey(action);
    }
    
    // Limpiar variables
    isRecordingKey = false;
    currentRecordingInput = null;
    
    // Actualizar mensaje de estado
    showControlStatus("Grabación detenida", true);
    
    console.log("Stopped key recording");
}

/**
 * Manejador global para eventos de teclado (cuando se presiona una tecla)
 * @param {KeyboardEvent} e - Evento de teclado
 */
function handleGlobalKeyDown(e) {
    // Solo procesar si estamos en modo de grabación
    if (!isRecordingKey || !currentRecordingInput) return;
    
    // Prevenir el comportamiento por defecto
    e.preventDefault();
    e.stopPropagation();
    
    // Obtener el nombre legible de la tecla
    const keyName = getKeyName(e);
    
    // Asignar la tecla al input actual
    assignKeyToInput(currentRecordingInput, keyName);
    
    // Detener la grabación
    stopKeyRecording();
    
    console.log(`Key assigned: ${keyName} (code: ${e.code})`);
}

/**
 * Manejador global para eventos de liberación de teclas
 * @param {KeyboardEvent} e - Evento de teclado
 */
function handleGlobalKeyUp(e) {
    // No es necesario para la funcionalidad básica, pero se mantiene para futuras expansiones
}

/**
 * Obtiene un nombre legible para una tecla basado en el evento de teclado
 * @param {KeyboardEvent} e - Evento de teclado
 * @returns {string} Nombre legible de la tecla
 */
function getKeyName(e) {
    // Intentar obtener el nombre del mapeo
    if (keyCodeMap[e.code]) {
        return keyCodeMap[e.code];
    }
    
    // Si no está en el mapeo, usar la tecla directamente
    // Para letras, convertir a mayúsculas
    if (e.key.length === 1 && e.key.match(/[a-zA-Z]/)) {
        return e.key.toUpperCase();
    }
    
    // Para otros caracteres, usar la tecla tal cual
    return e.key || e.code;
}

/**
 * Asigna una tecla a un input específico
 * @param {HTMLElement} input - Elemento input
 * @param {string} keyName - Nombre de la tecla a asignar
 */
function assignKeyToInput(input, keyName) {
    // Actualizar el valor del input
    input.value = keyName;
    
    // Obtener la acción asociada
    const action = input.dataset.action;
    
    // Actualizar la configuración actual
    currentControlConfig[action] = keyName;
    
    // Mostrar mensaje de confirmación
    showControlStatus(`Tecla asignada: ${keyName} para ${getActionLabel(action)}`, true);
}

/**
 * Obtiene la etiqueta legible para una acción
 * @param {string} action - Identificador de la acción
 * @returns {string} Etiqueta legible
 */
function getActionLabel(action) {
    const actionLabels = {
        'forward': 'AVANZAR',
        'backward': 'RETROCEDER',
        'left': 'IZQUIERDA',
        'right': 'DERECHA',
        'jump': 'SALTAR'
    };
    
    return actionLabels[action] || action;
}

/**
 * Obtiene la tecla predeterminada para una acción
 * @param {string} action - Identificador de la acción
 * @returns {string} Tecla predeterminada
 */
function getDefaultKey(action) {
    const defaultKeys = {
        'forward': 'W',
        'backward': 'S',
        'left': 'A',
        'right': 'D',
        'jump': 'ESPACIO'
    };
    
    return defaultKeys[action] || '?';
}

/**
 * Actualiza todos los inputs con la configuración actual
 */
function updateInputsFromConfig() {
    // Para cada acción en la configuración
    for (const action in currentControlConfig) {
        // Buscar el input correspondiente
        const input = document.querySelector(`.control-input[data-action="${action}"]`);
        if (input) {
            // Actualizar el valor
            input.value = currentControlConfig[action];
        }
    }
}

/**
 * Guarda la configuración actual en el almacenamiento local
 */
function saveControlConfig() {
    try {
        // Guardar en localStorage
        localStorage.setItem('aesirControlConfig', JSON.stringify(currentControlConfig));
        
        // Mostrar mensaje de confirmación
        showControlStatus("Configuración guardada correctamente en el navegador", true);
        
        console.log("Simplified control configuration saved to localStorage:", currentControlConfig);
    } catch (error) {
        // Mostrar mensaje de error
        showControlStatus("Error al guardar la configuración: " + error.message, false, true);
        
        console.error("Error saving control configuration:", error);
    }
}

/**
 * Carga la configuración guardada del almacenamiento local
 */
function loadControlConfig() {
    try {
        // Intentar cargar del localStorage
        const savedConfig = localStorage.getItem('aesirControlConfig');
        
        if (savedConfig) {
            // Parsear y aplicar la configuración
            currentControlConfig = JSON.parse(savedConfig);
            
            // Actualizar los inputs
            updateInputsFromConfig();
            
            // Mostrar mensaje de confirmación
            showControlStatus("Configuración cargada del navegador", true);
            
            console.log("Control configuration loaded from localStorage:", currentControlConfig);
        } else {
            // Usar configuración predeterminada
            resetControlConfig(true); // Silencioso
        }
    } catch (error) {
        // Mostrar mensaje de error
        showControlStatus("Error al cargar la configuración: " + error.message, false, true);
        
        console.error("Error loading control configuration:", error);
        
        // Usar configuración predeterminada
        resetControlConfig(true); // Silencioso
    }
}

/**
 * Exporta la configuración actual como un archivo JSON
 */
function exportControlConfig() {
    try {
        // Crear objeto con la configuración y metadatos
        const exportData = {
            config: currentControlConfig,
            metadata: {
                name: "AESIR Simplified Control Configuration",
                version: "1.0",
                exportDate: new Date().toISOString(),
                totalActions: Object.keys(currentControlConfig).length,
                description: "Configuración simplificada con 5 acciones: movimiento (4) + salto"
            }
        };
        
        // Convertir a JSON
        const jsonData = JSON.stringify(exportData, null, 2);
        
        // Crear un blob con los datos
        const blob = new Blob([jsonData], { type: 'application/json' });
        
        // Crear un enlace para descargar
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `aesir-simple-controls-${new Date().toISOString().slice(0, 10)}.json`;
        
        // Simular clic para descargar
        document.body.appendChild(a);
        a.click();
        
        // Limpiar
        setTimeout(() => {
            document.body.removeChild(a);
            URL.revokeObjectURL(url);
        }, 100);
        
        // Mostrar mensaje de confirmación
        showControlStatus("Configuración exportada como archivo JSON", true);
        
        console.log("Simplified control configuration exported:", exportData);
    } catch (error) {
        // Mostrar mensaje de error
        showControlStatus("Error al exportar la configuración: " + error.message, false, true);
        
        console.error("Error exporting control configuration:", error);
    }
}

/**
 * Restablece la configuración a los valores predeterminados
 * @param {boolean} silent - Si es true, no muestra mensaje de confirmación
 */
function resetControlConfig(silent = false) {
    // Restablecer a valores predeterminados
    currentControlConfig = {};
    
    // Para cada input, establecer su valor predeterminado
    const inputs = document.querySelectorAll('.control-input');
    inputs.forEach(input => {
        const action = input.dataset.action;
        const defaultValue = getDefaultKey(action);
        
        // Actualizar input
        input.value = defaultValue;
        
        // Actualizar configuración
        currentControlConfig[action] = defaultValue;
    });
    
    // Mostrar mensaje de confirmación (si no es silencioso)
    if (!silent) {
        showControlStatus("Configuración restablecida a valores predeterminados", true);
    }
    
    // Eliminar configuración guardada
    localStorage.removeItem('aesirControlConfig');
    
    console.log("Simplified control configuration reset to defaults (5 actions)");
}

/**
 * Muestra un mensaje de estado en la interfaz
 * @param {string} message - Mensaje a mostrar
 * @param {boolean} isSuccess - Si es true, muestra en verde; si es false, en rojo
 * @param {boolean} isError - Si es true, fuerza el color rojo (error)
 */
function showControlStatus(message, isSuccess = true, isError = false) {
    const statusElement = document.getElementById('control-status-message');
    if (!statusElement) return;
    
    // Establecer el mensaje
    statusElement.textContent = message;
    
    // Establecer el color
    if (isError) {
        statusElement.className = 'control-status-message control-status-error';
    } else {
        statusElement.className = 'control-status-message';
        statusElement.style.color = isSuccess ? '#4CAF50' : '#ff9800';
    }
    
    // Ocultar el mensaje después de 5 segundos (si no es un error)
    if (!isError) {
        setTimeout(() => {
            if (statusElement.textContent === message) {
                statusElement.textContent = '';
            }
        }, 5000);
    }
}

// ===========================================
// INICIALIZACIÓN AL CARGAR LA PÁGINA
// ===========================================

/**
 * Función de inicialización que se ejecuta cuando la página se carga
 */
function init() {
    // Inicializar la configuración de controles simplificada
    initControlConfig();
    
    // Configurar todas las ventanas de cámara como arrastrables
    for (let cameraNumber = 1; cameraNumber <= 3; cameraNumber++) {
        const windowElement = getCameraWindowElement(cameraNumber);
        makeWindowDraggable(windowElement, cameraNumber);
    }
    
    console.log("Page initialization complete - Simplified interface with 5 controls and floating cameras!");
}

// Ejecutar la inicialización cuando el DOM esté listo
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}

// ===========================================
// FUNCIONES DE WEBRTC (placeholder - puedes completarlas)
// ===========================================

function start() {
    console.log("Start transmission function called");
    // Aquí iría tu lógica de WebRTC
}

function stop() {
    console.log("Stop transmission function called");
    // Aquí iría tu lógica de WebRTC
}

function toggleCommunication() {
    console.log("Toggle communication function called");
    // Aquí iría tu lógica de comunicación
}

function toggleInputMode() {
    console.log("Toggle input mode function called");
    // Aquí iría tu lógica para cambiar entre controlador y teclado
}
