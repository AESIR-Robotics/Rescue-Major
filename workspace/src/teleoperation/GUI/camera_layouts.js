// ============================================
// AESIR INTERFACE — CAMERA LAYOUT MANAGER
// Handles layout modes, drag-to-reorder,
// animated transitions, keyboard shortcuts,
// terminal overlays, and UI integration.
// ============================================

(function () {
    'use strict';

    const MODES = {
        PRIMARY: 'primary', SECONDARY: 'secondary', TERTIARY: 'tertiary',
        MOSAIC_LEFT: 'mosaic-left', MOSAIC_TOP: 'mosaic-top', MOSAIC_RIGHT: 'mosaic-right'
    };
    const GAP = 8; // px gap between cameras
    const FLOAT_SIZE = 0.22; // fraction of area for floating cams
    const FLOAT_MARGIN = 16; // px margin for floating cams
    const STORAGE_KEY_LAYOUT = 'aesir-layout';
    const STORAGE_KEY_SHORTCUTS = 'aesir-shortcuts';

    // ---- State ----
    let currentMode = MODES.PRIMARY;
    let focusCamera = 1;        // which camera is "main" in secondary/tertiary
    let cameraOrder = [1, 2, 3]; // order for primary grid
    let areaRect = null;

    const defaultShortcuts = {
        layoutPrimary: { code: 'Digit1', alt: true, label: 'Layout Grid' },
        layoutSecondary: { code: 'Digit2', alt: true, label: 'Layout PiP' },
        layoutTertiary: { code: 'Digit3', alt: true, label: 'Layout Focus' },
        layoutMosaicLeft: { code: 'Digit4', alt: true, label: 'Layout Mosaico Izq' },
        layoutMosaicTop: { code: 'Digit5', alt: true, label: 'Layout Mosaico Sup' },
        layoutMosaicRight: { code: 'Digit6', alt: true, label: 'Layout Mosaico Der' },
        layoutCycle: { code: 'Tab', alt: true, label: 'Ciclar layout' },
        focusCam1: { code: 'KeyQ', alt: true, label: 'Foco Cámara 1' },
        focusCam2: { code: 'KeyW', alt: true, label: 'Foco Cámara 2' },
        focusCam3: { code: 'KeyE', alt: true, label: 'Foco Cámara 3' },
        focusCycle: { code: 'Space', alt: true, label: 'Ciclar foco cámara' },
        toggleTheme: { code: 'KeyT', alt: true, label: 'Modo claro/oscuro' },
        cycleColor: { code: 'KeyC', alt: true, label: 'Ciclar color tema' },
        toggleSettings: { code: 'Comma', alt: true, label: 'Abrir/cerrar config' },
        showShortcuts: { code: 'Slash', alt: true, label: 'Mostrar atajos' },
    };

    let shortcuts = {};

    // ---- DOM References ----
    const containers = {};
    let cameraArea = null;

    function init() {
        cameraArea = document.getElementById('camera-area');
        for (let i = 1; i <= 3; i++) {
            containers[i] = document.getElementById(`cam-container-${i}`);
        }

        loadState();
        loadShortcuts();
        computeAreaRect();
        applyLayout(false);

        bindLayoutButtons();
        bindFocusButtons();
        bindKeyboardShortcuts();
        bindSettingsPanel();
        bindTerminalOverlays();
        bindConnectionIndicators();
        bindStartStopIntegration();
        renderShortcutsConfig();
        renderShortcutsModal();
        initDragReorder();
        initVideoFitMode();

        window.addEventListener('resize', () => {
            computeAreaRect();
            applyLayout(false);
        });

        console.log('[CameraLayout] Initialized');
    }

    function computeAreaRect() {
        if (cameraArea) {
            areaRect = {
                w: cameraArea.clientWidth,
                h: cameraArea.clientHeight
            };
        }
    }

    function loadState() {
        try {
            const saved = localStorage.getItem(STORAGE_KEY_LAYOUT);
            if (saved) {
                const data = JSON.parse(saved);
                if (Object.values(MODES).includes(data.mode)) currentMode = data.mode;
                if ([1, 2, 3].includes(data.focus)) focusCamera = data.focus;
                if (Array.isArray(data.order) && data.order.length === 3) cameraOrder = data.order;
            }
        } catch (e) { }
    }

    function saveState() {
        try {
            localStorage.setItem(STORAGE_KEY_LAYOUT, JSON.stringify({
                mode: currentMode, focus: focusCamera, order: cameraOrder
            }));
        } catch (e) { }
    }

    function loadShortcuts() {
        shortcuts = JSON.parse(JSON.stringify(defaultShortcuts));
        try {
            const saved = localStorage.getItem(STORAGE_KEY_SHORTCUTS);
            if (saved) {
                const data = JSON.parse(saved);
                for (const action in data) {
                    if (shortcuts[action]) {
                        shortcuts[action].code = data[action].code || data[action].key || shortcuts[action].code;
                        shortcuts[action].alt = data[action].alt !== undefined ? data[action].alt : true;
                    }
                }
            }
        } catch (e) { }
    }

    function saveShortcuts() {
        try {
            const data = {};
            for (const action in shortcuts) {
                data[action] = { code: shortcuts[action].code, alt: shortcuts[action].alt };
            }
            localStorage.setItem(STORAGE_KEY_SHORTCUTS, JSON.stringify(data));
        } catch (e) { }
    }

    // ============================================
    // LAYOUT CALCULATION
    // ============================================

    function getPositions() {
        if (!areaRect) return {};
        const W = areaRect.w;
        const H = areaRect.h;
        const positions = {};

        switch (currentMode) {
            case MODES.PRIMARY: {
                // Three cameras side by side in order
                const camW = (W - GAP * 4) / 3;
                const camH = H - GAP * 2;
                cameraOrder.forEach((cam, idx) => {
                    positions[cam] = {
                        left: GAP + idx * (camW + GAP),
                        top: GAP,
                        width: camW,
                        height: camH,
                        floating: false
                    };
                });
                break;
            }
            case MODES.SECONDARY: {
                // Two cameras take space, one floats (small, PiP)
                const mainCams = cameraOrder.filter(c => c !== focusCamera);
                const pipCam = focusCamera;
                const mainW = (W - GAP * 3) / 2;
                const mainH = H - GAP * 2;
                mainCams.forEach((cam, idx) => {
                    positions[cam] = {
                        left: GAP + idx * (mainW + GAP),
                        top: GAP,
                        width: mainW,
                        height: mainH,
                        floating: false
                    };
                });
                // PiP in bottom-right of second camera area
                const pipW = W * FLOAT_SIZE;
                const pipH = H * 0.28;
                positions[pipCam] = {
                    left: W - pipW - FLOAT_MARGIN,
                    top: H - pipH - FLOAT_MARGIN,
                    width: pipW,
                    height: pipH,
                    floating: true
                };
                break;
            }
            case MODES.TERTIARY: {
                // One camera takes full space, two float on sides
                const mainCam = focusCamera;
                const sideCams = cameraOrder.filter(c => c !== focusCamera);
                positions[mainCam] = {
                    left: GAP,
                    top: GAP,
                    width: W - GAP * 2,
                    height: H - GAP * 2,
                    floating: false
                };
                // Small floats on bottom-left and bottom-right
                const sW = W * FLOAT_SIZE;
                const sH = H * 0.26;
                positions[sideCams[0]] = {
                    left: FLOAT_MARGIN,
                    top: H - sH - FLOAT_MARGIN,
                    width: sW,
                    height: sH,
                    floating: true
                };
                positions[sideCams[1]] = {
                    left: W - sW - FLOAT_MARGIN,
                    top: H - sH - FLOAT_MARGIN,
                    width: sW,
                    height: sH,
                    floating: true
                };
                break;
            }
            case MODES.MOSAIC_LEFT: {
                // Big camera left (4/7 width), two small stacked right (3/7 width)
                const mainCam = focusCamera;
                const sideCams = cameraOrder.filter(c => c !== focusCamera);
                const leftW = (W - GAP * 3) * 4 / 7;
                const rightW = (W - GAP * 3) * 3 / 7;
                const halfH = (H - GAP * 3) / 2;
                positions[mainCam] = {
                    left: GAP, top: GAP, width: leftW, height: H - GAP * 2, floating: false
                };
                positions[sideCams[0]] = {
                    left: GAP * 2 + leftW, top: GAP, width: rightW, height: halfH, floating: false
                };
                positions[sideCams[1]] = {
                    left: GAP * 2 + leftW, top: GAP * 2 + halfH, width: rightW, height: halfH, floating: false
                };
                break;
            }
            case MODES.MOSAIC_TOP: {
                // Big camera top (full width, 3/5 height), two small bottom side-by-side
                const mainCam = focusCamera;
                const sideCams = cameraOrder.filter(c => c !== focusCamera);
                const topH = (H - GAP * 3) * 3 / 5;
                const bottomH = (H - GAP * 3) * 2 / 5;
                const halfW = (W - GAP * 3) / 2;
                positions[mainCam] = {
                    left: GAP, top: GAP, width: W - GAP * 2, height: topH, floating: false
                };
                positions[sideCams[0]] = {
                    left: GAP, top: GAP * 2 + topH, width: halfW, height: bottomH, floating: false
                };
                positions[sideCams[1]] = {
                    left: GAP * 2 + halfW, top: GAP * 2 + topH, width: halfW, height: bottomH, floating: false
                };
                break;
            }
            case MODES.MOSAIC_RIGHT: {
                // Two small stacked left (3/8 width), big camera right (5/8 width)
                const mainCam = focusCamera;
                const sideCams = cameraOrder.filter(c => c !== focusCamera);
                const leftW = (W - GAP * 3) * 3 / 8;
                const rightW = (W - GAP * 3) * 5 / 8;
                const halfH = (H - GAP * 3) / 2;
                positions[sideCams[0]] = {
                    left: GAP, top: GAP, width: leftW, height: halfH, floating: false
                };
                positions[sideCams[1]] = {
                    left: GAP, top: GAP * 2 + halfH, width: leftW, height: halfH, floating: false
                };
                positions[mainCam] = {
                    left: GAP * 2 + leftW, top: GAP, width: rightW, height: H - GAP * 2, floating: false
                };
                break;
            }
        }
        return positions;
    }

    function applyLayout(animate = true) {
        const positions = getPositions();
        for (const cam in positions) {
            const el = containers[cam];
            if (!el) continue;
            const pos = positions[cam];

            if (!animate) {
                el.style.transition = 'none';
            } else {
                el.style.transition = '';
            }

            el.style.left = pos.left + 'px';
            el.style.top = pos.top + 'px';
            el.style.width = pos.width + 'px';
            el.style.height = pos.height + 'px';

            el.classList.toggle('is-floating', pos.floating);
            el.classList.toggle('is-focused', Number(cam) === focusCamera && currentMode !== MODES.PRIMARY);

            // Force reflow for no-animate
            if (!animate) {
                el.offsetHeight; // eslint-disable-line no-unused-expressions
                el.style.transition = '';
            }
        }

        updateLayoutButtons();
        updateLayoutStatus();
        saveState();
    }

    // ============================================
    // MODE SWITCHING
    // ============================================

    function setMode(mode) {
        if (!Object.values(MODES).includes(mode)) return;
        currentMode = mode;
        computeAreaRect();
        applyLayout(true);
    }

    function cycleMode() {
        const modes = [MODES.PRIMARY, MODES.SECONDARY, MODES.TERTIARY, MODES.MOSAIC_LEFT, MODES.MOSAIC_TOP, MODES.MOSAIC_RIGHT];
        const idx = modes.indexOf(currentMode);
        setMode(modes[(idx + 1) % modes.length]);
    }

    function setFocus(camNum) {
        if (![1, 2, 3].includes(camNum)) return;
        focusCamera = camNum;
        computeAreaRect();
        applyLayout(true);
    }

    function cycleFocus() {
        const next = (focusCamera % 3) + 1;
        setFocus(next);
    }

    // ============================================
    // UI UPDATES
    // ============================================

    function updateLayoutButtons() {
        document.querySelectorAll('.layout-btn').forEach(btn => {
            btn.classList.toggle('active', btn.dataset.layout === currentMode);
        });
    }

    function updateLayoutStatus() {
        const el = document.getElementById('layout-status');
        if (el) {
            const names = {
                primary: 'Grid', secondary: 'PiP', tertiary: 'Focus',
                'mosaic-left': 'Mosaico Izq', 'mosaic-top': 'Mosaico Sup', 'mosaic-right': 'Mosaico Der'
            };
            let text = `Layout: ${names[currentMode] || currentMode}`;
            if (currentMode !== MODES.PRIMARY) {
                text += ` (Cam ${focusCamera})`;
            }
            el.textContent = text;
        }
    }

    // ============================================
    // LAYOUT BUTTON BINDINGS
    // ============================================

    function bindLayoutButtons() {
        document.querySelectorAll('.layout-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const mode = btn.dataset.layout;
                if (mode) setMode(mode);
            });
        });
    }

    function bindFocusButtons() {
        document.querySelectorAll('.cam-focus-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                const camNum = parseInt(btn.dataset.cam);
                if (camNum) setFocus(camNum);
            });
        });
    }

    // ============================================
    // KEYBOARD SHORTCUTS
    // ============================================

    function bindKeyboardShortcuts() {
        // Use capture phase to fire BEFORE com_client.js handlers
        document.addEventListener('keydown', (e) => {
            // Don't capture if recording a key in control config
            if (e.target.classList.contains('control-input') ||
                e.target.classList.contains('shortcut-key') ||
                e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') {
                return;
            }

            // Use e.code for reliable detection (macOS Alt produces special chars in e.key)
            const code = e.code;
            const isAlt = e.altKey;

            // Escape always closes modals/settings
            if (code === 'Escape') {
                closeSettings();
                closeShortcutsModal();
                return;
            }

            // Check each shortcut
            for (const action in shortcuts) {
                const sc = shortcuts[action];
                const scAlt = sc.alt !== false;
                const scCode = sc.code;

                if (code === scCode && isAlt === scAlt) {
                    e.preventDefault();
                    e.stopPropagation();
                    executeShortcutAction(action);
                    return;
                }
            }
        }, true); // <-- capture phase
    }

    function executeShortcutAction(action) {
        switch (action) {
            case 'layoutPrimary': setMode(MODES.PRIMARY); break;
            case 'layoutSecondary': setMode(MODES.SECONDARY); break;
            case 'layoutTertiary': setMode(MODES.TERTIARY); break;
            case 'layoutMosaicLeft': setMode(MODES.MOSAIC_LEFT); break;
            case 'layoutMosaicTop': setMode(MODES.MOSAIC_TOP); break;
            case 'layoutMosaicRight': setMode(MODES.MOSAIC_RIGHT); break;
            case 'layoutCycle': cycleMode(); break;
            case 'focusCam1': setFocus(1); break;
            case 'focusCam2': setFocus(2); break;
            case 'focusCam3': setFocus(3); break;
            case 'focusCycle': cycleFocus(); break;
            case 'toggleTheme':
                if (window.AesirTheme) window.AesirTheme.toggleMode();
                break;
            case 'cycleColor':
                if (window.AesirTheme) window.AesirTheme.cycleColor();
                break;
            case 'toggleSettings': toggleSettings(); break;
            case 'showShortcuts': toggleShortcutsModal(); break;
        }
    }

    // ============================================
    // SETTINGS PANEL
    // ============================================

    function bindSettingsPanel() {
        const btn = document.getElementById('settings-btn');
        const close = document.getElementById('settings-close');
        const backdrop = document.getElementById('settings-backdrop');

        if (btn) btn.addEventListener('click', toggleSettings);
        if (close) close.addEventListener('click', closeSettings);
        if (backdrop) backdrop.addEventListener('click', closeSettings);
    }

    function toggleSettings() {
        const panel = document.getElementById('settings-panel');
        const backdrop = document.getElementById('settings-backdrop');
        if (!panel) return;
        const isOpen = panel.classList.contains('open');
        if (isOpen) {
            closeSettings();
        } else {
            panel.classList.add('open');
            if (backdrop) backdrop.classList.add('open');
        }
    }

    function closeSettings() {
        const panel = document.getElementById('settings-panel');
        const backdrop = document.getElementById('settings-backdrop');
        if (panel) panel.classList.remove('open');
        if (backdrop) backdrop.classList.remove('open');
    }

    // ============================================
    // SHORTCUTS MODAL
    // ============================================

    function toggleShortcutsModal() {
        const modal = document.getElementById('shortcuts-modal-backdrop');
        if (!modal) return;
        modal.classList.toggle('open');
    }

    function closeShortcutsModal() {
        const modal = document.getElementById('shortcuts-modal-backdrop');
        if (modal) modal.classList.remove('open');
    }

    function renderShortcutsModal() {
        const container = document.getElementById('shortcuts-list');
        if (!container) return;

        const btn = document.getElementById('shortcuts-modal-close');
        if (btn) btn.addEventListener('click', closeShortcutsModal);

        const hintBtn = document.getElementById('shortcuts-hint-btn');
        if (hintBtn) hintBtn.addEventListener('click', toggleShortcutsModal);

        const groups = {
            'Layouts': ['layoutPrimary', 'layoutSecondary', 'layoutTertiary', 'layoutMosaicLeft', 'layoutMosaicTop', 'layoutMosaicRight', 'layoutCycle'],
            'Cámaras': ['focusCam1', 'focusCam2', 'focusCam3', 'focusCycle'],
            'Interfaz': ['toggleTheme', 'cycleColor', 'toggleSettings', 'showShortcuts']
        };

        let html = '';
        for (const groupName in groups) {
            html += `<div class="shortcut-group">`;
            html += `<div class="shortcut-group-title">${groupName}</div>`;
            for (const action of groups[groupName]) {
                const sc = shortcuts[action];
                if (!sc) continue;
                const keyDisplay = formatShortcut(sc);
                html += `<div class="shortcut-item">
          <span class="shortcut-item-label">${sc.label}</span>
          <span class="shortcut-item-key">${keyDisplay}</span>
        </div>`;
            }
            html += `</div>`;
        }
        html += `<div class="shortcut-group">
      <div class="shortcut-group-title">General</div>
      <div class="shortcut-item">
        <span class="shortcut-item-label">Cerrar panel/modal</span>
        <span class="shortcut-item-key">Esc</span>
      </div>
    </div>`;

        container.innerHTML = html;
    }

    function formatShortcut(sc) {
        const parts = [];
        if (sc.alt !== false) parts.push('Alt');
        let keyLabel = sc.code || '';
        // Convert e.code names to friendly labels
        const codeMap = {
            'Digit1': '1', 'Digit2': '2', 'Digit3': '3', 'Digit4': '4',
            'Digit5': '5', 'Digit6': '6', 'Digit7': '7', 'Digit8': '8', 'Digit9': '9', 'Digit0': '0',
            'Space': 'Espacio', 'Tab': 'Tab', 'Comma': ',', 'Slash': '?',
            'Period': '.', 'Minus': '-', 'Equal': '=', 'Escape': 'Esc',
        };
        if (codeMap[keyLabel]) {
            keyLabel = codeMap[keyLabel];
        } else if (keyLabel.startsWith('Key')) {
            keyLabel = keyLabel.slice(3); // KeyQ -> Q
        }
        parts.push(keyLabel);
        return parts.join('+');
    }

    // ============================================
    // SHORTCUTS CONFIGURATOR (in settings)
    // ============================================

    let recordingAction = null;

    function renderShortcutsConfig() {
        const container = document.getElementById('shortcuts-config');
        if (!container) return;

        let html = '';
        for (const action in shortcuts) {
            const sc = shortcuts[action];
            const keyDisplay = formatShortcut(sc);
            html += `<div class="shortcut-row">
        <span class="shortcut-label">${sc.label}</span>
        <span class="shortcut-key" data-action="${action}" tabindex="0">${keyDisplay}</span>
      </div>`;
        }
        html += `<div style="margin-top:12px">
      <button class="btn btn-sm btn-danger" id="reset-shortcuts-btn"><svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 -960 960 960"
                            fill="currentColor" style="vertical-align:middle;margin-right:6px;display:inline-block;">
                            <path
                                d="M399-140q-101.5-27-165.75-110T169-441q0-57 19.75-108.75T245-643.5q10.5-11.5 25.75-11.5T298-643q10.5 10.5 10.75 25.5T298-590q-26.5 31-40.25 69T244-441q0 80 47.25 143t122.25 85q12.5 3.5 20.75 14t8.25 23q0 18.5-13 29.5T399-140Zm162 0q-17.5 4.5-30.5-6.75t-13-29.75q0-11.5 8.25-22.25T546.5-213q75-22.5 122.5-85.25T716.5-441q0-98.5-69-167.5t-167.5-69h-5l20 20q10.5 10.5 10.5 26t-10.5 26q-11 11-26.5 11t-26-11l-84-83.5q-5.5-5.5-8-12t-2.5-14q0-7.5 2.5-14t8-12l84-84q10.5-10.5 26-10.5T495-825q10.5 11 10.5 26.5t-10.5 26l-20 20h5q130 0 220.75 90.75T791.5-441q0 107.5-64.5 190.75T561-140Z" />
                        </svg> Reset atajos</button>
    </div>`;
        container.innerHTML = html;

        // Bind click-to-record
        container.querySelectorAll('.shortcut-key').forEach(el => {
            el.addEventListener('click', () => {
                if (recordingAction) {
                    // Cancel previous
                    const prev = container.querySelector(`.shortcut-key[data-action="${recordingAction}"]`);
                    if (prev) {
                        prev.classList.remove('recording');
                        prev.textContent = formatShortcut(shortcuts[recordingAction]);
                    }
                }
                recordingAction = el.dataset.action;
                el.classList.add('recording');
                el.textContent = 'Presiona...';
            });
        });

        // Listen for key to record
        document.addEventListener('keydown', (e) => {
            if (!recordingAction) return;
            e.preventDefault();
            e.stopPropagation();

            shortcuts[recordingAction].code = e.code;
            shortcuts[recordingAction].alt = e.altKey;
            saveShortcuts();

            const el = container.querySelector(`.shortcut-key[data-action="${recordingAction}"]`);
            if (el) {
                el.classList.remove('recording');
                el.textContent = formatShortcut(shortcuts[recordingAction]);
            }
            recordingAction = null;

            // Update modal too
            renderShortcutsModal();
        }, true);

        // Reset button
        const resetBtn = document.getElementById('reset-shortcuts-btn');
        if (resetBtn) {
            resetBtn.addEventListener('click', () => {
                shortcuts = JSON.parse(JSON.stringify(defaultShortcuts));
                saveShortcuts();
                renderShortcutsConfig();
                renderShortcutsModal();
            });
        }
    }

    // ============================================
    // DRAG TO REORDER (Primary mode)
    // ============================================

    function initDragReorder() {
        for (let cam = 1; cam <= 3; cam++) {
            const el = containers[cam];
            if (!el) continue;
            const handle = el.querySelector('.cam-drag-handle');

            // Drag handler (works from handle in PRIMARY, from full container when floating)
            const startDrag = (e) => {
                const isFloating = el.classList.contains('is-floating');
                const fromHandle = handle && handle.contains(e.target);

                // In primary mode only allow drag from handle
                if (currentMode === MODES.PRIMARY && !fromHandle) return;
                // In other modes, allow drag from anywhere on floating cams, or from handle
                if (!isFloating && !fromHandle) return;
                // Don't drag if clicking buttons inside overlay header
                if (e.target.closest('.cam-focus-btn') || e.target.closest('.camera-overlay-header button')) return;

                e.preventDefault();
                let isDragging = true;
                el.classList.add('is-dragging');
                const startX = e.clientX;
                const startY = e.clientY;
                const origLeft = el.offsetLeft;
                const origTop = el.offsetTop;

                const onMove = (ev) => {
                    if (!isDragging) return;
                    const dx = ev.clientX - startX;
                    const dy = ev.clientY - startY;
                    el.style.left = (origLeft + dx) + 'px';
                    el.style.top = (origTop + dy) + 'px';
                };

                const onUp = () => {
                    isDragging = false;
                    el.classList.remove('is-dragging');
                    document.removeEventListener('mousemove', onMove);
                    document.removeEventListener('mouseup', onUp);

                    if (currentMode === MODES.PRIMARY) {
                        // Check for swap
                        const dropX = el.offsetLeft + el.offsetWidth / 2;
                        for (let other = 1; other <= 3; other++) {
                            if (other === cam) continue;
                            const otherEl = containers[other];
                            if (!otherEl) continue;
                            const oLeft = otherEl.offsetLeft;
                            const oRight = oLeft + otherEl.offsetWidth;
                            if (dropX > oLeft && dropX < oRight) {
                                const idxA = cameraOrder.indexOf(cam);
                                const idxB = cameraOrder.indexOf(other);
                                if (idxA !== -1 && idxB !== -1) {
                                    [cameraOrder[idxA], cameraOrder[idxB]] = [cameraOrder[idxB], cameraOrder[idxA]];
                                }
                                break;
                            }
                        }
                        applyLayout(true);
                    }
                    // In other modes, just leave the floating cam where it was dropped
                };

                document.addEventListener('mousemove', onMove);
                document.addEventListener('mouseup', onUp);
            };

            // Bind drag from handle always
            if (handle) handle.addEventListener('mousedown', startDrag);
            // Bind drag from full container for floating cams
            el.addEventListener('mousedown', startDrag);
        }
    }

    // ============================================
    // TERMINAL OVERLAYS
    // ============================================

    function bindTerminalOverlays() {
        // Robot terminal
        const robotTerminal = document.getElementById('robot-terminal');
        const termMinBtn = document.getElementById('terminal-minimize');
        const termCloseBtn = document.getElementById('terminal-close');

        if (termMinBtn && robotTerminal) {
            termMinBtn.addEventListener('click', () => {
                robotTerminal.classList.toggle('minimized');
                termMinBtn.textContent = robotTerminal.classList.contains('minimized') ? '+' : '−';
            });
        }
        if (termCloseBtn && robotTerminal) {
            termCloseBtn.addEventListener('click', () => {
                robotTerminal.style.display = 'none';
            });
        }

        // DataChannel overlay
        const dcOverlay = document.getElementById('dc-overlay');
        const dcMinBtn = document.getElementById('dc-minimize');
        if (dcMinBtn && dcOverlay) {
            dcMinBtn.addEventListener('click', () => {
                dcOverlay.classList.toggle('minimized');
                dcMinBtn.textContent = dcOverlay.classList.contains('minimized') ? '+' : '−';
            });
        }

        // Make terminal overlays draggable
        makeDraggable(robotTerminal, document.getElementById('terminal-drag-handle'));
        makeDraggable(dcOverlay, document.getElementById('dc-drag-handle'));
    }

    function makeDraggable(element, handle) {
        if (!element || !handle) return;
        let isDragging = false;
        let startX, startY, origLeft, origTop;

        handle.addEventListener('mousedown', (e) => {
            if (e.target.closest('.terminal-btn')) return;
            e.preventDefault();
            isDragging = true;
            startX = e.clientX;
            startY = e.clientY;
            origLeft = element.offsetLeft;
            origTop = element.offsetTop;

            // Switch from bottom/right positioning to left/top
            element.style.left = origLeft + 'px';
            element.style.top = origTop + 'px';
            element.style.bottom = 'auto';
            element.style.right = 'auto';

            const onMove = (ev) => {
                if (!isDragging) return;
                element.style.left = (origLeft + ev.clientX - startX) + 'px';
                element.style.top = (origTop + ev.clientY - startY) + 'px';
            };
            const onUp = () => {
                isDragging = false;
                document.removeEventListener('mousemove', onMove);
                document.removeEventListener('mouseup', onUp);
            };
            document.addEventListener('mousemove', onMove);
            document.addEventListener('mouseup', onUp);
        });
    }

    // ============================================
    // VIDEO FIT MODE
    // ============================================

    const STORAGE_KEY_FIT = 'aesir-video-fit';

    function initVideoFitMode() {
        const select = document.getElementById('video-fit-mode');
        if (!select) return;

        // Load saved preference
        const saved = localStorage.getItem(STORAGE_KEY_FIT);
        if (saved) {
            select.value = saved;
            applyVideoFit(saved);
        } else {
            applyVideoFit('scale-down');
        }

        select.addEventListener('change', () => {
            const mode = select.value;
            applyVideoFit(mode);
            localStorage.setItem(STORAGE_KEY_FIT, mode);
        });
    }

    function applyVideoFit(mode) {
        document.querySelectorAll('.camera-container video, .camera-container .placeholder').forEach(el => {
            el.style.objectFit = mode;
        });
    }

    // ============================================
    // CONNECTION INDICATORS
    // ============================================

    function bindConnectionIndicators() {
        // Observe the original state spans and update indicators
        const mapping = [
            { spanId: 'ice-connection-state', indicatorId: 'ind-ice' },
            { spanId: 'signaling-state', indicatorId: 'ind-signal' },
            { spanId: 'data-channel', indicatorId: 'ind-data' }
        ];

        mapping.forEach(({ spanId, indicatorId }) => {
            const span = document.getElementById(spanId);
            const indicator = document.getElementById(indicatorId);
            if (!span || !indicator) return;

            const update = () => {
                const text = span.textContent.toLowerCase();
                let state = 'inactive';
                if (text.includes('connected') || text.includes('complete') || text.includes('stable') || text.includes('open')) {
                    state = 'connected';
                } else if (text.includes('checking') || text.includes('gathering') || text.includes('new') || text.includes('have-local')) {
                    state = 'connecting';
                } else if (text.includes('failed') || text.includes('disconnected') || text.includes('closed')) {
                    state = 'failed';
                }
                indicator.setAttribute('data-state', state);
            };

            // Use MutationObserver
            const observer = new MutationObserver(update);
            observer.observe(span, { childList: true, characterData: true, subtree: true });

            // Initial check
            update();
        });
    }

    // ============================================
    // START/STOP INTEGRATION
    // ============================================

    function bindStartStopIntegration() {
        // Wrap start/stop to toggle button visibility
        const originalStart = window.start;
        const originalStop = window.stop;

        if (typeof originalStart === 'function') {
            window.start = async function () {
                const startBtn = document.getElementById('start');
                const stopBtn = document.getElementById('stop');
                try {
                    await originalStart.call(this);
                    if (startBtn) startBtn.style.display = 'none';
                    if (stopBtn) stopBtn.style.display = '';
                } catch (e) {
                    console.error('Start error:', e);
                    if (startBtn) startBtn.style.display = '';
                }
            };
        }

        if (typeof originalStop === 'function') {
            window.stop = function () {
                originalStop.call(this);
                const startBtn = document.getElementById('start');
                const stopBtn = document.getElementById('stop');
                if (startBtn) startBtn.style.display = '';
                if (stopBtn) stopBtn.style.display = 'none';
            };
        }
    }

    // ============================================
    // INIT
    // ============================================

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }

    // Public API
    window.AesirLayout = {
        setMode, cycleMode, setFocus, cycleFocus,
        getState: () => ({ mode: currentMode, focus: focusCamera, order: [...cameraOrder] })
    };

})();
