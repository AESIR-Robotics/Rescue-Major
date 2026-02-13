// ============================================
// AESIR INTERFACE — THEME MANAGER
// Handles theme color & dark/light mode switching
// ============================================

(function () {
    'use strict';

    const STORAGE_KEY = 'aesir-theme';
    const COLORS = ['blue', 'green', 'pink', 'orange', 'classic'];
    const COLOR_NAMES = {
        blue: 'Azul', green: 'Verde', pink: 'Rosa',
        orange: 'Naranja', classic: 'Clásico'
    };
    const MODE_NAMES = { dark: 'Oscuro', light: 'Claro' };

    const DARK_SVG = '<svg xmlns="http://www.w3.org/2000/svg" width="25" height="25" viewBox="0 -960 960 960" fill="currentColor"><path d="M480-122.5q-149.5 0-253.5-104T122.5-480q0-134.5 89-236.5t224-119q15-2 25.25 2.5t15.75 14q5.5 9 4.5 22.25T471-769q-14.5 24.5-22 51.75T441.5-660q0 91 63.75 154.75T660-441.5q29.5 0 57.5-7.75T769-471q14.5-8.5 27.5-9.5t22 4q10 5 14.5 15.25t2.5 25.75q-15.5 133-118 223t-237.5 90Z"/></svg>';
    const LIGHT_SVG = '<svg xmlns="http://www.w3.org/2000/svg" width="25" height="25" viewBox="0 -960 960 960" fill="currentColor"><path d="M340.25-340.25Q282.5-398 282.5-480t57.75-139.75Q398-677.5 480-677.5t139.75 57.75Q677.5-562 677.5-480t-57.75 139.75Q562-282.5 480-282.5t-139.75-57.75ZM80-442.5q-15.5 0-26.5-11t-11-26.5q0-15.5 11-26.5t26.5-11h80q15.5 0 26.5 11t11 26.5q0 15.5-11 26.5t-26.5 11H80Zm720 0q-15.5 0-26.5-11t-11-26.5q0-15.5 11-26.5t26.5-11h80q15.5 0 26.5 11t11 26.5q0 15.5-11 26.5t-26.5 11h-80Zm-346.5-331q-11-11-11-26.5v-80q0-15.5 11-26.5t26.5-11q15.5 0 26.5 11t11 26.5v80q0 15.5-11 26.5t-26.5 11q-15.5 0-26.5-11Zm0 720q-11-11-11-26.5v-80q0-15.5 11-26.5t26.5-11q15.5 0 26.5 11t11 26.5v80q0 15.5-11 26.5t-26.5 11q-15.5 0-26.5-11ZM227.5-680l-43-42q-11.5-10.5-11-26t11.5-27q11-11.5 26.75-11.75T238-775.5l42 43q11 11.5 11 26.5t-11 26.5q-10.5 11.5-25.75 10.75T227.5-680ZM722-184.5l-42-43q-10.5-11.5-10.75-26.75T680-280.5q10.5-11.5 25.75-10.75T732.5-280l43 42q11.5 10.5 11 26T775-185q-11 11.5-26.75 11.75T722-184.5ZM679.5-680q-11.5-10.5-10.75-25.75T680-732.5l42-43q10.5-11.5 26-11t27 11.5q11.5 11 11.75 26.75T775.5-722l-43 42Q721-669 706-669t-26.5-11ZM185-185q-11.5-11-11.75-26.75T184.5-238l43-42q11.5-10.5 26.75-10.75T280.5-280q11.5 10.5 10.75 25.75T280-227.5l-42 43q-10.5 11.5-26 10.75T185-185Z"/></svg>';
    const SETTINGS_SVG = '<svg xmlns="http://www.w3.org/2000/svg" width="25" height="25" viewBox="0 -960 960 960" fill="currentColor"><path d="M427-90q-22 0-38.25-14.75T369.5-141l-10-73.5q-13-5.5-26.25-12.75T309-242.5l-68.5 28q-20.5 9-41 1.75t-32-26.25L114-331.5q-11.5-19-6.75-40.5t22.25-35.5l60-45.5q-1-6.5-1-12.5V-480q0-6 .25-13t1.25-16l-59.5-44Q113-567 108-588.25t6.5-40.75l53-92q11.5-19 32.25-26.25T241-745.5l69.5 29q10-8 22.25-15t26.75-13l10-75.5q3-21.5 19.5-36.25T427.5-871H533q22 0 38.25 14.75T590.5-820l10 74q14 5.5 25.75 12.75T649-716.5l70.5-29q20.5-9 41-1.75t32 26.25l53 92q11.5 19 6.5 40.5T829.5-553l-62 46q1 7 1 12.75v29q0 6.25-1.5 12.75l61 45q17.5 14 22.5 35.5t-6.5 40.5L790.5-239q-11.5 19-32.25 26.25T717-214.5l-68-29q-9.5 7.5-20.75 14T600.5-215l-10 74q-3 21.5-19.25 36.25T533-90H427Zm52.5-255q56 0 95.5-39.5t39.5-95.5q0-56-39.5-95.5T479.5-615q-56.5 0-95.75 39.5T344.5-480q0 56 39.25 95.5T479.5-345Z"/></svg>';

    let currentColor = 'blue';
    let currentMode = 'dark';

    function init() {
        load();
        applyTheme();
        bindUI();
        updateStatusText();
    }

    function load() {
        try {
            const saved = localStorage.getItem(STORAGE_KEY);
            if (saved) {
                const data = JSON.parse(saved);
                if (COLORS.includes(data.color)) currentColor = data.color;
                if (data.mode === 'light' || data.mode === 'dark') currentMode = data.mode;
            }
        } catch (e) { }
    }

    function save() {
        try {
            localStorage.setItem(STORAGE_KEY, JSON.stringify({ color: currentColor, mode: currentMode }));
        } catch (e) { }
    }

    function applyTheme() {
        const html = document.documentElement;
        html.setAttribute('data-theme', currentMode);
        html.setAttribute('data-color', currentColor);
        save();
        updateUI();
        updateStatusText();
    }

    function bindUI() {
        const toggle = document.getElementById('theme-mode-toggle');
        if (toggle) {
            toggle.checked = currentMode === 'dark';
            toggle.addEventListener('change', () => {
                currentMode = toggle.checked ? 'dark' : 'light';
                applyTheme();
            });
        }

        const swatchContainer = document.getElementById('color-swatches');
        if (swatchContainer) {
            swatchContainer.addEventListener('click', (e) => {
                const swatch = e.target.closest('.swatch');
                if (!swatch) return;
                const color = swatch.dataset.color;
                if (color && COLORS.includes(color)) {
                    currentColor = color;
                    applyTheme();
                }
            });
        }

        const quickToggle = document.getElementById('theme-toggle-btn');
        if (quickToggle) {
            quickToggle.addEventListener('click', () => {
                currentMode = currentMode === 'dark' ? 'light' : 'dark';
                applyTheme();
            });
        }
    }

    function updateUI() {
        const toggle = document.getElementById('theme-mode-toggle');
        if (toggle) toggle.checked = currentMode === 'dark';

        document.querySelectorAll('#color-swatches .swatch').forEach(s => {
            s.classList.toggle('active', s.dataset.color === currentColor);
        });

        const quickToggle = document.getElementById('theme-toggle-btn');
        if (quickToggle) {
            const iconEl = document.getElementById('theme-icon');
            if (iconEl) {
                iconEl.innerHTML = currentMode === 'dark' ? DARK_SVG : LIGHT_SVG;
            }
            const settingsEl = document.getElementById('settings-icon');
            if (settingsEl) {
                settingsEl.innerHTML = SETTINGS_SVG;
            }
        }
    }

    function updateStatusText() {
        const el = document.getElementById('theme-status');
        if (el) {
            el.textContent = `${COLOR_NAMES[currentColor]} ${MODE_NAMES[currentMode]}`;
        }
    }

    window.AesirTheme = {
        toggleMode() {
            currentMode = currentMode === 'dark' ? 'light' : 'dark';
            applyTheme();
        },
        setColor(color) {
            if (COLORS.includes(color)) {
                currentColor = color;
                applyTheme();
            }
        },
        cycleColor() {
            const idx = COLORS.indexOf(currentColor);
            currentColor = COLORS[(idx + 1) % COLORS.length];
            applyTheme();
        },
        getState() {
            return { color: currentColor, mode: currentMode };
        }
    };

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})();
