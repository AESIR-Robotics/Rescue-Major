
(function () {
    'use strict';

    let selectedOption = null;
    function init() {

        const options = document.querySelectorAll('.config-option');
        const contents = document.querySelectorAll('.settings-content-modal');
        const modal_content = document.getElementById('config-option-content');

        options.forEach(option => {
            option.addEventListener('click', () => {
                // Aquí puedes agregar lógica adicional para manejar la opción seleccionada
                contents.forEach(content => {
                    content.hidden = true;
                });
                modal_content.style.display = 'none';
                if (option === selectedOption) {
                    selectedOption = null;
                    option.classList.remove('selected');
                    return
                }; // No hacer nada si ya está seleccionado
                if (selectedOption) selectedOption.classList.remove('selected');
                option.classList.add('selected');
                selectedOption = option;

                const index = Array.from(options).indexOf(option);
                if (index !== -1) {
                    contents[index].hidden = false;
                    modal_content.style.display = 'block';
                }
            });
        });
    }

    // Auto-inicializar cuando el DOM esté listo
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }

})();
