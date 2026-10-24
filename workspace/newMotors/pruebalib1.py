import vesc_motores_pid as vm
import time

vm.ir_a_posicion(1, 15, 100, esperar=False)
vm.ir_a_posicion(2, -15, 75, esperar=False)
vm.ir_a_posicion(3, 15, 50, esperar=False)
vm.ir_a_posicion(4, -15, 25, esperar=False)
while not (vm.bllego(1) and vm.bllego(2) and vm.bllego(3) and vm.bllego(4)):
      time.sleep(0.05)