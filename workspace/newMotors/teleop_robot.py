#!/usr/bin/env python3
"""
teleop_robot.py  -  Maneja los 4 motores como un robot desde el teclado.

Motores izquierdos: 1 y 2   |   Motores derechos: 3 y 4

Teclas (el robot SOLO se mueve mientras mantienes la tecla; al soltarla, para):
    W / S        adelante / atrás
    A / D        giro a la izquierda / derecha
                 (A: 1,2 atrás y 3,4 frente  |  D: 1,2 frente y 3,4 atrás)
    W + A / W + D   curva (se pueden mantener dos teclas a la vez)
    ESPACIO / X  parar todo (y cancelar las vueltas en curso)
    + / -        subir / bajar la velocidad
    U / J        motores 1 y 2: una vuelta al frente / atrás
    I / K        motores 3 y 4: una vuelta al frente / atrás
    Q (o Esc)    salir (para los motores)

Para detectar cuándo sueltas una tecla se usa la librería pynput:
    pip install pynput
Necesita un escritorio gráfico (teclado conectado a la misma máquina). Atención: pynput
lee el teclado aunque la ventana de la terminal no tenga el foco.

Si pynput no está instalado o no hay pantalla (por ejemplo por SSH), el programa pasa solo
al "modo terminal": ahí la terminal no avisa cuándo sueltas una tecla, así que W/A/S/D
mantienen la última orden hasta que pulses otra tecla o ESPACIO.

Necesita Linux/macOS y vesc_motores_pid.py en la misma carpeta.
"""

import os
import select
import signal
import sys
import termios
import threading
import time
import tty

import vesc_motores_pid as vm

# =====================================================================
# CONFIGURACIÓN
# =====================================================================

IZQ = (1, 2)
DER = (3, 4)

# Sentido de cada motor. Si al pulsar W alguna rueda gira hacia atrás, cámbiale el
# signo aquí (1 -> -1). Normalmente los motores de un lado van montados al revés.
SIGNO = {1: 1, 2: 1, 3: 1, 4: 1}

VEL_INICIAL = 15.0   # % de duty al arrancar el programa
VEL_MAX = 50.0       # tope de velocidad con las teclas + / -
VEL_PASO = 5.0       # cuánto cambia con cada + / -
VEL_POS = 25.0       # % de duty máximo en los movimientos de una vuelta (U J I K)
VUELTAS = 1.0        # vueltas de cada pulsación de U J I K

MODO = 'auto'        # 'auto' = pynput si se puede, si no terminal | 'terminal' = forzar terminal

# ---- Detección de teclas (modo pynput) ----
RETARDO_SOLTAR = 0.06   # s. Tolera los "soltar+pulsar" falsos que genera la repetición automática
MAX_SIN_EVENTO = 1.5    # s. Una tecla sin ningún evento (ni repetición) en este tiempo se da por
                        # soltada: evita que el robot siga andando si se pierde el aviso de soltar.
                        # Si tu sistema no repite teclas al mantenerlas, ponlo en 0 (desactivado).
PERIODO = 0.05          # s entre lecturas del teclado

# tecla -> (motores, sentido)
UNA_VUELTA = {
    'u': (IZQ, 1),
    'j': (IZQ, -1),
    'i': (DER, 1),
    'k': (DER, -1),
}

NORMALIZAR = {'=': '+', '_': '-'}   # + y - sin/con shift cuentan igual

NOMBRES = {(1, 1): 'ADELANTE', (-1, -1): 'ATRAS', (-1, 1): 'GIRO IZQ',
           (1, -1): 'GIRO DER', (0, 0): 'PARADO'}

AYUDA = """\
Control de robot por teclado  [{modo}]
  W adelante   S atrás   A giro izq   D giro der   (W+A / W+D = curva)
  ESPACIO/X parar     + / -  velocidad     Q salir
  U / J  motores 1,2: 1 vuelta frente / atrás
  I / K  motores 3,4: 1 vuelta frente / atrás
"""


# =====================================================================
# ENTRADA DE TECLADO  (dos formas de leer, misma interfaz)
#   leer() -> (teclas, nuevas)
#     teclas: conjunto de teclas que están presionadas ahora
#     nuevas: pulsaciones nuevas desde la lectura anterior (sin repeticiones)
# =====================================================================

def _vaciar_terminal(fd):
    """Descarta lo que se haya tecleado en la terminal (para que no se acumule ni llegue al shell)."""
    if fd is None:
        return
    while select.select([fd], [], [], 0)[0]:
        if not os.read(fd, 256):
            break


class EntradaPynput:
    """Pulsar y soltar reales. El robot solo se mueve mientras mantienes la tecla."""
    descripcion = "mantener tecla"

    def __init__(self, keyboard, fd):
        self._kb = keyboard
        self._fd = fd
        self._lock = threading.Lock()
        self._teclas = {}      # tecla -> [abajo, instante del último evento]
        self._nuevas = []
        self._escuchador = keyboard.Listener(on_press=self._al_pulsar, on_release=self._al_soltar)
        self._escuchador.start()

    def _nombre(self, tecla):
        if tecla == self._kb.Key.space:
            return ' '
        if tecla == self._kb.Key.esc:
            return 'q'
        c = getattr(tecla, 'char', None)
        return NORMALIZAR.get(c.lower(), c.lower()) if c else None

    def _presionada(self, nombre, ahora):
        est = self._teclas.get(nombre)
        if est is None:
            return False
        abajo, t = est
        if abajo:
            return MAX_SIN_EVENTO <= 0 or ahora - t < MAX_SIN_EVENTO
        return ahora - t < RETARDO_SOLTAR

    def _al_pulsar(self, tecla):
        n = self._nombre(tecla)
        if n is None:
            return
        ahora = time.monotonic()
        with self._lock:
            if not self._presionada(n, ahora):     # las repeticiones no cuentan como pulsación nueva
                self._nuevas.append(n)
            self._teclas[n] = [True, ahora]

    def _al_soltar(self, tecla):
        n = self._nombre(tecla)
        if n is None:
            return
        with self._lock:
            self._teclas[n] = [False, time.monotonic()]

    def leer(self):
        time.sleep(PERIODO)
        _vaciar_terminal(self._fd)
        ahora = time.monotonic()
        with self._lock:
            teclas = {n for n in self._teclas if self._presionada(n, ahora)}
            nuevas, self._nuevas = self._nuevas, []
        return teclas, nuevas

    def cerrar(self):
        self._escuchador.stop()


class EntradaTerminal:
    """
    Solo recibe pulsaciones (no sabe cuándo se suelta). W/A/S/D mantienen la última
    orden hasta pulsar otra tecla o ESPACIO.
    """
    descripcion = "terminal: la orden se mantiene, ESPACIO para parar"

    def __init__(self, fd):
        self._fd = fd
        self._orden = set()

    def leer(self):
        nuevas = []
        datos = b''
        if select.select([self._fd], [], [], PERIODO)[0]:
            datos = os.read(self._fd, 64)
        if datos.startswith(b'\x1b'):                 # flechas y otras teclas especiales
            datos = b''
        for b in datos:
            t = chr(b).lower()
            t = NORMALIZAR.get(t, t)
            if t in ('w', 'a', 's', 'd'):
                self._orden = {t}
            else:
                nuevas.append(t)
        if ' ' in nuevas or 'x' in nuevas:
            self._orden = set()
        return set(self._orden), nuevas

    def cerrar(self):
        pass


def crear_entrada(fd):
    if MODO != 'terminal':
        try:
            from pynput import keyboard
            return EntradaPynput(keyboard, fd)
        except Exception as ex:       # sin pynput, sin pantalla (SSH), etc.
            print(f"[teleop] No se pudo usar pynput ({type(ex).__name__}); uso el modo terminal.")
    if fd is None:
        sys.exit("Sin pynput este programa necesita una terminal interactiva.")
    return EntradaTerminal(fd)


# =====================================================================
# ACCIONES
# =====================================================================

def orden_de_teclas(teclas):
    """Convierte las teclas presionadas en (izquierda, derecha), cada una de -1 a 1."""
    avance = ('w' in teclas) - ('s' in teclas)
    giro = ('d' in teclas) - ('a' in teclas)
    return (max(-1, min(1, avance + giro)), max(-1, min(1, avance - giro)))


def conducir(orden, vel):
    """Aplica una orden (izq, der) con el signo de cada motor y la velocidad dada."""
    izq, der = orden
    for m in IZQ:
        vm.mover(m, SIGNO[m] * izq * vel)
    for m in DER:
        vm.mover(m, SIGNO[m] * der * vel)


def una_vuelta(motores, sentido, objetivos):
    """
    Mueve esos motores VUELTAS hacia el frente (sentido=1) o atrás (-1).
    Si pulsas varias veces seguidas, las vueltas se suman (el objetivo se acumula).
    """
    for m in motores:
        base = objetivos[m]
        if base is None or vm.batascado(m):      # sin movimiento previo: parte de donde está
            base = vm.lposicion(m)
        if base is None:                         # sin lectura del motor: no se mueve
            continue
        objetivos[m] = base + sentido * SIGNO[m] * VUELTAS
        vm.ir_a_posicion(m, objetivos[m], VEL_POS, esperar=False)


def linea_estado(nombre, vel):
    partes = []
    for m in vm.MOTORES:
        pos = vm.lposicion(m)
        txt = f"{pos:+7.2f}" if pos is not None else "   --  "
        partes.append(f"M{m}{txt}{'!' if vm.batascado(m) else ' '}")
    return f"[{nombre:<10} {vel:3.0f}%] " + " ".join(partes) + "  (! = atascado)"


# =====================================================================
# BUCLE PRINCIPAL
# =====================================================================

def bucle(entrada):
    vel = VEL_INICIAL
    previo = (0, 0)                              # última orden de conducción enviada
    nombre = 'PARADO'
    objetivos = {m: None for m in vm.MOTORES}    # objetivos acumulados de U J I K
    ignoradas = set()                            # teclas que siguen abajo tras un ESPACIO
    t_estado = 0.0

    while True:
        teclas, nuevas = entrada.leer()

        for ev in nuevas:
            if ev == 'q':
                return
            elif ev in (' ', 'x'):               # parar todo, aunque haya teclas abajo
                conducir((0, 0), vel)
                previo, nombre = (0, 0), 'PARADO'
                objetivos = {m: None for m in vm.MOTORES}
                ignoradas = set(teclas)
            elif ev in UNA_VUELTA:
                motores, sentido = UNA_VUELTA[ev]
                una_vuelta(motores, sentido, objetivos)
                nombre = 'UNA VUELTA'
            elif ev == '+':
                vel = min(VEL_MAX, vel + VEL_PASO)
                if previo != (0, 0):
                    conducir(previo, vel)
            elif ev == '-':
                vel = max(VEL_PASO, vel - VEL_PASO)
                if previo != (0, 0):
                    conducir(previo, vel)

        ignoradas &= teclas                      # una tecla ignorada vuelve a valer al soltarla
        orden = orden_de_teclas(teclas - ignoradas)
        if orden != previo:                      # solo se envía cuando cambia
            conducir(orden, vel)
            previo = orden
            nombre = NOMBRES.get(orden, 'CURVA')
            objetivos = {m: None for m in vm.MOTORES}

        ahora = time.monotonic()
        if ahora - t_estado >= 0.1:
            sys.stdout.write("\r\x1b[K" + linea_estado(nombre, vel))
            sys.stdout.flush()
            t_estado = ahora


def _salir(*_):
    raise SystemExit(0)        # así el bloque finally de main() para los motores


def main():
    # Si se cierra la terminal o el SSH, también se detienen los motores
    signal.signal(signal.SIGTERM, _salir)
    signal.signal(signal.SIGHUP, _salir)

    try:
        vm.iniciar()
    except RuntimeError as ex:
        sys.exit(f"Error: {ex}")

    fd = sys.stdin.fileno() if sys.stdin.isatty() else None
    ajustes = termios.tcgetattr(fd) if fd is not None else None
    entrada = None
    try:
        if fd is not None:
            tty.setcbreak(fd)                    # sin eco y sin Enter; Ctrl+C sigue funcionando
        entrada = crear_entrada(fd)
        print(AYUDA.format(modo=entrada.descripcion))
        bucle(entrada)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            vm.cerrar()                          # para los 4 motores y cierra el puerto
        finally:
            if entrada is not None:
                entrada.cerrar()
            if fd is not None:
                termios.tcflush(fd, termios.TCIFLUSH)     # tira lo tecleado y no enviado
                termios.tcsetattr(fd, termios.TCSADRAIN, ajustes)
            print("\nMotores detenidos.")


if __name__ == '__main__':
    main()
