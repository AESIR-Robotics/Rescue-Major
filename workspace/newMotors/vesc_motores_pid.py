"""
vesc_motores_pid.py  -  Control sencillo de 4 motores VESC (1 master + 3 slaves por CAN)
Versión con PID de posición.

Uso rápido:
    import vesc_motores_pid as vm

    vm.mover(1, 20)                            # motor 1 gira al 20 % (hacia adelante)
    vm.mover(3, -15)                           # motor 3 gira al 15 % en reversa
    vm.ir_a_posicion(2, 5, 25)                 # motor 2 va a 5 vueltas (max 25 % de duty)

    vm.ir_a_posicion(1, 3, 50, esperar=False)  # lanza varios movimientos a la vez...
    vm.ir_a_posicion(4, -3, 50, esperar=False)
    vm.esperar_llegada(timeout=10)             # ...y espera (con límite de tiempo)

    print(vm.lposicion(2), vm.lrpm(2))
    vm.parar_todo()

Funciones de lectura (todas empiezan con "l"):
    lposicion, lrpm, lvoltaje, lestado
Funciones que devuelven True/False (empiezan con "b"):
    bllego, batascado

Dependencias: pip install pyserial PyCRC
"""

import atexit
import math
import struct
import threading
import time

import serial
from PyCRC.CRCCCITT import CRCCCITT

# =====================================================================
# 1. CONFIGURACIÓN (ajusta aquí)
# =====================================================================

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

# Número de motor -> ID CAN (None = el VESC master, conectado por USB)
IDS = {1: None, 2: 45, 3: 70, 4: 118}

POLE_PAIRS = 6       # pares de polos de tus motores
ESCALA = 16          # divisor que ya calibraste
TACHO_POR_VUELTA = 6 * POLE_PAIRS * ESCALA   # cuentas del tacómetro por vuelta de posición

# ---- Control de posición (PID) ----
KP = 100.0           # % de duty por cada vuelta de error
KI = 10.0            # % de duty por (vuelta * segundo) de error acumulado
KD = 10.0            # % de duty por cada (vuelta/s) de velocidad (frena al acercarse)
DUTY_MIN = 5.0       # % mínimo para vencer la fricción cuando aún falta llegar
TOLERANCIA = 0.001   # vueltas de error aceptadas para dar por "llegado"

INT_ZONA = 0.5       # el integral solo acumula si el error es menor a estas vueltas
INT_MAX = 15.0       # tope (en % de duty) de lo que puede aportar el integral
FILTRO_D = 0.5       # filtro de la velocidad para el término D (1 = sin filtro, menor = más suave)

# ---- Rampa de duty (suaviza arranques; pon RAMPA = 0 para desactivarla) ----
RAMPA = 50.0           # % de duty por segundo al ACELERAR
RAMPA_FRENADO = 100.0  # % de duty por segundo al REDUCIR velocidad

# ---- Seguridad ----
TIEMPO_ATASCO = 3.0    # s sin avanzar hacia el objetivo antes de parar el motor (0 = desactivar)
MOV_ATASCO = 0.01      # vueltas mínimas que debe avanzar en ese tiempo para no considerarse atascado
TOL_MIN = 1 / TACHO_POR_VUELTA   # menor tolerancia útil: una cuenta del tacómetro
TIMEOUT_LECTURA = 0.1  # s máximos esperando la respuesta de un VESC
PAUSA_CICLO = 0.005    # s de descanso entre vueltas completas del hilo de control

# ---- Comandos del protocolo VESC ----
COMM_FORWARD_CAN = 34
COMM_SET_DUTY = 5
COMM_GET_VALUES = 4

# =====================================================================
# 2. ESTADO INTERNO
# =====================================================================

MOTORES = sorted(IDS)

_ser = None
_hilo = None
_activo = False
_lock = threading.Lock()          # protege _estado (solo cálculos, nunca E/S serie)
_lock_inicio = threading.Lock()   # evita abrir el puerto dos veces

_crc = CRCCCITT()
# Formato de la respuesta GET_VALUES, en orden: temp_fet, temp_motor, i_motor, i_in, i_d,
# i_q, duty, erpm, v_in, ah, ah_chg, wh, wh_chg, tacho, tacho_abs, fault
_VALORES = struct.Struct('>hhiiiihihiiiiiib')


def _nuevo_estado():
    return {
        # --- Orden actual ---
        'modo': 'duty',      # 'duty' o 'pos'
        'duty': 0.0,         # % pedido en modo duty
        'objetivo': 0.0,     # vueltas objetivo en modo pos
        'vmax': 20.0,        # % máximo en modo pos
        'tol': TOLERANCIA,
        'llego': False,
        'atascado': False,
        # --- Lecturas ---
        'tacho_cero': None,
        'pos': None,         # vueltas
        'rpm': None,
        'v_in': None,
        'fault': None,
        'ok': False,         # última lectura válida
        # --- PID ---
        'integral': 0.0,     # aporte del término I (en % de duty)
        'vel': 0.0,          # velocidad filtrada (vueltas/s)
        'pos_prev': None,
        't_prev': None,
        'dt': 0.0,
        # --- Rampa ---
        'duty_out': 0.0,     # duty que realmente se envía
        't_ult': None,
        # --- Detección de atasco ---
        'ref_pos': None,
        't_ref': None,
    }


_estado = {m: _nuevo_estado() for m in MOTORES}


def _limitar(x, maximo):
    """Recorta x al rango [-maximo, +maximo]."""
    return max(-maximo, min(maximo, x))


# =====================================================================
# 3. COMUNICACIÓN CON EL VESC (bajo nivel)
# =====================================================================

def _enviar_payload(payload, can_id=None):
    if can_id is not None:
        payload = bytes([COMM_FORWARD_CAN, can_id]) + payload
    crc = _crc.calculate(payload)
    _ser.write(bytes([2, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 3]))


def _leer_respuesta(timeout=TIMEOUT_LECTURA):
    """Lee un paquete del puerto. Devuelve su payload, o None si no llegó uno válido a tiempo."""
    buf = bytearray()
    t_fin = time.monotonic() + timeout
    while time.monotonic() < t_fin:
        buf += _ser.read(_ser.in_waiting or 1)
        i = buf.find(2)                       # byte de inicio de paquete corto
        if i < 0:
            buf.clear()                       # basura sin inicio: se descarta
            continue
        if len(buf) < i + 2:
            continue
        n = buf[i + 1]
        fin = i + 2 + n + 3                   # inicio + largo + payload + CRC(2) + final
        if len(buf) < fin:
            continue
        payload = bytes(buf[i + 2: i + 2 + n])
        crc_rx = (buf[i + 2 + n] << 8) | buf[i + 3 + n]
        if buf[fin - 1] == 3 and crc_rx == _crc.calculate(payload):
            return payload
        del buf[:i + 1]                       # paquete corrupto: sigue buscando el siguiente
    return None


def _leer_valores(can_id):
    """Pide y decodifica los valores de un VESC. Devuelve dict o None si no respondió bien."""
    _ser.reset_input_buffer()
    _enviar_payload(bytes([COMM_GET_VALUES]), can_id)
    p = _leer_respuesta()
    if not p or p[0] != COMM_GET_VALUES or len(p) < 1 + _VALORES.size:
        return None
    _, _, _, _, _, _, _, erpm, v_in, _, _, _, _, tacho, _, fault = _VALORES.unpack_from(p, 1)
    return {
        'rpm': erpm / POLE_PAIRS / ESCALA,
        'v_in': v_in / 10,
        'tacho': tacho,
        'fault': fault,
    }


def _enviar_duty(motor, pct):
    """pct en % (-100 a 100). Equivale a SetDutyCycle(pct * 1000)."""
    if not math.isfinite(pct):
        pct = 0.0
    pct = _limitar(pct, 100.0)
    _enviar_payload(struct.pack('>Bi', COMM_SET_DUTY, int(pct * 1000)), IDS[motor])


def _reabrir_puerto():
    """Intenta recuperar el puerto serie tras un error (por ejemplo, un cable flojo)."""
    try:
        _ser.close()
        _ser.open()
    except Exception:
        pass


# =====================================================================
# 4. CONTROL (se ejecuta dentro del hilo de fondo)
# =====================================================================

def _actualizar_lectura(e, v, ahora):
    """Guarda la lectura nueva y actualiza posición y velocidad filtrada."""
    if v is None:
        e['ok'] = False
        return
    if e['tacho_cero'] is None:
        e['tacho_cero'] = v['tacho']
    pos = (v['tacho'] - e['tacho_cero']) / TACHO_POR_VUELTA

    # velocidad filtrada (término D). Si pasó mucho tiempo desde la lectura anterior
    # (hubo fallos de comunicación) no se usa, daría una velocidad falsa.
    gap = ahora - e['t_prev'] if e['t_prev'] is not None else 0.0
    if e['pos_prev'] is not None and 0 < gap <= 0.2:
        e['vel'] += FILTRO_D * ((pos - e['pos_prev']) / max(gap, 1e-3) - e['vel'])
    e['dt'] = min(gap, 0.2)
    e['pos_prev'] = pos
    e['t_prev'] = ahora

    e['pos'] = pos
    e['rpm'] = v['rpm']
    e['v_in'] = v['v_in']
    e['fault'] = v['fault']
    e['ok'] = True


def _pid(e, err):
    """Calcula el duty del PID de posición para un motor (err en vueltas)."""
    p = KP * err
    d = -KD * e['vel']                      # derivada sobre la medición (sin patada al cambiar objetivo)

    # Integral con anti-windup: solo acumula cerca del objetivo y si P+D no saturan
    if abs(err) < INT_ZONA:
        if abs(p + d) < e['vmax']:
            e['integral'] = _limitar(e['integral'] + KI * err * e['dt'], INT_MAX)
    else:
        e['integral'] = 0.0

    duty = _limitar(p + e['integral'] + d, e['vmax'])

    duty_min = min(DUTY_MIN, e['vmax'])     # nunca pasa del tope pedido
    if abs(duty) < duty_min:                # vence la fricción mientras no se llegue
        duty = duty_min if err > 0 else -duty_min
    return duty


def _sin_avance(e, ahora):
    """True si el motor lleva TIEMPO_ATASCO segundos sin avanzar (atorado o sin fuerza)."""
    if TIEMPO_ATASCO <= 0:
        return False
    if e['t_ref'] is None or abs(e['pos'] - e['ref_pos']) >= MOV_ATASCO:
        e['ref_pos'], e['t_ref'] = e['pos'], ahora   # avanzó: reinicia la ventana
        return False
    return ahora - e['t_ref'] > TIEMPO_ATASCO


def _calcular_duty(e, motor, ahora):
    """Duty que se quiere para este motor según su modo."""
    if e['modo'] != 'pos':
        return e['duty']

    if not e['ok']:                          # sin lectura: por seguridad, no mover
        e['t_ref'] = None
        return 0.0

    err = e['objetivo'] - e['pos']
    if abs(err) <= e['tol']:
        e['llego'] = True
        e['integral'] = 0.0
        e['t_ref'] = None
        return 0.0

    e['llego'] = False
    if _sin_avance(e, ahora):
        e['atascado'] = True
        e['modo'], e['duty'] = 'duty', 0.0   # se detiene y espera una orden nueva
        print(f"[vesc] Motor {motor} atascado (sin avanzar en {TIEMPO_ATASCO:g} s): detenido.")
        return 0.0
    return _pid(e, err)


def _aplicar_rampa(e, duty, ahora):
    """Limita cuánto puede cambiar el duty enviado por segundo."""
    if RAMPA <= 0:
        e['duty_out'] = duty
        return duty
    dt = min(ahora - e['t_ult'], 0.2) if e['t_ult'] is not None else 0.0
    e['t_ult'] = ahora
    tasa = RAMPA if abs(duty) > abs(e['duty_out']) else RAMPA_FRENADO
    paso = tasa * dt
    e['duty_out'] += max(-paso, min(paso, duty - e['duty_out']))
    return e['duty_out']


def _ciclo_motor(m):
    """Un ciclo completo para un motor: leer -> calcular -> enviar."""
    e = _estado[m]
    v = _leer_valores(IDS[m])                # la E/S serie va fuera del lock
    ahora = time.monotonic()
    with _lock:
        _actualizar_lectura(e, v, ahora)
        duty = _calcular_duty(e, m, ahora)
        duty_out = _aplicar_rampa(e, duty, ahora)
    _enviar_duty(m, duty_out)


def _bucle():
    """
    Hilo de fondo: atiende a cada motor y reenvía el duty constantemente para que
    el VESC no active su timeout. Un error en un motor nunca debe matar el hilo.
    """
    avisados = set()
    while _activo:
        for m in MOTORES:
            try:
                _ciclo_motor(m)
                avisados.discard(m)
            except Exception as ex:
                _estado[m]['ok'] = False
                if m not in avisados:        # avisa una vez por racha de errores, sin inundar
                    print(f"[vesc] Error con el motor {m}: {ex!r}")
                    avisados.add(m)
                if isinstance(ex, (serial.SerialException, OSError)):
                    time.sleep(0.5)
                    _reabrir_puerto()
        time.sleep(PAUSA_CICLO)


# =====================================================================
# 5. API PÚBLICA
# =====================================================================

def iniciar(puerto=None):
    """Abre el puerto y arranca el control. Se llama sola la primera vez que usas una función."""
    global _ser, _hilo, _activo
    if _activo:
        return
    with _lock_inicio:
        if _activo:
            return
        try:
            _ser = serial.Serial(puerto or SERIAL_PORT, baudrate=BAUDRATE, timeout=0.01)
        except Exception as ex:
            raise RuntimeError(f"No se pudo abrir el puerto serie: {ex}") from ex
        _activo = True
        _hilo = threading.Thread(target=_bucle, daemon=True)
        _hilo.start()

    # Espera la primera lectura de todos para fijar el cero de posición
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2:
        if all(_estado[m]['pos'] is not None for m in MOTORES):
            return
        time.sleep(0.02)
    sin = [m for m in MOTORES if _estado[m]['pos'] is None]
    print(f"[vesc] Aviso: sin respuesta de los motores {sin} (revisa cables, IDs CAN y alimentación).")


def _motor(motor):
    """Valida el número de motor, asegura que el control esté corriendo y devuelve su estado."""
    if motor not in IDS:
        raise ValueError(f"Motor inválido: {motor}. Usa {MOTORES}")
    iniciar()
    if not _hilo.is_alive():
        raise RuntimeError("El hilo de control se detuvo. Reinicia el programa.")
    return _estado[motor]


def _num(valor, nombre):
    """Convierte a float y rechaza texto, None, NaN e infinitos."""
    try:
        x = float(valor)
    except (TypeError, ValueError):
        raise ValueError(f"{nombre} debe ser un número, recibí {valor!r}") from None
    if not math.isfinite(x):
        raise ValueError(f"{nombre} debe ser un número finito, recibí {valor!r}")
    return x


def _esperar(motores, timeout):
    """Espera hasta que todos lleguen. False si hay timeout, atasco o se detiene el control."""
    t_fin = time.monotonic() + timeout
    while time.monotonic() < t_fin:
        estados = [_estado[m] for m in motores]
        if all(e['llego'] for e in estados):
            return True
        if any(e['atascado'] for e in estados) or not _hilo.is_alive():
            return False
        time.sleep(0.01)
    return False


# ---------------------------------------------------------------------
# Órdenes
# ---------------------------------------------------------------------

def mover(motor, velocidad):
    """
    Hace girar el motor a una velocidad dada (duty en %, de -100 a 100).
    Positivo = un sentido, negativo = el contrario, 0 = parar.
    Sigue girando hasta que llames a mover(motor, 0), parar() o parar_todo().
    """
    e = _motor(motor)
    v = _limitar(_num(velocidad, 'velocidad'), 100.0)
    with _lock:
        e['modo'] = 'duty'
        e['duty'] = v
        e['llego'] = False
        e['atascado'] = False


def ir_a_posicion(motor, vueltas, velocidad=20, relativo=False,
                  esperar=True, tolerancia=None, timeout=15):
    """
    Lleva el motor a una posición, medida en vueltas del motor.
    La posición 0 es donde estaba el motor al iniciar (o donde llamaste poner_cero).

    vueltas   : posición objetivo (puede ser negativa o decimal, ej. 2.5)
    velocidad : duty máximo en % que usará para llegar (siempre positivo)
    relativo  : si True, 'vueltas' es cuánto girar desde donde está ahora
    esperar   : si True, la función se bloquea hasta llegar (o hasta el timeout)
    Devuelve True si llegó. False si se agotó el timeout, el motor se atascó
    (ver batascado) o si esperar=False.
    """
    e = _motor(motor)
    vueltas = _num(vueltas, 'vueltas')
    vmax = min(abs(_num(velocidad, 'velocidad')), 100.0)
    tol = max(_num(tolerancia, 'tolerancia') if tolerancia is not None else TOLERANCIA, TOL_MIN)

    with _lock:
        if relativo:
            if e['pos'] is None:
                raise RuntimeError(f"Motor {motor} sin lectura de posición: no se puede mover en relativo.")
            vueltas += e['pos']
        e.update(objetivo=vueltas, vmax=vmax, tol=tol, llego=False, atascado=False,
                 integral=0.0, t_ref=None, modo='pos')   # el PID arranca limpio

    if not esperar:
        return False
    return _esperar([motor], timeout)


def esperar_llegada(motores=None, timeout=15):
    """
    Espera a que los motores indicados (por defecto los 4) lleguen a su posición.
    Devuelve True si todos llegaron; False si se agotó el timeout o alguno se atascó.
    Úsala en lugar de un while sobre bllego(): no se queda colgada para siempre.
    """
    lista = MOTORES if motores is None else list(motores)
    for m in lista:
        _motor(m)
    return _esperar(lista, timeout)


def poner_cero(motor=None):
    """Define la posición actual como 0. Sin argumento lo hace en los 4 motores."""
    iniciar()
    for m in (MOTORES if motor is None else [motor]):
        e = _motor(m)
        with _lock:
            if e['pos'] is None:
                continue
            pos = e['pos']
            e['tacho_cero'] += pos * TACHO_POR_VUELTA   # acumula el desplazamiento actual
            e['objetivo'] -= pos     # el objetivo pasa al nuevo cero (si no, el motor saltaría)
            e['pos'] = 0.0
            e['pos_prev'] = None     # evita un salto falso en la velocidad
            e['t_ref'] = None


def parar(motor):
    """Detiene un motor."""
    mover(motor, 0)


def parar_todo():
    """Detiene los 4 motores."""
    if not _activo:
        return
    for m in MOTORES:
        mover(m, 0)


# ---------------------------------------------------------------------
# Lecturas (empiezan con "l")
# ---------------------------------------------------------------------

def lposicion(motor):
    """Posición actual en vueltas del motor (relativa al cero). None si aún no hay lectura."""
    return _motor(motor)['pos']


def lrpm(motor):
    """Velocidad actual en rpm del motor."""
    return _motor(motor)['rpm']


def lvoltaje(motor):
    """Voltaje de entrada (batería) que ve ese VESC."""
    return _motor(motor)['v_in']


def bllego(motor):
    """True si el motor ya alcanzó la posición pedida con ir_a_posicion."""
    return _motor(motor)['llego']


def batascado(motor):
    """True si el motor se detuvo por no avanzar hacia su objetivo (ver TIEMPO_ATASCO)."""
    return _motor(motor)['atascado']


def lestado():
    """Imprime posición y rpm de los 4 motores (usa los últimos valores leídos)."""
    iniciar()
    partes = []
    for m in MOTORES:
        e = _estado[m]
        if not e['ok']:
            partes.append(f"M{m}: sin respuesta")
        else:
            marca = " ATASCADO" if e['atascado'] else ""
            partes.append(f"M{m}: {e['pos']:8.2f} rev | {e['rpm']:6.0f} rpm{marca}")
    print(" || ".join(partes))


# ---------------------------------------------------------------------
# Cierre
# ---------------------------------------------------------------------

def cerrar():
    """Para los motores y cierra el puerto. Se llama sola al salir del programa."""
    global _activo
    if not _activo:
        return
    parar_todo()
    time.sleep(0.2)          # deja que el hilo envíe el duty 0
    _activo = False
    if _hilo:
        _hilo.join(timeout=1)
    for m in MOTORES:        # cada motor por separado: un fallo no debe impedir parar los demás
        _estado[m]['duty_out'] = 0.0
        try:
            _enviar_duty(m, 0.0)
        except Exception:
            pass
    try:
        _ser.close()
    except Exception:
        pass


atexit.register(cerrar)


# =====================================================================
# DEMO:  python3 vesc_motores_pid.py
# =====================================================================

if __name__ == '__main__':
    print("Girando motor 1 al 20% por 3 s...")
    mover(1, 20)
    for _ in range(30):
        lestado()
        time.sleep(0.1)
    parar(1)

    print("Motor 2 a 3 vueltas...")
    ok = ir_a_posicion(2, 3, velocidad=25)
    print("Llegó" if ok else "No llegó", "| posición:", lposicion(2))

    print("Todos los motores a 0...")
    for m in MOTORES:
        ir_a_posicion(m, 0, velocidad=25, esperar=False)
    esperar_llegada(timeout=10)
    lestado()
    cerrar()
