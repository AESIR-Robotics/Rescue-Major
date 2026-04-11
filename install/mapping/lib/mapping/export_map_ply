#!/usr/bin/env python3
"""
export_map_ply.py — Equipo Aesir / RoboCup 2026
─────────────────────────────────────────────────
Exporta el mapa 3-D de RTAB-Map a PLY compatible con las reglas
de RoboCup Rescue 2026.

Datos fijos:
  • Año   : 2026
  • Equipo: Aesir

Datos manuales al ejecutar:
  • Número de misión
  • Fecha y hora de inicio (para el nombre del archivo y el header)

Requisitos del reglamento cumplidos:
  ✓ Formato ASCII PLY
  ✓ Campos x, y, z en metros (float)
  ✓ Campos RGB — bonus point multiplier
  ✓ Campos nx, ny, nz — normales (bonus)
  ✓ Nombre: RoboCup2026-Aesir-[Misión]-[HH-MM-SS]-map.ply
  ✓ Escala en metros (RTAB-Map trabaja en metros nativamente)
  ✓ Origen (0,0,0) = posición inicial del robot
  ✓ +Y = frente del robot, Z = vertical
  ✓ Header PLY con comentarios de equipo, tiempo y misión
  ✓ Objetos detectados por YOLO anotados en el header

Uso:
    python3 export_map_ply.py

    El script pedirá interactivamente:
      - Número de misión
      - Fecha y hora de inicio

Dependencias:
    pip install open3d numpy --break-system-packages
    rtabmap-export debe estar disponible en PATH
    (viene con la instalación de rtabmap en ROS 2 Humble)
"""

import datetime
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# DATOS FIJOS DEL EQUIPO
# ─────────────────────────────────────────────────────────────────────────────
TEAM_NAME    = 'Aesir'
ROBOCUP_YEAR = 2026

# ─────────────────────────────────────────────────────────────────────────────
# RUTAS POR DEFECTO (ajusta si cambian en tu sistema)
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_DB_PATH      = '~/.ros/rtabmap.db'
DEFAULT_OBJECTS_JSON = '~/.ros/detected_objects.json'
# Carpeta maps dentro del paquete mapping (relativa al script)
_SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR   = str(_SCRIPT_DIR.parent / 'maps')

# ─────────────────────────────────────────────────────────────────────────────
# PARÁMETROS DE CALIDAD DEL MAPA
# ─────────────────────────────────────────────────────────────────────────────
VOXEL_SIZE_M  = 0.05   # 5 cm — resolución final del PLY
NORMAL_RADIUS = 0.10   # radio para estimación de normales
NORMAL_MAX_NN = 30     # vecinos máximos para normales

# ─────────────────────────────────────────────────────────────────────────────
# Verificar open3d
# ─────────────────────────────────────────────────────────────────────────────
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print('[AVISO] open3d no está instalado. Se usará modo sin post-proceso.')
    print('        Para instalarlo: pip install open3d --break-system-packages\n')


# ─────────────────────────────────────────────────────────────────────────────
# INPUT INTERACTIVO
# ─────────────────────────────────────────────────────────────────────────────

def pedir_datos_mision() -> tuple[int, datetime.datetime]:
    """
    Pide al usuario el número de misión y la fecha/hora de inicio.
    Devuelve (mission_number, start_datetime).
    """
    print('=' * 60)
    print(f'  EXPORTADOR DE MAPA — Equipo {TEAM_NAME} / RoboCup {ROBOCUP_YEAR}')
    print('=' * 60)
    print()

    # ── Número de misión ─────────────────────────────────────────────────
    while True:
        try:
            mission = int(input('  Número de misión [1/2/3...]: ').strip())
            if mission > 0:
                break
            print('  El número debe ser mayor que 0.')
        except ValueError:
            print('  Ingresa un número entero válido.')

    # ── Fecha y hora de inicio ───────────────────────────────────────────
    print()
    print('  Fecha y hora de inicio de la misión:')
    print('  (Presiona Enter para usar el valor actual)')

    # Fecha
    while True:
        fecha_str = input('  Fecha [DD/MM/YYYY] (Enter = hoy): ').strip()
        if fecha_str == '':
            fecha = datetime.date.today()
            break
        try:
            fecha = datetime.datetime.strptime(fecha_str, '%d/%m/%Y').date()
            break
        except ValueError:
            print('  Formato inválido. Ejemplo: 15/07/2026')

    # Hora
    while True:
        hora_str = input('  Hora de inicio [HH:MM:SS] (Enter = ahora): ').strip()
        if hora_str == '':
            hora = datetime.datetime.now().time().replace(microsecond=0)
            break
        try:
            hora = datetime.datetime.strptime(hora_str, '%H:%M:%S').time()
            break
        except ValueError:
            print('  Formato inválido. Ejemplo: 14:30:00')

    start_dt = datetime.datetime.combine(fecha, hora)

    print()
    print(f'  ✓ Equipo  : {TEAM_NAME}')
    print(f'  ✓ Misión  : {mission}')
    print(f'  ✓ Inicio  : {start_dt.strftime("%d/%m/%Y  %H:%M:%S")}')
    print()

    confirmacion = input('  ¿Confirmar y exportar? [S/n]: ').strip().lower()
    if confirmacion == 'n':
        print('  Exportación cancelada.')
        sys.exit(0)

    return mission, start_dt


def make_filename(mission: int, start_time: datetime.datetime) -> str:
    """
    Formato reglamento: RoboCup[Year]-[Teamname]-[Mission]-[HH-MM-SS]-map.ply
    Ejemplo: RoboCup2026-Aesir-1-14-30-00-map.ply
    """
    ts = start_time.strftime('%H-%M-%S')
    return f'RoboCup{ROBOCUP_YEAR}-{TEAM_NAME}-{mission}-{ts}-map.ply'


# ─────────────────────────────────────────────────────────────────────────────
# EXPORTAR DB → PLY TEMPORAL
# ─────────────────────────────────────────────────────────────────────────────

def export_from_rtabmap_db(db_path: str, output_dir: str) -> str | None:
    """
    Exporta la nube de puntos de la base de datos RTAB-Map.
    Retorna la ruta del archivo PLY generado o None si falla.
    """
    db_path = str(Path(db_path).expanduser())
    output_dir = str(Path(output_dir).expanduser())
    
    # rtabmap-export guarda el archivo como {output_dir}/rtabmap_cloud.ply
    cmd = [
        'rtabmap-export',
        '--cloud',
        '--output_dir', output_dir,
        db_path
    ]
    print('[INFO] Ejecutando rtabmap-export…')
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f'[ERROR] rtabmap-export falló:\n{result.stderr}')
        return None
    
    # El archivo se genera como rtabmap_cloud.ply en output_dir
    generated_ply = Path(output_dir) / 'rtabmap_cloud.ply'
    if not generated_ply.exists():
        print(f'[ERROR] No se encontró el archivo generado: {generated_ply}')
        return None
    
    print(f'[INFO] Nube exportada a: {generated_ply}')
    return str(generated_ply)


# ─────────────────────────────────────────────────────────────────────────────
# POST-PROCESO CON OPEN3D
# ─────────────────────────────────────────────────────────────────────────────

def postprocess_cloud(tmp_ply: str) -> 'o3d.geometry.PointCloud':
    print('[INFO] Leyendo nube de puntos…')
    pcd = o3d.io.read_point_cloud(tmp_ply)
    n_orig = len(pcd.points)
    print(f'[INFO] Puntos originales: {n_orig:,}')

    # Verificar si la nube está vacía
    if n_orig == 0:
        print('[ERROR] La nube de puntos está vacía.')
        print('        Verifica que rtabmap.db tenga datos válidos.')
        print('        Asegúrate de haber mapeado algo antes de exportar.')
        sys.exit(1)

    pcd = pcd.voxel_down_sample(VOXEL_SIZE_M)
    print(f'[INFO] Tras voxel {VOXEL_SIZE_M*100:.0f}cm: {len(pcd.points):,} puntos')

    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f'[INFO] Tras filtro outlier: {len(pcd.points):,} puntos')

    # Solo estimar normales si hay suficientes puntos
    if len(pcd.points) < 3:
        print('[ERROR] Muy pocos puntos tras filtrar. Necesitas más datos.')
        sys.exit(1)

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=NORMAL_RADIUS, max_nn=NORMAL_MAX_NN
        )
    )
    pcd.orient_normals_towards_camera_location(
        camera_location=np.array([0.0, 0.0, 0.0])
    )
    print('[INFO] Normales estimadas y orientadas.')
    return pcd


# ─────────────────────────────────────────────────────────────────────────────
# LEER PLY MANUAL (fallback sin open3d)
# ─────────────────────────────────────────────────────────────────────────────

def read_ply_manual(ply_path: str):
    pts, colors, normals = [], [], []
    has_colors = has_normals = False
    in_header  = True

    with open(ply_path) as f:
        for line in f:
            line = line.strip()
            if in_header:
                if 'red' in line or 'green' in line or 'blue' in line:
                    has_colors = True
                if line.split()[-1] in ('nx', 'ny', 'nz'):
                    has_normals = True
                if line == 'end_header':
                    in_header = False
                continue
            vals = line.split()
            if len(vals) < 3:
                continue
            pts.append([float(vals[0]), float(vals[1]), float(vals[2])])
            if has_colors and len(vals) >= 6:
                colors.append([int(vals[3]), int(vals[4]), int(vals[5])])
            if has_normals:
                idx = 6 if has_colors else 3
                if len(vals) >= idx + 3:
                    normals.append([
                        float(vals[idx]),
                        float(vals[idx+1]),
                        float(vals[idx+2])
                    ])

    pts     = np.array(pts,     dtype=np.float32) if pts     else np.empty((0, 3))
    colors  = np.array(colors,  dtype=np.uint8)   if colors  else None
    normals = np.array(normals, dtype=np.float32) if normals else None
    return pts, colors, normals, has_colors, has_normals


# ─────────────────────────────────────────────────────────────────────────────
# ESCRIBIR PLY FINAL CONFORME AL REGLAMENTO
# ─────────────────────────────────────────────────────────────────────────────

def write_compliant_ply(
    data,
    out_path: Path,
    mission: int,
    start_time: datetime.datetime,
    detected_objects: list[dict],
):
    """
    Escribe el PLY con el header exacto del reglamento RoboCup Rescue.

    Conversión de coordenadas ROS → Reglamento:
      ROS map frame: x = frente del robot, y = izquierda, z = arriba
      Reglamento:    Y = frente del robot, X = derecha,   Z = arriba
      → PLY_X = -ROS_Y
        PLY_Y =  ROS_X
        PLY_Z =  ROS_Z
    """
    # ── Obtener arrays ───────────────────────────────────────────────────
    if HAS_OPEN3D and hasattr(data, 'points'):
        pts     = np.asarray(data.points, dtype=np.float32)
        has_colors  = data.has_colors()
        has_normals = data.has_normals()
        colors  = (np.asarray(data.colors) * 255).astype(np.uint8) if has_colors  else None
        normals = np.asarray(data.normals, dtype=np.float32)        if has_normals else None
    else:
        pts, colors, normals, has_colors, has_normals = read_ply_manual(data)

    n_pts = len(pts)
    if n_pts == 0:
        print('[ERROR] La nube de puntos está vacía.')
        sys.exit(1)

    # ── Rotación al sistema de coordenadas del reglamento ────────────────
    pts_ply = np.column_stack([
        -pts[:, 1],    # PLY_X = -ROS_Y
         pts[:, 0],    # PLY_Y =  ROS_X  (+Y = frente del robot)
         pts[:, 2],    # PLY_Z =  ROS_Z  (vertical)
    ])
    normals_ply = None
    if has_normals and normals is not None:
        normals_ply = np.column_stack([
            -normals[:, 1],
             normals[:, 0],
             normals[:, 2],
        ])

    # ── Header PLY según reglamento ──────────────────────────────────────
    header_lines = [
        'ply',
        'format ascii 1.0',
        f'comment {TEAM_NAME}',
        f'comment {start_time.strftime("%H:%M:%S")}',
        f'comment Mission {mission}',
    ]

    if detected_objects:
        header_lines.append(f'comment Detected_objects {len(detected_objects)}')
        for obj in detected_objects:
            header_lines.append(
                f'comment OBJECT label={obj["label"]} '
                f'x={obj["x"]:.3f} y={obj["y"]:.3f} z={obj["z"]:.3f} '
                f'conf={obj["confidence"]:.2f}'
            )

    header_lines += [
        f'element vertex {n_pts}',
        'property float x',
        'property float y',
        'property float z',
    ]
    if has_colors:
        header_lines += [
            'property uchar red',
            'property uchar green',
            'property uchar blue',
        ]
    if has_normals:
        header_lines += [
            'property float nx',
            'property float ny',
            'property float nz',
        ]
    header_lines.append('end_header')
    header = '\n'.join(header_lines) + '\n'

    # ── Escribir archivo ─────────────────────────────────────────────────
    print(f'[INFO] Escribiendo {n_pts:,} vértices en PLY…')
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with open(out_path, 'w') as f:
        f.write(header)
        CHUNK = 50_000
        for start in range(0, n_pts, CHUNK):
            end  = min(start + CHUNK, n_pts)
            rows = []
            for i in range(start, end):
                row = [
                    f'{pts_ply[i, 0]:.6f}',
                    f'{pts_ply[i, 1]:.6f}',
                    f'{pts_ply[i, 2]:.6f}',
                ]
                if has_colors:
                    row += [str(colors[i, 0]), str(colors[i, 1]), str(colors[i, 2])]
                if has_normals and normals_ply is not None:
                    row += [
                        f'{normals_ply[i, 0]:.6f}',
                        f'{normals_ply[i, 1]:.6f}',
                        f'{normals_ply[i, 2]:.6f}',
                    ]
                rows.append(' '.join(row))
            f.write('\n'.join(rows) + '\n')
            print(f'  Escribiendo… {end:,}/{n_pts:,}', end='\r')

    print()
    size_mb = out_path.stat().st_size / 1_048_576
    print(f'\n[✓] Archivo guardado exitosamente.')
    print(f'    Nombre   : {out_path.name}')
    print(f'    Ruta     : {out_path}')
    print(f'    Tamaño   : {size_mb:.1f} MB')
    print(f'    Vértices : {n_pts:,}')
    if has_colors:
        print(f'    Color    : SÍ — bonus point multiplier activo ✓')
    if has_normals:
        print(f'    Normales : SÍ — campo bonus activo ✓')


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    # ── Datos de la misión ───────────────────────────────────────────────
    mission, start_dt = pedir_datos_mision()

    filename = make_filename(mission, start_dt)
    out_path = Path(DEFAULT_OUTPUT_DIR).expanduser() / filename

    print()
    print(f'[INFO] Archivo de salida: {out_path}')
    print()

    # ── Cargar objetos detectados por YOLO ───────────────────────────────
    objects_path = Path(DEFAULT_OBJECTS_JSON).expanduser()
    detected_objects: list[dict] = []
    if objects_path.exists():
        with open(objects_path) as f:
            detected_objects = json.load(f)
        print(f'[INFO] {len(detected_objects)} objetos detectados cargados.')
    else:
        print(f'[AVISO] Sin archivo de objetos: {objects_path}')

    # ── Exportar DB → PLY temporal ───────────────────────────────────────
    # Usamos /tmp como directorio temporal para la exportación
    tmp_dir = tempfile.mkdtemp(prefix='rtabmap_export_')
    tmp_ply = export_from_rtabmap_db(DEFAULT_DB_PATH, tmp_dir)
    
    if tmp_ply is None:
        # Intentar usar PLY existente en ~/.ros/
        alt_ply = str(Path(DEFAULT_DB_PATH).expanduser().parent / 'rtabmap_cloud.ply')
        if os.path.exists(alt_ply):
            print(f'[INFO] Usando PLY existente: {alt_ply}')
            tmp_ply = alt_ply
        else:
            print()
            print('[ERROR] No se pudo exportar la nube de puntos.')
            print('        Verifica que rtabmap-export esté instalado:')
            print('        which rtabmap-export')
            sys.exit(1)

    # ── Post-proceso ─────────────────────────────────────────────────────
    data = postprocess_cloud(tmp_ply) if HAS_OPEN3D else tmp_ply

    # ── PLY final ────────────────────────────────────────────────────────
    write_compliant_ply(
        data=data,
        out_path=out_path,
        mission=mission,
        start_time=start_dt,
        detected_objects=detected_objects,
    )

    # ── Limpiar temporal ─────────────────────────────────────────────────
    try:
        import shutil
        if tmp_dir and os.path.isdir(tmp_dir):
            shutil.rmtree(tmp_dir)
    except Exception:
        pass

    print()
    print('─' * 60)
    print('  Listo para entregar en competencia.')
    print('  Copia el archivo al pendrive USB antes de la entrega.')
    print('─' * 60)


if __name__ == '__main__':
    main()
