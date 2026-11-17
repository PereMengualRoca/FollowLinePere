"""
FollowLine - Unibotics / JdeRobot Robotics Academy  -  VERSION P
================================================================
VERSION RAPIDA: es la version MAXIMO con estos parametros cambiados
(si se sale de la pista, usa followline_P.py, la MAXIMO original):
    LAT_TOL        = 0.5
    Z_MARGIN       = 2.0
    ACCEL          = 40.0
    DECEL          = 80.0
    LOOKAHEAD_MAX  = 12.0
    V_MAX          = 40.0
    BRAKE_PLAN     = 40.0
    V_CURVE        = 11.0
    A_LAT          = 35.0
    KP             = 5.0
    LOOKAHEAD_TIME = 0.4

Control reactivo visual para que el F1 siga la linea roja (Simple Circuit)
lo mas rapido posible.

Las tres versiones (P, PD y PID) son IDENTICAS salvo el control del volante:
  - misma percepcion  (mascara HSV + barrido fila a fila + ajuste de la linea)
  - mismo error       (angulo hacia un punto de la linea a L metros por delante)
  - misma velocidad   ("por tramos", segun los metros de recta que se ven delante)

Control del volante en esta version:
    W = KP * e

Geometria de la camara (sacada del modelo del F1 en Gazebo, CustomRobots):
  640x480, campo de vision 1.57 rad (90 grados), 20 imagenes/s, ve hasta 15 m,
  montada 0.45 m por delante y 0.04 m a la izquierda del centro del coche,
  a ~0.11 m del suelo y sin inclinacion (horizonte en la mitad de la imagen).

Pipeline de cada imagen nueva
-----------------------------
1. Percepcion : BGR -> HSV -> mascara de rojo -> limpieza morfologica.
2. Medicion   : barrido fila a fila de abajo arriba (paso fino cerca del
                horizonte) siguiendo la linea por continuidad.
3. Modelo     : ajuste por minimos cuadrados x_n = a*u + b + c/u de los puntos
                cercanos (a = desplazamiento, b = angulo, c = curvatura).
4. Error      : angulo (normalizado) hacia un punto de la linea situado a
                L = V*LOOKAHEAD_TIME metros por delante (entre LOOKAHEAD_MIN y
                LOOKAHEAD_MAX): e = (x_eje_coche - x_linea) / (w/2) en esa fila.
5. Control    : W = KP * e
6. Velocidad  : se mide cuantos metros de recta quedan por delante (z_free) y
                se va a la velocidad desde la que se puede frenar a V_CURVE en
                esa distancia: V = sqrt(V_CURVE^2 + 2*BRAKE_PLAN*z_free).
                En la curva, V <= sqrt(A_LAT / curvatura). Limites ACCEL/DECEL.
7. Seguridad  : si se pierde la linea, frena y gira hacia el ultimo lado.

Convenio de signos: e > 0  ->  linea a la IZQUIERDA  ->  W > 0 (giro a la izquierda).
"""

#IMPORTAR LIBRERIAS
import math
import time

import cv2
import numpy as np

import HAL
import Frequency
import WebGUI


# =====================================================================
#  PARAMETROS
# =====================================================================
LOOP_HZ = 60                       # el bucle va a 60 Hz; la camara da 20 imagenes/s y
                                   # solo se procesa cuando llega una imagen nueva

# ---------- Vision ----------
# HSV: H = tono, S = saturacion, V = brillo. En OpenCV H va de 0 a 179 y el
# rojo esta en LOS DOS extremos, por eso hay dos rangos. Trabajar en HSV hace
# que la deteccion aguante sombras (cambia V, pero H casi no).
RED_RANGES = (
    (np.array([0, 100, 70]),   np.array([10, 255, 255])),
    (np.array([165, 100, 70]), np.array([179, 255, 255])),
)
MIN_AREA = 40                      # px minimos de un trozo (solo para el modo TROZO)
KERNEL_OPEN = np.ones((3, 3), np.uint8)
KERNEL_CLOSE = np.ones((5, 5), np.uint8)

# ---------- Camara del F1 (modelo de Gazebo) ----------
CAM_HEIGHT = 0.109                 # m, altura de la camara sobre el suelo
CAM_HFOV = 1.57                    # rad, campo de vision horizontal
CAM_LATERAL = 0.04                 # m, camara desplazada a la IZQUIERDA del centro del coche
HORIZON = 0.499                    # fila del horizonte (fraccion de la altura); camara horizontal
LINE_WIDTH = 0.12                  # m, anchura real de la linea roja

# ---------- Barrido de la linea ----------
SCAN_BOTTOM = 0.70                 # primera fila analizada; mas abajo se ve el morro del coche
SCAN_TOP_PX = 2                    # ultima fila: 2 px por debajo del horizonte (~15 m)
SCAN_STEP_FRAC = 0.06              # paso entre filas = 6% de la distancia al horizonte
SCAN_STEP_MAX = 5                  # paso maximo (px)
FIT_TOP = 0.56                     # el ajuste de la linea solo usa filas por debajo de esta
MIN_RUN_PX = 2                     # anchura minima de un tramo rojo en una fila
MAX_RUN_K = 3.0                    # tramo > MAX_RUN_K * ancho esperado -> linea casi horizontal
MAX_JUMP_PX = 30                   # salto maximo entre filas consecutivas (o medio ancho de linea)
MAX_GAP_STEPS = 5                  # filas seguidas sin linea antes de dejar de subir
MIN_POINTS = 3                     # puntos minimos para considerar que se ve la linea
MIN_FIT_POINTS = 8                 # puntos minimos para estimar tambien la curvatura
OUTLIER_PX = 4.0                   # residuo maximo (px) para que un punto cuente en el ajuste
CURV_SMOOTH = 0.5                  # regularizacion de la curvatura hacia la de la imagen anterior

# ---------- Control lateral: P ----------
LOOKAHEAD_TIME = 0.4               # s: el error se mide en un punto de la linea a
                                   # L = V * LOOKAHEAD_TIME metros por delante de la camara
LOOKAHEAD_MIN = 1.0                # m: distancia de mira minima (curvas lentas)
LOOKAHEAD_MAX = 12.0               # m: distancia de mira maxima (rectas rapidas)
KP = 5.0                           # ganancia proporcional
W_MAX = 6.0                        # rad/s

# ---------- Velocidad lineal ("por tramos", segun lo que se ve delante) ----------
V_MAX = 40.0                       # m/s maximos (rectas largas)
V_CURVE = 11.0                     # m/s con los que se quiere llegar a una curva cerrada (R ~ 5 m)
A_LAT = 35.0                       # dentro de la curva: V <= sqrt(A_LAT / curvatura)
V_MIN = 2.5                        # m/s minimos siguiendo linea
BRAKE_PLAN = 40.0                  # m/s^2 de frenada que se supone al planificar
LAT_TOL = 0.5                      # m: si la linea se aparta mas que esto de la recta
                                   # prolongada, ahi empieza una curva
Z_MARGIN = 2.0                     # m de margen antes de la curva
CAM_FAR = 15.0                     # m: la camara no ve mas alla (far clip del modelo)
ACCEL = 40.0                       # m/s^2 maximos al acelerar
DECEL = 80.0                       # m/s^2 maximos al frenar
V_START = 2.0                      # m/s al arrancar (se acelera con ACCEL desde aqui)

# ---------- Recuperacion si se pierde la linea ----------
KP_BLOB = 1.5                      # P simple cuando solo se ve un trozo suelto
V_BLOB = 2.0                       # m/s en modo TROZO
LOST_GRACE = 6                     # imagenes manteniendo el ultimo giro antes de buscar
V_SEARCH = 0.5                     # m/s mientras busca
W_SEARCH = 1.2                     # rad/s mientras busca


# =====================================================================
#  UTILIDADES
# =====================================================================
def clip(x, lo, hi):
    """Deja x dentro de [lo, hi] (como el tope de giro de un volante)."""
    return max(lo, min(hi, x))


def focal_px(w):
    """Distancia focal en pixeles a partir del campo de vision."""
    return (w / 2.0) / math.tan(CAM_HFOV / 2.0)


def curv_scale(h, w):
    """Factor que pasa la curvatura del modelo 'c' a curvatura real (1/m)."""
    f = focal_px(w)
    return h * w / (f * f * CAM_HEIGHT)


def red_mask(img_bgr, y0, y1):
    """Mascara binaria de la linea roja entre las filas y0 e y1."""
    hsv = cv2.cvtColor(img_bgr[y0:y1], cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in RED_RANGES:
        mask |= cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL_OPEN)    # quita ruido suelto
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL_CLOSE)  # rellena huecos
    return mask


def find_runs(row):
    """Tramos rojos continuos de una fila de la mascara: (inicios, finales exclusivos)."""
    r = np.concatenate(([0], (row > 0).view(np.int8), [0]))
    d = np.diff(r)
    starts = np.flatnonzero(d == 1)
    ends = np.flatnonzero(d == -1)
    keep = (ends - starts) >= MIN_RUN_PX
    return starts[keep], ends[keep]


def scan_rows(h, y_h):
    """Filas a analizar, de abajo arriba. El paso se estrecha cerca del horizonte,
    donde cada fila representa mucha distancia en el suelo."""
    rows = []
    y = float(min(h - 1, int(SCAN_BOTTOM * h)))
    y_top = y_h + SCAN_TOP_PX
    while y >= y_top:
        rows.append(int(round(y)))
        y -= clip(SCAN_STEP_FRAC * (y - y_h), 1.0, SCAN_STEP_MAX)
    return sorted(set(rows), reverse=True)


def scan_line(mask, y0, rows, w, y_h, x_pred):
    """
    Recorre las filas de abajo arriba y en cada una se queda con el tramo rojo
    mas coherente con lo que se viene siguiendo (prediccion lineal).
    Devuelve una lista de (y, x_centro, anchura, toca_borde).
    """
    k_w = LINE_WIDTH / CAM_HEIGHT        # ancho esperado de la linea (px) = k_w * (y - y_h)
    pts = []
    slope = 0.0
    gap = 0
    y_last = None
    for y in rows:
        if y_last is not None and x_pred is not None:
            x_pred -= slope * (y_last - y) if pts else 0.0
        y_last = y
        starts, ends = find_runs(mask[y - y0])
        if len(starts) == 0:
            if pts:
                gap += 1
                if gap > MAX_GAP_STEPS:
                    break
            continue
        centers = (starts + ends - 1) * 0.5
        widths = ends - starts
        expected = k_w * (y - y_h)
        if x_pred is None:
            k = int(np.argmax(widths))                    # sin historia: el mas ancho
        else:
            k = int(np.argmin(np.abs(centers - x_pred)))  # el mas cercano a la prediccion
            tol = max(MAX_JUMP_PX, 0.5 * expected) * (1 + gap)
            if pts and abs(centers[k] - x_pred) > tol:
                gap += 1
                if gap > MAX_GAP_STEPS:
                    break
                continue
        if widths[k] > MAX_RUN_K * expected + 4:
            break                         # linea casi horizontal: curva muy cerrada, parar aqui
        xc = float(centers[k])
        if pts:
            y_prev, x_prev = pts[-1][0], pts[-1][1]
            slope = (xc - x_prev) / (y - y_prev)
        edge = starts[k] == 0 or ends[k] >= w
        pts.append((y, xc, int(widths[k]), edge))
        x_pred = xc
        gap = 0
    return pts


# ---------------------------------------------------------------------
# Modelo de la linea vista por la camara
# ---------------------------------------------------------------------
# Camara a altura H mirando al frente. Un punto del suelo a distancia Z y
# desplazado X a la izquierda aparece en:
#       fila     y - y_h = f*H/Z        (y_h = fila del horizonte)
#       columna  cx - x  = f*X/Z
# Si cerca del coche la linea es  X(Z) = d + psi*Z + kappa*Z^2/2
# (desplazamiento + orientacion + curvatura), sustituyendo queda
#       cx - x = (d/H)*u + f*psi + (f^2*H*kappa/2)/u        con u = y - y_h
# Lo proporcional a u es el desplazamiento, lo constante es la orientacion y
# lo que va en 1/u es la curvatura. Con un ajuste por minimos cuadrados se
# separan las tres cosas, promediando decenas de puntos.
# Aqui se trabaja normalizado: u = (y - y_h)/h  y  x_n = (cx - x)/(w/2).
def fit_line_model(pts, h, w, y_h, cx, c_prev):
    """Devuelve (a, b, c) o None si no hay puntos suficientes."""
    use = [p for p in pts if not p[3]]           # fuera los tramos cortados por el borde
    if len(use) < MIN_POINTS:
        use = pts
    if len(use) < MIN_POINTS:
        return None
    P = np.array([(p[0], p[1]) for p in use], np.float64)
    u = (P[:, 0] - y_h) / h
    xn = (cx - P[:, 1]) / (w / 2.0)
    ok = u > 0.5 / h
    if ok.sum() < MIN_POINTS:
        return None
    u, xn, P = u[ok], xn[ok], P[ok]
    span = P[:, 0].max() - P[:, 0].min()
    with_c = len(u) >= MIN_FIT_POINTS and span >= 0.04 * h

    def solve(sel):
        uu, xx = u[sel], xn[sel]
        if with_c:
            lam = math.sqrt(CURV_SMOOTH)
            A = np.column_stack((uu, np.ones_like(uu), 1.0 / uu))
            # regularizacion: si los puntos no bastan para fijar la curvatura,
            # se queda cerca de la de la imagen anterior (no la inventa)
            A = np.vstack((A, [0.0, 0.0, lam]))
            xx = np.append(xx, lam * c_prev)
            return np.linalg.lstsq(A, xx, rcond=None)[0]
        if span >= 6:
            A = np.column_stack((uu, np.ones_like(uu)))
            a, b = np.linalg.lstsq(A, xx, rcond=None)[0]
        else:
            a, b = 0.0, float(np.mean(xx))       # casi un solo punto: solo direccion
        return np.array([a, b, 0.0])

    sel = np.ones(len(u), bool)
    coef = solve(sel)
    # Segunda pasada sin los puntos que no encajan (brillos, otro trozo de linea...)
    res_px = np.abs(coef[0] * u + coef[1] + coef[2] / u - xn) * (w / 2.0)
    inl = res_px <= OUTLIER_PX
    if inl.sum() >= max(MIN_POINTS, 0.5 * len(u)) and not inl.all():
        coef = solve(inl)
    return float(coef[0]), float(coef[1]), float(coef[2])


def axis_x(y, cx, y_h):
    """Columna por la que pasa el EJE DEL COCHE en la fila y. Como la camara esta
    CAM_LATERAL m a la izquierda, el eje se ve algo a la derecha del centro."""
    return cx + CAM_LATERAL * (y - y_h) / CAM_HEIGHT


def lookahead_point(pts, y_target):
    """Columna de la linea en la fila y_target, interpolando entre los puntos del
    barrido. Si la linea no llega tan lejos, se usa el punto visible mas lejano."""
    if y_target >= pts[0][0]:
        return pts[0][1], pts[0][0]
    for (y1, x1, _, _), (y2, x2, _, _) in zip(pts, pts[1:]):
        if y2 <= y_target <= y1:
            t = (y1 - y_target) / float(y1 - y2) if y1 != y2 else 0.0
            return x1 + t * (x2 - x1), y_target
    return pts[-1][1], pts[-1][0]


def pick_component(mask, prev_cx):
    """Centroide x del trozo rojo mas fiable (solo se usa si falla el barrido)."""
    n, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    cands = [(stats[i, cv2.CC_STAT_AREA], centroids[i][0])
             for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= MIN_AREA]
    if not cands:
        return None
    if prev_cx is None:
        return max(cands)[1]
    biggest = max(a for a, _ in cands)
    cands = [c for c in cands if c[0] >= 0.3 * biggest]
    return min(cands, key=lambda c: abs(c[1] - prev_cx))[1]


def rate_limit(current, target, dt):
    """Limita aceleracion y frenada para evitar tirones."""
    return current + clip(target - current, -DECEL * dt, ACCEL * dt)


def free_distance(pts, coef, h, w, y_h, cx, f):
    """
    Metros (desde la camara) en los que la linea sigue RECTA por delante.
    Se prolonga la recta que forma la linea cerca del coche (a, b del ajuste)
    y se recorren los puntos de cerca a lejos: donde la linea se aparta mas de
    LAT_TOL metros de esa recta, empieza una curva. Si la linea se ve recta
    hasta el final, hay CAM_FAR metros libres.
    """
    if not pts:
        return 0.0
    a, b, _ = coef
    z_free = 0.0
    for y, x, _, _ in pts:                       # de cerca a lejos
        u_px = y - y_h
        if u_px < 0.5:
            break
        z = f * CAM_HEIGHT / u_px
        x_lin = cx - (a * u_px / h + b) * (w / 2.0)
        if abs(x - x_lin) * CAM_HEIGHT / u_px > LAT_TOL:
            return z
        z_free = z
    if pts[-1][0] - y_h <= SCAN_TOP_PX + 1.5:    # recta visible hasta el final
        return CAM_FAR
    return z_free                                # la linea desaparece antes: curva


# =====================================================================
#  CONTROLADOR
# =====================================================================
class ControladorP:
    """
    Control proporcional:  W = KP * e
    Gira mas cuanto mas lejos esta la linea del eje del coche. No tiene
    memoria: solo usa el error de esta imagen.
    """

    def __init__(self, kp):
        self.kp = kp

    def reset(self):
        pass

    def update(self, error, dt):
        return self.kp * error


# =====================================================================
#  BUCLE PRINCIPAL
# =====================================================================
def main():
    ctrl = ControladorP(KP)

    x_pred = None           # donde empezar a buscar la linea en la siguiente imagen
    c_prev = 0.0            # curvatura anterior (regularizacion del ajuste)
    prev_blob_cx = None
    last_side = 0.0         # ultimo lado donde se vio la linea (para buscar)
    lost_frames = 0
    v_cmd, w_cmd = V_START, 0.0
    prev_sig = None         # "firma" de la ultima imagen, para no procesarla dos veces
    rows = None
    t_prev = time.time()

    while True:
        Frequency.tick(LOOP_HZ)

        img = HAL.getImage()
        if img is None:                         # la camara aun no ha publicado
            continue
        sig = img[::40, ::40].copy()
        if prev_sig is not None and np.array_equal(sig, prev_sig):
            continue                            # misma imagen que antes: nada nuevo que procesar
        prev_sig = sig

        now = time.time()
        dt = clip(now - t_prev, 1e-3, 0.25)
        t_prev = now

        h, w = img.shape[:2]
        y_h = HORIZON * h
        cx = w / 2.0 - 0.5
        y0 = int(y_h) + 1
        y1 = int(SCAN_BOTTOM * h) + 1
        if rows is None:
            rows = scan_rows(h, y_h)
            k_curv = curv_scale(h, w)
            f_px = focal_px(w)

        # ---------- Percepcion ----------
        mask = red_mask(img, y0, y1)
        pts = scan_line(mask, y0, rows, w, y_h, x_pred)
        pts_near = [p for p in pts if p[0] >= FIT_TOP * h]
        if len(pts_near) < MIN_POINTS:
            pts_near = pts
        coef = fit_line_model(pts_near, h, w, y_h, cx, c_prev) if len(pts) >= MIN_POINTS else None

        if coef is not None:
            mode = "LINEA"
            c_prev = coef[2]
            x_pred = pts[0][1]
            prev_blob_cx = None

            # ---------- Error: angulo hacia un punto de la linea a L metros ----------
            # L crece con la velocidad (como en "pure pursuit"): en recta, rapido,
            # se apunta lejos (estable aunque haya retardo); en las curvas, lento,
            # se apunta cerca (preciso).
            look = clip(v_cmd * LOOKAHEAD_TIME, LOOKAHEAD_MIN, LOOKAHEAD_MAX)
            y_look = y_h + f_px * CAM_HEIGHT / look
            x_look, y_look = lookahead_point(pts, y_look)
            error = (axis_x(y_look, cx, y_h) - x_look) / (w / 2.0)
            last_side = 1.0 if error >= 0 else -1.0

            # ---------- Velocidad segun lo que se ve delante ----------
            # 1) frenar a tiempo: con z_free metros de recta hay que poder llegar
            #    a V_CURVE frenando con BRAKE_PLAN  ->  V^2 = V_CURVE^2 + 2*BRAKE_PLAN*d
            # 2) dentro de la curva: V <= sqrt(A_LAT / curvatura)
            z_free = free_distance(pts, coef, h, w, y_h, cx, f_px)
            v_target = math.sqrt(V_CURVE ** 2 + 2.0 * BRAKE_PLAN * max(0.0, z_free - Z_MARGIN))
            k_near = k_curv * abs(coef[2])                      # 1/m
            if k_near > 1e-4:
                v_target = min(v_target, math.sqrt(A_LAT / k_near))
            v_target = clip(v_target, V_MIN, V_MAX)
        else:
            x_pred = None
            blob_cx = pick_component(mask, prev_blob_cx)
            prev_blob_cx = blob_cx
            mode = "TROZO" if blob_cx is not None else None

        lost_frames = 0 if mode is not None else lost_frames + 1

        v_cmd_prev = v_cmd
        if mode == "LINEA":
            # ---------- Control del volante (P) ----------
            v_cmd = rate_limit(v_cmd, v_target, dt)
            w_cmd = clip(ctrl.update(error, dt), -W_MAX, W_MAX)
        elif mode == "TROZO":
            e = (cx - blob_cx) / (w / 2.0)
            last_side = 1.0 if e >= 0 else -1.0
            ctrl.reset()                         # el error viene de otra fuente: empezar de cero
            v_cmd = rate_limit(v_cmd, V_BLOB, dt)
            w_cmd = clip(KP_BLOB * e, -W_MAX, W_MAX)
        elif lost_frames <= LOST_GRACE:
            # perdida breve: mantener el ultimo giro y frenar
            v_cmd = rate_limit(v_cmd, min(v_cmd_prev, V_BLOB), dt)
        else:
            # perdida real: girar hacia el ultimo lado donde se vio la linea
            ctrl.reset()
            c_prev = 0.0
            v_cmd = rate_limit(v_cmd, V_SEARCH, dt)
            w_cmd = W_SEARCH if last_side >= 0 else -W_SEARCH

        HAL.setV(v_cmd)
        HAL.setW(w_cmd)
        WebGUI.showImage(img)                    # imagen de la camara en la web

main()