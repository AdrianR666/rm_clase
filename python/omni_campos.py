import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import socket

# ====================== Configuración de UDP ======================
UDP_IP = "192.168.137.104"  # Dirección IP de la ESP32
UDP_PORT = 12345
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ====================== Configuración ArUco =======================
MARKER_SIZE_METERS = 0.1
MARKER_IDS = [0, 1, 2]
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# ================= Configuración de la Cámara =====================
cap = cv.VideoCapture(1, cv.CAP_DSHOW)

camera_params = [1999.17838, 2006.06097, 928.811574, 478.347147]
camera_matrix = np.array([[camera_params[0], 0, camera_params[2]],
                          [0, camera_params[1], camera_params[3]],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([0.0906358995, 0.318998982, -0.00288859938, 0.000180882259, -2.99885280])

# =================== Parámetros de Control ========================
kv = 5
kw = 8
rp = 0.05
ap = 5.0

def mapear_velocidad_l298n(vel):
    return max(-255, min(255, int(vel)))

def calcular_velocidades_ruedas(vx, vy, w):
    r = 0.030
    b = 0.1425
    d = 0.078

    rueda1 = (vx / r) - (vy / r) - (((b + d) / r) * w)
    rueda2 = (vx / r) + (vy / r) + (((b + d) / r) * w)
    rueda3 = (vx / r) + (vy / r) - (((b + d) / r) * w)
    rueda4 = (vx / r) - (vy / r) + (((b + d) / r) * w)
    return rueda1, rueda2, rueda3, rueda4

def enviar_velocidades_udp(v1, v2, v3, v4):
    try:
        data = np.array([v1, v2, v3, v4], dtype=np.float32).tobytes()
        udp_socket.sendto(data, (UDP_IP, UDP_PORT))
        print(f"Enviado -> v1: {v1:.2f}, v2: {v2:.2f}, v3: {v3:.2f}, v4: {v4:.2f}")
    except Exception as e:
        print(f"Error al enviar datos UDP: {e}")

def calcular_distancia(p1, p2):
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)

# ==================== Loop Principal ==============================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None and len(ids) > 0:
        aruco.drawDetectedMarkers(frame, corners, ids)

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_METERS, camera_matrix, dist_coeffs)
        marker_positions = {}
        orientations = {}

        for i, id in enumerate(ids.flatten()):
            if id in MARKER_IDS:
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]
                marker_positions[id] = tvec
                orientations[id] = rvec
                cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                x, y, z = tvec
                cv.putText(frame, f"ID {id}: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m", 
                           (10, 30 + 30 * id), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if 0 in marker_positions and 1 in marker_positions and 2 in marker_positions:
            x_r, y_r, _ = marker_positions[1]
            x_o, y_o, _ = marker_positions[2]

            d_obj = np.sqrt((x_o - x_r)**2 + (y_o - y_r)**2)
            a_o = np.degrees(np.arctan2(y_o - y_r, x_o - x_r))
            if a_o < 0:
                a_o += 360

            a_r = np.degrees(orientations[1][2])
            da_obj = a_o - a_r
            if da_obj > 180:
                da_obj -= 360
            elif da_obj < -180:
                da_obj += 360

            v = kv * d_obj if d_obj >= rp else 0
            w = kw * np.sin(np.radians(da_obj)) if abs(da_obj) >= ap else 0

            vx = v * np.cos(np.radians(a_o))
            vy = v * np.sin(np.radians(a_o))

            rueda1, rueda2, rueda3, rueda4 = calcular_velocidades_ruedas(vx, vy, w)

            v1 = mapear_velocidad_l298n(rueda1)
            v2 = mapear_velocidad_l298n(rueda2)
            v3 = mapear_velocidad_l298n(rueda3)
            v4 = mapear_velocidad_l298n(rueda4)

            enviar_velocidades_udp(v1, v2, v3, v4)
        else:
            # Faltan marcadores: enviar velocidad cero y mostrar mensaje
            enviar_velocidades_udp(0, 0, 0, 0)
            cv.putText(frame, "Marcadores faltantes: Robot detenido", 
                       (10, 450), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
            print("⚠️ Marcadores faltantes. Se envió velocidad cero.")
    else:
        # No se detectó ningún marcador: detener y mostrar mensaje
        enviar_velocidades_udp(0, 0, 0, 0)
        cv.putText(frame, "Sin detección de marcadores", 
                   (10, 450), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
        print("⚠️ Sin detección. Robot detenido.")

    cv.imshow('Aruco Marker Detection', frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
