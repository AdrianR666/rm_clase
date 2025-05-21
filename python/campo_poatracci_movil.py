import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import socket

# ====================== Configuración de UDP ======================
UDP_IP = "192.168.137.131"  # Dirección IP de la ESP32
UDP_PORT = 58625          # Puerto configurado en la ESP32
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ====================== Configuración ArUco =======================
MARKER_SIZE_METERS = 0.1  # Tamaño del marcador ArUco en metros
MARKER_IDS = [0, 1, 2]     # IDs de los marcadores para referencia, robot, objetivo
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# ================= Configuración de la Cámara =====================
cap = cv.VideoCapture(1, cv.CAP_DSHOW)  # 0 indica cámara de la computadora y 1 cámara externa (WebCam)

camera_params = [1999.17838, 2006.06097, 928.811574, 478.347147]  # [fx, fy, cx, cy]
camera_matrix = np.array([[camera_params[0], 0, camera_params[2]],
                          [0, camera_params[1], camera_params[3]],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([ 0.0906358995,  0.318998982, -0.00288859938,  0.000180882259, -2.99885280 ])
#=====================Robot manipulador============================
def calculate_attraction_force(current_pos, target_pos, gain=0.1):
    """
    Calcula un vector de fuerza de atracción hacia el punto objetivo.
    """
    return {axis: current_pos[axis] + gain * (target_pos[axis] - current_pos[axis]) for axis in target_pos}
#=====================Robot movi===================================
# =================== Parámetros de Control ========================
kv = 3  # Constante de velocidad lineal
kw = 5 # Constante de velocidad angular
rp = 0.03  # Radio de proximidad al objetivo[m]
ap = 5.0   # Precisión angular en grados
dx= 0.18
dy=0
dz=0.086

def mapear_velocidad_featherwing(vel):
    # Limitamos la velocidad al rango [-255, 255]
    return max(-255, min(255, int(vel)))  # Convertir a entero y limitar


# ===================== Cinemática Inversa =========================
def calcular_velocidades_ruedas(vx, vy, w):
    # Parámetros del robot
    r = 0.1  # Radio de las llantas [m]
    b = 0.1425   # Distancia entre las ruedas en el eje longitudinal [m]
    d = 0.078 # Distancia entre las ruedas en el eje transversal [m]

    # Ecuaciones de la cinemática inversa
    rueda1 = (vx / r) - (vy / r) - (((b + d) / r) * w)  # Frontal izquierda
    rueda2 = (vx / r) + (vy / r) + (((b + d) / r) * w)  # Frontal derecha
    rueda3 = (vx / r) + (vy / r) - (((b + d) / r) * w)  # Trasera izquierda
    rueda4 = (vx / r) - (vy / r) + (((b + d) / r) * w ) # Trasera deercha
    return rueda1, rueda2, rueda3, rueda4

# =================== Enviar Velocidades por UDP ===================
def enviar_velocidades_udp(v1, v2, v3, v4):
    try:
        data = np.array([v1, v2, v3, v4], dtype=np.float32).tobytes()
        udp_socket.sendto(data, (UDP_IP, UDP_PORT))
        print(f"Enviado -> v1: {v1:.2f}, v2: {v2:.2f}, v3: {v3:.2f}, v4: {v4:.2f}")
    except Exception as e:
        print(f"Error al enviar datos UDP: {e}")

# ===================== Función para calcular la distancia =======================
def calcular_distancia(p1, p2):
    # Calcular distancia Euclidiana en 3D
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)

def desplazamiento_robot():
    return np.array([0.18,0,0.086])

# ==================== Loop Principal ==============================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detectar marcadores ArUco
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None and len(ids) > 0:
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Obtener poses de los marcadores
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
                # Mostrar las coordenadas en metros (x, y, z)
                x, y, z = tvec
                cv.putText(frame, f"ID {id}: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m", 
                               (10, 30 + 30 * id), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Verificar si se detectaron ambos marcadores (incluyendo el marcador de referencia 0)
        if 0 in marker_positions and 1 in marker_positions and 2 in marker_positions:
            x_ref, y_ref, z_ref = marker_positions[0]  # Posición del marcador de referencia

            # Mostrar distancias a los otros marcadores (1 y 2)
            for id in [1, 2]:  # Marcadores de robot (ID 1) y objetivo (ID 2)
                x, y, z = marker_positions[id]
                distancia = calcular_distancia([x_ref, y_ref, z_ref], [x, y, z])
                cv.putText(frame, f"Distancia a Marcador {id}: {distancia:.2f}m", 
                           (10, 60 + 30 * id), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Cálculo de control
            x_r, y_r, _ = marker_positions[1]
            x_o, y_o, _ = marker_positions[2]

            d_obj = np.sqrt((x_o - x_r)**2 + (y_o - y_r)**2)
            #d_obj = np.sqrt((x_r- x_o)**2+ (y_r - y_o)**2)
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

            w = kw *np.sin(np.radians(da_obj))if abs(da_obj) >= ap else 0
            #w=0

            vx = v * np.cos(np.radians(a_o))
            #vx =-1*( v * np.cos(np.radians(a_o)))
            
            
            vy = v * np.sin(np.radians(a_o))
            #vy=0   

            # Calcular velocidades de las ruedas
            rueda1, rueda2, rueda3, rueda4 = calcular_velocidades_ruedas(vx, vy, w)

            # Enviar velocidades por UDP

            v1 = mapear_velocidad_featherwing(rueda1)
            v2 = mapear_velocidad_featherwing(rueda2)
            v3 = mapear_velocidad_featherwing(rueda3)
            v4 = mapear_velocidad_featherwing(rueda4)

            enviar_velocidades_udp(v1, v2, v3, v4)
    # Mostrar el video
    cv.imshow('Aruco Marker Detection', frame)

    # Salir si se presiona 'q'
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar recursos
cap.release()
cv.destroyAllWindows()
