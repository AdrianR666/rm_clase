import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import socket

# =============== UDP ===============
UDP_IP = "192.168.137.224"
UDP_PORT = 12345
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# =============== ArUco ===============
MARKER_SIZE_METERS = 0.1
MARKER_IDS = [0, 1]
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# =============== Cámara ===============
cap = cv.VideoCapture(1, cv.CAP_DSHOW)
camera_params = [1999.17838, 2006.06097, 928.811574, 478.347147]
camera_matrix = np.array([[camera_params[0], 0, camera_params[2]],
                          [0, camera_params[1], camera_params[3]],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([0.0906, 0.3189, -0.0028, 0.00018, -2.998])

# =============== Robot y trayectoria ===============
kv = 5
kw = 8
r_rueda = 0.03
l_eje = 0.12
r_trayectoria = 0.01
n_trayectoria = 0.02
xc, yc = 0.6, 0.45
t0 = cv.getTickCount()
frec = cv.getTickFrequency()

# =============== Funciones =================
def obtener_matriz_homogenea(rvec, tvec):
    R, _ = cv.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T

def transformar_a_referencia_0(T_obj, T_ref):
    T_0_cam = np.linalg.inv(T_ref)
    return T_0_cam @ T_obj

def mapear_velocidad(val):
    return max(-255, min(255, int(val)))

def enviar_velocidades_udp(vl, vr):
    try:
        data = f"{vl:.2f},{vr:.2f}".encode()
        udp_socket.sendto(data, (UDP_IP, UDP_PORT))
        print(f"Enviado -> vL: {vl:.2f}, vR: {vr:.2f}")
    except Exception as e:
        print(f"Error UDP: {e}")

def calcular_campos_potenciales(xr, yr, a_r, xo, yo):
    d = np.sqrt((xo - xr)**2 + (yo - yr)**2)
    v = kv * d

    a_o = np.degrees(np.arctan2(yo - yr, xo - xr)) % 360
    a_r_deg = np.degrees(a_r)
    da = a_o - a_r_deg
    if da > 180: da -= 360
    if da < -180: da += 360
    w = kw * np.sin(np.radians(da))
    return v, w

def diferencial_inverse_kinematics(v, w, r, l):
    vr = (2*v + w*l) / (2*r)
    vl = (2*v - w*l) / (2*r)
    return vl, vr

# =============== Loop principal ===============
while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_METERS, camera_matrix, dist_coeffs)

        poses = {}
        for i, id in enumerate(ids.flatten()):
            poses[id] = (tvecs[i][0], rvecs[i][0])
            cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i][0], tvecs[i][0], 0.05)

        if 0 in poses and 1 in poses:
            T_cam_0 = obtener_matriz_homogenea(*poses[0])
            T_cam_1 = obtener_matriz_homogenea(*poses[1])
            T_0_1 = transformar_a_referencia_0(T_cam_1, T_cam_0)

            xr, yr = T_0_1[0, 3], T_0_1[1, 3]
            theta_r = np.arctan2(T_0_1[1, 0], T_0_1[0, 0])

            t_actual = (cv.getTickCount() - t0) / frec
            xo = xc + r_trayectoria * np.cos(n_trayectoria * t_actual)
            yo = yc + r_trayectoria * np.sin(n_trayectoria * t_actual)

            v, w = calcular_campos_potenciales(xr, yr, theta_r, xo, yo)
            vl, vr = diferencial_inverse_kinematics(v, w, r_rueda, l_eje)
            enviar_velocidades_udp(mapear_velocidad(vl), mapear_velocidad(vr))
        else:
            enviar_velocidades_udp(0, 0)
            print("⚠️ Faltan marcadores")
    else:
        enviar_velocidades_udp(0, 0)
        print("⚠️ Sin detección")

    cv.imshow("Trayectoria Circular - Ref ArUco 0", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
