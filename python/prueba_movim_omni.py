import numpy as np
import socket
import time

# ====================== Configuración de UDP ======================
UDP_IP = "192.168.137.3"  # Dirección IP de la ESP32
UDP_PORT = 58625          # Puerto configurado en la ESP32
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ===================== Cinemática Inversa =========================
def calcular_velocidades_ruedas(vx, vy, w):
    # Parámetros del robot
    r = 0.030  # Radio de las llantas [m]
    b = 0.1425   # Distancia entre las ruedas en el eje longitudinal [m]
    d = 0.078    # Distancia entre las ruedas en el eje transversal [m]

    # Ecuaciones de la cinemática inversa
    rueda1 = (vx / r) - (vy / r) - (((b + d) / r) * w)  # Frontal izquierda
    rueda2 = (vx / r) + (vy / r) + (((b + d) / r) * w)  # Frontal derecha
    rueda3 = (vx / r) + (vy / r) - (((b + d) / r) * w)  # Trasera izquierda
    rueda4 = (vx / r) - (vy / r) + (((b + d) / r) * w)  # Trasera derecha
    return rueda1, rueda2, rueda3, rueda4

# =================== Enviar Velocidades por UDP ===================
def enviar_velocidades_udp(v1, v2, v3, v4):
    try:
        data = np.array([v1, v2, v3, v4], dtype=np.float32).tobytes()
        udp_socket.sendto(data, (UDP_IP, UDP_PORT))
        print(f"Enviado -> v1: {v1:.2f}, v2: {v2:.2f}, v3: {v3:.2f}, v4: {v4:.2f}")
    except Exception as e:
        print(f"Error al enviar datos UDP: {e}")

# ===================== Mapeo de Velocidad para Adafruit FeatherWing =====================
def mapear_velocidad_featherwing(vel):
    # Limitamos la velocidad al rango [-255, 255]
    return max(-255, min(255, int(vel)))  # Convertir a entero y limitar

# ==================== Loop Principal ==============================
print("Introduce las velocidades vx, vy, w (en m/s y rad/s):")
print("Escribe 'q' para detener el robot y finalizar el programa.")

continuar = True
while continuar:
    try:
        # Solicitar las velocidades al usuario
        vx = float(input("Introduce vx: "))
        vy = float(input("Introduce vy: "))
        w = float(input("Introduce w: "))

        while True:
            # Calcular velocidades de las ruedas
            rueda1, rueda2, rueda3, rueda4 = calcular_velocidades_ruedas(vx, vy, w)

            v1 = mapear_velocidad_featherwing(rueda1)
            v2 = mapear_velocidad_featherwing(rueda2)
            v3 = mapear_velocidad_featherwing(rueda3)
            v4 = mapear_velocidad_featherwing(rueda4)

            # Enviar velocidades por UDP
            enviar_velocidades_udp(v1, v2, v3, v4)

            # Agregar un delay
            time.sleep(0.1)

    except ValueError:
        print("Entrada inválida. Asegúrate de ingresar un número válido.")
    except KeyboardInterrupt:
        # Detener el robot enviando velocidades cero
        enviar_velocidades_udp(0, 0, 0, 0)  # Velocidad mapeada a cero
        print("\nRobot detenido. Programa finalizado.")
        continuar = False
    except Exception as e:
        print(f"Error: {e}")
