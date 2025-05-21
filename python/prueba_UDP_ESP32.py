import socket

def enviar_mensaje_udp(mensaje, ip_esp32, puerto_esp32):
    """
    Envía un mensaje UDP a una ESP32.

    :param mensaje: Mensaje a enviar (str)
    :param ip_esp32: Dirección IP de la ESP32 (str)
    :param puerto_esp32: Puerto UDP de la ESP32 (int)
    """
    # Crear el socket UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # Codificar el mensaje a bytes y enviarlo
        sock.sendto(mensaje.encode('utf-8'), (ip_esp32, puerto_esp32))
        print(f"Mensaje enviado a {ip_esp32}:{puerto_esp32}: {mensaje}")
    except Exception as e:
        print(f"Error al enviar el mensaje: {e}")
    finally:
        # Cerrar el socket
        sock.close()

# Parámetros de conexión
IP_ESP32 = "192.168.0.100"  # Cambia esto por la IP de tu ESP32
PUERTO_ESP32 = 12345        # Cambia esto por el puerto configurado en tu ESP32

# Mensaje a enviar
mensaje = "Hola desde Python!"

# Llamar a la función para enviar el mensaje
enviar_mensaje_udp(mensaje, IP_ESP32, PUERTO_ESP32)
