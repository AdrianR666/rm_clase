import socket

# Configura la IP y el puerto destino
IP_DESTINO = '192.168.137.197'  # Cambia por la IP de destino
PUERTO_DESTINO = 12345         # Cambia por el puerto destino

# Crea el socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("Escribe palabras para enviar por UDP. Escribe 'salir' para terminar.")

while True:
    mensaje = input(">> ")

    if mensaje.lower() == 'salir':
        print("Finalizando envío UDP.")
        break

    # Envía el mensaje codificado en UTF-8
    sock.sendto(mensaje.encode('utf-8'), (IP_DESTINO, PUERTO_DESTINO))

# Cierra el socket al terminar
sock.close()
