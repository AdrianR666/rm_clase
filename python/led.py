import socket

esp_ip = "192.168.137.224"  # Cambia por la IP del ESP32
port = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    msg = input("Escribe ON u OFF para controlar el LED: ").strip()
    if msg in ["ON", "OFF"]:
        sock.sendto(msg.encode(), (esp_ip, port))
    else:
        print("Comando inválido.")
