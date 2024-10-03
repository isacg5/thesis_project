import socket
import struct
import time

def send_message(host, port, message):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        while True:
            encoded_message = message.encode('utf-8')
            message_length = struct.pack('!I', len(encoded_message))  # '!I' significa número entero sin signo en big-endian
            sock.sendall(message_length + encoded_message)

            response_length = recv_exact(sock, 4)
            if not response_length:
                break
            response_length = struct.unpack('!I', response_length)[0]
            response = recv_exact(sock, response_length)
            if not response:
                break
            print(f"Respuesta del servidor: {response.decode('utf-8')}")
            
            #time.sleep(1)  # Esperar un segundo antes de enviar el siguiente mensaje

def recv_exact(sock, num_bytes):
    data = b''
    while len(data) < num_bytes:
        packet = sock.recv(num_bytes - len(data))
        if not packet:
            return None
        data += packet
    return data

if __name__ == "__main__":
    host = '127.0.0.1'
    port = 65433
    message = "3/42.44798278808594/IDLE/[[305.06448364 438.24191284]\n [329.00183105 460.74438477]\n [ -1.          -1.        ]\n [ -1.          -1.        ]\n [ -1.          -1.        ]\n [ -1.          -1.        ]\n [370.98684692 471.99783325]\n [ -1.          -1.        ]\n [299.12524414 533.82592773]\n [ -1.          -1.        ]\n [ -1.          -1.        ]\n [347.10568237 533.83502197]\n [ -1.          -1.        ]\n [317.09417725 618.16821289]\n [305.08868408 432.61590576]\n [317.10122681 432.61627197]\n [ -1.          -1.        ]\n [341.0713501  438.24676514]]/[54.53672409 43.87835693 30.50309753         nan         nan 38.97347641\n 24.93242264         nan 18.81970978         nan         nan 29.61429024\n         nan 18.73027802 31.78191566 71.58540344         nan 48.52221298]/"
    send_message(host, port, message)
