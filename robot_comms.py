from imports import socket
from env_var import SERVER_IP, SERVER_PORT

def send_tcp_packet(message: str):
    # Send a message to the server and receive the response.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((SERVER_IP, SERVER_PORT))
        try:
            client_socket.sendall(message.encode('utf-8'))
            response = client_socket.recv(1024).decode('utf-8')
            return response
        except socket.error as e:
            print(f"Socket error: {e}")
            return None