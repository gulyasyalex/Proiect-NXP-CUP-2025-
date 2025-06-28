import socket
import numpy as np
import cv2

# Set up client socket
HOST = '172.20.10.7'  # Replace with the actual IP address of the Debix
PORT = 9999

print(f"Connecting to {HOST}:{PORT}...")

try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))
    print("Connected to server.")
except Exception as e:
    print(f"Connection failed: {e}")
    exit()


data = b''
payload_size = 4  # First receive 4 bytes for the size of each frame

def receive_frame(client_socket):
    global data  

    try:
        # Receive the size of the frame
        while len(data) < payload_size:
            packet = client_socket.recv(4096)
            if not packet:
                print("Connection closed by server.")
                return None
            data += packet

        packed_size = data[:payload_size]
        data = data[payload_size:]

        frame_size = int.from_bytes(packed_size, byteorder='big')
        print(f"Expected frame size: {frame_size} bytes")

        # If frame_size is zero or extremely large, there's a problem.
        if frame_size <= 0 or frame_size > 1000000:  # Arbitrary large size check
            print("Invalid frame size received.")
            return None

        # Receive the actual frame data
        while len(data) < frame_size:
            packet = client_socket.recv(4096)
            if not packet:
                print("Connection closed by server.")
                return None
            data += packet

        print(f"Received frame data size: {len(data)} bytes")

        # Extract frame data and reset buffer
        frame_data = data[:frame_size]
        data = data[frame_size:]  # Keep extra bytes for next frame

        # Decode the frame
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        if frame is None:
            print("Frame decoding failed.")
            return None

        return frame

    except Exception as e:
        print(f"Error during frame reception: {e}")
        return None


while True:
    frame1 = receive_frame(client_socket)
    if frame1 is None:  # Exit if there's an error or the connection is closed
        print("Exiting due to frame reception failure.")
        break
    cv2.imshow('Client Frame 1', frame1)

    frame2 = receive_frame(client_socket)
    if frame2 is None:  # Exit if there's an error or the connection is closed
        print("Exiting due to frame reception failure.")
        break
    cv2.imshow('Client Frame 2', frame2)

    '''frame3 = receive_frame(client_socket)
    if frame3 is None:  # Exit if there's an error or the connection is closed
        print("Exiting due to frame reception failure.")
        break
    cv2.imshow('Client Frame 3', frame3)'''

    if cv2.waitKey(1) == 27:  # Escape key to exit
        print("Escape key pressed. Exiting.")
        break

# Clean up resources
client_socket.close()
cv2.destroyAllWindows()