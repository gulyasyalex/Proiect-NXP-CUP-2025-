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

fourcc = cv2.VideoWriter_fourcc(*'XVID')
fps = 100
layout_writer = None  # Will initialize later when we know layout size

def receive_frame(client_socket):
    global data
    try:
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

        if frame_size <= 0 or frame_size > 1000000:
            print("Invalid frame size received.")
            return None

        while len(data) < frame_size:
            packet = client_socket.recv(4096)
            if not packet:
                print("Connection closed by server.")
                return None
            data += packet

        frame_data = data[:frame_size]
        data = data[frame_size:]

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
    if frame1 is None:
        print("Exiting due to frame reception failure.")
        break

    frame2 = receive_frame(client_socket)
    if frame2 is None:
        print("Exiting due to frame reception failure.")
        break

    '''frame3 = receive_frame(client_socket)
    if frame3 is None:
        print("Exiting due to frame reception failure.")
        break
    '''
    # Determine the highest frame height
    #target_height = max(frame1.shape[0], frame2.shape[0], frame3.shape[0])
    target_height = max(frame1.shape[0], frame2.shape[0])

    # Resize all frames to the same height while maintaining aspect ratio
    def resize_to_height(frame, height):
        h, w = frame.shape[:2]
        scale = height / h
        return cv2.resize(frame, (int(w * scale), height))

    frame1 = resize_to_height(frame1, target_height)
    frame2 = resize_to_height(frame2, target_height)
    #frame3 = resize_to_height(frame3, target_height)

    # Combine frames horizontally
    #layout = cv2.hconcat([frame1, frame2, frame3])
    layout = cv2.hconcat([frame1, frame2])

    # Initialize video writer when layout size is known
    if layout_writer is None:
        layout_h, layout_w = layout.shape[:2]
        layout_writer = cv2.VideoWriter('layout.avi', fourcc, fps, (layout_w, layout_h))

    # Save and show
    layout_writer.write(layout)
    cv2.imshow('Layout View', layout)

    if cv2.waitKey(1) == 27:  # Escape key
        print("Escape key pressed. Exiting.")
        break

# Clean up
client_socket.close()
cv2.destroyAllWindows()
if layout_writer:
    layout_writer.release()
