from flask import Flask, jsonify, Response, render_template, request
import socket
import threading
import cv2
import numpy as np
import json

HOST = '172.20.10.7'

app = Flask(__name__)

# Shared variables
latest_frame_0 = None
latest_frame_1 = None
latest_config = {}

# Global TCP sockets for reuse
config_socket = None
image_socket = None

config_socket_lock = threading.Lock()

def connect_to_debix():
    global config_socket, image_socket

    config_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    config_socket.connect((HOST, 8888))

    image_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    image_socket.connect((HOST, 9999))

# TCP image reception (port 9999)
def receive_images():
    global latest_frame_0, latest_frame_1
    data = b""
    payload_size = 4

    while True:
        for i in range(2):  # read 2 frames
            while len(data) < payload_size:
                packet = image_socket.recv(4096)
                if not packet:
                    return
                data += packet

            frame_size = int.from_bytes(data[:payload_size], byteorder='big')
            data = data[payload_size:]

            while len(data) < frame_size:
                data += image_socket.recv(4096)

            frame_data = data[:frame_size]
            data = data[frame_size:]

            frame_np = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)

            target_width = 370
            target_height = 400

            if frame is not None:
                h, w = frame.shape[:2]
                #if w != target_width or h != target_height:
                    #frame = cv2.resize(frame, (300, 400))  # (width, height)
                
                if i == 0:
                    latest_frame_0 = frame
                else:
                    latest_frame_1 = frame

@app.route("/frame0")
def frame0():
    def generate():
        while True:
            if latest_frame_0 is not None:
                _, buffer = cv2.imencode('.jpg', latest_frame_0)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/frame1")
def frame1():
    def generate():
        while True:
            if latest_frame_1 is not None:
                _, buffer = cv2.imencode('.jpg', latest_frame_1)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')



@app.route("/write-config", methods=["POST"])
def write_config():
    try:
        config = request.get_json()
        print("Received updated config from frontend:")
        print(config)

        # Serialize with WRITE prefix
        json_data = "WRITE:" + json.dumps(config) + "\n"

        with config_socket_lock:
            config_socket.sendall(json_data.encode('utf-8'))

        return jsonify(success=True)

    except Exception as e:
        print("Failed to send config to Debix:", e)
        return jsonify(success=False, error=str(e)), 500
    
@app.route("/read-config", methods=["GET"])
def read_config():
    try:
        # Send the READ command
        with config_socket_lock:
            config_socket.sendall(b"READ\n")

        # Receive the full JSON response (you can use a buffer)
        buffer = b""
        while True:
            part = config_socket.recv(4096)
            if not part:
                break
            buffer += part
            try:
                config = json.loads(buffer.decode())
                break  # Successfully parsed, exit loop
            except json.JSONDecodeError:
                continue  # Wait for more data

        return jsonify(config)

    except Exception as e:
        print("Failed to read config from Debix:", e)
        return jsonify(success=False, error=str(e)), 500
    
@app.route("/stop", methods=["POST"])
def stop_command():
    try:
        with config_socket_lock:
            config_socket.sendall(b"STOP\n")
        return jsonify(success=True)
    except Exception as e:
        print("Failed to send STOP to Debix:", e)
        return jsonify(success=False, error=str(e)), 500


@app.route('/')
def index():
    return render_template("index.html")

if __name__ == '__main__':
    connect_to_debix()
    threading.Thread(target=receive_images, daemon=True).start()
    app.run(host='0.0.0.0')
