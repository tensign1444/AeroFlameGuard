import io
import socket
import struct
import time
import picamera

client_socket = socket.socket()
client_socket.connect(('192.168.1.6', 8000))

connection = client_socket.makefile('wb')
try:
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.framerate = 24
        time.sleep(2)
        start_time = time.time()
        stream = io.BytesIO()
        for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
            connection.write(struct.pack('<L', stream.tell()))
            connection.flush()
            stream.seek(0)
            connection.write(stream.read())
            stream.seek(0)
            stream.truncate()
finally:
    connection.close()
    client_socket.close()
