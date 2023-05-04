import sys
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QDialog, QLabel, QPushButton
from YOLO_Fire import FireDetector
import socket
import struct
import cv2
import numpy as np


class VideoPlayer(QDialog):
    def __init__(self):
        """
        Constructor for VideoPlayer class.
        """

        super().__init__()
        model_path = 'models/best.pt'
        self.detector = FireDetector(model_path)

        # set up GUI
        self.image_label = QLabel(self)
        self.image_label.resize(640, 480)
        self.image_label.move(0, 0)

        self.button = QPushButton('Connect', self)
        self.button.move(640, 240)
        self.button.resize(160, 80)
        self.button.clicked.connect(self.connect_to_video)

        self.toggle_button = QPushButton('Detect Fire', self)
        self.toggle_button.move(640, 140)
        self.toggle_button.resize(160, 80)
        self.toggle_button.setCheckable(True)
        self.toggle_button.clicked.connect(self.toggle_detection)

        # set up timer to update video stream
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)

        self.connection = None
        self.server_socket = None
        self.is_detecting_fire = False

    def __del__(self):
        """
        Destructor for VideoPlayer class.
        """
        if self.connection:
            self.connection.close()
        if self.server_socket:
            self.server_socket.close()
        if self.detector:
            del self.detector

    def toggle_detection(self):
        """
        Toggles fire detection on or off.
        """
        try:
            self.is_detecting_fire = not self.is_detecting_fire
            print(f"{self.is_detecting_fire}")
        except:
            print("Error in toggle.")

    def connect_to_video(self):
        """
        Connects to video stream from drone and sets up fire detector.
        """
        try:
            self.server_socket = socket.socket()
            self.server_socket.bind(('192.168.1.7', 8000))  # 192.168.1.8
            self.server_socket.listen(0)
            self.connection = self.server_socket.accept()[0].makefile('rb')
            self.timer.start(1)
            self.button.setEnabled(False)
        except:
            print("Error connecting to drone")

    def update_frame(self):
        """
        Updates the current video frame and detects fire if enabled.
        """
        if self.connection:
            frame_length = struct.unpack('<L', self.connection.read(struct.calcsize('<L')))[0]
            frame_data = self.connection.read(frame_length)

            # Convert the frame data into a format that OpenCV can use
            frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)

            # detect fire and update button text

            if self.is_detecting_fire:
                boxes = self.detector.detect(frame)
                if len(boxes) > 0:
                    for box in boxes:
                        cv2.rectangle(frame, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (0, 0, 255), 2)
                        cv2.putText(frame, "Fire", (box[0], box[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            q_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()

            pixmap = QPixmap(q_image)
            self.image_label.setPixmap(pixmap)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    player = VideoPlayer()
    player.resize(800, 480)
    player.show()
    sys.exit(app.exec_())