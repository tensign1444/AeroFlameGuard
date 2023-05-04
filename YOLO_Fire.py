import cv2
import torch
from pathlib import Path

from yolov5.utils.augmentations import letterbox

from yolov5.models.experimental import attempt_load
from yolov5.utils.general import non_max_suppression, scale_boxes
from yolov5.utils.torch_utils import select_device

class FireDetector:
    def __init__(self, model_path, img_size=640, conf_thres=0.1, iou_thres=0.1):
        self.device = select_device('')
        self.model = attempt_load(model_path)

        self.img_size = img_size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres

    def detect(self, image):
        img = letterbox(image, self.img_size, stride=32)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1).copy()
        img = torch.from_numpy(img).float().to(self.device)

        img /= 255.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        pred = self.model(img, augment=False)[0]
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)

        boxes = []
        for det in pred:
            if det is not None and len(det):
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], image.shape).round()
                for *xyxy, conf, cls in det:
                    if int(cls) == 0:
                        x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                        boxes.append([x1, y1, x2 - x1, y2 - y1])

        return boxes

    def isFire(self, image):
        detections = self.detect(image)
        return len(detections) > 0

"""model_path = 'models/best.pt'

fire_detector = FireDetector(model_path)
cap = cv2.VideoCapture('input.mp4')
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    detections = fire_detector.detect(frame)
    for (x, y, w, h) in detections:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
    cv2.imshow('Fire Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()"""
