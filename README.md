# AeroFlameGuard - Drone Fire Detection 🔥
AeroFlameGuard is an autonmous drone fire detection alogrithm that uses computer vision-based fire detection systems utilizing machine learning algorithms to detect fire in images and videos in real-time. The primary goal of this project is to provide a reliable and fast fire detection solution that can be easily integrated into various applications and platforms. The Fire Detection algorithm takes video feed from a drone that has a raspberry pi attatched on it and runs the algorithm on a local machine. This is a small implamentation of what could potentially be used in the fire prevention field in the future.

# Development

In recent years, California has faced an escalating wildfire crisis. To address this challenge, our Software Development Group, The Tie Fighters, consisting of [@nathanMDev](https://github.com/nathanMDev), [@koa-afusia](https://github.com/koa-afusia), and [@tensign1444](https://github.com/tensign1444), has developed an autonomous flying drone capable of navigating to specific GPS coordinates, hovering over a fire, and utilizing the cutting-edge YOLOv5 algorithm to detect the presence of flames. By leveraging this advanced technology, we aim to significantly improve wildfire response times while minimizing the risk to human lives involved in the detection and assessment process.

## Features

- Real-time fire detection in images and videos
- Easy integration with various platforms and applications
- Autonomous flight to GPS coordinates using a Raspberry Pi

## Installation

To install the required dependencies, run the following command:

1.) Install requirements on local machine and Raspberry Pi

```bash
pip install -r requirements.txt
```

2.) Install ServerSide.py on Raspberry Pi via SFTP

3.) All other files run locally on a machine.

## Fire Detection Current Usage

1.) Assuming requirements are installed and ServerSide.py is on the Raspberry Pi, 
adjust ```Local Machine IP``` in ServerSide.py and main.py to the IP you wish to display the video feed on.

2.) Run main.py

## Fire Detection Customizable Usage

1.) Import the fire detection module:

```
from YOLO_Fire import FireDetector
```

2.)Initialize the Fire Detection object with the desired parameters:

```
self.detector = FireDetector(model_path) #Model path will be the file path to best.pt
```

3.) Detect fire in an frame (image or frame from video):
```
#This will draw a box around the fire, if detected.

  boxes = self.detector.detect(frame)
                if len(boxes) > 0:
                    for box in boxes:
                        cv2.rectangle(frame, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (0, 0, 255), 2)
                        cv2.putText(frame, "Fire", (box[0], box[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
```

## Autonomous Drone Flight Usage

 Please feel free to use our [Mav Proxy Library](https://github.com/tensign1444/DroneFireDetection/blob/master/MavProxyLib.py) if you are looking to do autonomous drone flight with a Raspberry Pi and Pixhawk.

 1.) Create a drone object of the library:

 ```
 BigTie = Drone(False, 10)
 ```

 2.) Takeoff (movement is in meters): 

 ```
 BigTie.takeoff(5)
 ```
 
 3.) Use as intended. Please remember to follow all [FAA rules and regulations for drones](https://www.faa.gov/uas).

 4.) Land when done.

 ```
 BigTie.land()
 ```

## Contributing
 
 Thank you to [@spacewalk01](https://github.com/spacewalk01) for having his pretrained fire detection model open source and allowing us to use it for this project. Thank you to Professor Tallman for teaching us about Yolo and how to programatically fly drones. Thank you to CUI for providing the drones and Raspberry pi's.

Other contributions are welcome! If you have any ideas or suggestions to improve the Drone Fire Detection system, please open an issue or create a pull request.

## License 

This project is licensed under the [MIT License](https://github.com/tensign1444/DroneFireDetection/blob/master/LICENSE).
