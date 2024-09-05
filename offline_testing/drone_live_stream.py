import mss
import numpy as np
import cv2
import torch
import warnings

warnings.filterwarnings("ignore", category=FutureWarning)

# Load YOLOv5 model with custom weights
model = torch.hub.load('yolov5', 'custom', path='best.pt', source='local')

# Define custom colors for each class
custom_colors = {
    'non_oak': (128, 128, 128),        # Grey
    'low_risk_oak': (0, 255, 0),       # Green
    'medium_risk_oak': (0, 255, 255),  # Yellow
    'high_risk_oak': (0, 0, 255)       # Red
}

# Define the region of the screen to capture
screen_region = {'left': 0, 'top': 200, 'width': 640, 'height': 480}

# Start capturing the screen using mss
with mss.mss() as sct:
    while True:
        # Capture the screen
        screen_frame = sct.grab(screen_region)

        # Convert the captured image to a numpy array
        frame = np.array(screen_frame)

        # Convert RGB to BGR (OpenCV format)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Perform inference using YOLOv5 model
        results = model(frame)

        # Extract bounding boxes, class names, and confidences
        for det in results.xyxy[0]:  # Loop through detections
            xyxy = det[:4]  # Bounding box coordinates
            conf = det[4]   # Confidence score
            cls_id = int(det[5])  # Class ID

            # Get the class name
            class_name = results.names[cls_id]
            # Get the color for the class name
            color = custom_colors.get(class_name, (255, 255, 255))  # Default to white

            # Draw the bounding box
            cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color, 2)

            # Draw the label
            label = f'{class_name} {conf:.2f}'
            cv2.putText(frame, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # Display the frame
        cv2.imshow('YOLOv5 Detection', frame)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release OpenCV windows
cv2.destroyAllWindows()
