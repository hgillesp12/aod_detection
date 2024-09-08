import cv2
import os

# Define paths to the directory containing images and YOLO label files
image_directory = '/Users/hannahgillespie/aod_detection/blender_automation/bark/renderings/images'  # Replace with the path to your images
label_directory = '/Users/hannahgillespie/aod_detection/blender_automation/bark/renderings/labels'  # Replace with the path to your YOLO label files
output_directory = '/Users/hannahgillespie/aod_detection/blender_automation/bark/renderings/output_images'  # Replace with the path to save images with bounding boxes

# Define colors for each class
class_colors = {
    0: (0, 255, 0),      # Green for low_risk_oak
    1: (0, 255, 255),    # Yellow for medium_risk_oak
    2: (0, 0, 255),       # Red for high_risk_oak
    3: (128, 128, 128)  # Grey for non_oak
}

# Ensure the output directory exists
os.makedirs(output_directory, exist_ok=True)

# Loop through all label files in the label directory
for label_file in os.listdir(label_directory):
    if label_file.endswith('.txt'):
        # Read the corresponding image file
        image_name = label_file.replace('.txt', '.png')  # Adjust if images have a different format
        image_path = os.path.join(image_directory, image_name)

        # Check if the image exists
        if not os.path.exists(image_path):
            print(f"Image {image_path} not found, skipping...")
            continue

        # Load the image
        image = cv2.imread(image_path)
        height, width, _ = image.shape

        # Read the YOLO label file
        with open(os.path.join(label_directory, label_file), 'r') as file:
            for line in file:
                # Parse the label line
                label_data = line.strip().split()
                class_id = int(label_data[0])
                x_center, y_center, box_width, box_height = map(float, label_data[1:])

                # Convert YOLO normalized coordinates to absolute pixel values
                x_center_abs = int(x_center * width)
                y_center_abs = int(y_center * height)
                box_width_abs = int(box_width * width)
                box_height_abs = int(box_height * height)

                # Calculate the top-left and bottom-right corners of the bounding box
                x1 = int(x_center_abs - box_width_abs / 2)
                y1 = int(y_center_abs - box_height_abs / 2)
                x2 = int(x_center_abs + box_width_abs / 2)
                y2 = int(y_center_abs + box_height_abs / 2)

                # Get the color for the bounding box based on class
                color = class_colors.get(class_id, (255, 255, 255))  # Default to white if class_id is not found

                # Draw the bounding box on the image
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                # Optional: Put the class label on the bounding box
                cv2.putText(image, f'Class {class_id}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Save the image with the bounding box to the output directory
        output_path = os.path.join(output_directory, image_name)
        cv2.imwrite(output_path, image)
        print(f"Processed and saved {output_path}")
