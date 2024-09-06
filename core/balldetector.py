import cv2
import numpy as np
import mss
import math
from geopy.distance import geodesic
from ultralytics import YOLO
import json
import time
import os
import errno

# Initial coordinates and size of the target window
x, y, width, height = 1779, 238, 1081, 875

# Load the YOLOv8 model
model = YOLO('./yolov8n.pt')

# Truck class index in COCO dataset
truck_class_index = 32

# Camera parameters (update these with your drone's actual camera specs)
vfov = 98.89  # Vertical FOV in degrees
hfov = 114.59  # Horizontal FOV in degrees

status_fifo_path = '/tmp/gpt_status_fifo'
obcoords_fifo_path = '/tmp/gpt_obcoords_fifo'

def capture_region(x, y, width, height):
    with mss.mss() as sct:
        monitor = {"top": y, "left": x, "width": width, "height": height}
        sct_img = sct.grab(monitor)
        img = np.array(sct_img)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img

def calculate_gps_coordinates(lat, lon, bearing, distance):
    origin = (lat, lon)
    destination = geodesic(meters=distance).destination(origin, bearing)
    return destination.latitude, destination.longitude

def calculate_object_position(frame, bbox, lat, lon, altitude, bearing):
    frame_height, frame_width = frame.shape[:2]
    x1, y1, x2, y2 = bbox
    centroid_x = (x1 + x2) / 2
    centroid_y = (y1 + y2) / 2

    # Calculate angles from the center of the frame
    angle_x = ((centroid_x / frame_width) - 0.5) * hfov
    angle_y = -((centroid_y / frame_height) - 0.5) * vfov  # Negative because image is inverted

    # Calculate ground distance using both angles
    ground_distance = altitude * math.tan(math.radians(math.sqrt(angle_x**2 + angle_y**2)))
    
    # Calculate the relative bearing to the object
    relative_bearing = math.degrees(math.atan2(angle_x, angle_y))
    
    # Calculate the absolute bearing to the object
    object_bearing = (bearing + relative_bearing) % 360

    # Calculate GPS coordinates of the object
    object_lat, object_lon = calculate_gps_coordinates(lat, lon, object_bearing, ground_distance)

    return object_lat, object_lon

def draw_bbox_and_label(frame, bbox, label, color=(0, 255, 0)):
    x1, y1, x2, y2 = bbox
    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
    cv2.putText(frame, label, (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def read_status():
    try:
        with open(status_fifo_path, 'r') as fifo:
            status_json = fifo.readline().strip()
            return json.loads(status_json)
    except Exception as e:
        print(f"Error reading status: {e}")
        return None

def write_obcoords(data):
    try:
        if not data:
            data = "nothing detected yet"
        with open(obcoords_fifo_path, 'w') as fifo:
            fifo.write(data + "\n")
    except Exception as e:
        print(f"Error writing to FIFO: {e}")

def create_fifo(fifo_path):
    try:
        os.mkfifo(fifo_path)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

def draw_transparent_text(image, text, position, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=0.8, font_thickness=1, text_color=(255, 255, 255), bg_color=(0, 0, 0), bg_opacity=0.5):
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size

    x, y = position
    bg_rect = (x, y - text_h, x + text_w, y)

    overlay = image.copy()
    cv2.rectangle(overlay, (bg_rect[0] - 2, bg_rect[1] - 2), (bg_rect[2] + 2, bg_rect[3] + 2), bg_color, -1)
    cv2.addWeighted(overlay, bg_opacity, image, 1 - bg_opacity, 0, image)

    cv2.putText(image, text, position, font, font_scale, text_color, font_thickness)

def main():
    cv2.namedWindow("Drone View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Drone View", 640, 480)

    current_lat, current_lon, altitude, bearing = None, None, None, None
    last_status_read = time.time()
    last_write_time = time.time()

    create_fifo(obcoords_fifo_path)

    while True:
        frame = capture_region(x, y, width, height)
        frame = cv2.flip(frame, -1)  # Keep the image inversion
        resized_frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)

        if time.time() - last_status_read > 0.2:
            status = read_status()
            last_status_read = time.time()
            if status:
                current_lat = status["latitude"]
                current_lon = status["longitude"]
                altitude = status["altitude"]
                bearing = status["bearing"]

        if all([current_lat is not None, current_lon is not None, altitude is not None, bearing is not None]):
            results = model(resized_frame, classes=[truck_class_index], conf=0.12)

            object_data = []
            object_count = 1

            for result in results:
                if result.boxes:
                    for box in result.boxes:
                        bbox = box.xyxy[0].tolist()
                        object_lat, object_lon = calculate_object_position(
                            resized_frame, bbox, current_lat, current_lon, altitude, bearing
                        )
                        object_name = f"Ball{object_count}"
                        object_data.append(f"{object_name}: {object_lat:.6f}, {object_lon:.6f}")
                        draw_bbox_and_label(resized_frame, bbox, object_name)
                        object_count += 1

            # Display the coordinates on the screen in a transparent way
            for i, obj in enumerate(object_data):
                draw_transparent_text(resized_frame, obj, (10, 20 * (i + 1)))

            if time.time() - last_write_time > 0.5:
                if object_data:
                    obcoords_string = ", ".join(object_data)
                    write_obcoords(obcoords_string)
                else:
                    write_obcoords(None)  # Ensure "nothing detected yet" is written if no objects detected
                last_write_time = time.time()

            os.system('clear')  # For Unix-based systems
            print(f"Drone Location: Lat={current_lat:.6f}, Lon={current_lon:.6f}")
            print(f"Altitude: {altitude:.3f}m, Bearing: {bearing:.1f}°")

        cv2.imshow("Drone View", resized_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


