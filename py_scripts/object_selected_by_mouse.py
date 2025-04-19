import cv2
from ultralytics import YOLO

# Load the model
yolo = YOLO('yolov8s.pt')

# Load the video capture
videoCap = cv2.VideoCapture(6)

# Store detected boxes globally to access in mouse callback
detected_boxes = []

# Store clicked info globally
clicked_info = None

# Mouse callback function
def mouse_callback(event, x, y, flags, param):
    global clicked_info
    if event == cv2.EVENT_LBUTTONDOWN:
        for (x1, y1, x2, y2, class_name, conf) in detected_boxes:
            if x1 <= x <= x2 and y1 <= y <= y2:
                print(f"Clicked on: {class_name}, Confidence: {conf:.2f}")
                clicked_info = (class_name, conf)
                break

# Set up mouse callback on window
cv2.namedWindow("frame")
cv2.setMouseCallback("frame", mouse_callback)

# Function to get class colors
def getColours(cls_num):
    base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    color_index = cls_num % len(base_colors)
    increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
    color = [base_colors[color_index][i] + increments[color_index][i] * 
    (cls_num // len(base_colors)) % 256 for i in range(3)]
    return tuple(color)

while True:
    ret, frame = videoCap.read()
    if not ret:
        continue

    detected_boxes.clear()  # Clear previous frame's boxes
    results = yolo.track(frame, stream=True)

    for result in results:
        classes_names = result.names
        for box in result.boxes:
            if box.conf[0] > 0.65:
                [x1, y1, x2, y2] = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cls = int(box.cls[0])
                class_name = classes_names[cls]
                colour = getColours(cls)

                # Save box info for mouse checking
                detected_boxes.append((x1, y1, x2, y2, class_name, box.conf[0]))

                # Draw rectangle and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                cv2.putText(frame, f'{class_name} {box.conf[0]:.2f}', (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)

    # Show clicked info on frame
    if clicked_info:
        print("works")
        text = f'Selected: {clicked_info[0]} ({clicked_info[1]:.2f})'
        cv2.putText(frame, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 0, 255), 2)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

videoCap.release()
cv2.destroyAllWindows()
