import cv2

# Callback function to handle mouse events
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        print(f"Mouse moved to: ({x}, {y})")
    elif event == cv2.EVENT_LBUTTONDOWN:
        print(f"Left click at: ({x}, {y})")

# Open video stream (0 = default webcam)
cap = cv2.VideoCapture(0)

cv2.namedWindow("Stream")
cv2.setMouseCallback("Stream", mouse_callback)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Stream", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):  # Press q to exit
        break

cap.release()
cv2.destroyAllWindows()
