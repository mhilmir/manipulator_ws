import cv2
import numpy as np

# Start video capture from the default webcam (index 0)
cap = cv2.VideoCapture(6)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    output = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.medianBlur(gray, 5)

    # Detect circles
    circles = cv2.HoughCircles(
        gray_blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=30,
        param1=100,
        param2=50,
        minRadius=20,
        maxRadius=100
    )

    # Draw circles
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # Outer circle
            cv2.circle(output, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # Center
            cv2.circle(output, (i[0], i[1]), 2, (0, 0, 255), 3)

    cv2.imshow("Live Circle Detection", output)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release and close
cap.release()
cv2.destroyAllWindows()
