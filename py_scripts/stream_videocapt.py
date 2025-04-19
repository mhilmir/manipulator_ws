import cv2
import sys

if len(sys.argv) < 2:
    print("error: please give an argument for camera id")
    exit()

# Load the video capture
videoCap = cv2.VideoCapture(int(sys.argv[1]))

while True:
    ret, frame = videoCap.read()
    if not ret:
        continue

    # show the image
    cv2.imshow('frame', frame)

    # break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# release the video capture and destroy all windows
videoCap.release()
cv2.destroyAllWindows()
