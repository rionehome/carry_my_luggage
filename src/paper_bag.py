import cv2
import numpy as np

cap = cv2.VideoCapture(0)

lower_white = np.array([0, 0, 200])
upper_white = np.array([255, 50, 255])

while True:
    ret, frame = cap.read()

    # Apply color thresholding
    mask = cv2.inRange(frame, lower_white, upper_white)
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

    # Convert to grayscale
    gray = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)

    # Threshold the image
    _, thresh = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)
    # Find contours in the image
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour (the paper bag)
    if (contours):
      largest_contour = max(contours, key=cv2.contourArea)

      # Get the bounding box of the largest contour
      x, y, w, h = cv2.boundingRect(largest_contour)

      # Calculate the center of the bounding box
      center_x = x + w // 2
      center_y = y + h // 2

      # Draw a rectangle around the paper bag
      cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

      # Calculate the distance to the paper bag
      focal_length = 600  # Replace with your own camera's focal length
      known_width = 30  # Width of the paper bag in cm
      distance = (known_width * focal_length) / w

      # Calculate the size of the paper bag
      width_cm = 45  # Width of the paper bag in cm
      height_cm = 30  # Height of the paper bag in cm
      size = (w * width_cm) / known_width, (h * height_cm) / known_width

      # Print the distance and center coordinates
      print(f"Distance to paper bag: {distance:.2f} cm")
      print(f"Center coordinates: ({center_x}, {center_y})")
      print(f"Paper bag size: ({size[0]:.2f} cm, {size[1]:.2f} cm)")

      # Show the frame
      cv2.imshow('frame', frame)

      if cv2.waitKey(1) == ord('q'):
          break

cap.release()
cv2.destroyAllWindows()
