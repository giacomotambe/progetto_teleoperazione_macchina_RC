import cv2

# Select the appropriate video device. Replace '2' with the correct device number.
cap = cv2.VideoCapture('/dev/video6')

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()

# Get the video frame width and height
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# Define the codec and create VideoWriter object
out = cv2.VideoWriter('odometry1.avi', 
                      cv2.VideoWriter_fourcc('M','J','P','G'), 
                      30, 
                      (frame_width, frame_height))

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if ret:
        # Write the frame into the file 'output.avi'
        out.write(frame)
        
        # Display the resulting frame
        cv2.imshow('Frame', frame)
        
        # Press 'q' on the keyboard to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()
