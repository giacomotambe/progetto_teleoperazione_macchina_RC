import cv2

# Select the appropriate video device. Replace '2' with the correct device number.
cap = cv2.VideoCapture('odometry1.avi')

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()



while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if ret:
        # Write the frame into the file 'output.avi'
        
        # Display the resulting frame
        cv2.imshow('Frame', frame)
        
        # Press 'q' on the keyboard to exit
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()

cv2.destroyAllWindows()




