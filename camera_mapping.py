import cv2

print("=== OPENCV ACTIVE CAMERA INDEX SCAN ===")
for idx in range(10):
    # cv2.CAP_DSHOW forces Windows to talk to identical cameras cleanly
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW) 
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            window_name = f"Camera Index {idx}"
            # Show the feed so you can see which lane it is
            cv2.imshow(window_name, cv2.resize(frame, (320, 240)))
            print(f"✅ Found active camera at Index: {idx}")
            
print("\nPress any key with the windows open to close the scan.")
cv2.waitKey(0)
cv2.destroyAllWindows()