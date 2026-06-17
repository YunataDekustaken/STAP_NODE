import cv2
import win32com.client

print("[STAP Test] Connecting to Windows Management Interface...")
try:
    wmi = win32com.client.GetObject("winmgmts:")
    devices = wmi.InstancesOf("Win32_PnPEntity")
    print("[STAP Test] Connected successfully! Scanning for USB cameras...")
    
    camera_count = 0
    for device in devices:
        # Check for Video Devices
        if device.PNPClass == "Image" or "Camera" in str(device.Name):
            print(f"\n🎥 Camera Name: {device.Name}")
            print(f"🆔 Permanent Hardware DeviceID:\n   {device.DeviceID}")
            print("-" * 50)
            camera_count += 1
            
    if camera_count == 0:
        print("[STAP Test] ⚠️ No hardware USB cameras detected by the OS.")
    else:
        print(f"\n[STAP Test] Scan complete. Found {camera_count} camera(s).")
        
except Exception as e:
    print(f"[STAP Test] ❌ Error scanning registry: {e}")

print("\n[STAP Test] Checking if standard OpenCV can open Index 0...")
cap = cv2.VideoCapture(0)
if cap.isOpened():
    print("✅ OpenCV successfully opened Camera Index 0!")
    cap.release()
else:
    print("❌ OpenCV failed to open Camera Index 0 (either busy, invalid, or no index).")