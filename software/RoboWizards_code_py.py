import serial
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

MODE = input("Enter CALIBRATE to calibrate and DETECT to detect : ")  
SERIAL_PORT = 'COM11'  #change as per requirement
BAUD_RATE = 115200     #change as per requirement

PAN_MAX = 180
TILT_MAX = 180
PAN_STEP = 3
TILT_STEP = 5

GRID_W = (PAN_MAX // PAN_STEP) + 1
GRID_H = (TILT_MAX // TILT_STEP) + 1

DIFF_THRESH_MM = 100 
MIN_BLOB_AREA = 6     

def init_serial():
    try:
        s = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(3) 
        print(f"[CONNECTED] Arduino on {SERIAL_PORT}")
        return s
    except Exception as e:
        print(f"[ERROR] Serial Connection Failed: {e}")
        sys.exit()

def get_depth_map(arduino):
    arduino.reset_input_buffer()
    arduino.write(b'S') 
    
    depth_map = np.full((GRID_H, GRID_W), 8190, dtype=np.int16)
    print(f"[{MODE}] Scanning...")
    
    while True:
        line = arduino.readline().decode(errors='ignore').strip()
        if line == "END_SCAN": break
        if line == "START_SCAN": continue
            
        try:
            parts = line.split(',')
            if len(parts) == 3:
                p, t, d = map(int, parts)
                col = p // PAN_STEP
                row = t // TILT_STEP
                if 0 <= col < GRID_W and 0 <= row < GRID_H:
                    depth_map[row, col] = d
        except ValueError:
            pass
            
    return cv2.medianBlur(depth_map.astype(np.uint16), 3)

def grid_to_cartesian(row, col, dist):
    pan = col * PAN_STEP
    tilt = row * TILT_STEP
    
    theta = np.radians(pan)
    phi = np.radians(tilt)
    
    z = dist * np.sin(phi)
    xy_proj = dist * np.cos(phi)
    x = xy_proj * np.cos(theta)
    y = xy_proj * np.sin(theta)
    
    return x, y, z

def calculate_3d_centroid(contour, depth_map):
    x_sum = 0
    y_sum = 0
    z_sum = 0
    count = 0
    
    for point in contour:
        col = point[0][0]
        row = point[0][1]
        
        dist = depth_map[row, col]
        
        if 20 < dist < 8000:
            x, y, z = grid_to_cartesian(row, col, dist)
            x_sum += x
            y_sum += y
            z_sum += z
            count += 1
            
    if count == 0: return None
    
    return (x_sum / count, y_sum / count, z_sum / count)

def visualize_3d_result(current_map, target_mask, centroids):
    print("Generating 3D Spatial Map...")
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    bg_points = []
    target_points = []
    
    for row in range(GRID_H):
        for col in range(GRID_W):
            dist = current_map[row, col]
            if dist > 8000 or dist < 20: continue 
                
            x, y, z = grid_to_cartesian(row, col, dist)
            
            if target_mask[row, col] == 255:
                target_points.append([x, y, z])
            else:
                if (row * col) % 8 == 0: 
                    bg_points.append([x, y, z])

    if bg_points:
        bg = np.array(bg_points)
        ax.scatter(bg[:,0], bg[:,1], bg[:,2], c='lightgray', s=5, alpha=0.3, label='Enclosure')

    if target_points:
        tg = np.array(target_points)
        ax.scatter(tg[:,0], tg[:,1], tg[:,2], c='red', s=20, label='Target Clusters')

    for i, center in enumerate(centroids):
        cx, cy, cz = center
        ax.scatter([cx], [cy], [cz], c='black', marker='*', s=300, zorder=100)
        ax.text(cx, cy, cz + 50, f"  TARGET {i+1}", color='black', fontsize=10, fontweight='bold')

    ax.scatter([0], [0], [0], c='blue', marker='^', s=100, label='Base')

    ax.set_title(f"Detected {len(centroids)} Targets")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    
    try: ax.set_box_aspect([1,1,0.5])
    except: pass
        
    plt.legend()
    plt.show()

def main():
    arduino = init_serial()
    
    if MODE == 'CALIBRATE':
        print("\n>>> CALIBRATION MODE <<<")
        input("Press Enter to Start...")
        baseline = get_depth_map(arduino)
        np.save('depth_calib.npy', baseline)
        print("[SUCCESS] Calibration Saved.")
        plt.imshow(baseline, cmap='viridis'); plt.show()

    elif MODE == 'DETECT':
        if not os.path.exists('depth_calib.npy'):
            print("Run CALIBRATE first!"); return
            
        baseline = np.load('depth_calib.npy')
        
        print("\n>>> DETECTION MODE <<<")
        input("Press Enter to Scan...")
        current = get_depth_map(arduino)
        
        diff = baseline.astype(np.int32) - current.astype(np.int32)
        mask = np.where(diff > DIFF_THRESH_MM, 255, 0).astype(np.uint8)
        
        kernel = np.ones((5,5), np.uint8)
        solid_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        clean_mask = cv2.morphologyEx(solid_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        
        contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        centroids_3d = []
        valid_count = 0
        
        print("\n--- TARGET REPORT ---")
        for cnt in contours:
            if cv2.contourArea(cnt) >= MIN_BLOB_AREA:
                valid_count += 1
                
                
                center_3d = calculate_3d_centroid(cnt, current)
                
                if center_3d:
                    centroids_3d.append(center_3d)
                    cx, cy, cz = center_3d
                    print(f"Target #{valid_count}: Center at ({cx:.0f}, {cy:.0f}, {cz:.0f}) mm")

        print(f"TOTAL: {valid_count} Targets")
        
        visualize_3d_result(current, clean_mask, centroids_3d)

if __name__ == "__main__":
    main()
