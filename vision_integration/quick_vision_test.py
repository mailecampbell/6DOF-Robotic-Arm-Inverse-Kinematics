#!/usr/bin/env python3
"""
Quick Vision Test - Detect cubes and show results
"""

import cv2
import numpy as np
import time

print("=" * 60)
print("QUICK VISION TEST")
print("=" * 60)

# Open camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

time.sleep(1)  # Let camera warm up

ret, frame = cap.read()
if not ret:
    print("ERROR: Could not read from camera")
    cap.release()
    exit(1)

print(f"Frame size: {frame.shape[1]}x{frame.shape[0]}")

# Convert to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# Color ranges to detect
colors = [
    ("ORANGE", (8, 150, 80), (18, 255, 255), (0, 165, 255)),
    ("RED", (0, 120, 50), (6, 255, 255), (0, 0, 255)),
    ("RED2", (173, 120, 50), (179, 255, 255), (0, 0, 255)),
    ("BLACK", (0, 0, 0), (180, 255, 50), (50, 50, 50)),
    ("GREEN", (35, 80, 80), (85, 255, 255), (0, 255, 0)),
    ("BLUE", (100, 80, 80), (130, 255, 255), (255, 0, 0)),
    ("YELLOW", (20, 100, 100), (35, 255, 255), (0, 255, 255)),
    ("PINK", (140, 50, 100), (170, 255, 255), (255, 0, 255)),
]

print("\nDetecting colors...")
display = frame.copy()
detections = []

for color_name, lower, upper, bgr in colors:
    # Skip RED2 for display (it's combined with RED)
    if color_name == "RED2":
        continue

    # Create mask
    lower_np = np.array(lower)
    upper_np = np.array(upper)
    mask = cv2.inRange(hsv, lower_np, upper_np)

    # Add second red range
    if color_name == "RED":
        mask2 = cv2.inRange(hsv, np.array((173, 120, 50)), np.array((179, 255, 255)))
        mask = cv2.bitwise_or(mask, mask2)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:  # Minimum area threshold
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = w / h if h > 0 else 0

            # Filter by aspect ratio (roughly square)
            if 0.5 < aspect < 2.0:
                cx, cy = x + w//2, y + h//2
                detections.append({
                    'color': color_name,
                    'x': x, 'y': y, 'w': w, 'h': h,
                    'cx': cx, 'cy': cy,
                    'area': area
                })

                # Draw on display
                cv2.rectangle(display, (x, y), (x+w, y+h), bgr, 3)
                cv2.putText(display, color_name, (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, bgr, 2)
                cv2.circle(display, (cx, cy), 5, bgr, -1)

print(f"\nFound {len(detections)} objects:")
for d in detections:
    print(f"  {d['color']}: center=({d['cx']}, {d['cy']}), size={d['w']}x{d['h']}, area={d['area']}")

# Detect the board (black border)
print("\nDetecting board border...")
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
_, black_mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

# Find largest black contour (the border)
contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if contours:
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    # Approximate to polygon
    epsilon = 0.02 * cv2.arcLength(largest, True)
    approx = cv2.approxPolyDP(largest, epsilon, True)

    if len(approx) == 4:
        print(f"  Found quadrilateral border, area={area}")
        cv2.drawContours(display, [approx], -1, (0, 255, 0), 2)
        for pt in approx:
            cv2.circle(display, tuple(pt[0]), 8, (0, 255, 0), -1)
    else:
        print(f"  Border has {len(approx)} vertices (expected 4)")

# Detect divider line
print("\nDetecting vertical divider...")
# Look for vertical black line in the middle region
h, w = frame.shape[:2]
middle_region = black_mask[:, w//3:2*w//3]

# Use Hough lines
edges = cv2.Canny(middle_region, 50, 150)
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=100, maxLineGap=10)

if lines is not None:
    # Find most vertical line
    best_line = None
    best_score = 0
    for line in lines:
        x1, y1, x2, y2 = line[0]
        length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        # Check if mostly vertical
        if abs(x2 - x1) < 20:  # Nearly vertical
            if length > best_score:
                best_score = length
                best_line = line[0]

    if best_line is not None:
        x1, y1, x2, y2 = best_line
        # Adjust for region offset
        x1 += w//3
        x2 += w//3
        print(f"  Found divider at x={x1} to x={x2}")
        cv2.line(display, (x1, y1), (x2, y2), (255, 0, 255), 3)
else:
    print("  No divider detected")

# Save result
output_path = "/tmp/vision_test_result.jpg"
cv2.imwrite(output_path, display)
print(f"\nSaved annotated image to: {output_path}")

cap.release()
print("\nDone!")
