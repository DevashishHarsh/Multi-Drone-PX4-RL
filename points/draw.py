import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import json
from pathlib import Path

def get_user_drawing(canvas_size=(512, 512)):
    drawing = False
    exiting = False
    points = []
    undo_stack = []
    redo_stack = []
    status = None  # 'n' for next, 'enter' to finish

    def draw(event, x, y, flags, param):
        nonlocal drawing, points, undo_stack, redo_stack
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            redo_stack.clear()
            undo_stack.append(points.copy())
            points.append((x, y))
        elif event == cv2.EVENT_MOUSEMOVE and drawing:
            points.append((x, y))
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False

    canvas = np.ones(canvas_size, dtype=np.uint8) * 255
    window_name = "Draw Shape (Press Enter to finish, 'n' for next)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, draw)

    while True:
        temp_canvas = canvas.copy()
        for i in range(1, len(points)):
            cv2.line(temp_canvas, points[i - 1], points[i], 0, 2)

        cv2.imshow(window_name, temp_canvas)
        key = cv2.waitKey(1) & 0xFF

        if key == 13:  # Enter key
            status = 'enter'
            break
        elif key == ord('n'):
            status = 'n'
            break
        elif key == 26 and undo_stack:  # Ctrl+Z
            redo_stack.append(points.copy())
            points = undo_stack.pop()
        elif key == 25 and redo_stack:  # Ctrl+Y
            undo_stack.append(points.copy())
            points = redo_stack.pop()
        elif key == 27:
            exiting = True
            break

    for i in range(1, len(points)):
        cv2.line(canvas, points[i - 1], points[i], 0, 2)

    cv2.destroyWindow(window_name)
    return canvas, status, exiting

def extract_outline(image):
    _, thresh = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contours[0] if contours else None

def distribute_points_evenly(contour, num_points):
    contour = contour[:, 0, :]
    distances = [0]
    for i in range(1, len(contour)):
        dist = np.linalg.norm(contour[i] - contour[i - 1])
        distances.append(distances[-1] + dist)

    total_length = distances[-1]
    if total_length == 0:
        return np.zeros((num_points, 2))
    spacing = total_length / num_points
    target_dists = [i * spacing for i in range(num_points)]

    result_points = []
    j = 0
    for d in target_dists:
        while j < len(distances) - 1 and distances[j + 1] < d:
            j += 1
        if j == len(distances) - 1:
            # if we reached the end, append the last point
            result_points.append(contour[-1].astype(float))
            continue
        denom = (distances[j + 1] - distances[j])
        t = 0.0 if denom == 0 else (d - distances[j]) / denom
        pt = (1 - t) * contour[j] + t * contour[j + 1]
        result_points.append(pt)

    return np.array(result_points)

def show_all_previous_drawings(json_path, offset, canvas_size=(512, 512), scale=40):
    p = Path(json_path)
    if not p.exists():
        print("No previous drawings found.")
        return

    with p.open("r") as f:
        try:
            data = json.load(f)
        except json.JSONDecodeError:
            print("Invalid JSON format.")
            return

    for idx, drone_points in data.items():
        canvas = np.ones(canvas_size, dtype=np.uint8) * 255

        for key, coord in drone_points.items():
            x = int(float(coord[0]) * scale)
            y = int((float(coord[1]) - offset) * scale)
            cv2.circle(canvas, (x, y), 4, 0, -1)
            cv2.putText(canvas, str(key), (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0, 1)

        window_name = f"Drawing #{idx}"
        cv2.imshow(window_name, canvas)

    print("\nPress Enter in any window to close all and exit.")
    while True:
        if cv2.waitKey(1) & 0xFF == 13:  # Enter
            break

    cv2.destroyAllWindows()

def main():
    # Ensure JSON file sits next to this script regardless of working directory
    filename = Path(__file__).parent / "drone_points.json"

    data = {}

    index = 0
    try:
        drones = int(input("Enter the number of drones : "))
    except Exception:
        print("Invalid input. Using 2 drones by default.")
        drones = 2

    offset = 2 # int(input("Enter the offset in X axis : "))
    scale = 40 # int(input("Enter the scale of the drawing for the drone points (20 - 50) : "))

    while True:
        print("\nDraw shape on canvas. Press 'n' for next or Enter to finish and view all drawings.")
        img, status, exiting = get_user_drawing()
        contour = extract_outline(img)

        if contour is None:
            if exiting:
                print("Exiting the program...")
                break
            print("No shape detected. Try again.")
            continue

        points = distribute_points_evenly(contour, drones)

        drone_points = {}
        for i in range(len(points)):
            # store coordinates relative to canvas scaled to "world" units
            drone_points[str(i + 2)] = [
                round(float(points[i][0]) / scale, 4),
                round(float(points[i][1]) / scale + offset, 4),
                -2.0
            ]

        data[str(index)] = drone_points
        index += 1

        with filename.open("w") as f:
            json.dump(data, f, indent=2)

        print(f"Saved drawing #{index - 1} with {len(drone_points)} points.")

        if status == 'n':
            continue
        elif status == 'enter':
            show_all_previous_drawings(filename, offset)
            break

if __name__ == "__main__":
    main()
