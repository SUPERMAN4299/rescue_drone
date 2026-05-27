from ultralytics import YOLO
import cv2

# ─────────────────────────────────────
# LOAD MODEL
# ─────────────────────────────────────

model = YOLO("yolov8n.pt")

# ─────────────────────────────────────
# OPEN WEBCAM
# ─────────────────────────────────────

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Webcam not found")
    exit()

# ─────────────────────────────────────
# DRONE STATES
# ─────────────────────────────────────

STATE_IDLE = "IDLE"
STATE_TRACKING = "TRACKING"
STATE_SEARCHING = "SEARCHING"

drone_state = STATE_IDLE

# ─────────────────────────────────────
# AI DECISION ENGINE
# ─────────────────────────────────────

def ai_decision(boxes, frame_w):

    global drone_state

    if len(boxes) == 0:

        drone_state = STATE_SEARCHING

        return "SEARCH"

    # Biggest human
    biggest = max(
        boxes,
        key=lambda b: (b[2]-b[0]) * (b[3]-b[1])
    )

    x1, y1, x2, y2 = biggest

    human_x = (x1 + x2) // 2

    center_x = frame_w // 2

    drone_state = STATE_TRACKING

    # LEFT
    if human_x < center_x - 80:
        return "LEFT"

    # RIGHT
    elif human_x > center_x + 80:
        return "RIGHT"

    # CENTER
    else:
        return "FORWARD"

# ─────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────

while True:

    ret, frame = cap.read()

    if not ret:
        break

    frame_h, frame_w = frame.shape[:2]

    # YOLO Detection
    results = model(frame)

    human_boxes = []

    for result in results:

        for box in result.boxes:

            cls = int(box.cls[0])

            # PERSON CLASS
            if cls == 0:

                x1, y1, x2, y2 = map(
                    int,
                    box.xyxy[0]
                )

                human_boxes.append(
                    (x1, y1, x2, y2)
                )

                # DRAW BOX
                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (0,255,0),
                    2
                )

    # AI Decision
    decision = ai_decision(
        human_boxes,
        frame_w
    )

    # CENTER POINT
    center_x = frame_w // 2
    center_y = frame_h // 2

    cv2.circle(
        frame,
        (center_x, center_y),
        5,
        (0,0,255),
        -1
    )

    # HUD
    cv2.putText(
        frame,
        f"State: {drone_state}",
        (10,30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255,255,255),
        2
    )

    cv2.putText(
        frame,
        f"Decision: {decision}",
        (10,70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0,255,255),
        2
    )

    cv2.putText(
        frame,
        f"Humans: {len(human_boxes)}",
        (10,110),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255,255,255),
        2
    )

    # SHOW
    cv2.imshow(
        "AI Drone Brain",
        frame
    )

    # EXIT
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()