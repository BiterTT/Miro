from hand_gesture_detection import HandGestureDetector
import cv2

detector = HandGestureDetector()

while True:
    frame, results = detector.read_frame()
    if frame is None:
        break

    gesture = detector.detect_gesture(frame, results)
    cv2.putText(frame, f'Gesture: {gesture}', (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Gesture Window', frame)
    if cv2.waitKey(10) & 0xFF == 27:
        break

detector.release()
cv2.destroyAllWindows()
