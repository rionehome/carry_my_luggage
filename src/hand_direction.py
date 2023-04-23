#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import mediapipe as mp


def get_direction(n):
    cap = cv2.VideoCapture(0)
    mp_drawing = mp.solutions.drawing_utils
    mp_hands = mp.solutions.hands
    right_count = 0
    left_count = 0
    direction = None
    while True:
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5).process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.multi_hand_landmarks:
            if len(results.multi_hand_landmarks) == 2:
                left_hand = results.multi_hand_landmarks[0]
                right_hand = results.multi_hand_landmarks[1]
                left_wrist = left_hand.landmark[0]
                right_wrist = right_hand.landmark[0]
                if abs(left_wrist.y - right_wrist.y) < 0.1:
                    if left_wrist.x < right_wrist.x:
                        if direction != "Right":
                            right_count = 1
                            left_count = 0
                            direction = "Right"
                        else:
                            right_count += 1
                            if right_count >= n:
                                cap.release()
                                cv2.destroyAllWindows()
                                return "Right"
                    else:
                        if direction != "Left":
                            left_count = 1
                            right_count = 0
                            direction = "Left"
                        else:
                            left_count += 1
                            if left_count >= n:
                                cap.release()
                                cv2.destroyAllWindows()
                                return "Left"
                else:
                    if left_wrist.y < right_wrist.y:
                        if direction != "Right":
                            right_count = 1
                            left_count = 0
                            direction = "Right"
                        else:
                            right_count += 1
                            if right_count >= n:
                                cap.release()
                                cv2.destroyAllWindows()
                                return "Right"
                    else:
                        if direction != "Left":
                            left_count = 1
                            right_count = 0
                            direction = "Left"
                        else:
                            left_count += 1
                            if left_count >= n:
                                cap.release()
                                cv2.destroyAllWindows()
                                return "Left"
            else:
                for hand_landmarks in results.multi_hand_landmarks:
                    wrist = hand_landmarks.landmark[0]
                    thumb_tip = hand_landmarks.landmark[4]
                    index_tip = hand_landmarks.landmark[8]
                    mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    if index_tip.x < wrist.x:
                        if direction != "Left":
                            left_count = 1
                            right_count = 0
                            direction = "Left"
                        else:
                            left_count += 1
                            if left_count >= n:
                                cap.release()
                                cv2.destroyAllWindows()
                                return "Left"
                    else:
                        if direction != "Right":
                            right_count = 1
                            left_count = 0
                            direction = "Right"
                        else:
                            right_count += 1
                            if right_count >= n:
                                cap.release()
                                cv2.destroyAllWindows()
                                return "Right"

        cv2.imshow("MediaPipe Hands", image)
        if cv2.waitKey(5) & 0xFF == 27:
            break
    cap.release()
