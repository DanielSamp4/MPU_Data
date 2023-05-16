import cv2
import mediapipe as mp

def run_mediapipe():
    # Setup
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_pose = mp.solutions.pose

    # Inicializar o OpenCV para capturar o vídeo da webcam
    cap = cv2.VideoCapture(0)  # O argumento 0 indica que a webcam padrão será usada

    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while cap.isOpened():
            # Ler cada quadro do vídeo
            success, image = cap.read()
            if not success:
                print("Ignoring No Video in frame")
                continue
        
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS, landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

            # Mostrar a imagem resultante
            cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
        
            # Verificar se a tecla 'q' foi pressionada para sair
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Liberar recursos
    cap.release()
    cv2.destroyAllWindows()
