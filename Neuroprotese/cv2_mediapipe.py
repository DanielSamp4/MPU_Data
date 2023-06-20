import cv2
import mediapipe as mp
import math

# def calculate_angle(a, b, c):
#     # Calcula o vetor AB
#     vec_ab = [b.x - a.x, b.y - a.y, b.z - a.z]

#     # Calcula o vetor BC
#     vec_bc = [c.x - b.x, c.y - b.y, c.z - b.z]

#     # Calcula o produto escalar AB . BC
#     dot_product = sum([vec_ab[i] * vec_bc[i] for i in range(3)])

#     # Calcula o comprimento dos vetores AB e BC
#     magnitude_ab = math.sqrt(sum([vec_ab[i] ** 2 for i in range(3)]))
#     magnitude_bc = math.sqrt(sum([vec_bc[i] ** 2 for i in range(3)]))

#     # Calcula o ângulo em radianos
#     angle_rad = math.acos(dot_product / (magnitude_ab * magnitude_bc))

#     # Converte o ângulo para graus
#     angle_deg = math.degrees(angle_rad)

#     return angle_deg

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

            # if results.pose_landmarks is not None:
            #     # Obter as coordenadas dos pontos do calcanhar e joelho
            #     left_heel = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HEEL]
            #     left_knee = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_KNEE]
            #     left_hip = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP]
            #     # Calcular o ângulo entre os pontos do calcanhar e joelho com o plano do eixo Z
            #     angle_deg = calculate_angle(left_heel, left_knee, left_hip)

            #     # Desenhar o ângulo no frame
            #     cv2.putText(image, f'Angle: {angle_deg:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)


            mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS, landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

            # Mostrar a imagem resultante
            cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
        
            # Verificar se a tecla 'q' foi pressionada para sair
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Liberar recursos
    cap.release()
    cv2.destroyAllWindows()
