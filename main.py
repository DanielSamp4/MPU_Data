import graph
import cv2_mediapipe
# import readSerial
from threading import Thread

def main():
    # Criar as threads para as funções
    # serial_thread = Thread(target=readSerial.get_serial)
    mediapipe_thread = Thread(target=cv2_mediapipe.run_mediapipe)
    graph_thread = Thread(target=graph.plot_graph)

    # Iniciar as threads
    # serial_thread.start()
    mediapipe_thread.start()
    graph_thread.start()

    # Aguardar o término das threads
    # serial_thread.join()
    mediapipe_thread.join()
    graph_thread.join()


if __name__ == '__main__':
    main()
