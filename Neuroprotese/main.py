import graph as graph
import cv2_mediapipe as cv2_mediapipe
import circularBuffer as buff
import matplotlib.pyplot as plt
import readSerial as Serial
from threading import Thread

buffer = buff.CircularBuffer(2000)
# Configuração do tema Monokai
plt.style.use('cyberpunk')

# Configuração inicial do gráfico
plt.ion()  # Ativa o modo de interatividade
fig, ax = plt.subplots()  # Cria uma figura e um conjunto de eixos
# fig2, ax2 = plt.subplots()  # Cria uma figura e um conjunto de eixos

def main():
    # Criar as threads para as funções
    # serial_thread = Thread(target=readSerial.get_serial)
    # mediapipe_thread = Thread(target=cv2_mediapipe.run_mediapipe)
    graph_thread = Thread(target=graph.plot_graph(fig, ax, buffer))
    serial_thread = Thread(target=Serial.get_serial(buffer))

    # Iniciar as threads
    # serial_thread.start()
    # mediapipe_thread.start()
    graph_thread.start()
    serial_thread.start()

    # Aguardar o término das threads
    # serial_thread.join()
    # mediapipe_thread.join()
    # graph_thread.join()


if __name__ == '__main__':
    main()
