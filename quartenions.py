import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
import serial
import time

ser = serial.Serial('COM5', 115200)

def read_values():
        # Ler os valores da porta serial
        data = ser.readline().decode('utf-8').rstrip()
        if data.startswith("a&"):
            
            # print("Data serial: %s",data)
            data1 = data.split('&')[1]
            # Separar os valores de aceleração e giroscópio
            # accel_values, gyro_values = data1.split(',')
            # Converter os valores em números de ponto flutuante
            q0, q1, q2, q3 = map(float, data1.split(','))
            # gyro_x, gyro_y, gyro_z = map(float, gyro_values.split(':'))
            # Retornaros valores de aceleração e giroscópio
            return (q0, q1, q2, q3)
        return (0, 0, 0, 0,)

def update_cube( q0, q1, q2, q3):
    fig.clear()
     # Normaliza os quaternions
    norm = np.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    q0 /= norm
    q1 /= norm
    q2 /= norm
    q3 /= norm
    

    # Define os vértices do cubo
    vertices = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
    ])

    print("data insert -> q0:%f q1:%f q2:%f q3:%f", q0, q1, q2, q3)
    # Aplica a rotação nos vértices usando os quaternions
    R = np.array([
        [1-2*q2*q2-2*q3*q3, 2*q1*q2-2*q3*q0, 2*q1*q3+2*q2*q0],
        [2*q1*q2+2*q3*q0, 1-2*q1*q1-2*q3*q3, 2*q2*q3-2*q1*q0],
        [2*q1*q3-2*q2*q0, 2*q2*q3+2*q1*q0, 1-2*q1*q1-2*q2*q2]
    ])
    rotated_vertices = np.dot(vertices, R)

    # Plota o cubo
    ax = fig.add_subplot(111, projection='3d')

    # Desenha as arestas do cubo
    edges = [
        [0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6],
        [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]
    ]
    for edge in edges:
        ax.plot3D(
            [rotated_vertices[edge[0], 0], rotated_vertices[edge[1], 0]],
            [rotated_vertices[edge[0], 1], rotated_vertices[edge[1], 1]],
            [rotated_vertices[edge[0], 2], rotated_vertices[edge[1], 2]],
            'r'
        )

    # Define os limites do gráfico
    ax.set_xlim([-2, 2])    
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Atualiza o gráfico
    plt.draw()
    plt.pause(0.000001)

   

# Cria a figura
fig = plt.figure()

# Valores iniciais de exemplo para os quaternions
q0 = 0.5
q1 = 0.3
q2 = 0.2
q3 = 0.1

# Chama a função de atualização do cubo em loop
while True:
    time_loop = time.time()
    # if ser.in_waiting > 0:
    while ser.in_waiting == 0:
            pass  # Aguarda até que novos dados estejam disponíveis na porta serial
        
    q0_now, q1_now, q2_now, q3_now = read_values()
# Gera valores aleatórios para os quaternions
    q0 += q0_now
    q1 += q1_now
    q2 += q2_now
    q3 += q3_now
    # q0 += np.random.uniform(-0.005, 0.005)
    # q1 += np.random.uniform(-0.005, 0.005)
    # q2 += np.random.uniform(-0.005, 0.005)
    # q3 += np.random.uniform(-0.005, 0.005)

    # Chama a função para atualizar o cubo com os novos quaternions
    update_cube(q0, q1, q2, q3)

    # Aguarda um intervalo de tempo
    time.sleep(0.001)
    print(time.time() - time_loop)
    
    # Verifica se o usuário quer encerrar o programa
    if plt.get_fignums() == []:
        # ser.close()
        break
# Finaliza a conexão serial
# ser.close()