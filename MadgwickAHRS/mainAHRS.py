import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from MadgwickAHRS import MadgwickAHRS

# Função para ler os valores da porta serial
def read_values():
    # Ler os valores da porta serial
    data = ser.readline().decode('utf-8').rstrip()
    print(data);
    if data.startswith("a&"):
        data1 = data.split('&')[1]
        # Separar os valores de aceleração e giroscópio
        accel_values, gyro_values = data1.split(',')
        # Converter os valores em números de ponto flutuante
        accel_x, accel_y, accel_z = map(float, accel_values.split(':'))
        gyro_x, gyro_y, gyro_z = map(float, gyro_values.split(':'))
        # Retornar os valores de aceleração e giroscópio
        return (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
    return (0, 0, 0, 0, 0, 0)

# Configurar a porta serial
ser = serial.Serial('COM9', 115200)

# Criar lista vazia para armazenar os dados
# gyroscope_data = []
# accelerometer_data = []

# Tempo inicial
start_time = time.time()

def update_cube( q0, q1, q2, q3):
    fig.clear()
     # Normaliza os quaternions
    # norm = np.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    # q0 /= norm
    # q1 /= norm
    # q2 /= norm
    # q3 /= norm
    

    # Define os vértices do cubo
    vertices = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
    ])

    print( q0, q1, q2, q3)
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
q0 = 1
q1 = 0
q2 = 0
q3 = 0

mag = np.array([np.random.uniform(-0.0005, 0.0005),
               np.random.uniform(-0.0005, 0.0005),
               np.random.uniform(-0.0005, 0.0005)])

# Ler os valores do sensor em tempo real
while True:
    time_loop = time.time()
    current_time = time.time() - start_time
    while ser.in_waiting == 0:
            pass  # Aguarda até que novos dados estejam disponíveis na porta serial
    values = read_values()
    # accel_data = values[:3]
    # gyro_data = values[3:]
    # # print(accel_data)
    # # print(gyro_data)

    # # Verificar se os dados são válidos
    # # if accel_data != (0, 0, 0) and gyro_data != (0, 0, 0):
    # #     gyroscope_data.append(gyro_data)
    # #     accelerometer_data.append(accel_data)
    

    # gyro_data = np.array(gyro_data).reshape(-1, 3)    
    # accel_data = np.array(accel_data).reshape(-1, 3)

    # # Processar os dados através do algoritmo
    # AHRS = MadgwickAHRS(SamplePeriod=1/256, Beta=0.1)
    # quaternion = np.zeros((len(gyro_data), 4))

    # for i in range(len(gyro_data)):
    #     AHRS.Update(gyro_data[i] * (np.pi / 180), accel_data[i], mag)
    #     print(AHRS.Quaternion)

    #     quaternion[i] += AHRS.Quaternion
    #     print("quaternions")
    #     print(quaternion)
    #     # print(quaternion[0, 0])
    #     # Chama a função para atualizar o cubo com os novos quaternions
    #     # print("np quartenions")
    #     # print(type(quaternion[0, 0]))
    #     # print(quaternion[0, 1])
    #     # print(quaternion[0, 2])
    #     # print(quaternion[0, 3])
    # q0 = quaternion[0, 0].item()
    # q1 = quaternion[0, 1].item()
    # q2 = quaternion[0, 2].item()
    # q3 = quaternion[0, 3].item()
    # # q0 = np.random.uniform(0.96, 0.95)
    # # q1 = np.random.uniform(-0.0005, 0.0005)
    # # q2 = np.random.uniform(-0.0005, 0.0005)
    # # q3 = np.random.uniform(-0.0005, 0.0005)
    # # print(type(q0))

    # # update_cube(quaternion[0, 0], quaternion[0, 1], quaternion[0, 2], quaternion[0, 3])
    # update_cube(q0, q1, q2, q3)

    # Aguarda um intervalo de tempo
    # time.sleep(0.001)
    # print(time.time() - time_loop)
    
    # Verifica se o usuário quer encerrar o programa
    if plt.get_fignums() == []:
        ser.close()
        break
    # # Definir a duração da coleta dos dados
    # if current_time >= 3:
    #     break


# # Converter os dados para arrays numpy
# # gyroscope_data = np.array(gyroscope_data)
# gyroscope_data = np.array(gyroscope_data).reshape(-1, 3)

# # accelerometer_data = np.array(accelerometer_data)
# accelerometer_data = np.array(accelerometer_data).reshape(-1, 3)

# # Processar os dados através do algoritmo
# AHRS = MadgwickAHRS(SamplePeriod=1/256, Beta=0.1)
# quaternion = np.zeros((len(gyroscope_data), 4))
# # Mag = np.zeros(4)
# # Mag[0] = 0
# # Mag[1] = 0.2
# # Mag[2] = 0.3
# # Mag[3] = 0.4
# for i in range(len(gyroscope_data)):
#     AHRS.Update(gyroscope_data[i] * (np.pi / 180), accelerometer_data[i], [0.0003, 0.0003, 0.0003])
#     print(quaternion)
#     quaternion[i] = AHRS.Quaternion

# # Plotar os dados do giroscópio
# time_data = np.linspace(0, len(gyroscope_data) / 256, len(gyroscope_data))


# fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
# fig.suptitle('Sensor Data')

# for i in range(3):
#     axes[0].plot(time_data, gyroscope_data[:, i])
#     axes[1].plot(time_data, accelerometer_data[:, i])
#     axes[0].set_ylabel('Angular rate (deg/s)')
#     axes[1].set_ylabel('Acceleration (g)')
#     axes[1].set_xlabel('Time (s)')
#     axes[0].set_title('Gyroscope')
#     axes[1].set_title('Accelerometer')

# plt.tight_layout()
# plt.show()
