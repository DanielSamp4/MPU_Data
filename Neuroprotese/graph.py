import matplotlib
matplotlib.use('TkAgg')  # Define o backend do Matplotlib como TkAgg
import matplotlib.pyplot as plt
import numpy as np
import time
import mplcyberpunk
import serial

def plot_graph( fig, ax, buffer):
    # Conectar ao dispositivo pela porta serial
    ser = serial.Serial('COM5', 115200)
    width_window = 0.0001

    # # Configuração do tema Monokai
    # plt.style.use('cyberpunk')

    # # Configuração inicial do gráfico
    # plt.ion()  # Ativa o modo de interatividade
    # fig, ax = plt.subplots()  # Cria uma figura e um conjunto de eixos
    # # fig2, ax2 = plt.subplots()  # Cria uma figura e um conjunto de eixos

    # Dados iniciais
    x = np.linspace(0, width_window, 100)  # Exemplo de dados x
    # y_axe_accel_x = np.zeros_like(x)  # Exemplo de dados y (inicialmente zeros)
    # y_axe_accel_y = np.zeros_like(x)  # Exemplo de dados y (inicialmente zeros)
    # y_axe_accel_z = np.zeros_like(x)  # Exemplo de dados y (inicialmente zeros)
    # y_axe_gyro_x = np.zeros_like(x)  # Exemplo de dados y (inicialmente zeros)
    # y_axe_gyro_y = np.zeros_like(x)  # Exemplo de dados y (inicialmente zeros)
    y_axe_gyro_z = np.zeros_like(x)  # Exemplo de dados y (inicialmente zeros)

    start_time = time.time()  # Tempo de início do programa
    # Criação do objeto de linha
    # line1, = ax.plot(x, y_axe_accel_x, label='ax')
    # line2, = ax.plot(x, y_axe_accel_y, label='ay')
    # line3, = ax.plot(x, y_axe_accel_z, label='az')

    # # line4, = ax2.plot(x, y_axe_gyro_x, label='Gx')
    # # line5, = ax2.plot(x, y_axe_gyro_y, label='Gy')
    # # line6, = ax2.plot(x, y_axe_gyro_z, label='Gz')
    # line4, = ax.plot(x, y_axe_gyro_x, label='Gx')
    # line5, = ax.plot(x, y_axe_gyro_y, label='Gy')
    line6, = ax.plot(x, y_axe_gyro_z, label='Gz')

    # Adicionar legendas
    legend = ax.legend(loc='upper left', bbox_to_anchor=(0, 1))
    # legend2 = ax2.legend(loc='upper left', bbox_to_anchor=(0, 1))

    # Definir a função para ler os valores do dispositivo
    # def read_values():
    #     # Ler os valores da porta serial
    #     data = ser.readline().decode('utf-8').rstrip()
    #     if data.startswith("a&"):
    #         # print(data)
    #         data1 = data.split('&')[1]
    #         # Separar os valores de aceleração e giroscópio
    #         accel_values, gyro_values = data1.split(',')
    #         # Converter os valores em números de ponto flutuante
    #         accel_x, accel_y, accel_z = map(float, accel_values.split(':'))
    #         gyro_x, gyro_y, gyro_z = map(float, gyro_values.split(':'))
    #         # Retornaros valores de aceleração e giroscópio
    #         return (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
    #     return (0, 0, 0, 0, 0, 0)
    # Tempo inicial
    start_time_2 = time.time()
    # Função para ler os valores da porta serial
    # def read_values():
    #     # try:
    #         # Ler os valores da porta serial
    #     data = ser.readline().decode('utf-8').rstrip()

    #     if data.startswith("AI"):
    #         data_coma = data.split("I,")[1]
    #         # Separar os valores de aceleração e giroscópio
    #         packet, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = map(float, data_coma.split(','))
    #         # Retornar os valores de aceleração e giroscópio
    #         return (packet, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z)

    #     # except (ValueError, IndexError):
    #     #     print("Error reading values from serial port:", data)

    #     # Return default values if there was an error or the data doesn't match the expected format
    #     return (0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    # # Variáveis para contar o número de leituras
    num_readings_last_second = 0

    # Atualização em tempo real
    while True:
        current_time = time.time() - start_time_2
        # Simula a obtenção de dados do MPU
        # Ler os valores do dispositivo
        # time_loop = time.time()
        # if ser.in_waiting > 0:
        #     accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = read_values()

        # while ser.in_waiting == 0:
        #     pass  # Aguarda até que novos dados estejam disponíveis na porta serial
        
        # packet,accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = read_values()
        
        if current_time <= 1:
            num_readings_last_second += 1
        else:
            print("Número de leituras no último segundo  {}".format( num_readings_last_second))
            num_readings_last_second = 1
            start_time_2 = time.time()
        # Atualiza os dados y com o novo valor
        # y_axe_accel_x[:-1] = y_axe_accel_x[1:]  # Desloca todos os valores para a esquerda
        # y_axe_accel_x[-1] = accel_x  # Insere o novo valor no final
        # line1.set_ydata(y_axe_accel_x)

        # # Atualiza os dados y com o novo valor
        # y_axe_accel_y[:-1] = y_axe_accel_y[1:]  # Desloca todos os valores para a esquerda
        # y_axe_accel_y[-1] = accel_y  # Insere o novo valor no final
        # line2.set_ydata(y_axe_accel_y)

        # # Atualiza os dados y com o novo valor
        # y_axe_accel_z[:-1] = y_axe_accel_z[1:]  # Desloca todos os valores para a esquerda
        # y_axe_accel_z[-1] = accel_z  # Insere o novo valor no final
        # line3.set_ydata(y_axe_accel_z)

        # # Atualiza os dados y com o novo valor
        # y_axe_gyro_x[:-1] = y_axe_gyro_x[1:]  # Desloca todos os valores para a esquerda
        # y_axe_gyro_x[-1] = gyro_x  # Insere o novo valor no final
        # line4.set_ydata(y_axe_gyro_x)

        # # Atualiza os dados y com o novo valor
        # y_axe_gyro_y[:-1] = y_axe_gyro_y[1:]  # Desloca todos os valores para a esquerda
        # y_axe_gyro_y[-1] = gyro_y  # Insere o novo valor no final
        # line5.set_ydata(y_axe_gyro_y)

        # Atualiza os dados y com o novo valor
        y_axe_gyro_z[:-1] = y_axe_gyro_z[1:]  # Desloca todos os valores para a esquerda
        y_axe_gyro_z[-1] = buffer.get_oldest()  # Insere o novo valor no final
        line6.set_ydata(y_axe_gyro_z)

        if time.time() - start_time > width_window:
            x[:-1] = x[1:]
            x[-1] = time.time() - start_time

        # Atualiza a linha com os novos dados
        if time.time() - start_time > width_window:
            # line1.set_xdata(x)
            # line2.set_xdata(x)
            # line3.set_xdata(x)
            # line4.set_xdata(x)
            # line5.set_xdata(x)
            line6.set_xdata(x)

        # Redesenha o gráfico
        ax.relim()  # Atualiza os limites dos eixos
        ax.autoscale_view()  # Ajusta a escala dos eixos
        # ax2.relim()  # Atualiza os limites dos eixos
        # ax2.autoscale_view()  # Ajusta a escala dos eixos
        fig.canvas.draw()  # Redesenha a figura principal
        # fig2.canvas.draw()  # Redesenha a figura principal

       

        # Atualiza a figura principal
        plt.pause(0.00001)
        # print("Tempo de loop")
        # print(time.time() - time_loop)
        # print("\n")
        # Verifica se o usuário quer encerrar o programa
        if plt.get_fignums() == []:
            ser.close()
            break

    # Finaliza a conexão serial
    ser.close()
