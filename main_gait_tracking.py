import serial
import time
import csv

# Função para ler os valores da porta serial
def read_values():
    # Ler os valores da porta serial
    data = ser.readline().decode('utf-8').rstrip()
    # print(data)
    if data.startswith("AI"):
        data_coma = data.split("I,")[1]
        # print(data_coma)
        # Separar os valores de aceleração e giroscópio
        packet, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = map(float, data_coma.split(','))
        # Retornar os valores de aceleração e giroscópio
        return (packet, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z)
    return (0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

# Configurar a porta serial
ser = serial.Serial('COM9')

# Tempo inicial
start_time = time.time()

# Variáveis para contar o número de leituras
num_readings = 0
num_readings_last_second = 0

# Abrir arquivo CSV para escrita
with open('dados_sensor.csv', 'w', newline='') as csvfile:
    fieldnames = ['Packet number', 'Gyroscope X (deg/s)', 'Gyroscope Y (deg/s)', 'Gyroscope Z (deg/s)', 'Accelerometer X (g)', 'Accelerometer Y (g)', 'Accelerometer Z (g)', 'Magnetometer X (G)', 'Magnetometer Y (G)', 'Magnetometer Z (G)']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    
    # Escrever cabeçalho no arquivo CSV
    writer.writeheader()
    
    # Ler os valores do sensor em tempo real
    while True:
        # time_loop = time.time()
        current_time = time.time() - start_time
        if ser.in_waiting > 0:
            # pass  # Aguarda até que novos dados estejam disponíveis na porta serial
            packet, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = read_values()
            
            # Contar o número de leituras
            # num_readings += 1
            if current_time <= 1:
                num_readings_last_second += 1
            else:
                print("Número de leituras no último segundo:", num_readings_last_second)
                num_readings_last_second = 1
                start_time = time.time()
            
            # Escrever os valores no arquivo CSV
            writer.writerow({'Packet number': int(packet), 'Gyroscope X (deg/s)': gyro_x, 'Gyroscope Y (deg/s)': gyro_y, 'Gyroscope Z (deg/s)': gyro_z,
                            'Accelerometer X (g)': accel_x, 'Accelerometer Y (g)': accel_y, 'Accelerometer Z (g)': accel_z,
                            'Magnetometer X (G)': mag_x, 'Magnetometer Y (G)': mag_y, 'Magnetometer Z (G)': mag_z})

            
            # Exibir os valores lidos
            # print("Packet:", packet)
            # print("Aceleração (X, Y, Z):", accel_x, accel_y, accel_z)
            # print("Giroscópio (X, Y, Z):", gyro_x, gyro_y, gyro_z)
            # print("Magnetômetro (X, Y, Z):", mag_x, mag_y, mag_z)
            
            # Aguardar 1 segundo entre as leituras
            # time.sleep(0.0078)
