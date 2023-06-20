import serial
import time
import csv
import threading
from threading import Thread
from threading import Lock
import sys

    

# Criar um Lock para controlar o acesso ao arquivo CSV
csv_lock = Lock()
serial_lock = Lock()

# Função para ler os valores da porta serial
def read_values():
    try:
        # Ler os valores da porta serial
        data = ser.readline().decode('utf-8').rstrip()

        if data.startswith("AI"):
            data_coma = data.split("I,")[1]
            # Separar os valores de aceleração e giroscópio
            packet, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = map(float, data_coma.split(','))
            # Retornar os valores de aceleração e giroscópio
            return (packet, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z)

    except (ValueError, IndexError):
        print("Error reading values from serial port:", data)

    # Return default values if there was an error or the data doesn't match the expected format
    return (0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


# Configurar a porta serial
ser = serial.Serial('COM5', 115200)



# Abrir arquivo CSV para escrita
csvfile = open('dados_sensor.csv', 'w', newline='')
fieldnames = ['Packet number', 'Gyroscope X (deg/s)', 'Gyroscope Y (deg/s)', 'Gyroscope Z (deg/s)', 'Accelerometer X (g)', 'Accelerometer Y (g)', 'Accelerometer Z (g)', 'Magnetometer X (G)', 'Magnetometer Y (G)', 'Magnetometer Z (G)']
writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
writer.writeheader()
    
def getValue_WriteCSV():
    # Tempo inicial
    start_time = time.time()

    # Variáveis para contar o número de leituras
    num_readings = 0
    num_readings_last_second = 0
    # Ler os valores do sensor em tempo real
    try:
        while True:
            # time_loop = time.time()
            current_time = time.time() - start_time
            if ser.in_waiting > 0:
                # pass  # Aguarda até que novos dados estejam disponíveis na porta serial
                with serial_lock:
                    packet, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = read_values()
                
                # Contar o número de leituras
                # num_readings += 1
                if current_time <= 1:
                    num_readings_last_second += 1
                else:
                    print("Número de leituras no último segundo (Thread {}): {}".format(threading.current_thread().name, num_readings_last_second))
                    num_readings_last_second = 1
                    start_time = time.time()
                # Bloquear o acesso ao arquivo CSV
                with csv_lock:
                    # Escrever os valores no arquivo CSV
                    writer.writerow({'Packet number': int(packet), 'Gyroscope X (deg/s)': gyro_x, 'Gyroscope Y (deg/s)': gyro_y, 'Gyroscope Z (deg/s)': gyro_z,
                                    'Accelerometer X (g)': accel_x, 'Accelerometer Y (g)': accel_y, 'Accelerometer Z (g)': accel_z,
                                    'Magnetometer X (G)': mag_x, 'Magnetometer Y (G)': mag_y, 'Magnetometer Z (G)': mag_z})

    except KeyboardInterrupt:
        print("Script encerrado.")
        sys.exit(0)
            # Exibir os valores lidos
            # print("Packet:", packet)
            # print("Aceleração (X, Y, Z):", accel_x, accel_y, accel_z)
            # print("Giroscópio (X, Y, Z):", gyro_x, gyro_y, gyro_z)
            # print("Magnetômetro (X, Y, Z):", mag_x, mag_y, mag_z)
            
            # Aguardar 1 segundo entre as leituras
            # time.sleep(0.0078)


def main(): 
    # Criar as threads para as funções
    # serial_thread = Thread(target=readSerial.get_serial)
    Thread1 = Thread(target=getValue_WriteCSV)
    # Thread2 = Thread(target=getValue_WriteCSV)
    # Thread2 = Thread(target=getValue_WriteCSV)
    # Thread3 = Thread(target=getValue_WriteCSV)
    # Thread4 = Thread(target=getValue_WriteCSV)
    # Thread5 = Thread(target=getValue_WriteCSV)
    # Thread6 = Thread(target=getValue_WriteCSV)

    # Iniciar as threads
    # serial_thread.start()
    # mediapipe_thread.start()
    Thread1.start()
    # Thread2.start()
    # Thread2.start()
    # Thread3.start()
    # Thread4.start()
    # Thread5.start()
    # Thread6.start()
    
    # Aguardar o término das threads
    # serial_thread.join()
    # mediapipe_thread.join()
    # Thread1.join()
    # Thread2.join()
    # Thread3.join()
    # Thread4.join()
    # Thread5.join()
    # Thread6.join()


if __name__ == '__main__':
    main()
