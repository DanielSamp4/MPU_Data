import serial

def get_serial():
    ser = serial.Serial('COM3', 115200)
    
    while True:
        # time_loop = time.time()
        # Ler os valores da porta serial
        data = ser.readline().decode('utf-8').rstrip()
        if data.startswith("a&"):
            print(data)
            data1 = data.split('&')[1]
            # Separar os valores de aceleração e giroscópio
            accel_values, gyro_values = data1.split(',')
            # Converter os valores em números de ponto flutuante
            accel_x, accel_y, accel_z = map(float, accel_values.split(':'))
            gyro_x, gyro_y, gyro_z = map(float, gyro_values.split(':'))
            # Retornaros valores de aceleração e giroscópio
            return (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
        return (0, 0, 0, 0, 0, 0)