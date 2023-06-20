import serial

def get_serial(self, buffer):
    ser = serial.Serial('COM5', 115200)
    
    while True:
        # time_loop = time.time()
        # Ler os valores da porta serial
        data = ser.readline().decode('utf-8').rstrip()

        if data.startswith("AI"):
            data_coma = data.split("I,")[1]
            # Separar os valores de aceleração e giroscópio
            packet, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = map(float, data_coma.split(','))
            # Retornar os valores de aceleração e giroscópio
            # return (packet, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z)
            buffer.push(gyro_z)
            # Retornaros valores de aceleração e giroscópio

            # return (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
        # return (0, 0, 0, 0, 0, 0)