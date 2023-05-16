import graph_async
import cv2_mediapipe_async
import serial
from threading import Thread

# Conectar ao dispositivo pela porta serial
ser = serial.Serial('COM3', 115200)


async def read_values():
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').rstrip()
            if data.startswith("a&"):
                data1 = data.split('&')[1]
                accel_values, gyro_values = data1.split(',')
                accel_x, accel_y, accel_z = map(float, accel_values.split(':'))
                gyro_x, gyro_y, gyro_z = map(float, gyro_values.split(':'))
                return (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
        await asyncio.sleep(0.001)  # Aguarda um curto período antes de verificar novamente

async def main():
    # Criar as threads para as funções
    mediapipe_thread = Thread(target=cv2_mediapipe.run_mediapipe)
    graph_thread = Thread(target=graph.plot_graph)

    # Iniciar as threads
    mediapipe_thread.start()
    graph_thread.start()

    # Aguardar o término das threads
    mediapipe_thread.join()
    graph_thread.join()

    # Resto do código omitido

    while True:
        # Resto do código omitido

        if ser.in_waiting > 0:
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = await read_values()

        # Resto do código omitido

if __name__ == '__main__':
    asyncio.run(main())
