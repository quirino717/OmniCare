# import serial
# import struct
# import threading
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from collections import deque

# # Configurar serial
# ser = serial.Serial('/dev/stm', 115200, timeout=1)

# # Buffers
# max_len = 500
# velocidade = deque(maxlen=max_len)
# erro = deque(maxlen=max_len)
# pwm = deque(maxlen=max_len)
# amostras = deque(maxlen=max_len)

# contador = 0

# def ler_dados():
#     global contador
#     while True:
#         try:
#             # Espera 16 bytes (4 * int32)
#             if ser.in_waiting >= 16:
#                 data = ser.read(16)
#                 v, e, dt, p = struct.unpack('<iiii', data)  # little-endian int32
#                 velocidade.append(v)
#                 erro.append(e)
#                 pwm.append(p)
#                 amostras.append(contador)
#                 contador += 1
#         except Exception as ex:
#             print("Erro na leitura:", ex)

# # Thread de leitura
# t = threading.Thread(target=ler_dados, daemon=True)
# t.start()

# # Gráfico
# def animar(i):
#     ax1.clear()
#     ax2.clear()
#     ax1.plot(amostras, velocidade, label="Velocidade")
#     ax1.plot(amostras, pwm, label="PWM")
#     ax2.plot(amostras, erro, label="Erro", color="red")
#     ax1.set_ylabel("Vel/PWM")
#     ax2.set_ylabel("Erro")
#     ax2.set_xlabel("Amostra")
#     ax1.legend()
#     ax2.legend()
#     ax1.grid(True)
#     ax2.grid(True)

# fig, (ax1, ax2) = plt.subplots(2, 1)
# ani = animation.FuncAnimation(fig, animar, interval=100)
# plt.tight_layout()

# # Envio de comando
# def entrada_usuario():
#     while True:
#         comando = input("Digite comando (char): ")
#         ser.write(comando.encode())

# te = threading.Thread(target=entrada_usuario, daemon=True)
# te.start()

# plt.show()


import serial
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Configurar serial
ser = serial.Serial('/dev/stm', 115200, timeout=1)

# Buffers
max_len = 500
velocidade = deque(maxlen=max_len)
erro = deque(maxlen=max_len)
pwm = deque(maxlen=max_len)
amostras = deque(maxlen=max_len)

contador = 0

def ler_dados():
    global contador
    while True:
        try:
            linha = ser.readline().decode().strip()
            partes = linha.split()
            if len(partes) == 4:
                v = int(partes[0])
                e = int(partes[1])
                dt = int(partes[2])  # ignorado no gráfico, mas você pode usar
                p = int(partes[3])
                velocidade.append(v)
                erro.append(e)
                pwm.append(p)
                amostras.append(contador)
                contador += 1
        except Exception as ex:
            print("Erro na leitura:", ex)

# Thread de leitura
t = threading.Thread(target=ler_dados, daemon=True)
t.start()

# Gráfico
def animar(i):
    ax1.clear()
    ax2.clear()
    ax1.plot(amostras, velocidade, label="Velocidade")
    ax1.plot(amostras, pwm, label="PWM")
    ax2.plot(amostras, erro, label="Erro", color="red")
    ax1.set_ylabel("Vel/PWM")
    ax2.set_ylabel("Erro")
    ax2.set_xlabel("Amostra")
    ax1.legend()
    ax2.legend()
    ax1.grid(True)
    ax2.grid(True)

fig, (ax1, ax2) = plt.subplots(2, 1)
ani = animation.FuncAnimation(fig, animar, interval=100)
plt.tight_layout()

# Envio de comando
def entrada_usuario():
    while True:
        comando = input("Digite comando (char): ")
        ser.write(comando.encode())

te = threading.Thread(target=entrada_usuario, daemon=True)
te.start()

plt.show()
