import time
from dynamixel_sdk import *

LOG_FILE = "registro_movimentos.txt"

class DynamixelMotor:
    def __init__(self, motor_id, port_handler, protocol=2.0):
        self.motor_id = motor_id
        self.protocol = protocol
        self.port_handler = port_handler
        self.packet_handler = PacketHandler(protocol)

        if protocol == 2.0:
            self.ADDR_TORQUE_ENABLE = 64
            self.ADDR_GOAL_POSITION = 116
            self.ADDR_PROFILE_VELOCITY = 112
            self.ADDR_PROFILE_ACCELERATION = 108
        else:  # protocolo 1.0 (RX-28)
            self.ADDR_TORQUE_ENABLE = 24
            self.ADDR_GOAL_POSITION = 30
            self.ADDR_PROFILE_VELOCITY = None
            self.ADDR_PROFILE_ACCELERATION = None

    def enable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 1)

    def disable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 0)

    def set_profile(self, acceleration, velocity):
        if self.protocol == 2.0:
            self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_PROFILE_ACCELERATION, acceleration)
            self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_PROFILE_VELOCITY, velocity)

    def set_goal_position(self, position):
        self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_GOAL_POSITION, int(position))


def reproduzir_movimentos():
    port_handler = PortHandler('COM3')

    if not port_handler.openPort():
        print("Erro ao abrir a porta.")
        return
    if not port_handler.setBaudRate(1000000):
        print("Erro ao configurar a baudrate.")
        return

    motores = {}

    try:
        with open(LOG_FILE, "r") as f:
            linhas = f.readlines()

        for linha in linhas:
            try:
                linha = linha.strip()
                if linha == "":
                    continue

                # Caso especial para MOTORES:6&7
                if "MOTORES:6&7" in linha:
                    partes = linha.split(";")
                    pos_6 = int(partes[3].split(":")[1])
                    pos_7 = int(partes[4].split(":")[1])

                    for motor_id, posicao in [(6, pos_6), (7, pos_7)]:
                        if motor_id not in motores:
                            motores[motor_id] = DynamixelMotor(motor_id, port_handler, protocol=1.0)
                            motores[motor_id].enable_torque()
                            motores[motor_id].set_profile(acceleration=30, velocity=90)

                        print(f"[Reproduzindo Espelhado] Motor {motor_id} -> Posição {posicao}")
                        motores[motor_id].set_goal_position(posicao)

                # Caso normal: MOTOR:X
                elif "MOTOR:" in linha:
                    partes = linha.split(";")
                    motor_id = int(partes[1].split(":")[1])
                    posicao = int(partes[2].split(":")[1])

                    protocolo = 1.0 if motor_id in [6, 7] else 2.0

                    if motor_id not in motores:
                        motores[motor_id] = DynamixelMotor(motor_id, port_handler, protocol=protocolo)
                        motores[motor_id].enable_torque()
                        motores[motor_id].set_profile(acceleration=30, velocity=90)

                    print(f"[Reproduzindo] Motor {motor_id} -> Posição {posicao}")
                    motores[motor_id].set_goal_position(posicao)

                # time.sleep(0.5)  # opcional entre comandos

            except Exception as e:
                print(f"Erro ao processar linha: {linha} -> {e}")

    finally:
        for motor in motores.values():
            motor.disable_torque()
        port_handler.closePort()
        print("✅ Porta fechada. Reprodução finalizada.")

if __name__ == "__main__":
    reproduzir_movimentos()
