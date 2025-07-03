import time
import keyboard
from datetime import datetime
from dynamixel_sdk import *

LOG_FILE = "registro_movimentos.txt"

class DynamixelMotor:
    def __init__(self, motor_id, port_handler, protocol=2.0):
        self.motor_id = motor_id
        self.port_handler = port_handler
        self.protocol = protocol
        self.packet_handler = PacketHandler(protocol)

        if protocol == 2.0:
            self.ADDR_TORQUE_ENABLE = 64
            self.ADDR_GOAL_POSITION = 116
            self.ADDR_PRESENT_POSITION = 132
            self.ADDR_PROFILE_VELOCITY = 112
            self.ADDR_PROFILE_ACCELERATION = 108
        else:
            self.ADDR_TORQUE_ENABLE = 24
            self.ADDR_GOAL_POSITION = 30
            self.ADDR_PRESENT_POSITION = 36
            self.ADDR_PROFILE_VELOCITY = None
            self.ADDR_PROFILE_ACCELERATION = None

        self.goal_position = self.get_present_position() or 0

    def enable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 1)

    def disable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 0)

    def set_profile(self, acceleration, velocity):
        if self.protocol == 2.0:
            self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_PROFILE_ACCELERATION, acceleration)
            self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_PROFILE_VELOCITY, velocity)

    def set_goal_position(self, position):
        self.goal_position = position
        self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_GOAL_POSITION, int(position))
        self.log_movimento(position)

    def get_present_position(self):
        position, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_PRESENT_POSITION)
        return position

    def log_movimento(self, position):
        with open(LOG_FILE, "a") as f:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            f.write(f"{timestamp};MOTOR:{self.motor_id};POSICAO:{position}\n")


class DynamixelMotorDuplo:
    def __init__(self, motor_a: DynamixelMotor, motor_b: DynamixelMotor):
        self.motor_a = motor_a  # Motor 6
        self.motor_b = motor_b  # Motor 7
        self.goal_position_a = motor_a.get_present_position() or 0
        self.goal_position_b = motor_b.get_present_position() or 0

    def enable_torque(self):
        self.motor_a.enable_torque()
        self.motor_b.enable_torque()

    def disable_torque(self):
        self.motor_a.disable_torque()
        self.motor_b.disable_torque()

    def set_profile(self, acceleration, velocity):
        self.motor_a.set_profile(acceleration, velocity)
        self.motor_b.set_profile(acceleration, velocity)

    def set_goal_position(self, delta):
        self.goal_position_a += delta
        self.goal_position_b -= delta
        self.motor_a.set_goal_position(self.goal_position_a)
        self.motor_b.set_goal_position(self.goal_position_b)
        self.log_movimento(delta)

    def log_movimento(self, delta):
        with open(LOG_FILE, "a") as f:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            f.write(f"{timestamp};MOTORES:6&7;DELTA:{delta};POS_6:{self.goal_position_a};POS_7:{self.goal_position_b}\n")


def controle_motor_individual(motor_id, motores):
    incremento = 5
    motor = motores[motor_id]
    print(f"\n➡️ Controlando Motor {motor_id} (↑ aumenta, ↓ diminui, ESC volta ao menu)")

    try:
        while True:
            if keyboard.is_pressed('up'):
                if isinstance(motor, DynamixelMotorDuplo):
                    motor.set_goal_position(incremento)
                    print(f"[Motor 6↑ | Motor 7↓] Pos6: {motor.goal_position_a} | Pos7: {motor.goal_position_b}")
                else:
                    motor.goal_position += incremento
                    motor.set_goal_position(motor.goal_position)
                    print(f"[Motor {motor_id}] ↑ Posição: {motor.goal_position}")
                #time.sleep(0.1)

            if keyboard.is_pressed('down'):
                if isinstance(motor, DynamixelMotorDuplo):
                    motor.set_goal_position(-incremento)
                    print(f"[Motor 6↓ | Motor 7↑] Pos6: {motor.goal_position_a} | Pos7: {motor.goal_position_b}")
                else:
                    motor.goal_position -= incremento
                    motor.set_goal_position(motor.goal_position)
                    print(f"[Motor {motor_id}] ↓ Posição: {motor.goal_position}")
                #time.sleep(0.1)

            if keyboard.is_pressed('esc'):
                print("🔙 Voltando ao menu...")
                break

            #time.sleep(0.05)
    except KeyboardInterrupt:
        print("Encerrando controle...")


def main():
    port_handler = PortHandler('COM3')

    if not port_handler.openPort():
        print("❌ Erro ao abrir a porta.")
        return
    if not port_handler.setBaudRate(1000000):
        print("❌ Erro ao configurar a baudrate.")
        return

    # Criação de motores individuais
    motores_individuais = {
        1: DynamixelMotor(1, port_handler),
        2: DynamixelMotor(2, port_handler),
        3: DynamixelMotor(3, port_handler),
        4: DynamixelMotor(4, port_handler),
        5: DynamixelMotor(10, port_handler),
        6: DynamixelMotor(6, port_handler, protocol=1.0),
        7: DynamixelMotor(7, port_handler, protocol=1.0),
    }

    # Motor combinado espelhado
    motores = motores_individuais.copy()
    motores[8] = DynamixelMotorDuplo(motores_individuais[6], motores_individuais[7])

    # Ativa torque e define perfil
    for motor in motores.values():
        motor.enable_torque()
        motor.set_profile(acceleration=30, velocity=60)

    # Limpa o log no início
    open(LOG_FILE, "w").close()

    try:
        while True:
            print("\n=== Controle de Motores Dynamixel ===")
            print("Escolha um motor para controlar:")
            for i in range(1, 8):
                print(f"{i} - Motor {i}")
            print("8 - Motor 6 e 7 (Espelhados)")
            print("0 - Sair")
            opcao = input("Digite o número do motor: ")

            if opcao == '0':
                print("Encerrando...")
                break

            try:
                motor_id = int(opcao)
                if motor_id in motores:
                    controle_motor_individual(motor_id, motores)
                else:
                    print("⚠️ Motor inválido.")
            except ValueError:
                print("⚠️ Entrada inválida.")

    finally:
        for motor in motores.values():
            motor.disable_torque()
        port_handler.closePort()
        print("✅ Porta serial fechada.")

if __name__ == "__main__":
    main()
