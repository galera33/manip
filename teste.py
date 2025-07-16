import math
import time
from dynamixel_sdk import PortHandler
from dynamixel_motor import DynamixelMotor

# === CONFIGURAÇÃO SERIAL ===
DEVICENAME = 'COM3'
BAUDRATE = 1000000

# === OFFSETS (zero DH → motor) ===
offsets = {
    1: 270,
    2: 94,
    3: 188,
    4: 73,
    5: 180
}

# === Ângulos DH atuais ===
A = {i: 0.0 for i in range(1, 6)}

# === CONVERSÃO TICKS <-> GRAUS ===
def ticks_para_graus(ticks):
    return (ticks % 4096) * 360.0 / 4096.0

def calcular_T15(A1, A2, A3, A4, A5):
    c1 = math.cos(math.radians(A1))
    s1 = math.sin(math.radians(A1))
    c5 = math.cos(math.radians(A5))
    s5 = math.sin(math.radians(A5))

    soma234 = A2 + A3 + A4
    c234 = math.cos(math.radians(soma234))
    s234 = math.sin(math.radians(soma234))

    c23 = math.cos(math.radians(A2 + A3))
    s23 = math.sin(math.radians(A2 + A3))

    c2 = math.cos(math.radians(A2))
    s2 = math.sin(math.radians(A2))

    T15 = [
        [
            s1 * s5 + c234 * c1 * c5,
            c5 * s1 - c234 * c1 * s5,
            s234 * c1,
            (c1 * (41 * c23 + 37 * c2)) / 200 + (219 * s234 * c1) / 1250
        ],
        [
            c234 * c5 * s1 - c1 * s5,
            - c1 * c5 - c234 * s1 * s5,
            s234 * s1,
            (s1 * (41 * c23 + 37 * c2)) / 200 + (219 * s234 * s1) / 1250
        ],
        [
            s234 * c5,
            - s234 * s5,
            - c234,
            (41 * s23) / 200 - (219 * c234) / 1250 + (37 * s2) / 200 + 1963 / 20000
        ],
        [0, 0, 0, 1]
    ]
    return T15

def monitorar_posicao(motores):
    print("Monitorando posição. Pressione CTRL+C para sair e desligar torque...")
    try:
        while True:
            for i in range(1, 6):
                ang_motor = motores[i - 1].get_present_position_deg()
                A[i] = ang_motor - offsets[i]

            T = calcular_T15(A[1], A[2], A[3], A[4], A[5])
            x, y, z = T[0][3], T[1][3], T[2][3]

            angulos_str = " | ".join([f"A{i}={A[i]:6.2f}°" for i in range(1, 6)])

            print(f"\r📍 Posição: X={x:.3f} m Y={y:.3f} m Z={z:.3f} m | Ângulos: {angulos_str}", end='', flush=True)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n⏹️ Monitoramento interrompido pelo usuário.")

# === PROGRAMA PRINCIPAL ===
def main():
    port_handler = PortHandler(DEVICENAME)
    if not port_handler.openPort():
        print("❌ Erro ao abrir a porta serial.")
        return
    if not port_handler.setBaudRate(BAUDRATE):
        print("❌ Erro ao configurar a taxa de transmissão.")
        return

    motores = {i: DynamixelMotor(i + 1, port_handler) for i in range(5)}

    # Se quiser, pode ativar torque
    # for m in motores.values():
    #     m.enable_torque()

    monitorar_posicao(motores)

    print("🔴 Desligando torque dos motores...")
    for m in motores.values():
        m.disable_torque()

    port_handler.closePort()
    print("✔️ Torque desligado e porta serial fechada. Programa encerrado.")

if __name__ == "__main__":
    main()
