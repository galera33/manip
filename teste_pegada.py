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

# === MATRIZ DE TRANSFORMAÇÃO T15 (usando A1 e A2) ===
def calcular_T15(A1, A2):
    c1, s1 = math.cos(math.radians(A1)), math.sin(math.radians(A1))
    c2, s2 = math.cos(math.radians(A2)), math.sin(math.radians(A2))
    c12 = math.cos(math.radians(2 * A1 + 2 * A2))
    s12 = math.sin(math.radians(2 * A1 + 2 * A2))
    c22 = math.cos(math.radians(2 * A2))
    s22 = math.sin(math.radians(2 * A2))
    cA1p2A2 = math.cos(math.radians(A1 + 2 * A2))
    sA1p2A2 = math.sin(math.radians(A1 + 2 * A2))

    T15 = [
        [
            c2 * (c12 / 2 + c22 / 2) + s1 * s2,
            c2 * s1 - s2 * (c12 / 2 + c22 / 2),
            (s12 + s22) / 2,
            (219 * s12)/2500 + (219 * s22)/2500 + (c1 * (37*c2 + 82*c2**2 - 41))/200
        ],
        [
            c2*(s12/2 - s22/2) - c1*s2,
            -s2*(s12/2 - s22/2) - c1*c2,
            (c22 - c12)/2,
            (219*c22)/2500 - (219*c12)/2500 + (s1*(37*c2 + 82*c2**2 - 41))/200
        ],
        [
            c2 * sA1p2A2,
            -s2 * sA1p2A2,
            -cA1p2A2,
            (41*s22)/200 - (219*cA1p2A2)/1250 + (37*s2)/200 + 1963/20000
        ],
        [0, 0, 0, 1]
    ]
    return T15

# === MONITOR DE POSIÇÃO EM TEMPO REAL COM ÂNGULOS IMPRESSOS ===
def monitorar_posicao(motores):
    try:
        while True:
            for i in range(1, 6):
                ang_motor = motores[i - 1].get_present_position_deg()
                A[i] = ang_motor - offsets[i]

            T = calcular_T15(A[1], A[2])
            x, y, z = T[0][3], T[1][3], T[2][3]

            angulos_str = " | ".join([f"A{i}={A[i]:6.2f}°" for i in range(1, 6)])

            print(f"\r📍 Posição: X={x:.3f} m Y={y:.3f} m Z={z:.3f} m | Ângulos: {angulos_str}", end='', flush=True)
            time.sleep(0.1)  # atualiza a cada 100ms

    except KeyboardInterrupt:
        print("\n⏹️ Monitoramento encerrado pelo usuário.")

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
    # Para monitorar posição apenas, torque pode estar desativado
    # Se precisar ativar torque, descomente as linhas abaixo:
    # for m in motores.values():
    #     m.enable_torque()

    print("🔄 Iniciando monitoramento em tempo real (CTRL+C para sair)...\n")
    monitorar_posicao(motores)

    for m in motores.values():
        m.disable_torque()
    port_handler.closePort()
    print("✔️ Encerrado.")

if __name__ == "__main__":
    main()
