import json
import os
import time
import pyrealsense2 as rs
import numpy as np
import cv2
from pupil_apriltags import Detector
from dynamixel_sdk import (
    PortHandler, GroupSyncWrite,
    DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD, COMM_SUCCESS
)
from dynamixel_motor import DynamixelMotor

CAMINHO_JSON = "containers.json"

def carregar_estado_containers():
    if not os.path.exists(CAMINHO_JSON):
        estado_inicial = {
            "A": {"ocupado": False, "id": None},
            "B": {"ocupado": False, "id": None},
            "C": {"ocupado": False, "id": None}
        }
        with open(CAMINHO_JSON, 'w') as f:
            json.dump(estado_inicial, f, indent=4)
        return estado_inicial
    with open(CAMINHO_JSON, 'r') as f:
        return json.load(f)

def salvar_estado_containers(estado):
    with open(CAMINHO_JSON, 'w') as f:
        json.dump(estado, f, indent=4)

def resetar_containers():
    salvar_estado_containers({
        "A": {"ocupado": False, "id": None},
        "B": {"ocupado": False, "id": None},
        "C": {"ocupado": False, "id": None}
    })
    print("🧹 Containers resetados.")

enderecos_containers = {
    "A": [
        {"motor": 2, "pos": 1800},
        {"motor": 3, "pos": 3250},
        {"motor": 1, "pos": 2790},
        {"motor": 4, "pos": 2230},
        {"motor": 2, "pos": 2100}
    ],
    "B": [
        {"motor": 2, "pos": 1800},
        {"motor": 3, "pos": 3000},
        {"motor": 1, "pos": 3060},
        {"motor": 4, "pos": 2157},
        {"motor": 2, "pos": 2280}
    ],
    "C": [
        {"motor": 2, "pos": 1800},
        {"motor": 3, "pos": 3340},
        {"motor": 1, "pos": 3380},
        {"motor": 4, "pos": 2150},
        {"motor": 2, "pos": 2000}
    ]
}

POSICAO_INICIAL = {
    1: 1055,
    2: 1800,
    3: 3400,
    4: 2100
}

def aguardar_movimento(motor, posicao_alvo, tolerancia=20, timeout=5.0):
    inicio = time.time()
    while time.time() - inicio < timeout:
        pos_atual = motor.get_present_position()
        if abs(pos_atual - posicao_alvo) <= tolerancia:
            print(f"✅ Motor {motor.motor_id} chegou em {pos_atual}")
            return True
        time.sleep(0.05)
    print(f"⚠️ Timeout motor {motor.motor_id}. Última posição: {pos_atual}")
    return False

def controlar_garra(m6, m7, delta_ticks):
    pos6 = m6.get_present_position()
    pos7 = m7.get_present_position()

    nova_pos6 = pos6 + delta_ticks
    nova_pos7 = pos7 - delta_ticks

    print(f"🔧 Garra SINCRONIZADA → M6: {pos6} ➜ {nova_pos6} | M7: {pos7} ➜ {nova_pos7}")

    m6.set_profile(10, 40)
    m7.set_profile(10, 40)

    group_sync_write = GroupSyncWrite(m6.port_handler, m6.packet_handler, m6.ADDR_GOAL_POSITION, 4)

    param_m6 = [DXL_LOBYTE(DXL_LOWORD(nova_pos6)), DXL_HIBYTE(DXL_LOWORD(nova_pos6)),
                DXL_LOBYTE(DXL_HIWORD(nova_pos6)), DXL_HIBYTE(DXL_HIWORD(nova_pos6))]

    param_m7 = [DXL_LOBYTE(DXL_LOWORD(nova_pos7)), DXL_HIBYTE(DXL_LOWORD(nova_pos7)),
                DXL_LOBYTE(DXL_HIWORD(nova_pos7)), DXL_HIBYTE(DXL_HIWORD(nova_pos7))]

    group_sync_write.addParam(m6.motor_id, bytes(param_m6))
    group_sync_write.addParam(m7.motor_id, bytes(param_m7))

    result = group_sync_write.txPacket()
    if result != COMM_SUCCESS:
        print(f"❌ Erro ao enviar comando SyncWrite: {m6.packet_handler.getTxRxResult(result)}")

    group_sync_write.clearParam()

    aguardar_movimento(m6, nova_pos6)
    aguardar_movimento(m7, nova_pos7)

def voltar_posicao_inicial(motores):
    print("🔙 Retornando para posição inicial na ordem 2 → 3 → 4 → 1...")
    for motor_id in [2, 3, 4, 1]:
        pos = POSICAO_INICIAL[motor_id]
        print(f"🔄 Retornando motor {motor_id} para posição {pos}")
        motores[motor_id].set_goal_position(pos)
        aguardar_movimento(motores[motor_id], pos)

def armazenar_objeto(motores):
    estado = carregar_estado_containers()
    destino = next((nome for nome, dados in estado.items() if not dados["ocupado"]), None)

    if destino is None:
        print("❌ Todos os containers estão ocupados.")
        return

    print(f"📦 Armazenando objeto no container {destino}...")

    # === Inicia câmera para capturar ID da AprilTag ===
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    detector = Detector(families="tag36h11", nthreads=4)

    tag_id = None
    try:
        print("🔍 Procurando AprilTag...")
        for tentativa in range(30):  # tenta por 30 frames (~3 segundos)
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray)
            if tags:
                tag = tags[0]
                if hasattr(tag, 'id'):
                    tag_id = int(tag.id)
                    print(f"✅ AprilTag detectada na tentativa {tentativa+1}: ID = {tag_id}")
                    break
                else:
                    print("⚠️ Tag detectada mas sem atributo 'id'.")
            time.sleep(0.1)
        else:
            print("⚠️ Nenhuma AprilTag detectada. Armazenando com ID nulo.")
    finally:
        pipeline.stop()

    # === Movimentos até o container ===
    for movimento in enderecos_containers[destino]:
        motor_id = movimento["motor"]
        pos = movimento["pos"]
        print(f"🔄 Movendo motor {motor_id} para posição {pos}")
        motores[motor_id].set_goal_position(pos)
        aguardar_movimento(motores[motor_id], pos)

    time.sleep(0.5)
    controlar_garra(motores[6], motores[7], 600)
    time.sleep(0.5)

    # === Atualiza o JSON ===
    estado[destino]["ocupado"] = True
    estado[destino]["id"] = tag_id
    salvar_estado_containers(estado)
    print(f"✅ Objeto com ID {tag_id} armazenado no container {destino}.")

    voltar_posicao_inicial(motores)
    print("🏁 Ciclo concluído.\n")

def funfa():
    port_handler = PortHandler('COM3')
    if not port_handler.openPort():
        print("❌ Não foi possível abrir a porta COM3.")
        return
    if not port_handler.setBaudRate(1000000):
        print("❌ Não foi possível configurar a taxa de baud.")
        return
    print("✅ Porta COM3 aberta com sucesso.")

    motores = {i: DynamixelMotor(i, port_handler) for i in range(1, 8)}
    for m in motores.values():
        m.enable_torque()
    print("✅ Torque habilitado.")

    armazenar_objeto(motores)

    port_handler.closePort()
    print("🔌 Porta serial encerrada.")
    

if __name__ == "__main__":
    funfa()
    # resetar_containers()
