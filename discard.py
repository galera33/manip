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
CAMINHO_TAGS = "retira.json"

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

def carregar_estado_containers():
    if not os.path.exists(CAMINHO_JSON):
        raise FileNotFoundError("Arquivo containers.json não encontrado.")
    with open(CAMINHO_JSON, 'r') as f:
        return json.load(f)

def salvar_estado_containers(estado):
    with open(CAMINHO_JSON, 'w') as f:
        json.dump(estado, f, indent=4)

def carregar_tags_para_retirar():
    if not os.path.exists(CAMINHO_TAGS):
        raise FileNotFoundError("Arquivo tags.json não encontrado.")
    with open(CAMINHO_TAGS, 'r') as f:
        return json.load(f)["tags"]


def aguardar_movimento(motor, posicao_alvo, tolerancia=20, timeout=10):
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

def retirar_objeto(motores, id_alvo):
    estado = carregar_estado_containers()
    destino = next((nome for nome, dados in estado.items() if dados["id"] == id_alvo), None)

    if destino is None:
        print(f"❌ ID {id_alvo} não encontrado em nenhum container.")
        return

    print(f"📤 Retirando objeto do container {destino} com ID {id_alvo}...")

    for movimento in enderecos_containers[destino]:
        motor_id = movimento["motor"]
        pos = movimento["pos"]
        print(f"🔄 Movendo motor {motor_id} para posição {pos}")
        motores[motor_id].set_goal_position(pos)
        aguardar_movimento(motores[motor_id], pos)

    time.sleep(0.5)
    controlar_garra(motores[6], motores[7], -600)  # Abrir garra
    time.sleep(0.5)

    estado[destino]["ocupado"] = False
    estado[destino]["id"] = None
    salvar_estado_containers(estado)
    print(f"✅ Objeto com ID {id_alvo} retirado do container {destino}.")

    voltar_posicao_inicial(motores)
    controlar_garra(motores[6], motores[7], 600)
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
        m.set_profile(10, 20)
    print("✅ Torque habilitado.")

    tags = carregar_tags_para_retirar()
    print(f"📜 Lista de tags a retirar: {tags}")
    for tag_id in tags:
        retirar_objeto(motores, tag_id)

    port_handler.closePort()
    print("🔌 Porta serial encerrada.")

if __name__ == "__main__":
    funfa()
