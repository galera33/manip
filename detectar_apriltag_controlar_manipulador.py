import pyrealsense2 as rs
import numpy as np
import cv2
import time

from pupil_apriltags import Detector
from dynamixel_sdk import PortHandler
from dynamixel_motor import DynamixelMotor

# === Inicialização da porta e motores ===
port_handler = PortHandler('COM3')
if not port_handler.openPort():
    print("❌ Erro ao abrir a porta serial.")
    exit()
if not port_handler.setBaudRate(1000000):
    print("❌ Erro ao configurar a baudrate.")
    exit()

# Inicializa motores 1, 2, 3 e 4
motores = {i: DynamixelMotor(i, port_handler) for i in [1, 2, 3, 4]}

# Ativa torque e configura velocidade/accel limitadas
for m in motores.values():
    m.enable_torque()
    m.set_profile(acceleration=10, velocity=20)

# === RealSense setup ===
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# === AprilTag detector ===
detector = Detector(families="tag36h11", nthreads=4)

# === Flag para parada de movimento após centralização ===
centralizado_flag = False

def seguir_em_x_y_z(coord_xyz, pixel_c, centro_bloco_5_px, motores,
                    tolerancia_px=1, tolerancia_z=0.03,
                    max_ajuste_x=10, max_ajuste_y=10, max_ajuste_z=20):
    cx, cy = pixel_c
    centro_x, centro_y = centro_bloco_5_px
    _, _, Z = coord_xyz

    comandos = {}

    # 1️⃣ Alinhamento em X com motor 1
    diff_x = cx - centro_x
    if abs(diff_x) > tolerancia_px:
        ajuste_x = int(diff_x * 3)
        atual = motores[1].get_present_position()
        ajuste_x = max(-max_ajuste_x, min(max_ajuste_x, ajuste_x))
        nova_pos = atual + ajuste_x
        comandos[1] = max(0, min(4095, nova_pos))
        return comandos  # Corrige X primeiro

    # 2️⃣ Alinhamento em Y com motor 4
    diff_y = cy - centro_y
    if abs(diff_y) > tolerancia_px:
        ajuste_y = int(diff_y * 5)
        atual = motores[4].get_present_position()
        ajuste_y = max(-max_ajuste_y, min(max_ajuste_y, ajuste_y))
        nova_pos = atual + ajuste_y
        comandos[4] = max(0, min(4095, nova_pos))
        return comandos  # Corrige Y depois

    # 3️⃣ Alinhamento em Z (profundidade) com motores 2 e 3
    if abs(Z - 0.4) > tolerancia_z:
        ajuste_z = int((Z - 0.4) * 120)
        for mid in [2, 3]:
            atual = motores[mid].get_present_position()
            ajuste = -ajuste_z
            ajuste = max(-max_ajuste_z, min(max_ajuste_z, ajuste))
            nova_pos = atual + ajuste
            comandos[mid] = max(0, min(4095, nova_pos))

    return comandos

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        # Desenha malha 3x3
        terco_h, terco_w = h // 3, w // 3
        for i in range(1, 3):
            cv2.line(color_image, (0, i * terco_h), (w, i * terco_h), (255, 255, 255), 1)
            cv2.line(color_image, (i * terco_w, 0), (i * terco_w, h), (255, 255, 255), 1)

        # Centro do bloco 5
        centro_bloco_5_x = terco_w * 1 + terco_w // 2
        centro_bloco_5_y = terco_h * 1 + terco_h // 2

        # Detecta tags
        tags = detector.detect(gray)
        for tag in tags:
            cx, cy = int(tag.center[0]), int(tag.center[1])
            distancia = depth_frame.get_distance(cx, cy)
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            coord_xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], distancia)

            campo = (cy // terco_h) * 3 + (cx // terco_w) + 1
            cv2.putText(color_image, f"Campo: {campo}", (cx - 30, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Determina parâmetros com base no campo
            if campo == 5:
                tolerancia_px = 1
                tolerancia_z = 0.02
                max_ajuste_x = 5
                max_ajuste_y = 5
                max_ajuste_z = 10
                suavizando = True
            else:
                tolerancia_px = 3
                tolerancia_z = 0.05
                max_ajuste_x = 15
                max_ajuste_y = 15
                max_ajuste_z = 25
                suavizando = False

            # Se já centralizado, só exibe mensagem e pula
            if centralizado_flag:
                cv2.putText(color_image, "✅ Centralizado!", (30, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                break

            comandos = seguir_em_x_y_z(
                coord_xyz, (cx, cy),
                (centro_bloco_5_x, centro_bloco_5_y),
                motores,
                tolerancia_px=tolerancia_px,
                tolerancia_z=tolerancia_z,
                max_ajuste_x=max_ajuste_x,
                max_ajuste_y=max_ajuste_y,
                max_ajuste_z=max_ajuste_z
            )

            for mid in sorted(comandos):
                motores[mid].set_goal_position(comandos[mid])

            # Checa se está centralizado
            if campo == 5:
                diff_x = abs(cx - centro_bloco_5_x)
                diff_y = abs(cy - centro_bloco_5_y)
                diff_z = abs(coord_xyz[2] - 0.4)

                if diff_x <= 1 and diff_y <= 1 and diff_z <= 0.02:
                    cv2.putText(color_image, "✅ Centralizado!", (30, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    print("✅ Alvo centralizado com sucesso!")
                    centralizado_flag = True  # === FLAG ATIVADA ===

            break  # Apenas uma tag por loop

        cv2.imshow("Seguindo Tag (X,Y,Z) baseado em malha", color_image)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC para sair
            break

finally:
    for m in motores.values():
        m.disable_torque()
    port_handler.closePort()
    pipeline.stop()
    cv2.destroyAllWindows()
