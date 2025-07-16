import pyrealsense2 as rs
import numpy as np
import cv2
from pupil_apriltags import Detector

# === Inicializar RealSense ===
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# === Detector de AprilTags ===
detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

print("[INFO] Pressione ESC para sair...")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        h, w, _ = color_image.shape
        terco_h = h // 3
        terco_w = w // 3

        # === Desenhar malha 3x3 ===
        # Linhas horizontais
        cv2.line(color_image, (0, terco_h), (w, terco_h), (255, 255, 255), 2)
        cv2.line(color_image, (0, 2 * terco_h), (w, 2 * terco_h), (255, 255, 255), 2)
        # Linhas verticais
        cv2.line(color_image, (terco_w, 0), (terco_w, h), (255, 255, 255), 2)
        cv2.line(color_image, (2 * terco_w, 0), (2 * terco_w, h), (255, 255, 255), 2)
        # Números dos campos
        font = cv2.FONT_HERSHEY_SIMPLEX
        for i in range(3):
            for j in range(3):
                campo = 3 * i + j + 1
                cx_txt = j * terco_w + terco_w // 2
                cy_txt = i * terco_h + terco_h // 2
                cv2.putText(color_image, str(campo), (cx_txt - 10, cy_txt + 10), font, 1, (0, 255, 255), 2)

        # === Detectar AprilTags ===
        tags = detector.detect(gray)

        for tag in tags:
            cx, cy = int(tag.center[0]), int(tag.center[1])

            # Posição 3D
            distancia = depth_frame.get_distance(cx, cy)
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            coord_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, [cx, cy], distancia)

            # Desenhar bounding box
            for idx in range(4):
                p1 = (int(tag.corners[idx][0]), int(tag.corners[idx][1]))
                p2 = (int(tag.corners[(idx + 1) % 4][0]), int(tag.corners[(idx + 1) % 4][1]))
                cv2.line(color_image, p1, p2, (0, 255, 0), 2)

            cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
            texto = f"ID:{tag.tag_id} Z:{distancia:.2f}m"
            cv2.putText(color_image, texto, (cx - 40, cy - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Determinar campo (1 a 9)
            coluna = cx // terco_w  # 0, 1, 2
            linha = cy // terco_h
            campo = 3 * linha + coluna + 1
            print(f"[TAG {tag.tag_id}] Posição 3D: X={coord_xyz[0]:.2f}, Y={coord_xyz[1]:.2f}, Z={coord_xyz[2]:.2f} — Campo: {campo}")

        # === Mostrar imagem com sobreposição ===
        cv2.imshow("Detecção de AprilTags com Malha 3x3", color_image)

        # ESC para sair
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
