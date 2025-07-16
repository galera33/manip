from dynamixel_sdk import PortHandler
from dynamixel_motor import DynamixelMotor
from manipulador import Manipulador5DOF
from sequencia import executar_sequencia


if __name__ == "__main__":
    port_handler = PortHandler('COM3')

    if not port_handler.openPort():
        print("❌ Erro ao abrir a porta.")
        exit()
    if not port_handler.setBaudRate(1000000):
        print("❌ Erro ao configurar a baudrate.")
        exit()

    motores = {
        1: DynamixelMotor(1, port_handler),
        2: DynamixelMotor(2, port_handler),
        3: DynamixelMotor(3, port_handler),
        4: DynamixelMotor(4, port_handler),
        5: DynamixelMotor(5, port_handler),
    }

    for motor in motores.values():
        motor.enable_torque()
        motor.set_profile(acceleration=5, velocity=10)
        motor.goal_position = motor.get_present_position()

    manipulador = Manipulador5DOF()

    try:
        while True:
            entrada = input("\n🎯 Digite sequências de posições (bits) separadas por linha ou 'sair':\n"
                            "Ex: 994,1854,1259,1990,2137\n     1024,2048,2048,2048,2048\n>>> ")

            if entrada.strip().lower() in ['sair', 'exit', 'q']:
                print("⏹️ Saindo do sistema...")
                break

            linhas = entrada.strip().split('\n')
            comandos = []
            for linha in linhas:
                try:
                    posicoes = [int(p.strip()) for p in linha.split(',')]
                    if len(posicoes) != 5:
                        print(f"⚠️ Comando inválido: {linha} — esperado 5 posições.")
                        continue
                    comandos.append(posicoes)
                except ValueError:
                    print(f"❌ Comando inválido (não numérico): {linha}")

            if not comandos:
                continue

            # Separar comandos válidos e comandos com risco de colisão
            fila_execucao = []
            fila_colisao = []

            for comando in comandos:
                if manipulador.check_collision(comando):
                    print(f"🚫 Colisão detectada. Aguardando para tentar depois: {comando}")
                    fila_colisao.append(comando)
                else:
                    fila_execucao.append(comando)

            # Executa os comandos seguros
            if fila_execucao:
                print(f"\n▶️ Executando {len(fila_execucao)} comandos seguros...")
                executar_sequencia(fila_execucao, motores, manipulador)

            # Tentativa repetida para comandos com colisão
            tentativas = 3
            while fila_colisao and tentativas > 0:
                print(f"\n🔁 Tentativa de execução dos {len(fila_colisao)} comandos com colisão... ({tentativas} restantes)")
                nova_fila_colisao = []
                for comando in fila_colisao:
                    if manipulador.check_collision(comando):
                        print(f"🚫 Ainda em colisão: {comando}")
                        nova_fila_colisao.append(comando)
                    else:
                        print(f"✅ Agora seguro: {comando}")
                        executar_sequencia([comando], motores, manipulador)
                fila_colisao = nova_fila_colisao
                tentativas -= 1

            if fila_colisao:
                print(f"\n⚠️ Os seguintes comandos foram ignorados após {3} tentativas por risco persistente de colisão:")
                for c in fila_colisao:
                    print(f"   - {c}")

    finally:
        for m in motores.values():
            m.disable_torque()
        port_handler.closePort()
        print("✅ Porta serial fechada.")
