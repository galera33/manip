import time
from dynamixel_sdk import *  # Importa a SDK do Dynamixel

class DynamixelMotor:
    def __init__(self, motor_id, port_handler):
        self.motor_id = motor_id
        self.port_handler = port_handler
        self.packet_handler = PacketHandler(2.0)
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_GOAL_VELOCITY = 104
        self.ADDR_PRESENT_VELOCITY = 128
        self.ADDR_PROFILE_ACCELERATION = 108
        self.ADDR_PROFILE_VELOCITY = 112

    def enable_torque(self):
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 1)
        if result != COMM_SUCCESS:
            print(f"Erro ao habilitar torque no motor {self.motor_id}: {self.packet_handler.getTxRxResult(result)}")
            return False
        return True

    def disable_torque(self):
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 0)
        if result != COMM_SUCCESS:
            print(f"Erro ao desabilitar torque no motor {self.motor_id}: {self.packet_handler.getTxRxResult(result)}")
            return False
        return True

    def set_goal_position(self, position):
        result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_GOAL_POSITION, position)
        if result != COMM_SUCCESS:
            print(f"Erro ao definir posição no motor {self.motor_id}: {self.packet_handler.getTxRxResult(result)}")
            return

        # Monitoramento da posição atual
        attempts = 0
        while attempts < 200:  # Limita o número de tentativas para evitar loop infinito
            present_position, _, error = self.packet_handler.read4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_PRESENT_POSITION)
            if error != 0:
                print(f"Erro ao ler posição: {self.packet_handler.getRxPacketError(error)}")
                break
            print(f"Posição Atual: {present_position}")

            if abs(position - present_position) < 20:
                print(f"Alcançou a posição {position}")
                break
            time.sleep(0.1)
            attempts += 1
        if attempts == 200:
            print(f"Limite de tentativas alcançado. Posição não foi alcançada.")

    def set_profile(self, acceleration, velocity):
        # Define o perfil de aceleração
        result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_PROFILE_ACCELERATION, acceleration)
        if result != COMM_SUCCESS:
            print(f"Erro ao definir aceleração no motor {self.motor_id}: {self.packet_handler.getTxRxResult(result)}")
            return

        # Define o perfil de velocidade
        result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_PROFILE_VELOCITY, velocity)
        if result != COMM_SUCCESS:
            print(f"Erro ao definir velocidade no motor {self.motor_id}: {self.packet_handler.getTxRxResult(result)}")
            return
    
    def move_slow(self, goal_position):
        # Definir uma aceleração muito baixa e uma velocidade mínima
        min_velocity = 30      # Valor mínimo para a velocidade
        min_acceleration = 10   # Valor baixo para aceleração

        # Definir o perfil de movimento com aceleração e velocidade baixas
        self.set_profile(min_acceleration, min_velocity)

        # Movimentar o motor para a posição desejada com a menor velocidade possível
        self.set_goal_position(goal_position)

# Função para inicializar as posições dos motores
def inicial(motor1, motor2, motor3, motor4, motor5, motor6):#POSIÇÃO INICIAL VERIFICADO(19/2)
    motor1.move_slow(1090)
    motor2.move_slow(3452)
    motor3.move_slow(1050)
    motor4.move_slow(860)#ABERTO
    #motor2.move_slow(1781)
    #otor1.move_slow(1222)

# Função para inicializar as posições dos motores
def manipula(motor1, motor2, motor3, motor4, motor5, motor6):#POSIÇÃO INICIAL VERIFICADO(19/2)
    motor1.move_slow(3130)
    motor2.move_slow(3330)
    motor3.move_slow(3540)
    motor4.move_slow(1300)#ABERTO
    #motor2.move_slow(1781)
    #otor1.move_slow(1222)

# Função para inicializar as posições dos motores
def container1(motor1, motor2, motor3, motor4, motor5, motor6):#POSIÇÃO INICIAL VERIFICADO(19/2)
    motor2.move_slow(2730)
    motor4.move_slow(740)
    motor1.move_slow(1050)
    #motor4.move_slow(860)#ABERTO
    #motor2.move_slow(1781)
    #otor1.move_slow(1222)


# Código principal
def funfa():
    port_handler = None
    try:
        # Abrindo a porta única
        port_handler = PortHandler('COM3')  # A porta de comunicação

        # Verifica se a porta foi aberta corretamente
        if not port_handler.openPort():
            raise Exception("Não foi possível abrir a porta COM9.")
        if not port_handler.setBaudRate(1000000):
            raise Exception("Não foi possível configurar a taxa de baud.")

        print("Porta COM9 aberta com sucesso.")
        
        # Criação dos motores, ambos usando a mesma porta
        motor6 = DynamixelMotor(motor_id=6, port_handler=port_handler)
        motor2 = DynamixelMotor(motor_id=2, port_handler=port_handler)
        motor3 = DynamixelMotor(motor_id=3, port_handler=port_handler)
        motor4 = DynamixelMotor(motor_id=7, port_handler=port_handler)
        motor5 = DynamixelMotor(motor_id=5, port_handler=port_handler)
        motor1 = DynamixelMotor(motor_id=1, port_handler=port_handler)
        
        # Habilita o torque para todos os motores
        for motor in [motor1, motor2, motor3, motor4, motor5, motor6]:
            motor.enable_torque()
        print("Torque habilitado com sucesso em todos os motores.")

        # Executa a movimentação inicial
        inicial(motor1, motor2, motor3, motor4, motor5, motor6)
        manipula(motor1, motor2, motor3, motor4, motor5, motor6)
        container1(motor1, motor2, motor3, motor4, motor5, motor6)
        inicial(motor1, motor2, motor3, motor4, motor5, motor6)

    except Exception as e:
        print(f"Erro: {str(e)}")
    finally:
        # Verifica se o port_handler foi inicializado antes de chamar closePort()
        if port_handler is not None and port_handler.is_open():
            port_handler.closePort()
            print("Comunicação com a porta finalizada.")

funfa()
