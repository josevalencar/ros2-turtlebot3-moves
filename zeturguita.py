import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, tty, termios
from std_srvs.srv import Empty

VEL_LIN_MAX = 0.22
VEL_ANG_MAX = 2.84
VEL_LIN_INCREMENTO = 0.01
VEL_ANG_INCREMENTO = 0.1

msg = """
Controle o Zeturguita! 
---------------------------
Para se mover:
    ↑
←       →
    ↓

↑ : aumenta a velocidade linear (~ 0.22)
↓ : diminui a velocidade linear (~ 0.22)
← : aumenta velocidade angular (~ 2.84)
→ : diminui velocidade angular (~ 2.84)

Tecla de espaço: força a pausa

Pressione S para encerrar
"""

e = """
Comunicação falhou
"""

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return f"Atualmente:\tlinear vel {target_linear_vel}\t angular vel {target_angular_vel}"

def suavizarVelocidade(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

def limite(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    return input

def verificarLimiteVelLinear(vel):
    return limite(vel, -VEL_LIN_MAX, VEL_LIN_MAX)

def verificarLimiteVelAngular(vel):
    return limite(vel, -VEL_ANG_MAX, VEL_ANG_MAX)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel_linear_alvo = 0.0
        self.vel_angular_alvo = 0.0
        self.vel_linear_controle = 0.0
        self.vel_angular_controle = 0.0
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info(msg)
        self.timer = self.create_timer(0.1, self.update)
        self.srv = self.create_service(Empty, 'parar_robo', self.callback_parar_robo)

    def callback_parar_robo(self, request, response):
        self.vel_linear_alvo = 0.0
        self.vel_angular_alvo = 0.0
        self.vel_linear_controle = 0.0
        self.vel_angular_controle = 0.0
        self.publisher.publish(Twist())  # Publicar Twist com velocidades zero
        self.get_logger().info('Serviço de parada do robô acionado, encerrando...')
        self.shutdown()
        return response
    
    def shutdown(self):
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def update(self):
        key = getKey(self.settings)
        if key == '\x1b[A':  # Seta para cima
            self.vel_linear_alvo = verificarLimiteVelLinear(self.vel_linear_alvo + VEL_LIN_INCREMENTO)
            self.get_logger().info(vels(self.vel_linear_alvo, self.vel_angular_alvo))
        elif key == '\x1b[B':  # Seta para baixo
            self.vel_linear_alvo = verificarLimiteVelLinear(self.vel_linear_alvo - VEL_LIN_INCREMENTO)
            self.get_logger().info(vels(self.vel_linear_alvo, self.vel_angular_alvo))
        elif key == '\x1b[D':  # Seta para esquerda
            self.vel_angular_alvo = verificarLimiteVelAngular(self.vel_angular_alvo + VEL_LIN_INCREMENTO)
            self.get_logger().info(vels(self.vel_linear_alvo, self.vel_angular_alvo))
        elif key == '\x1b[C':  # Seta para direita# Seta para cima
            self.vel_angular_alvo = verificarLimiteVelAngular(self.vel_angular_alvo - VEL_LIN_INCREMENTO)
            self.get_logger().info(vels(self.vel_linear_alvo, self.vel_angular_alvo))
        elif key == ' ':
            self.vel_linear_alvo = 0.0
            self.vel_angular_alvo = 0.0
            self.vel_linear_controle = 0.0
            self.vel_angular_controle = 0.0
            self.get_logger().info(vels(self.vel_linear_alvo, self.vel_angular_alvo))
        else:
            if key.lower() == 's':  # Tecla 'S'
                self.get_logger().info("Solicitando serviço de parada do robô")
                client = self.create_client(Empty, 'parar_robo')
                while not client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Aguardando o serviço de parada do robô...')
                request = Empty.Request()
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    print("Robô parado e comunicação encerrada com sucesso")
                else:
                    print("Falha ao encerrar a comunicação com o robô")
                return

        self.vel_linear_controle = suavizarVelocidade(self.vel_linear_controle, self.vel_linear_alvo, (VEL_LIN_INCREMENTO / 2.0))
        self.vel_angular_controle = suavizarVelocidade(self.vel_angular_controle, self.vel_angular_alvo, (VEL_LIN_INCREMENTO / 2.0))

        twist = Twist()
        twist.linear.x = self.vel_linear_controle
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.vel_angular_controle

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)

if __name__ == '__main__':
    main()
