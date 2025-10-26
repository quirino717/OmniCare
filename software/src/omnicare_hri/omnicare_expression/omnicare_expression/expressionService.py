#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt16, UInt8, Bool

from omnicare_msgs.srv import OmniMove, MoveArm, LEDandBuzz

class ExpressionServicesNode(Node):
    """
    Nó de serviços do OmniCare (HRI/Expression):
      - /omnicare/expression/move_omni : move lateral/omnidirecional em vai-e-vem
      - /omnicare/expression/move_arm  : comanda uma ação do manipulador
      - /omnicare/expression/buzzer    : toca buzzer com parâmetros simples
    """

    def __init__(self):
        super().__init__('expression_services_node')

        # Publishers subjacentes (ajuste tópicos conforme seu stack):
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Simples: publicamos uma string com a ação do braço (troque por msg/serviço próprio do seu manipulador)
        self.arm_cmd_pub = self.create_publisher(String, '/omnicare/arm/cmd', 10)

        # Buzzer simples: frequência em Hz e pattern (tópicos separados para clareza)
        self.led_and_buzzer_pub = self.create_publisher(Bool, '/omnicare/hri/anthem', 10)
        self.buzzer_pub = self.create_publisher(Bool, '/omnicare/hri/alarm', 10)

        # Finish pub
        self.finish_pub = self.create_publisher(String, 'omnicare/hri/idle_done', 10)

        # Serviços
        self.srv_move = self.create_service(OmniMove,
                                            '/omnicare/expression/move_omni',
                                            self.handle_move_omni)

        self.srv_arm  = self.create_service(MoveArm,
                                            '/omnicare/expression/move_arm',
                                            self.handle_move_arm)

        self.srv_buzz = self.create_service(LEDandBuzz,
                                            '/omnicare/expression/buzzer',
                                            self.handle_buzzer)

        # Mutex para garantir um movimento por vez (evita sobreposição de threads de movimento)
        self._move_lock = threading.Lock()
        self._move_thread: Optional[threading.Thread] = None
        self._stop_move_flag = False

        self.get_logger().info('expression_services_node pronto.')

    # ---------- Serviço 1: Movimento omnidirecional (vai-e-vem) ----------
    def handle_move_omni(self, request: OmniMove.Request, response: OmniMove.Response):
        speed = max(0.0, float(request.speed))
        duration = max(0.0, float(request.duration))
        freq = max(0.0, float(request.frequency))
        axis = (request.axis or 'y').lower()

        if duration <= 0.0:
            response.success = False
            response.message = 'duration deve ser > 0.'
            return response

        if axis not in ('x', 'y'):
            response.success = False
            response.message = 'axis deve ser "x" ou "y".'
            return response

        # Se já está em movimento, peça para parar antes
        if self._move_lock.locked():
            response.success = False
            response.message = 'Movimento já em execução. Tente novamente após concluir/parar.'
            return response

        def do_motion():
            with self._move_lock:
                self._stop_move_flag = False
                self.get_logger().info(f'Iniciando vai-e-vem: axis={axis}, speed={speed}, duration={duration}s, freq={freq}Hz')

                start = time.time()
                rate_hz = 50.0  # 50 Hz de atualização
                dt = 1.0 / rate_hz
                omega = 2.0 * math.pi * (freq if freq > 0.0 else 0.5)  # default 0.5 Hz se freq=0

                while (time.time() - start) < duration and not self._stop_move_flag and rclpy.ok():
                    t = time.time() - start
                    v = speed * math.sin(omega * t)  # perfil senoidal

                    twist = Twist()
                    if axis == 'y':
                        twist.linear.y = v
                    else:
                        twist.linear.x = v

                    self.cmd_vel_pub.publish(twist)
                    time.sleep(dt)

                # Parar
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info('Movimento concluído.')

                # Notifica fim do movimento 
                self.finish_pub.publish(String(data='done'))

        self._move_thread = threading.Thread(target=do_motion, daemon=True)
        self._move_thread.start()

        response.success = True
        response.message = 'Movimento iniciado.'
        return response

    # ---------- Serviço 2: Manipulador ----------
    # TODO: implemente conforme seu manipulador real (Esperando o @Thiago implementar)
    def handle_move_arm(self, request: MoveArm.Request, response: MoveArm.Response):
        action = (request.action or '').strip()
        dur = max(0.0, float(request.duration))
        self.get_logger().info(f'Requisição de movimento do braço: action="{action}", duration={dur:.2f}s')
        
        # Notifica fim do movimento (apenas para simular)
        self.finish_pub.publish(String(data='done'))

        if not action:
            response.success = False
            response.message = 'action não pode ser vazio.'
            return response

        # Publica a ação como string (troque por seu protocolo real)
        msg = String()
        msg.data = f'{action}:{dur:.2f}'
        self.arm_cmd_pub.publish(msg)

        self.get_logger().info(f'Manipulador acionado: action="{action}" duration={dur:.2f}s')
        response.success = True
        response.message = 'Comando enviado ao manipulador.'
        return response

    # # ---------- Serviço 3: Buzzer ----------
    def handle_buzzer(self, request: LEDandBuzz.Request, response: LEDandBuzz.Response):
        dur = max(0.0, float(request.duration))

        # Ativa o buzzer
        self.buzzer_pub.publish(Bool(data=True))

        # Publica parâmetros (implemente o lado do firmware para interpretar)
        self.led_and_buzzer_pub.publish(Bool(data=True))
        
        self.get_logger().info(f'Buzzer: duration={dur:.2f}s')

        # Manter padrão por "dur" e depois silenciar (sem bloquear o servidor)
        def auto_stop():
            time.sleep(dur)
            # Silencia 
            self.buzzer_pub.publish(Bool(data=False))
            self.get_logger().info('Buzzer: stop automático após duração.')

            # Notifica fim do buzzer
            self.finish_pub.publish(String(data='done'))


        if dur > 0.0:
            threading.Thread(target=auto_stop, daemon=True).start()

        response.success = True
        response.message = 'Buzzer acionado.'
        return response

    # ---------- Método para parar movimento em andamento ----------
    def stop_motion(self):
        self._stop_move_flag = True
        # zera cmd_vel
        self.cmd_vel_pub.publish(Twist())


def main():
    rclpy.init()
    node = ExpressionServicesNode()

    # Executor multithread para não travar serviços enquanto threads rodam
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
