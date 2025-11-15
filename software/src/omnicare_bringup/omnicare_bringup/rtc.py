#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
import subprocess
from datetime import datetime, timezone


class OneShotTimeSyncNode(Node):
    def __init__(self):
        super().__init__('one_shot_time_sync')
        self.sub = self.create_subscription(Time, '/pc_time', self.cb_time, 10)
        self.synced = False  # flag para garantir que só roda uma vez
        self.get_logger().info('Aguardando hora do notebook em /pc_time...')

    def cb_time(self, msg: Time):
        if self.synced:
            return  # já sincronizou, não faz mais nada

        # Converte Time (sec + nanosec) para datetime
        ts = msg.sec + msg.nanosec * 1e-9
        dt = datetime.fromtimestamp(ts, tz=timezone.utc).astimezone()  # converte pro fuso local
        dt_str = dt.strftime('%Y-%m-%d %H:%M:%S')

        self.get_logger().info(f'Recebido horário do notebook: {dt_str}')
        self.get_logger().info('Ajustando relógio da Jetson...')

        # IMPORTANTE: precisa ter permissão pra rodar date -s
        subprocess.run(['sudo', 'date', '-s', dt_str])

        self.synced = True
        self.get_logger().info('Relógio sincronizado uma única vez. Não serão feitas novas correções.')

        # Desinscreve para não gastar nada
        self.sub.destroy()

    def shutdown_node(self):
        self.get_logger().info('Encerrando nó one_shot_time_sync.')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = OneShotTimeSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
