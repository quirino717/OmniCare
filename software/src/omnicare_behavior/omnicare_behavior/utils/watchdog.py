from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration

class Watchdog:
    """
    Watchdog simples baseado em timer e Clock STEADY.
    - keep_alive(): renova o prazo
    - on_timeout(): chamado uma única vez quando o prazo expira
    """
    def __init__(self, node: Node, timeout_sec: float, check_period_sec: float = 0.2, on_timeout=None):
        self._node = node
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timeout = Duration(seconds=float(timeout_sec))
        self._check_period = float(check_period_sec)
        self._on_timeout = on_timeout
        self._tripped = False  # evita disparo repetido

        now = self._steady_clock.now()
        self._deadline = now + self._timeout

        # Timer para verificar o deadline
        self._timer = self._node.create_timer(self._check_period, self._check_deadline)

        self._node.get_logger().info(f"[Watchdog] iniciado (timeout={timeout_sec:.2f}s, check={check_period_sec:.2f}s)")

    def keep_alive(self):
        """Renova o prazo do watchdog (o famoso 'kick')."""
        self._deadline = self._steady_clock.now() + self._timeout
        self._tripped = False  # volta a permitir um novo disparo no futuro

    def _check_deadline(self):
        now = self._steady_clock.now()
        if now > self._deadline and not self._tripped:
            self._tripped = True
            self._node.get_logger().warn("[Watchdog] TIMEOUT alcançado!")
            if callable(self._on_timeout):
                try:
                    self._on_timeout()
                except Exception as e:
                    self._node.get_logger().error(f"[Watchdog] erro no on_timeout: {e}")

    def cancel(self):
        self._timer.cancel()
        self._node.get_logger().info("[Watchdog] cancelado")