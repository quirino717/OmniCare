#!/usr/bin/env python3
import os, time, tempfile
import vlc
from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

PKG = 'omnicare_expression'
SHARE_DIR = get_package_share_directory(PKG)
FACE_DIR = os.path.join(SHARE_DIR, 'config')
MASTER   = os.path.join(FACE_DIR, 'main_video.mp4')   # único arquivo


# -----------------------------
# Marcadores (segundos)
# -----------------------------
LABELS = {
    "pre_moviments_anchor":     15.40,   # quando a apresentação pede para “ir ao idle”
    "pos_moviments_anchor":     16.00,   # quando a apresentação pede para “ir ao idle”
    "idle_start":               48.02,   # IDLE_START
    "idle_loop":                56.00,   # início do loop idle no vídeo master
    "idle_end":                 64.01,   # fim do trecho idle no master 
    "transition_end":           72.00,   # fim da transição do idle para a continuação
}

TIMELINE = {
    1:                   "pre_moviments_anchor",
    2:                   "pos_moviments_anchor",
    3:                   "manipulation_anchor",
    4:                            "led_anchor",
    5:                         "buzzer_anchor"
}
# tolerância (ms) para disparar um evento ao redor do tempo alvo
EPS_MS = 120

class ExpressionsPlayer(Node):
    def __init__(self):
        super().__init__('omnicare_expressions')
        if not os.path.exists(MASTER):
            self.get_logger().error(f"Master não encontrado: {MASTER}")

        # VLC
        self.instance = vlc.Instance("--no-video-title-show", "--quiet", "--avcodec-hw=any", "--fullscreen")
        self.player = self.instance.media_player_new()
        em = self.player.event_manager()
        em.event_attach(vlc.EventType.MediaPlayerPlaying, self._on_playing)

        # estado
        self._pending_seek_ms = None
        self._fired_ids = set()          # ids (índices) de eventos já disparados
        self.exit_idle_pending = False     # pedido para sair do idle ao fim do ciclo atual
        self.resume_target_ms = None       # para onde pular quando sair do idle
        self.count_state = 1

        # estado de timeline/idle
        self.anchor_fired = False
        self.idle_loop_active = False
        self.resume_ms = None  # para onde continuar após o idle (idle_end)

        # ROS: sinal externo para sair do idle
        self.sub_idle_done = self.create_subscription(String, '/hri/idle_done', self._idle_done_cb, 10)

        # timer
        self.timer = self.create_timer(0.05, self._tick)

        # start
        self._play_master()

    # ------------ util ------------
    def _s_to_ms(self, s: float) -> int:
        return int(round(s * 1000.0))

    def _resolve_label_to_ms(self, label_or_sec) -> int:
        if isinstance(label_or_sec, (int, float)):
            return self._s_to_ms(float(label_or_sec))
        if label_or_sec in LABELS:
            return self._s_to_ms(LABELS[label_or_sec])
        raise KeyError(f"Label '{label_or_sec}' não encontrado em LABELS")

    # ------------ reprodução / seek ------------
    def _play_master(self):
        media = self.instance.media_new(MASTER)
        self.player.set_media(media)
        self.player.play()

    def _seek_ms(self, ms: int):
        ok = self.player.set_time(ms)
        if not ok:
            self._pending_seek_ms = ms

    # ==== eventos VLC ====
    def _on_playing(self, _evt):
        try:
            self.player.set_fullscreen(True)
        except Exception:
            pass
        if self._pending_seek_ms is not None:
            ms = self._pending_seek_ms
            if self.player.set_time(ms):
                self._pending_seek_ms = None

    # ==== callbacks ROS ====
    def _idle_done_cb(self, _msg: String):
        """Chamado quando a ação física terminou -> sair do loop do idle e continuar."""
        if not self.idle_loop_active:
            return
        # Desativa loop e pula para idle_end (continuação natural da apresentação)
        # self.idle_loop_active = False
        # target = self._s_to_ms(LABELS[TIMELINE[self.count_state]])
        # self.get_logger().info(f"[idle_done] Saindo do loop idle -> seek para {LABELS[TIMELINE[self.count_state]]:.2f}s")
        # self._seek_ms(target)
        self.count_state += 1
        self.resume_target_ms = self._s_to_ms(LABELS[TIMELINE[self.count_state]])
        self.exit_idle_pending = True
        self.get_logger().info("[idle_done] Sinal recebido: sair do idle ao fim do ciclo atual.")

    # ==== controle principal ====
    def _tick(self):
        state = self.player.get_state()
        if state != vlc.State.Playing:
            # aplicar seek pendente assim que possível
            if self._pending_seek_ms is not None:
                if self.player.set_time(self._pending_seek_ms):
                    self._pending_seek_ms = None
                    self.fade_in(180)
            return

        t = self.player.get_time()  # ms
        if t < 0:
            return
        # 1) Quando passar do pre_idle_anchor -> entrar no loop do idle
        if not self.anchor_fired:
            anchor_ms = self._s_to_ms(LABELS[TIMELINE[self.count_state]])
            if t + EPS_MS >= anchor_ms:
                self.anchor_fired = True
                self.resume_ms = self._s_to_ms(LABELS["idle_end"])  # para onde continuar depois do idle
                # entrar no idle (loopa idle_start..idle_end)
                self.idle_loop_active = True
                self.get_logger().info(f"[anchor] Entrando no idle: seek {LABELS['idle_start']:.2f}s, loop até {LABELS['idle_end']:.2f}s")
                self._seek_ms(self._s_to_ms(LABELS["idle_start"]))
                return  # espera o seek aplicar

        # 2) Se estiver em loop do idle, mantenha looping
        if self.idle_loop_active:
            idle_start_ms = self._s_to_ms(LABELS["idle_loop"])
            idle_end_ms   = self._s_to_ms(LABELS["idle_end"]) if not self.exit_idle_pending else self._s_to_ms(LABELS["transition_end"])

            # Estamos perto do fim do trecho idle?
            if t >= idle_end_ms - EPS_MS:
                if self.exit_idle_pending and self.resume_target_ms is not None:
                    # Sair do idle: NÃO relupar. Faz seek para o alvo da apresentação.
                    self.idle_loop_active = False
                    self.exit_idle_pending = False
                    target = self.resume_target_ms
                    self.resume_target_ms = None
                    self.get_logger().info(f"[idle] Saindo ao fim do ciclo -> seek para {target/1000:.3f}s")
                    self._seek_ms(target)
                    return
                else:
                    # Continuar loopando o idle (reinicia o trecho)
                    self._seek_ms(idle_start_ms)
            return  # permanece gerenciado aqui enquanto estiver em idle
        
        # 3) Segue a apresentação normal (sem nada para fazer aqui)
def main():
    rclpy.init()
    node = ExpressionsPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
