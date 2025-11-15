#!/usr/bin/env python3
import os, time, tempfile
import vlc
from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from omnicare_msgs.srv import OmniMove, MoveArm, LEDandBuzz

from ament_index_python.packages import get_package_share_directory

os.environ["DISPLAY"] = ":1"
os.environ["XDG_RUNTIME_DIR"] = "/run/user/1000"  # ajuste o UID se for outro

PKG = 'omnicare_expression'
SHARE_DIR = get_package_share_directory(PKG)
FACE_DIR = os.path.join(SHARE_DIR, 'config')
MASTER   = os.path.join(FACE_DIR, 'main_video.mp4')   # único arquivo


# -----------------------------
# Marcadores (segundos)
# -----------------------------
LABELS = {
    "start":                    01.00,   # Start
    "pre_moviments_anchor":     19.20,   # movimentação -> idle
    "pos_moviments_anchor":     20.40,   # idle         -> manipulação
    "manipulation_anchor":      29.00,   # manipulação  -> idle
    "led_and_buzz_anchor":      45.12,   # led_and_buzz -> idle
    "idle_start":               56.00,   # IDLE_START
    "idle_loop":                64.01,   # início do loop idle no vídeo master
    "idle_end":                 72.00,   # fim do trecho idle no master 
    "transition_end":           79.76,   # fim da transição do idle para a continuação
}

TIMELINE = {
    0:                                  "start",
    1:                   "pre_moviments_anchor",
    2:                   "pos_moviments_anchor",
    3:                    "manipulation_anchor",
    4:                    "led_and_buzz_anchor",
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
        em.event_attach(vlc.EventType.MediaPlayerEndReached, self._on_end)

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
        self.current_label = None

        # ROS2: sinal externo para sair do idle
        self.sub_idle_done = self.create_subscription(String, 'omnicare/hri/idle_done', self._idle_done_cb, 10)

        # Services
        self.omni_move_client = self.create_client(OmniMove, '/omnicare/expression/move_omni')
        self.move_arm_client  = self.create_client(MoveArm, '/omnicare/expression/move_arm')
        self.buzzer_client    = self.create_client(LEDandBuzz, '/omnicare/expression/buzzer')

        # self.manipulation_client = 

        # timer
        self.timer = self.create_timer(0.05, self._tick)

        # start
        self._play_master()

    # ------------ util ------------
    def _s_to_ms(self, s: float) -> int:
        return int(round(s * 1000.0))

    def _get_ms(self, label: str) -> int:
        return self._s_to_ms(LABELS[label])
    
    def _curr_anchor_label(self) -> str | None:
        return TIMELINE.get(self.count_state)
    
    def _advance_to_next_anchor(self):
        self.count_state += 1
        self.anchor_fired = False


    def _resolve_label_to_ms(self, label_or_sec) -> int:
        if isinstance(label_or_sec, (int, float)):
            return self._s_to_ms(float(label_or_sec))
        if label_or_sec in LABELS:
            return self._s_to_ms(LABELS[label_or_sec])
        raise KeyError(f"Label '{label_or_sec}' não encontrado em LABELS")

    # ------------ reprodução / seek ------------
    def _play_master(self):
        media = self.instance.media_new(MASTER)
        media.add_option(":file-caching=800")     
        media.add_option(":avcodec-fast")   
        media.add_option(":start-time=2.0")      
        self.player.set_media(media)
        self.player.play()

    def _seek_ms(self, ms: int):
        ok = self.player.set_time(ms)
        if not ok:
            self._pending_seek_ms = ms

    def _enter_idle_loop_forever(self):
        """Loopar idle_loop→idle_end indefinidamente (standby)."""
        # self.standby_idle = True
        self.anchor_fired = True         # evita disparos de novos anchors
        self.idle_loop_active = True
        self.exit_idle_pending = False
        self.resume_target_ms = None
        self.count_state = 1             # opcional: reseta timeline
        self.current_label = None
        # self._last_loop_seek_ms = -1
        self.get_logger().info("[standby] Entrando em loop de idle infinito.")
        self._seek_ms(self._s_to_ms(LABELS["idle_start"]))

    def _leave_idle_loop_and_restart(self):
        """Sair do standby e recomeçar o master (apresentação)."""
        # self.standby_idle = False
        self.anchor_fired = False
        # self.idle_loop_active = False
        self.exit_idle_pending = True
        self.resume_target_ms = 2
        self.count_state = 0
        self.current_label = None
        self.get_logger().info("[standby] Saindo do idle e reiniciando a apresentação.")
        # recomeça do início (ou troque para um label com self._seek_ms(...))
        # self._seek_ms(0)


    # ==== eventos VLC ====
    def _on_playing(self, _evt):
        try:
            self.player.set_fullscreen(True)
            self.player.audio_set_volume(100)
        except Exception:
            pass
        if self._pending_seek_ms is not None:
            ms = self._pending_seek_ms
            if self.player.set_time(ms):
                self._pending_seek_ms = None

    def _on_end(self, _evt):
        self._seek_ms(self._s_to_ms(0))
        # quando o master chega no fim, entra no loop do idle até mandarem começar
        self.get_logger().info("[EOF] Vídeo terminou. Entrando em standby (idle infinito).")
        self._enter_idle_loop_forever()


    # ==== callbacks ROS2 ====
    def _exit_idle(self):
        # if self.current_label == "pre_moviments_anchor": self.resume_target_ms = self._s_to_ms(LABELS["pos_moviments_anchor"])
        if self._curr_anchor_label() is None: 
            self._leave_idle_loop_and_restart()
        else:
            self.resume_target_ms = self._s_to_ms(LABELS[TIMELINE[self.count_state]])
        
        self.exit_idle_pending = True


    def _idle_done_cb(self, _msg: String):
        """Chamado quando a ação física terminou -> sair do loop do idle e continuar."""
        if not self.idle_loop_active:
            return
        self._exit_idle()
        self.get_logger().info("[idle_done] Sinal recebido: sair do idle ao fim do ciclo atual.")

    def _call_services(self, event_id: str):
        self.get_logger().info(f"Evento disparado: {event_id}")
        if event_id == "pre_moviments_anchor":
            # Chamar o serviço de movimento OmniMove
            if not self.omni_move_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Serviço /omnicare/expression/move_omni indisponível.')
                return
            req = OmniMove.Request()
            req.speed = 5.0
            req.duration = 15.0
            req.frequency = 0.15
            req.axis = 'y'
            self.omni_move_client.call_async(req)
        elif event_id == "manipulation_anchor":
            if not self.move_arm_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Serviço /omnicare/expression/move_arm indisponível.')
                self._exit_idle()
                return
            req = MoveArm.Request()
            req.action = "Hello"
            req.duration = 10.0
            self.move_arm_client.call_async(req)
        elif event_id == "led_and_buzz_anchor":
            if not self.buzzer_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Serviço /omnicare/expression/buzzer indisponível.')
                return
            req = LEDandBuzz.Request()
            req.duration = 15.0
            self.buzzer_client.call_async(req)
        


    
    
    # ==== controle principal ====
    def _tick(self):
        state = self.player.get_state()
        if state != vlc.State.Playing:
            # aplicar seek pendente assim que possível
            if self._pending_seek_ms is not None:
                if self.player.set_time(self._pending_seek_ms):
                    self._pending_seek_ms = None
            self.get_logger().info("VLC is not playing")
            return

        t = self.player.get_time()  # ms
        if t < 0:
            self.get_logger().info(f"time: {t}")
            return
        
        if t >= 56000: self.player.audio_set_volume(50)
        else: self.player.audio_set_volume(100)

        
        # 1) Quando passar do pre_idle_anchor -> entrar no loop do idle
        if not self.anchor_fired:
            self.current_label = self._curr_anchor_label()
            self.get_logger().info("Will go to idle to do a service")
            if self.current_label is not None:
                if self.current_label == "pos_moviments_anchor":
                    self._advance_to_next_anchor()
                    self.current_label = self._curr_anchor_label()

                anchor_ms = self._get_ms(self.current_label)
                self.get_logger().info(f"t: {t} + {EPS_MS} >= {anchor_ms}" )
                if t + EPS_MS >= anchor_ms and self.count_state:
                    self.get_logger().info("Will change")
                    self.anchor_fired = True
                    self.idle_loop_active = True
                    self.resume_ms = self._s_to_ms(LABELS["idle_end"])  # para onde continuar depois do idle
                    
                    self.get_logger().info(f"[{self.current_label}] → idle: seek {LABELS['idle_start']:.2f}s")
                    self._seek_ms(self._s_to_ms(LABELS["idle_start"]))
                    self._call_services(self.current_label)
                    return  # espera o seek aplicar
                

        # 2) Se estiver em loop do idle, mantenha looping
        idle_start_ms = self._s_to_ms(LABELS["idle_loop"])
        idle_end_ms   = self._s_to_ms(LABELS["idle_end"]) if not self.exit_idle_pending else self._s_to_ms(LABELS["transition_end"])
        if self.idle_loop_active:

            self.get_logger().info("In in looping")
            self.get_logger().info(f"t: {t} >= {idle_end_ms} - {EPS_MS}" )
            # Estamos perto do fim do trecho idle?
            if t >= idle_end_ms - EPS_MS:
                self.get_logger().info("Near the end")
                
                if self.exit_idle_pending and self.resume_target_ms is not None:
                    # Sair do idle: NÃO relupar. Faz seek para o alvo da apresentação.
                    self.idle_loop_active = False
                    self.exit_idle_pending = False
                    target = self.resume_target_ms
                    self.resume_target_ms = None
                    
                    self.get_logger().info(f"[idle] Saindo ao fim do ciclo -> seek para {target/1000:.3f}s")
                    self._seek_ms(target)
                    self._advance_to_next_anchor()

                    return
                else:
                    
                    # Quando for recomeçar a apresentação, para não ter que  esperar o idle inteiro
                    if not self.count_state:
                        return
                    
                    # Continuar loopando o idle (reinicia o trecho)
                    self._seek_ms(idle_start_ms)
            return  # permanece gerenciado aqui enquanto estiver em idle
    
        # 3) Se não tiver nenhum estado mais e no final do video, quer dizer que estamos no final da apresentação
        #    portanto loop no idle
        self.current_label = self._curr_anchor_label()
        if (self.current_label is None and t > 75000):
            self.get_logger().info("In the endless loop")
            self._seek_ms(idle_start_ms)
            self.idle_loop_active = True

        
def main():
    rclpy.init()
    node = ExpressionsPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
