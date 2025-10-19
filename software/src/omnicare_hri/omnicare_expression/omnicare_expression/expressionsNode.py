#!/usr/bin/env python3
import os
import time
import vlc
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

PKG = 'omnicare_expression'
SHARE_DIR = get_package_share_directory(PKG)

# Coloque seus vídeos em <share>/config/
FACE_DIR = os.path.join(SHARE_DIR, 'config')
IDLE     = os.path.join(FACE_DIR, 'idle.mp4')          # vídeo em loop
PRESENT  = os.path.join(FACE_DIR, 'presentation.mp4')  # vídeo único da apresentação

class ExpressionsPlayer(Node):
    def __init__(self):
        super().__init__('omnicare_expressions')

        self.get_logger().info("Iniciando nó de expressões OmniCare…")
        self.get_logger().info(f"share dir: {SHARE_DIR}")
        self.get_logger().info(f"Idle: {IDLE}")
        self.get_logger().info(f"Presentation: {PRESENT}")

        # --- libVLC base ---
        self.instance = vlc.Instance(
            "--no-video-title-show",
            "--quiet",
            "--avcodec-hw=any",     # mude para nvdec/vaapi se quiser forçar
            "--fullscreen"
        )

        # MediaPlayer “de verdade” (janela)
        self.player = self.instance.media_player_new()

        # MediaListPlayer para tocar listas com modo loop
        self.list_player = vlc.MediaListPlayer()
        self.list_player.set_media_player(self.player)

        # Eventos do MediaPlayer (usamos para fullscreen e fim de mídia)
        em = self.player.event_manager()
        em.event_attach(vlc.EventType.MediaPlayerPlaying, self._on_playing)
        em.event_attach(vlc.EventType.MediaPlayerEndReached, self._on_end)

        # Subscriptions ROS
        self.sub_present = self.create_subscription(String, '/hri/present', self._present_cb, 10)
        self.sub_state   = self.create_subscription(String, '/hri/state',    self._state_cb,    10)

        # Estado de apresentação
        self.mode = "idle"
        self._milestones = []  # (ms, payload)
        self._fired = set()    # marcos já disparados

        self._last_time_ms = -1          # último tempo lido do player (ms)
        self._no_progress_ms = 0         # tempo acumulado sem progresso (ms)

        # Timer para checar o tempo do vídeo durante a apresentação (20 Hz)
        self.timer = self.create_timer(0.05, self._tick)

        # Inicia em idle (loop robusto via MediaListPlayer)
        self.play_idle_loop()

    # ==========================
    # Helpers VLC / Overlays
    # ==========================
    def _on_playing(self, _evt):
        # Garante fullscreen quando a janela existir
        try:
            self.player.set_fullscreen(True)
        except Exception as e:
            self.get_logger().warn(f"Falha ao setar fullscreen: {e}")

    def _on_end(self, _evt):
        # Este evento pode disparar quando a apresentação termina
        # No idle (loop) ignoramos; no presenting, voltamos ao idle
        if self.mode == "presenting":
            self.get_logger().info("Apresentação concluída. Voltando para idle.")
            self.mode = "idle"
            self._fired.clear()
            self._clear_logo()
            self.play_idle_loop()

    def _show_marquee(self, text: str, ms: int = 4500, x: int = 40, y: int = 60, size: int = 36, opacity: int = 255):
        self.player.video_set_marquee_int(vlc.VideoMarqueeOption.Enable, 1)
        self.player.video_set_marquee_int(vlc.VideoMarqueeOption.X, x)
        self.player.video_set_marquee_int(vlc.VideoMarqueeOption.Y, y)
        self.player.video_set_marquee_int(vlc.VideoMarqueeOption.Size, size)
        self.player.video_set_marquee_int(vlc.VideoMarqueeOption.Timeout, ms)
        self.player.video_set_marquee_int(vlc.VideoMarqueeOption.Opacity, opacity)
        self.player.video_set_marquee_string(vlc.VideoMarqueeOption.Text, text)

    def _show_logo(self, png_path: str, x: int = 40, y: int = 120, opacity: int = 255):
        if not os.path.exists(png_path):
            self.get_logger().warn(f"Logo não encontrado: {png_path}")
            return
        self.player.video_set_logo_int(vlc.VideoLogoOption.enable, 1)
        self.player.video_set_logo_int(vlc.VideoLogoOption.x, x)
        self.player.video_set_logo_int(vlc.VideoLogoOption.y, y)
        self.player.video_set_logo_int(vlc.VideoLogoOption.opacity, opacity)
        self.player.video_set_logo_string(vlc.VideoLogoOption.file, png_path)

    def _clear_logo(self):
        self.player.video_set_logo_int(vlc.VideoLogoOption.logo_enable, 0)

    # ==========================
    # Reprodutores
    # ==========================
    def play_idle_loop(self):
        """Loop robusto do vídeo idle usando MediaListPlayer."""
        if not os.path.exists(IDLE):
            self.get_logger().error(f"Arquivo idle não encontrado: {IDLE}")
            return
        ml = self.instance.media_list_new([IDLE])
        self.list_player.set_media_list(ml)
        self.list_player.set_playback_mode(vlc.PlaybackMode.loop)  # << loop real
        self.list_player.play()

    def play_presentation_once(self):
        """Toca o vídeo de apresentação apenas uma vez, sem lista."""
        if not os.path.exists(PRESENT):
            self.get_logger().error(f"Arquivo de apresentação não encontrado: {PRESENT}")
            return
        # Para o list_player se estiver em idle
        try:
            self.list_player.stop()
        except Exception:
            pass
        media = self.instance.media_new(PRESENT)
        self.player.set_media(media)
        self.player.play()

    # ==========================
    # ROS Callbacks
    # ==========================
    def _present_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ("start", "play", "apresentar"):
            self.mode = "presenting"
            self.get_logger().info("Iniciando apresentação…")
            self._prepare_milestones()
            self._fired.clear()
            self._clear_logo()
            self.play_presentation_once()
        elif cmd in ("stop", "cancel", "abort"):
            self.get_logger().info("Interrompendo apresentação e voltando ao idle…")
            self.mode = "idle"
            self._fired.clear()
            self._clear_logo()
            self.play_idle_loop()

    def _state_cb(self, msg: String):
        if msg.data.strip().lower() == "idle" and self.mode != "idle":
            self.mode = "idle"
            self._fired.clear()
            self._clear_logo()
            self.play_idle_loop()

    # ==========================
    # Milestones / Timer
    # ==========================
    def _prepare_milestones(self):
        """Define os capítulos (tempo em ms) da apresentação."""
        self._milestones = [
            ( 2000,  {"text": "🧠 OmniCare: Assistente hospitalar autônomo", "dur": 4500}),
            ( 8000,  {"text": "🛞 Movimento omnidirecional: precisão em espaços apertados", "dur": 5000}),
            (15000,  {"text": "🤖 Manipulador: entrega segura de medicamentos", "dur": 5000}),
            (22000,  {"text": "💡 LED & 🔊 Buzzer: feedback claro para a equipe", "dur": 5000}),
            # Exemplo com logo:
            # (15000, {"logo": os.path.join(FACE_DIR, "icons", "arm.png")}),
        ]

    def _tick(self):
        """Checa o tempo do vídeo durante a apresentação e dispara overlays."""
        if self.mode != "presenting":
            return
        
        state = self.player.get_state()
        L = self.player.get_length()  # ms (ou -1 se ainda não disponível
        t = self.player.get_time()  # ms (ou -1 se ainda não disponível)
        if t < 0:
            return
        for ms, payload in self._milestones:
            if ms in self._fired or t < ms:
                continue
            # Dispara
            if "text" in payload:
                self._show_marquee(payload["text"], ms=payload.get("dur", 4500))
            if "logo" in payload:
                self._show_logo(payload["logo"])
            # Aqui você pode também publicar algo (ex.: acionar LED real)
            # self.pub_led.publish(Bool(data=True))
            self._fired.add(ms)
        
            # --- Fallbacks para detectar término e voltar ao idle ---

        # 1) Estados de término/erro conhecidos do VLC
        if state in (vlc.State.Ended, vlc.State.Stopped, vlc.State.Error):
            self.get_logger().info(f"Fim/erro detectado por estado VLC: {state}. Voltando ao idle.")
            self.mode = "idle"
            self._fired.clear()
            self._clear_logo()
            self.play_idle_loop()
            # reset watchdog
            self._last_time_ms = -1
            self._no_progress_ms = 0
            return

        # 2) Checagem por tempo alcançando o comprimento do vídeo
        if t >= 0 and L and L > 0:
            # margem de 200 ms para considerar 'acabou'
            if t >= (L - 200):
                self.get_logger().info("Apresentação concluída. Voltando para idle.")
                self.get_logger().info("Fim detectado por tempo (t≈len). Voltando ao idle.")
                self.mode = "idle"
                self._fired.clear()
                self._clear_logo()
                self.play_idle_loop()
                self._last_time_ms = -1
                self._no_progress_ms = 0
                return

        # 3) Watchdog de progresso (resolve caso o player “congele” sem emitir EndReached)
        # Incrementa 50ms por tick (o timer roda a cada 0.05 s)
        if t >= 0:
            if t == self._last_time_ms:
                self._no_progress_ms += 50
            else:
                self._no_progress_ms = 0
                self._last_time_ms = t

            # Se ficar 3s sem progredir, consideramos encerrado e voltamos ao idle
            if self._no_progress_ms >= 3000:
                self.get_logger().warn("Sem progresso no tempo de mídia por 3s. Voltando ao idle (watchdog).")
                self.mode = "idle"
                self._fired.clear()
                self._clear_logo()
                self.play_idle_loop()
                self._last_time_ms = -1
                self._no_progress_ms = 0

def main():
    rclpy.init()
    node = ExpressionsPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
