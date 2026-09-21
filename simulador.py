"""
Simulador de la ESP32 para desarrollar y probar el gemelo sin hardware.

Responde a los mismos comandos que el firmware (F, R, S, E, Vxx, Mxx, Ox, STATUS),
con las mismas respuestas de texto, y emite telemetria con el mismo formato.
Reproduce la rampa de aceleracion, el cambio de sentido seguro, el encoder
(actualizado cada 1 s con algo de ruido) y un objeto que recorre la cinta.
"""

import collections
import math
import random
import threading
import time

import config as C
from comunicacion import Transporte

PERIODO_TELEMETRIA_S = 0.2
OFFSET_HC_CM = 4.0
LARGO_OBJETO_CM = 4.0


class TransporteSimulado(Transporte):
    descripcion = "Simulador (sin hardware)"

    def conectar(self):
        self.lock = threading.Lock()
        self.respuestas = collections.deque()
        self.dir_sol = "STOP"
        self.dir_act = "STOP"
        self.vpct = 50
        self.micro = 8
        self.rpm_cmd = 0.0
        self.rpm_obj = 0.0
        self.en_regimen = False
        ahora = time.time()
        self.t_prev = ahora
        self.t_tel = 0.0
        self.t_enc = ahora
        self.vueltas = 0.0
        self.rpm_enc = 0.0
        self.pos = 8.0
        self.pos_filt = None
        self.visible = True
        self.t_reaparece = 0.0

    # ----------------------------------------------------------
    def _responder(self, texto):
        # latencia simulada del enlace
        self.respuestas.append((time.time() + random.uniform(0.015, 0.045), texto))

    def enviar(self, texto):
        cmd = texto.strip().upper()
        with self.lock:
            if cmd == "F":
                self.dir_sol = "FWD"
                self._responder("Direccion: ADELANTE")
            elif cmd == "R":
                self.dir_sol = "REV"
                self._responder("Direccion: REVERSA")
            elif cmd == "S":
                self.dir_sol = "STOP"
                self._responder("STOP")
            elif cmd == "E":
                self.dir_sol = self.dir_act = "STOP"
                self.rpm_cmd = 0.0
                self._responder("STOP INMEDIATO")
            elif cmd.startswith("V"):
                try:
                    v = int(cmd[1:])
                except ValueError:
                    v = -1
                if 0 <= v <= 100:
                    self.vpct = v
                    if v == 0:
                        self.dir_sol = "STOP"
                    self._responder(f"Velocidad: {v}%")
                else:
                    self._responder("V debe ser 0-100")
            elif cmd.startswith("M"):
                try:
                    n = int(cmd[1:])
                except ValueError:
                    n = -1
                if n in (2, 4, 8, 16):
                    if self.dir_act != "STOP" or self.rpm_cmd > 0:
                        self._responder("Detener (S) antes de cambiar M")
                    else:
                        self.micro = n
                        self._responder(f"Microstep: 1/{n}")
                else:
                    self._responder("Use M2 M4 M8 M16")
            elif cmd.startswith("O"):
                self._responder(f"Offset HC: {cmd[1:]}")
            elif cmd == "STATUS":
                self.t_tel = 0.0
            else:
                self._responder("Comando invalido")

    # ----------------------------------------------------------
    def _fisica(self, ahora):
        dt = min(ahora - self.t_prev, 0.1)
        self.t_prev = ahora

        if self.dir_act == "STOP" and self.dir_sol != "STOP" and self.rpm_cmd < 0.1:
            self.dir_act = self.dir_sol

        self.rpm_obj = 0.0 if self.dir_sol == "STOP" else C.MAX_MOTOR_RPM * self.vpct / 100.0
        objetivo = self.rpm_obj
        if self.dir_act != self.dir_sol and self.dir_act != "STOP":
            objetivo = 0.0

        if self.rpm_cmd < objetivo:
            self.rpm_cmd = min(objetivo, self.rpm_cmd + C.ACCEL_RPM_S * dt)
        elif self.rpm_cmd > objetivo:
            self.rpm_cmd = max(objetivo, self.rpm_cmd - C.DECEL_RPM_S * dt)

        if self.rpm_cmd < 0.1 and self.dir_act != self.dir_sol:
            self.rpm_cmd = 0.0
            self.dir_act = self.dir_sol

        self.en_regimen = self.rpm_obj > 0 and abs(self.rpm_cmd - self.rpm_obj) < 2.0

        rpm_rod = self.rpm_cmd * C.RELACION
        vel = rpm_rod * math.pi * C.DIAMETRO_RODILLO_CM / 60.0
        self.vueltas += rpm_rod / 60.0 * dt

        # Objeto sobre la banda: al llegar a un extremo "se retira"
        # y a los 2,5 s se coloca uno nuevo en el otro extremo.
        signo = {"FWD": C.SIGNO_POS_FWD, "REV": -C.SIGNO_POS_FWD}.get(self.dir_act, 0)
        if self.visible:
            self.pos += signo * vel * dt
            if self.pos > C.LARGO_CINTA_CM or self.pos < 0:
                self.visible = False
                self.t_reaparece = ahora + 2.5
        elif ahora >= self.t_reaparece:
            self.visible = True
            self.pos = 3.0 if signo >= 0 else C.LARGO_CINTA_CM - 3.0
            self.pos_filt = None

        # Encoder: calculo cada 1 s (como la ventana del firmware)
        if ahora - self.t_enc >= 1.0:
            self.rpm_enc = self.vueltas / (ahora - self.t_enc) * 60.0
            self.rpm_enc *= 1.0 + random.gauss(0, 0.006)
            self.vueltas = 0.0
            self.t_enc = ahora

    def _telemetria(self):
        if self.visible:
            medida = self.pos + random.gauss(0, 0.3)
            if self.pos_filt is None:
                self.pos_filt = medida
            else:
                self.pos_filt = 0.35 * medida + 0.65 * self.pos_filt
            pos_txt = f"{min(max(self.pos_filt, 0.0), C.LARGO_CINTA_CM):.1f}"
            dist = medida + OFFSET_HC_CM
        else:
            pos_txt = "NA"
            dist = C.LARGO_CINTA_CM + 12.0

        rpm_t = self.rpm_cmd * C.RELACION
        k = math.pi * C.DIAMETRO_RODILLO_CM / 60.0
        err = abs(self.rpm_enc - rpm_t) / rpm_t * 100.0 if rpm_t > 1.0 else 0.0
        estado = "RAMP" if (not self.en_regimen and self.dir_act != "STOP") else "OK"

        return (f"DIR={self.dir_act},V={self.vpct},M={self.micro},"
                f"RPM_M={self.rpm_cmd:.1f},RPM_T={rpm_t:.1f},RPM_R={self.rpm_enc:.1f},"
                f"VEL_T={rpm_t * k:.2f},VEL_R={self.rpm_enc * k:.2f},"
                f"DIST={dist:.1f},POS={pos_txt},ERR={err:.1f},STATE={estado}")

    # ----------------------------------------------------------
    def leer_linea(self):
        limite = time.time() + 0.2
        while True:
            ahora = time.time()
            with self.lock:
                self._fisica(ahora)
                if self.respuestas and self.respuestas[0][0] <= ahora:
                    return self.respuestas.popleft()[1]
                if ahora - self.t_tel >= PERIODO_TELEMETRIA_S:
                    self.t_tel = ahora
                    return self._telemetria()
            if ahora > limite:
                return None
            time.sleep(0.01)
