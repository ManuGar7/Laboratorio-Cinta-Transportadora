"""
Nucleo del Gemelo Digital.

- Mantiene la conexion con la ESP32 en un hilo propio (se reconecta sola).
- Interpreta la telemetria y guarda el historial.
- Ejecuta el modelo digital:
    * velocidad: compara RPM real (encoder) con la teorica (STEP enviados)
    * posicion: predice la posicion del objeto integrando la velocidad del
      modelo y la compara con la medida del HC-SR04
- Mide la latencia de los comandos (envio -> respuesta de la ESP32).
- Permite inyectar fallas simuladas sobre los datos recibidos.
"""

import collections
import threading
import time

import config as C

CAMPOS_NUMERICOS = ["V", "M", "RPM_M", "RPM_T", "RPM_R", "VEL_T", "VEL_R", "DIST", "ERR"]

FALLAS = {
    "perdida_vel": "Pérdida de velocidad (la medición cae un 25 %)",
    "sobrecarga": "Sobrecarga progresiva del motor",
    "deslizamiento": "Deslizamiento del objeto sobre la banda",
}


def parsear_telemetria(linea):
    """Convierte 'DIR=FWD,V=60,...' en un diccionario. None si no es telemetria valida."""
    if not linea.startswith("DIR=") or "STATE=" not in linea:
        return None
    pares = {}
    for trozo in linea.split(","):
        if "=" in trozo:
            clave, valor = trozo.split("=", 1)
            pares[clave.strip()] = valor.strip()
    try:
        tel = {k.lower(): float(pares[k]) for k in CAMPOS_NUMERICOS}
    except (KeyError, ValueError):
        return None  # linea incompleta o corrupta
    try:
        tel["pos"] = float(pares.get("POS", "NA"))
    except ValueError:
        tel["pos"] = None  # "NA": sin objeto valido
    tel["dir"] = pares.get("DIR", "STOP")
    tel["state"] = pares.get("STATE", "OK")
    if tel["dir"] not in ("FWD", "REV", "STOP"):
        return None
    return tel


class Gemelo:
    def __init__(self, transporte):
        self.tr = transporte
        self.lock = threading.RLock()
        self.activo = False
        self.conectado = False
        self.t0 = time.time()
        self.ultimo_rx = None
        self.ultima = None

        self.serie = collections.deque(maxlen=int(C.HISTORIAL_S * 12))
        self.eventos = collections.deque(maxlen=80)
        self.pendientes = collections.deque()
        self.latencias = collections.deque(maxlen=10)

        self.fallas = {k: None for k in FALLAS}  # None = inactiva, float = instante de activacion
        self.ancla_desliz = None

        self.t_tel_prev = None
        self.dir_prev = "STOP"
        self.fase_banda = 0.0

        self.t_regimen = None
        self.t_cond_vel = None
        self.div_vel = False

        self.pos_modelo = None
        self.err_pos = None
        self.lim_pos = None
        self.t_cond_pos = None
        self.div_pos = False

    # ------------------------------------------------------------
    # Hilo de comunicacion
    # ------------------------------------------------------------
    def iniciar(self):
        self.activo = True
        threading.Thread(target=self._bucle, daemon=True).start()

    def detener(self):
        self.activo = False

    def _bucle(self):
        while self.activo:
            try:
                self._evento(f"Conectando: {self.tr.descripcion}")
                self.tr.conectar()
                with self.lock:
                    self.conectado = True
                self._evento("Conexión establecida", "ok")
                while self.activo:
                    linea = self.tr.leer_linea()
                    if linea:
                        self._procesar_linea(linea)
            except Exception as e:
                with self.lock:
                    self.conectado = False
                self._evento(f"Sin conexión ({e}). Reintento en 3 s", "alarma")
                try:
                    self.tr.cerrar()
                except Exception:
                    pass
                time.sleep(3)

    def _evento(self, texto, tipo="info"):
        with self.lock:
            self.eventos.append((time.time(), tipo, texto))

    # ------------------------------------------------------------
    # Comandos
    # ------------------------------------------------------------
    def enviar(self, cmd):
        with self.lock:
            if not self.conectado:
                self._evento(f"Sin conexión: no se envió {cmd}", "alarma")
                return False
            self.pendientes.append((cmd, time.time()))
        try:
            self.tr.enviar(cmd)
        except Exception as e:
            self._evento(f"Error al enviar {cmd}: {e}", "alarma")
            return False
        self._evento(f"Enviado {cmd}", "cmd")
        return True

    def set_fallas(self, activas):
        with self.lock:
            for clave, nombre in FALLAS.items():
                on = clave in activas
                if on and self.fallas[clave] is None:
                    self.fallas[clave] = time.time()
                    self._evento(f"Falla simulada activada: {nombre}", "sim")
                elif not on and self.fallas[clave] is not None:
                    self.fallas[clave] = None
                    if clave == "deslizamiento":
                        self.ancla_desliz = None
                    self._evento(f"Falla simulada desactivada: {nombre}", "sim")

    # ------------------------------------------------------------
    # Procesamiento de lineas
    # ------------------------------------------------------------
    def _procesar_linea(self, linea):
        tel = parsear_telemetria(linea)
        ahora = time.time()
        with self.lock:
            self.ultimo_rx = ahora
            if tel is not None:
                self._procesar_telemetria(tel, ahora)
                return
            # Respuesta de texto a un comando -> latencia
            while self.pendientes and ahora - self.pendientes[0][1] > 2.0:
                self.pendientes.popleft()
            if self.pendientes:
                _, t_envio = self.pendientes.popleft()
                lat = (ahora - t_envio) * 1000.0
                self.latencias.append(lat)
                self._evento(f"ESP32: {linea} ({lat:.0f} ms)", "resp")
            elif "ERROR" in linea.upper():
                self._evento(f"ESP32: {linea}", "alarma")
            # el resto (menu de arranque, etc.) se ignora

    def _procesar_telemetria(self, tel, ahora):
        dt = 0.0 if self.t_tel_prev is None else min(ahora - self.t_tel_prev, 0.5)
        self.t_tel_prev = ahora

        tel = self._aplicar_fallas(tel, ahora)
        signo = {"FWD": C.SIGNO_POS_FWD, "REV": -C.SIGNO_POS_FWD}.get(tel["dir"], 0)
        self.fase_banda = (self.fase_banda + signo * tel["vel_t"] * dt) % 5.0

        self._divergencia_velocidad(tel, ahora)
        self._modelo_posicion(tel, signo, dt, ahora)

        self.dir_prev = tel["dir"]
        self.ultima = tel
        self.serie.append((ahora, tel["vel_t"], tel["vel_r"], tel["pos"],
                           self.pos_modelo, self.div_vel, self.div_pos))

    def _aplicar_fallas(self, tel, ahora):
        t = dict(tel)
        t["simulada"] = False
        factor = 1.0
        if self.fallas["perdida_vel"] is not None:
            factor *= 0.75
        if self.fallas["sobrecarga"] is not None:
            factor *= max(0.45, 1.0 - 0.06 * (ahora - self.fallas["sobrecarga"]))
        if factor < 1.0:
            t["rpm_r"] *= factor
            t["vel_r"] *= factor
            t["err"] = abs(t["rpm_r"] - t["rpm_t"]) / t["rpm_t"] * 100.0 if t["rpm_t"] > 1.0 else 0.0
            t["simulada"] = True
        if self.fallas["deslizamiento"] is not None:
            t["simulada"] = True
            if t["pos"] is None:
                self.ancla_desliz = None
            else:
                if self.ancla_desliz is None:
                    self.ancla_desliz = t["pos"]
                # el objeto avanza solo el 40 % de lo que avanza la banda
                t["pos"] = self.ancla_desliz + 0.4 * (t["pos"] - self.ancla_desliz)
        return t

    # ------------------------------------------------------------
    # Modelo: divergencia de velocidad
    # ------------------------------------------------------------
    def _divergencia_velocidad(self, tel, ahora):
        en_marcha = tel["dir"] != "STOP" and tel["state"] != "RAMP"
        if en_marcha:
            if self.t_regimen is None:
                self.t_regimen = ahora
        else:
            self.t_regimen = None

        estable = self.t_regimen is not None and ahora - self.t_regimen >= C.ESPERA_REGIMEN_S
        condicion = estable and tel["rpm_t"] > 5.0 and tel["err"] > C.LIMITE_ERROR_VEL_PCT
        if condicion:
            self.t_cond_vel = self.t_cond_vel or ahora
        else:
            self.t_cond_vel = None

        nueva = ((self.t_cond_vel is not None and ahora - self.t_cond_vel >= C.PERSISTENCIA_VEL_S)
                 or tel["state"] == "DIVERG")

        if nueva and not self.div_vel:
            extra = " [falla simulada]" if tel["simulada"] else ""
            self._evento(f"Divergencia de velocidad: real {tel['vel_r']:.2f} cm/s, "
                         f"modelo {tel['vel_t']:.2f} cm/s (error {tel['err']:.0f} %){extra}", "alarma")
        elif self.div_vel and not nueva:
            self._evento("Velocidad de nuevo dentro de tolerancia", "ok")
        self.div_vel = nueva

    # ------------------------------------------------------------
    # Modelo: posicion del objeto
    # ------------------------------------------------------------
    def _modelo_posicion(self, tel, signo, dt, ahora):
        pos = tel["pos"]
        nueva = False
        if pos is None:
            self.pos_modelo = None
            self.err_pos = None
            self.t_cond_pos = None
        else:
            extremo = (pos < C.MARGEN_EXTREMOS_CM or
                       pos > C.LARGO_CINTA_CM - C.MARGEN_EXTREMOS_CM)
            self.lim_pos = C.LIMITE_POS_BASE_CM + C.LIMITE_POS_POR_VEL_S * abs(tel["vel_t"])
            if (self.pos_modelo is None or signo == 0 or
                    tel["dir"] != self.dir_prev or extremo):
                # re-sincronizacion del modelo con la medicion
                self.pos_modelo = pos
                self.err_pos = 0.0
                self.t_cond_pos = None
            else:
                self.pos_modelo += signo * tel["vel_t"] * dt
                self.pos_modelo = min(max(self.pos_modelo, 0.0), C.LARGO_CINTA_CM)
                self.err_pos = pos - self.pos_modelo
                if abs(self.err_pos) > self.lim_pos:
                    self.t_cond_pos = self.t_cond_pos or ahora
                else:
                    self.t_cond_pos = None
                nueva = (self.t_cond_pos is not None and
                         ahora - self.t_cond_pos >= C.PERSISTENCIA_POS_S)

        if nueva and not self.div_pos:
            extra = " [falla simulada]" if tel["simulada"] else ""
            self._evento(f"Divergencia de posición: medido {pos:.1f} cm, "
                         f"modelo {self.pos_modelo:.1f} cm{extra}", "alarma")
        elif self.div_pos and not nueva:
            if pos is None:
                self._evento("El objeto salió del rango del sensor; se reinicia el modelo de posición")
            else:
                self._evento("Posición de nuevo coherente con el modelo", "ok")
        self.div_pos = nueva

    # ------------------------------------------------------------
    # Estado para la interfaz
    # ------------------------------------------------------------
    def estado(self):
        with self.lock:
            ahora = time.time()
            return {
                "ahora": ahora,
                "conectado": self.conectado,
                "descripcion": self.tr.descripcion,
                "edad": None if self.ultimo_rx is None else ahora - self.ultimo_rx,
                "ultima": dict(self.ultima) if self.ultima else None,
                "pos_modelo": self.pos_modelo,
                "err_pos": self.err_pos,
                "lim_pos": self.lim_pos,
                "div_vel": self.div_vel,
                "div_pos": self.div_pos,
                "latencia": self.latencias[-1] if self.latencias else None,
                "latencia_prom": (sum(self.latencias) / len(self.latencias)) if self.latencias else None,
                "fallas": {k: v is not None for k, v in self.fallas.items()},
                "eventos": list(self.eventos)[-14:][::-1],
                "serie": list(self.serie),
                "fase": self.fase_banda,
            }
