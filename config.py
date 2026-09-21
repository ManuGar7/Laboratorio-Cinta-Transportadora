"""
Configuracion del Gemelo Digital de la cinta transportadora.
Todo lo que normalmente hay que tocar esta en este archivo.
"""

# -----------------------------------------------------------------
# CONEXION CON LA ESP32
# -----------------------------------------------------------------
# "serial"   -> USB o Bluetooth (ambos aparecen como puerto COM)
# "tcp"      -> cliente-servidor por Wi-Fi (cuando la ESP32 tenga servidor TCP)
# "simulado" -> sin hardware: un simulador responde como la ESP32
MODO = "simulado"

# USB: el COM del CP2102 (Administrador de dispositivos).
# Bluetooth: el COM *saliente* de CINTA_UTEC
#   (Configuracion > Bluetooth > Mas opciones de Bluetooth > Puertos COM).
# Linux: "/dev/ttyUSB0" (USB) o "/dev/rfcomm0" (Bluetooth).
PUERTO_SERIE = "COM9"
BAUDIOS = 115200

# Para el modo "tcp" (futuro cliente-servidor)
TCP_HOST = "192.168.4.1"
TCP_PUERTO = 3333

# -----------------------------------------------------------------
# PARAMETROS FISICOS (deben coincidir con el firmware)
# -----------------------------------------------------------------
MAX_MOTOR_RPM = 150.0
RELACION = 19.0 / 40.0          # engranaje motor / engranaje rodillo
DIAMETRO_RODILLO_CM = 2.95
LARGO_CINTA_CM = 45.0
ACCEL_RPM_S = 120.0
DECEL_RPM_S = 300.0

# +1 si al ir ADELANTE (F) el objeto se aleja del HC-SR04 (la posicion aumenta).
# -1 si al ir ADELANTE el objeto se acerca al sensor.
SIGNO_POS_FWD = +1

# -----------------------------------------------------------------
# DETECCION DE DIVERGENCIAS EN EL GEMELO
# -----------------------------------------------------------------
# Velocidad: error entre RPM real (encoder) y teorica (modelo)
LIMITE_ERROR_VEL_PCT = 15.0
ESPERA_REGIMEN_S = 1.5          # tiempo en regimen antes de evaluar
PERSISTENCIA_VEL_S = 1.0        # la condicion debe mantenerse este tiempo

# Posicion: objeto medido por el HC-SR04 vs posicion predicha por el modelo
# Limite = base + (por_vel x velocidad). El termino proporcional absorbe
# el retardo del filtro exponencial del HC-SR04.
LIMITE_POS_BASE_CM = 3.0
LIMITE_POS_POR_VEL_S = 0.3
PERSISTENCIA_POS_S = 0.6
MARGEN_EXTREMOS_CM = 1.5        # cerca de los extremos se re-sincroniza el modelo

# -----------------------------------------------------------------
# INTERFAZ WEB
# -----------------------------------------------------------------
HISTORIAL_S = 60                # segundos visibles en las tendencias
REFRESCO_MS = 300
HOST_WEB = "127.0.0.1"          # "0.0.0.0" para abrirla desde otro equipo de la red
PUERTO_WEB = 8050
