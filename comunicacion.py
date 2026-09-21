"""
Capa de comunicacion con la ESP32.

Todas las clases exponen la misma interfaz, asi el resto del programa
no sabe (ni le importa) si los datos llegan por USB, Bluetooth, TCP
o desde el simulador:

    conectar()      abre la conexion (lanza excepcion si falla)
    leer_linea()    devuelve una linea de texto, o None si no llego nada
    enviar(texto)   envia un comando (se agrega el salto de linea)
    cerrar()
"""

import socket
import time

import config as C

try:
    import serial  # pyserial
except ImportError:  # se informa recien al intentar conectar
    serial = None


class Transporte:
    descripcion = ""

    def conectar(self):
        raise NotImplementedError

    def leer_linea(self):
        raise NotImplementedError

    def enviar(self, texto):
        raise NotImplementedError

    def cerrar(self):
        pass


class TransporteSerial(Transporte):
    """USB (CP2102) o Bluetooth SPP: en la PC ambos son un puerto COM."""

    def __init__(self, puerto, baudios):
        self.puerto = puerto
        self.baudios = baudios
        self.ser = None
        self.descripcion = f"Serie {puerto}"

    def conectar(self):
        if serial is None:
            raise RuntimeError("falta pyserial (pip install pyserial)")
        self.ser = serial.Serial(self.puerto, self.baudios, timeout=0.2)
        # Por USB, abrir el puerto reinicia la ESP32: esperamos que arranque.
        time.sleep(2.0)
        self.ser.reset_input_buffer()

    def leer_linea(self):
        datos = self.ser.readline()
        if not datos:
            return None
        return datos.decode("utf-8", errors="ignore").strip()

    def enviar(self, texto):
        self.ser.write((texto + "\n").encode("utf-8"))

    def cerrar(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None


class TransporteTCP(Transporte):
    """Cliente TCP, para cuando la ESP32 funcione como servidor por Wi-Fi.
    El protocolo es el mismo: lineas de texto terminadas en '\\n'."""

    def __init__(self, host, puerto):
        self.host = host
        self.puerto = puerto
        self.sock = None
        self.buffer = b""
        self.descripcion = f"TCP {host}:{puerto}"

    def conectar(self):
        self.sock = socket.create_connection((self.host, self.puerto), timeout=5)
        self.sock.settimeout(0.2)
        self.buffer = b""

    def leer_linea(self):
        while b"\n" not in self.buffer:
            try:
                datos = self.sock.recv(1024)
            except socket.timeout:
                return None
            if not datos:
                raise ConnectionError("el servidor cerro la conexion")
            self.buffer += datos
        linea, self.buffer = self.buffer.split(b"\n", 1)
        return linea.decode("utf-8", errors="ignore").strip()

    def enviar(self, texto):
        self.sock.sendall((texto + "\n").encode("utf-8"))

    def cerrar(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None


def crear_transporte():
    if C.MODO == "serial":
        return TransporteSerial(C.PUERTO_SERIE, C.BAUDIOS)
    if C.MODO == "tcp":
        return TransporteTCP(C.TCP_HOST, C.TCP_PUERTO)
    if C.MODO == "simulado":
        from simulador import TransporteSimulado
        return TransporteSimulado()
    raise ValueError(f"MODO desconocido en config.py: {C.MODO!r}")
