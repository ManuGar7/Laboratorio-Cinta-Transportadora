# Gemelo digital de la cinta transportadora

Interfaz web (Python + Dash) que se comunica con la ESP32, compara el modelo
cinemático con las mediciones del encoder y del HC-SR04 y detecta divergencias.

## Instalación

    pip install -r requirements.txt

## Uso

1. Elegir la conexión en `config.py` (`MODO`):
   - `"simulado"`: sin hardware, para probar la interfaz.
   - `"serial"`: USB o Bluetooth. Poner en `PUERTO_SERIE` el COM correspondiente.
   - `"tcp"`: cliente-servidor por Wi-Fi (cuando la ESP32 tenga servidor TCP).
2. Ejecutar `python app.py`.
3. Abrir http://127.0.0.1:8050 en el navegador.

## Archivos

- `config.py`: conexión, parámetros físicos y umbrales de divergencia.
- `comunicacion.py`: transportes serie, TCP y selección según `MODO`.
- `simulador.py`: imita a la ESP32 (mismos comandos, respuestas y telemetría).
- `gemelo.py`: hilo de comunicación, modelo digital, divergencias, latencia y fallas simuladas.
- `app.py`: interfaz web.
- `assets/estilo.css`: estilos (Dash lo carga automáticamente).
