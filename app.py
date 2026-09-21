"""
Gemelo Digital de la cinta transportadora - interfaz web (Dash).

Uso:
    pip install -r requirements.txt
    python app.py
y abrir http://127.0.0.1:8050 en el navegador.

La conexion (simulado / USB / Bluetooth / TCP) se elige en config.py.
"""

import time

import plotly.graph_objects as go
from dash import Dash, Input, Output, State, ctx, dcc, html

import config as C
from comunicacion import crear_transporte
from gemelo import FALLAS, Gemelo

# Paletas para los graficos (mismos colores que assets/estilo.css).
# Criterio de HMI industrial: grises para lo normal, color solo para
# lo que requiere atencion. En ambos modos se respeta el mismo criterio.
PALETAS = {
    "claro": {
        "texto": "#1E2830", "texto2": "#56626C", "estructura": "#6E7880",
        "banda": "#3B434A", "marca": "#9AA3AA", "pieza": "#E4E7EA",
        "real": "#1F5A85", "modelo": "#7A8793", "alarma": "#B8322A", "grilla": "#D3D7DB",
    },
    "oscuro": {
        "texto": "#E3E7EA", "texto2": "#A3ADB5", "estructura": "#8A949C",
        "banda": "#AEB6BD", "marca": "#4A535B", "pieza": "#3A4148",
        "real": "#6FA8D6", "modelo": "#96A2AD", "alarma": "#E0564B", "grilla": "#3A4148",
    },
}
FUENTE = "Barlow, 'Segoe UI', Roboto, Arial, sans-serif"

DIRECCIONES = {"FWD": "adelante", "REV": "reversa", "STOP": "detenida"}

gemelo = Gemelo(crear_transporte())
gemelo.iniciar()

app = Dash(
    __name__,
    title="Gemelo digital de la cinta",
    external_stylesheets=[
        "https://fonts.googleapis.com/css2?family=Barlow:wght@400;500;600;700&display=swap"
    ],
)


# =================================================================
# LAYOUT
# =================================================================
def boton(texto, id_, clase="btn"):
    return html.Button(texto, id=id_, n_clicks=0, className=clase)


app.layout = html.Div(className="pagina", children=[
    html.Header(className="cabecera", children=[
        html.Div([
            html.H1("Gemelo digital de la cinta transportadora"),
            html.P("ESP32 + TMC2208 + NEMA 17. Modelo cinemático contra medición del encoder y del HC-SR04.",
                   className="bajada"),
        ]),
        html.Div(className="cabecera-der", children=[
            html.Div(id="conexion", className="con"),
            html.Button(id="btn-tema", n_clicks=0, className="btn-tema"),
        ]),
    ]),

    html.Section(className="panel vista", children=[
        dcc.Graph(id="vista-banda", config={"displayModeBar": False, "staticPlot": True},
                  style={"height": "250px"}),
        html.Div(id="resumen", className="resumen"),
    ]),

    html.Div(className="fila3", children=[
        html.Section(className="panel", children=[
            html.H2("Mando"),
            html.Div(className="botonera", children=[
                boton("Adelante", "btn-f"),
                boton("Reversa", "btn-r"),
                boton("Detener", "btn-s"),
                boton("Paro inmediato", "btn-e", "btn btn-paro"),
            ]),
            html.Label("Velocidad (% de 150 RPM de motor)", htmlFor="vel", className="etiqueta"),
            dcc.Slider(id="vel", min=0, max=100, step=5, value=50,
                       marks={0: "0", 25: "25", 50: "50", 75: "75", 100: "100"},
                       updatemode="mouseup"),
            html.Label("Micropaso", htmlFor="micro", className="etiqueta"),
            dcc.Dropdown(id="micro", clearable=False, searchable=False, value=8,
                         options=[{"label": f"1/{n}", "value": n} for n in (2, 4, 8, 16)]),
            html.P("El micropaso solo se cambia con la cinta detenida.", className="ayuda"),
            html.P(id="mando-respuesta", className="respuesta"),
        ]),

        html.Section(className="panel", children=[
            html.H2("Medición y modelo"),
            html.Table(id="tabla", className="tabla"),
        ]),

        html.Section(className="panel", children=[
            html.H2("Divergencias"),
            html.Div(id="alarmas"),
            html.H3("Simulación de fallas"),
            dcc.Checklist(
                id="fallas", className="fallas", value=[],
                options=[{"label": nombre, "value": clave} for clave, nombre in FALLAS.items()],
            ),
            html.P("Se aplican sobre los datos recibidos; el hardware no cambia.", className="ayuda"),
        ]),
    ]),

    html.Div(className="fila2", children=[
        html.Section(className="panel", children=[
            html.H2("Velocidad de la banda"),
            dcc.Graph(id="graf-vel", config={"displayModeBar": False}, style={"height": "260px"}),
        ]),
        html.Section(className="panel", children=[
            html.H2("Posición del objeto"),
            dcc.Graph(id="graf-pos", config={"displayModeBar": False}, style={"height": "260px"}),
        ]),
    ]),

    html.Section(className="panel", children=[
        html.H2("Registro de eventos"),
        html.Ul(id="registro", className="registro"),
    ]),

    dcc.Interval(id="tick", interval=C.REFRESCO_MS),
    # "tema": eleccion guardada en el navegador (None = seguir al sistema operativo)
    # "tema-efectivo": el que se esta usando realmente
    dcc.Store(id="tema", storage_type="local", data=None),
    dcc.Store(id="tema-efectivo", data="claro"),
])


# =================================================================
# AUXILIARES
# =================================================================
def fmt(x, dec=1, sufijo=""):
    return "—" if x is None else f"{x:.{dec}f}{sufijo}"


def fmt_signo(x, dec=1, sufijo=""):
    return "—" if x is None else f"{x:+.{dec}f}{sufijo}"


def figura_banda(est, P):
    L = C.LARGO_CINTA_CM
    R = 1.6
    u = est["ultima"]
    formas = []
    notas = []

    # Rodillos (el motriz y el conducido)
    for xc in (0.0, L):
        formas.append(dict(type="circle", x0=xc - R, x1=xc + R, y0=-R, y1=R,
                           line=dict(color=P["estructura"], width=2), fillcolor=P["pieza"]))
    # Tramos superior e inferior de la banda
    for y in (R, -R):
        formas.append(dict(type="rect", x0=0, x1=L, y0=y - 0.35, y1=y + 0.35,
                           fillcolor=P["banda"], line_width=0))
    # Marcas que se desplazan con la banda
    for k in range(-1, int(L / 5) + 2):
        x = k * 5 + est["fase"]
        if 0.4 < x < L - 0.4:
            formas.append(dict(type="line", x0=x, x1=x, y0=R - 0.35, y1=R + 0.35,
                               line=dict(color=P["marca"], width=2)))

    # HC-SR04 en el extremo de la posicion 0
    formas.append(dict(type="rect", x0=-6.2, x1=-3.6, y0=R + 0.5, y1=R + 3.3,
                       fillcolor=P["pieza"], line=dict(color=P["estructura"], width=2)))
    notas.append(dict(x=-4.9, y=R + 4.3, text="HC-SR04", showarrow=False,
                      font=dict(size=12, color=P["texto2"])))
    notas.append(dict(x=L, y=-R - 2.0, text="encoder", showarrow=False,
                      font=dict(size=11, color=P["texto2"])))

    if u is None:
        notas.append(dict(x=L / 2, y=R + 4, text="Esperando telemetría de la ESP32",
                          showarrow=False, font=dict(size=15, color=P["texto2"])))
    else:
        pos = u["pos"]
        pm = est["pos_modelo"]
        if pos is not None:
            y_haz = R + 1.9
            formas.append(dict(type="line", x0=-3.6, x1=pos, y0=y_haz, y1=y_haz,
                               line=dict(color=P["marca"], width=1.5, dash="dot")))
            borde = P["alarma"] if est["div_pos"] else P["real"]
            formas.append(dict(type="rect", x0=pos, x1=pos + 4, y0=R + 0.35, y1=R + 3.9,
                               fillcolor=P["real"], line=dict(color=borde, width=3)))
            notas.append(dict(x=pos + 2, y=R + 5.0, text=f"medido {pos:.1f} cm", showarrow=False,
                              font=dict(size=13, color=P["real"])))
        if pm is not None and pos is not None and abs(pm - pos) > 0.3:
            formas.append(dict(type="rect", x0=pm, x1=pm + 4, y0=R + 0.35, y1=R + 3.9,
                               fillcolor="rgba(0,0,0,0)",
                               line=dict(color=P["modelo"], width=2, dash="dash")))
            if abs(pm - pos) > 5:
                notas.append(dict(x=pm + 2, y=R + 5.0, text=f"modelo {pm:.1f} cm", showarrow=False,
                                  font=dict(size=13, color=P["modelo"])))
        if pos is None:
            notas.append(dict(x=L / 2, y=R + 4, text="Sin objeto en el rango del sensor",
                              showarrow=False, font=dict(size=13, color=P["texto2"])))

        # Sentido de avance
        signo = {"FWD": C.SIGNO_POS_FWD, "REV": -C.SIGNO_POS_FWD}.get(u["dir"], 0)
        if signo != 0:
            x0, x1 = (L * 0.35, L * 0.65) if signo > 0 else (L * 0.65, L * 0.35)
            notas.append(dict(x=x1, y=-R - 2.2, ax=x0, ay=-R - 2.2, xref="x", yref="y",
                              axref="x", ayref="y", showarrow=True, arrowhead=2, arrowsize=1.2,
                              arrowwidth=2, arrowcolor=P["texto2"], text=""))
            notas.append(dict(x=L / 2, y=-R - 3.6, showarrow=False,
                              text=f"{DIRECCIONES[u['dir']]}, {u['vel_r']:.1f} cm/s",
                              font=dict(size=12, color=P["texto2"])))

    fig = go.Figure()
    fig.update_layout(
        shapes=formas, annotations=notas,
        xaxis=dict(range=[-8, L + 3], visible=False, fixedrange=True),
        yaxis=dict(range=[-6.2, 8.5], visible=False, fixedrange=True, scaleanchor="x", scaleratio=1),
        margin=dict(l=4, r=4, t=4, b=4),
        paper_bgcolor="rgba(0,0,0,0)", plot_bgcolor="rgba(0,0,0,0)",
        font=dict(family=FUENTE),
    )
    return fig


def figura_tendencia(est, cual, P):
    ahora = est["ahora"]
    serie = [s for s in est["serie"] if ahora - s[0] <= C.HISTORIAL_S]
    t = [s[0] - ahora for s in serie]
    fig = go.Figure()

    if cual == "vel":
        modelo = [s[1] for s in serie]
        real = [s[2] for s in serie]
        marcas = [(ti, s[2]) for ti, s in zip(t, serie) if s[5]]
        titulo_y, rango = "cm/s", None
    else:
        modelo = [s[4] for s in serie]
        real = [s[3] for s in serie]
        marcas = [(ti, s[3]) for ti, s in zip(t, serie) if s[6] and s[3] is not None]
        titulo_y, rango = "cm", [0, C.LARGO_CINTA_CM]

    fig.add_trace(go.Scatter(x=t, y=modelo, name="modelo", mode="lines",
                             line=dict(color=P["modelo"], width=2, dash="dash")))
    fig.add_trace(go.Scatter(x=t, y=real, name="medido", mode="lines",
                             line=dict(color=P["real"], width=2.5)))
    if marcas:
        fig.add_trace(go.Scatter(x=[m[0] for m in marcas], y=[m[1] for m in marcas],
                                 name="divergencia", mode="markers",
                                 marker=dict(color=P["alarma"], size=6)))

    fig.update_layout(
        margin=dict(l=48, r=12, t=8, b=40),
        paper_bgcolor="rgba(0,0,0,0)", plot_bgcolor="rgba(0,0,0,0)",
        font=dict(family=FUENTE, size=12, color=P["texto"]),
        legend=dict(orientation="h", x=0, y=1.12, bgcolor="rgba(0,0,0,0)"),
        xaxis=dict(title="segundos", range=[-C.HISTORIAL_S, 0], gridcolor=P["grilla"], zeroline=False),
        yaxis=dict(title=titulo_y, range=rango, gridcolor=P["grilla"], zeroline=False,
                   rangemode="tozero"),
        uirevision=cual,
    )
    return fig


def fila_tabla(nombre, real, modelo, dif, alarma=False):
    return html.Tr([
        html.Th(nombre, scope="row"),
        html.Td(real, className="real"),
        html.Td(modelo, className="modelo"),
        html.Td(dif, className="dif alarma-txt" if alarma else "dif"),
    ])


def tabla(est):
    u = est["ultima"] or {}
    g = u.get
    cabecera = html.Thead(html.Tr([html.Th(""), html.Th("Medido"), html.Th("Modelo"),
                                   html.Th("Diferencia")]))
    dif_rpm = None if not u else g("rpm_r") - g("rpm_t")
    filas = [
        fila_tabla("Velocidad de la banda (cm/s)", fmt(g("vel_r"), 2), fmt(g("vel_t"), 2),
                   fmt(g("err"), 1, " %"), est["div_vel"]),
        fila_tabla("Giro del rodillo (RPM)", fmt(g("rpm_r")), fmt(g("rpm_t")),
                   fmt_signo(dif_rpm), est["div_vel"]),
        fila_tabla("Posición del objeto (cm)", fmt(g("pos")), fmt(est["pos_modelo"]),
                   fmt_signo(est["err_pos"]), est["div_pos"]),
        fila_tabla("Motor, comandado (RPM)", "", fmt(g("rpm_m")), ""),
        fila_tabla("Distancia bruta HC-SR04 (cm)", fmt(g("dist")), "", ""),
    ]
    return [cabecera, html.Tbody(filas)]


def resumen(est):
    u = est["ultima"]
    if not u:
        return []
    datos = [
        ("Sentido", DIRECCIONES.get(u["dir"], u["dir"])),
        ("Consigna", f"{u['v']:.0f} %"),
        ("Micropaso", f"1/{u['m']:.0f}"),
        ("Estado de la ESP32", {"OK": "normal", "RAMP": "en rampa", "DIVERG": "divergencia"}
         .get(u["state"], u["state"])),
    ]
    if u.get("simulada"):
        datos.append(("Datos", "con falla simulada"))
    return [html.Div([html.Span(k, className="k"), html.Span(v, className="v")], className="dato")
            for k, v in datos]


def insignia(texto, tipo):
    return html.Span(texto, className=f"estado {tipo}")


def alarmas(est):
    u = est["ultima"]
    bloques = []

    # Velocidad
    if not u:
        vel = (insignia("Sin datos", "inactivo"), "")
    elif est["div_vel"]:
        vel = (insignia("Divergencia", "alarma"),
               f"Error {u['err']:.0f} %. Revisar pérdida de pasos, engranajes o atasco de la banda.")
    elif u["state"] == "RAMP":
        vel = (insignia("En rampa", "aviso"), "No se evalúa hasta alcanzar el régimen.")
    elif u["dir"] == "STOP":
        vel = (insignia("Detenida", "inactivo"), "")
    else:
        vel = (insignia("Dentro de tolerancia", "normal"),
               f"Error {u['err']:.1f} % (límite {C.LIMITE_ERROR_VEL_PCT:.0f} %).")
    bloques.append(("Velocidad", *vel))

    # Posicion
    if not u or u["pos"] is None:
        pos = (insignia("Sin objeto", "inactivo"), "")
    elif est["div_pos"]:
        pos = (insignia("Divergencia", "alarma"),
               f"Desvío {est['err_pos']:+.1f} cm. Posible deslizamiento de la banda o del objeto.")
    else:
        lim = est["lim_pos"]
        pos = (insignia("Coherente", "normal"),
               f"Desvío {fmt_signo(est['err_pos'])} cm (límite ±{lim:.1f} cm)." if lim else "")
    bloques.append(("Posición", *pos))

    # Comunicacion
    edad = est["edad"]
    if not est["conectado"]:
        com = (insignia("Desconectada", "alarma"), "Reintentando conexión.")
    elif edad is None or edad > 3:
        com = (insignia("Sin datos", "alarma"),
               "La ESP32 no envía telemetría." if edad is None else f"Último dato hace {edad:.0f} s.")
    else:
        lat = est["latencia"]
        com = (insignia("Normal", "normal"),
               "Latencia de comandos: " + ("sin medir aún" if lat is None else
                                           f"{lat:.0f} ms (promedio {est['latencia_prom']:.0f} ms)"))
    bloques.append(("Comunicación", *com))

    return [html.Div(className="alarma-fila", children=[
        html.Div([html.Span(nombre, className="alarma-nombre"), ins]),
        html.P(detalle, className="alarma-detalle") if detalle else None,
    ]) for nombre, ins, detalle in bloques]


def conexion(est):
    edad = est["edad"]
    if est["conectado"] and edad is not None and edad < 3:
        return f"Conectado: {est['descripcion']}", "con ok"
    if est["conectado"]:
        return f"Conectado sin datos: {est['descripcion']}", "con aviso"
    return f"Buscando la ESP32: {est['descripcion']}", "con error"


def registro(est):
    items = []
    for t, tipo, texto in est["eventos"]:
        items.append(html.Li([html.Span(time.strftime("%H:%M:%S", time.localtime(t)), className="hora"),
                              html.Span(texto)], className=f"ev-{tipo}"))
    return items


# =================================================================
# CALLBACKS
# =================================================================
@app.callback(
    Output("conexion", "children"), Output("conexion", "className"),
    Output("vista-banda", "figure"), Output("resumen", "children"),
    Output("tabla", "children"), Output("alarmas", "children"),
    Output("graf-vel", "figure"), Output("graf-pos", "figure"),
    Output("registro", "children"),
    Input("tick", "n_intervals"), Input("tema-efectivo", "data"),
)
def actualizar(_, tema):
    P = PALETAS.get(tema, PALETAS["claro"])
    est = gemelo.estado()
    texto_con, clase_con = conexion(est)
    return (texto_con, clase_con, figura_banda(est, P), resumen(est), tabla(est), alarmas(est),
            figura_tendencia(est, "vel", P), figura_tendencia(est, "pos", P), registro(est))


@app.callback(
    Output("mando-respuesta", "children"),
    Input("btn-f", "n_clicks"), Input("btn-r", "n_clicks"),
    Input("btn-s", "n_clicks"), Input("btn-e", "n_clicks"),
    Input("vel", "value"), Input("micro", "value"),
    prevent_initial_call=True,
)
def mandar(_f, _r, _s, _e, vel, micro):
    origen = ctx.triggered_id
    comandos = {"btn-f": "F", "btn-r": "R", "btn-s": "S", "btn-e": "E"}
    if origen in comandos:
        cmd = comandos[origen]
    elif origen == "vel":
        cmd = f"V{int(vel)}"
    elif origen == "micro":
        cmd = f"M{int(micro)}"
    else:
        return ""
    return f"Enviado {cmd}" if gemelo.enviar(cmd) else f"Sin conexión: {cmd} no se envió"


@app.callback(Output("fallas", "className"), Input("fallas", "value"))
def fallas(valores):
    gemelo.set_fallas(valores or [])
    return "fallas activas" if valores else "fallas"


# Tema: se resuelve en el navegador (sin ir al servidor) y se aplica a la pagina
app.clientside_callback(
    """
    function(tema) {
        if (!tema) {
            tema = window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches
                ? "oscuro" : "claro";
        }
        document.documentElement.setAttribute("data-theme", tema);
        return [tema, tema === "oscuro" ? "Modo claro" : "Modo oscuro"];
    }
    """,
    Output("tema-efectivo", "data"), Output("btn-tema", "children"),
    Input("tema", "data"),
)


@app.callback(Output("tema", "data"), Input("btn-tema", "n_clicks"),
              State("tema-efectivo", "data"), prevent_initial_call=True)
def cambiar_tema(_, actual):
    return "claro" if actual == "oscuro" else "oscuro"


if __name__ == "__main__":
    # debug=False: el modo debug abre dos procesos y el puerto serie no se puede abrir dos veces.
    app.run(host=C.HOST_WEB, port=C.PUERTO_WEB, debug=False)
