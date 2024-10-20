# frontend/app.py

import dash
from dash import html, dcc, Output, Input, State, MATCH, ALL
import dash_bootstrap_components as dbc
import plotly.graph_objs as go
from dash.exceptions import PreventUpdate
import flask
from system_stats import SystemStats
import socket
import threading

# Initialize the SystemStats class and start collecting data
system_stats = SystemStats()
system_stats.start()

system_stats_thread = threading.Thread(target=system_stats.start, daemon=True)
system_stats_thread.start()

# Function to get local IP address
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        s.connect(('8.8.8.8', 1))
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = 'localhost'
    return local_ip

# Initialize Dash app with external scripts for Plotly
app = dash.Dash(__name__, 
                external_stylesheets=[dbc.themes.BOOTSTRAP], 
                external_scripts=['https://cdn.plot.ly/plotly-latest.min.js'],  # Add Plotly script
                suppress_callback_exceptions=True)
app.title = "ROS2 Robot HMI Interface"


# Navbar Layout
navbar = dbc.NavbarSimple(
    brand="ROS2 HMI Interface",
    brand_href="/",
    color="primary",
    dark=True,
    children=[
        dbc.NavItem(dbc.NavLink("Home", href="/")),
        dbc.NavItem(dbc.NavLink("System Metrics", href="/metrics")),
        dbc.DropdownMenu(
            label="Topics",
            children=[
                dbc.DropdownMenuItem("Loading...", header=True, id="loading-dropdown")
            ],
            nav=True,
            in_navbar=True,
            id="topics-dropdown-menu"
        ),
    ]
)

local_ip = get_local_ip()

app.layout = dbc.Container([
    html.Script(f"const local_ip = '{local_ip}';", type='text/javascript'),
    dcc.Location(id='url', refresh=False),
    navbar,
    html.Div(id='page-content')
], fluid=True)

# Home Page Layout with SLAM map, camera feed, and control panel
home_layout = dbc.Container([

    dbc.Row([
        dbc.Col([
            html.H3("Live Camera Feed"),
            html.Img(id="camera-feed", style={"width": "100%", "height": "600px"}),
        ], width=6),
        dbc.Col([
            html.H3("SLAM Map"),
            dcc.Graph(id="map-graph", style={"width": "100%", "height": "600px"})
        ], width=6),
    ]),
    dbc.Row([
        dbc.Col([
            html.H3("ROS2 Terminal Interface"),
            html.Iframe(id="terminal-iframe", src=f"http://{local_ip}:8080", style={"width": "100%", "height": "400px", "border": "1px solid #ccc"}),
        ], width=12),
    ]),
    dbc.Row([
        dbc.Col([
            html.Div(id='control-status', children="", style={"marginTop": "20px", "textAlign": "center", "fontWeight": "bold"}),
        ], width=12),
    ])
], fluid=True)

# Metrics Page Layout
metrics_layout = dbc.Container([
    dbc.Row([dbc.Col([html.H1("System Metrics")], width=12)]),
    dbc.Row([
        dbc.Col([html.H4("CPU Usage"), dcc.Graph(id="cpu-gauge")], width=6),
        dbc.Col([html.H4("Memory Usage"), dcc.Graph(id="memory-gauge")], width=6),
    ]),
    dbc.Row([
        dbc.Col([html.H4("CPU Core Usage"), dcc.Graph(id="cpu-core-line")], width=6),
        dbc.Col([html.H4("Network Traffic"), dcc.Graph(id="network-line")], width=6),
    ]),
    dbc.Row([
        dbc.Col([html.H4("Disk Usage"), dcc.Graph(id="disk-usage-gauge")], width=6),
        dbc.Col([html.H4("CPU Temperature"), dcc.Graph(id="cpu-temp-gauge")], width=6),
    ]),
    dcc.Interval(id='metrics-interval', interval=1000, n_intervals=0)
], fluid=True)

# Layout and navigation
app.layout = dbc.Container([
    dcc.Location(id='url', refresh=False),
    navbar,
    html.Div(id='page-content')
], fluid=True)

# Callback to display page content based on URL
@app.callback(Output('page-content', 'children'), [Input('url', 'pathname')])
def display_page(pathname):
    if pathname == '/metrics':
        return metrics_layout
    else:
        return home_layout

# Callback to update system metrics
@app.callback([
    Output('cpu-gauge', 'figure'),
    Output('memory-gauge', 'figure'),
    Output('cpu-core-line', 'figure'),
    Output('network-line', 'figure'),
    Output('disk-usage-gauge', 'figure'),
    Output('cpu-temp-gauge', 'figure'),
], [Input('metrics-interval', 'n_intervals')])
def update_metrics(n):
    stats = system_stats.get_stats()

    cpu_gauge = go.Figure(go.Indicator(mode="gauge+number", value=sum(stats['cpu_percent']) / len(stats['cpu_percent']), title={"text": "CPU Usage (%)"}, gauge={'axis': {'range': [None, 100]}}))
    memory_gauge = go.Figure(go.Indicator(mode="gauge+number", value=stats['memory_percent'], title={"text": "Memory Usage (%)"}, gauge={'axis': {'range': [None, 100]}}))
    cpu_core_line = go.Figure(go.Scatter(y=stats['cpu_percent'], mode='lines+markers', name='CPU Core Usage', line_shape='spline'))
    cpu_core_line.update_layout(title="CPU Core Usage Over Time", xaxis_title="Core", yaxis_title="Usage (%)")
    network_line = go.Figure(go.Scatter(y=[stats['net_io'].bytes_sent, stats['net_io'].bytes_recv], mode='lines+markers', name='Network Traffic', line_shape='spline'))
    disk_gauge = go.Figure(go.Indicator(mode="gauge+number", value=stats['disk_usage_percent'], title={"text": "Disk Usage (%)"}, gauge={'axis': {'range': [None, 100]}}))
    cpu_temp_gauge = go.Figure(go.Indicator(mode="gauge+number", value=stats['temperature'] if stats['temperature'] else 0, title={"text": "CPU Temperature (°C)"}, gauge={'axis': {'range': [None, 100]}}))

    return cpu_gauge, memory_gauge, cpu_core_line, network_line, disk_gauge, cpu_temp_gauge

if __name__ == '__main__':
    app.run_server(debug=True, host="0.0.0.0", port=8050)