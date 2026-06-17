"""
Hardware-in-the-Loop (HIL) Motor Control Telemetry Dashboard.
Provides a modern CustomTkinter interface to dynamically inject PID and 
Digital Compensator parameters into an STM32 microcontroller via USART.
Features asynchronous threading for non-blocking serial communication and 
real-time Oscilloscope-style plotting of tracking errors and actuation efforts.

Author: Daniel Ruiz Perez
date 2026-06-15
"""

import customtkinter as ctk # type: ignore
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import sys
import serial # type: ignore
import threading
import serial.tools.list_ports  
import time
from collections import deque   

# -----------------------------------------------------------------------------
# Configuración global de CustomTkinter
# -----------------------------------------------------------------------------
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

class MotorDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("⚙️ Motor Control Dashboard")
        self.geometry("1100x750") 
        
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.grid_columnconfigure(0, weight=1) 
        self.grid_columnconfigure(1, weight=3) 
        self.grid_rowconfigure(0, weight=1)

        self.modo_sistema_actual = "Velocidad"
        self.sistema_corriendo = False  
        self.es_autotuning = False # Bandera estática para el relevador

        # --- DICCIONARIOS ACTUALIZADOS (Sin PID Suavizado) ---
        self.perfiles_velocidad = {
            "Control P":   {"K1": 8.000, "K2": 0.000, "K3": 0.000, "K4": 0.000, "K5": 0.000},
            "Control PI":  {"K1": 12.000, "K2": 10.000, "K3": 0.000, "K4": 0.000, "K5": 0.000},
            "Control PD":  {"K1": 8.000, "K2": 0.000, "K3": 0.150, "K4": 0.000, "K5": 0.000},
            "Control PID": {"K1": 12.250, "K2": 10.700, "K3": 0.700, "K4": 0.000, "K5": 0.000},
            "Adelanto":        {"K1": 15.000, "K2": 5.000, "K3": 0.500, "K4": 0.000, "K5": 0.000},
            "Atraso":          {"K1": 8.079, "K2": -7.919, "K3": -0.9998, "K4": 0.000, "K5": 0.000},
            "Adelanto-Atraso": {"K1": 29.2497, "K2": -42.2139, "K3": 14.2499, "K4": -0.9524, "K5": -0.0476},
            "Sintonizar P":    {"K1": 0.000, "K2": 0.000, "K3": 0.000, "K4": 0.000, "K5": 0.000},
            "Sintonizar PI":   {"K1": 0.000, "K2": 0.000, "K3": 0.000, "K4": 0.000, "K5": 0.000},
            "Sintonizar PD":   {"K1": 0.000, "K2": 0.000, "K3": 0.000, "K4": 0.000, "K5": 0.000},
            "Sintonizar PID":  {"K1": 0.000, "K2": 0.000, "K3": 0.000, "K4": 0.000, "K5": 0.000}
        }
        
        self.perfiles_posicion = {
            "Control P":   {"K1": 13.000, "K2": 0.000, "K3": 0.000, "K4": 0.000, "K5": 0.000},
            "Control PI":  {"K1": 11.920, "K2": 2.130, "K3": 0.000, "K4": 0.000, "K5": 0.000},
            "Control PD":  {"K1": 13.000, "K2": 0.000, "K3": 0.020, "K4": 0.000, "K5": 0.000},
            "Control PID": {"K1": 12.360, "K2": 1.816, "K3": 0.900, "K4": 0.000, "K5": 0.000},
            "Adelanto":        {"K1": 12.24, "K2": 5.466, "K3": 0.455, "K4": 0.000, "K5": 0.000},
            "Atraso":          {"K1": 17.150, "K2": -16.813, "K3": -0.998, "K4": 0.000, "K5": 0.000},
           # "Adelanto-Atraso": {"K1": 107.400, "K2": -193.800, "K3": 87.180, "K4": -1.151, "K5": 0.1495},
            "Adelanto-Atraso": {"K1": 107.400, "K2": -182.800, "K3": 80.180, "K4": -1.151, "K5": 0.1495},
            "Sintonizar P":    {"K1": 0.000, "K2": 0.000, "K3": 0.000, "K4": 0.000, "K5": 0.000},
            "Sintonizar PI":   {"K1": 0.000, "K2": 0.000, "K3": 0.000, "K4": 0.000, "K5": 0.000},
            "Sintonizar PD":   {"K1": 0.000, "K2": 0.000, "K3": 0.000, "K4": 0.000, "K5": 0.000},
            "Sintonizar PID":  {"K1": 0.000, "K2": 0.000, "K3": 0.000, "K4": 0.000, "K5": 0.000}
        }

        self.puerto_serial = None
        self.leyendo_serial = False
        self.ultimos_datos_uart = None 
        self.contador_tx = 0 

        self.crear_panel_izquierdo()
        self.crear_panel_derecho()

    # =========================================================================
    # COLUMNA IZQUIERDA: PANEL DE CONEXIÓN Y ESTADO 
    # =========================================================================
    def crear_panel_izquierdo(self):
        self.frame_izquierdo = ctk.CTkFrame(self, corner_radius=10)
        self.frame_izquierdo.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        self.lbl_titulo_conexion = ctk.CTkLabel(self.frame_izquierdo, text="🔌 CONEXIÓN SERIAL", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_titulo_conexion.pack(pady=(15, 10))

        self.frame_serial = ctk.CTkFrame(self.frame_izquierdo, fg_color="transparent")
        self.frame_serial.pack(fill="x", padx=15)

        self.frame_serial.grid_columnconfigure(0, weight=1)
        self.frame_serial.grid_columnconfigure(1, weight=1)

        self.cmb_puertos = ctk.CTkComboBox(self.frame_serial, values=["Buscando..."])
        self.cmb_puertos.grid(row=0, column=0, padx=(0, 5), pady=5, sticky="ew")

        self.btn_actualizar = ctk.CTkButton(self.frame_serial, text="🔄 Actualizar", 
                                            fg_color="#3B8ED0", hover_color="#36719F", 
                                            command=self.actualizar_puertos)
        self.btn_actualizar.grid(row=0, column=1, padx=(5, 0), pady=5, sticky="ew")

        self.btn_conectar = ctk.CTkButton(self.frame_serial, text="Conectar", command=self.conectar_desconectar)
        self.btn_conectar.grid(row=1, column=0, padx=(0, 5), pady=(10, 5), sticky="ew")

        self.frame_estado = ctk.CTkFrame(self.frame_serial, fg_color="transparent")
        self.frame_estado.grid(row=1, column=1, padx=(5, 0), pady=(10, 5), sticky="e")
        
        self.lbl_texto_estado = ctk.CTkLabel(self.frame_estado, text="Desconectado", text_color="gray60", font=ctk.CTkFont(size=12, weight="bold"))
        self.lbl_texto_estado.pack(side="left", padx=(0, 8))

        self.lbl_estado_conexion = ctk.CTkLabel(self.frame_estado, text="🔴", font=ctk.CTkFont(size=16))
        self.lbl_estado_conexion.pack(side="right")

        self.separador = ctk.CTkFrame(self.frame_izquierdo, height=2, fg_color="gray30")
        self.separador.pack(fill="x", padx=10, pady=(20, 10))

        self.lbl_tipo_sistema = ctk.CTkLabel(self.frame_izquierdo, text="VARIABLE A CONTROLAR", font=ctk.CTkFont(size=12, weight="bold"), text_color="gray60")
        self.lbl_tipo_sistema.pack(pady=(0, 5))

        self.seg_sistema = ctk.CTkSegmentedButton(self.frame_izquierdo, values=["Velocidad (RPM)", "Posición (Grados)"], command=self.cambiar_modo_sistema)
        self.seg_sistema.set("Velocidad (RPM)")
        self.seg_sistema.pack(fill="x", padx=15, pady=(0, 15))

        self.lbl_titulo_control = ctk.CTkLabel(self.frame_izquierdo, text="🎛️ ESTADO DEL CONTROLADOR", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_titulo_control.pack(pady=(5, 10))

        self.tab_control = ctk.CTkTabview(self.frame_izquierdo, command=self.actualizar_ganancias_visuales)
        self.tab_control.pack(fill="both", expand=True, padx=15, pady=(0, 15))

        self.tab_control.add("Clásico")
        self.tab_control.add("Compensador")
        self.tab_control.add("Autotuning")

        # -------------------------------------------------------------
        # PESTAÑA: CLÁSICO
        # -------------------------------------------------------------
        self.cmb_modo_clasico = ctk.CTkComboBox(self.tab_control.tab("Clásico"), values=["Control P", "Control PI", "Control PD", "Control PID"], command=self.actualizar_ganancias_visuales)
        self.cmb_modo_clasico.set("Control PI") 
        self.cmb_modo_clasico.pack(pady=(10, 10), fill="x", padx=20)
        
        self.lbl_info_clasico = ctk.CTkLabel(self.tab_control.tab("Clásico"), text="Ganancias Internas del MCU:", text_color="gray60")
        self.lbl_info_clasico.pack(pady=(0, 5))

        self.frame_vars_clasico = ctk.CTkFrame(self.tab_control.tab("Clásico"), fg_color="transparent")
        self.frame_vars_clasico.pack(expand=True, fill="both", pady=(5, 10))
        self.lbls_clasicos = self.crear_etiquetas_lectura(self.frame_vars_clasico, 
                                                          ["Kp:", "Ki:", "Kd:", "K4 (-):", "K5 (-):"], 
                                                          ["0.000", "0.000", "0.000", "0.000", "0.000"])

        # -------------------------------------------------------------
        # PESTAÑA: COMPENSADOR
        # -------------------------------------------------------------
        self.cmb_modo_comp = ctk.CTkComboBox(self.tab_control.tab("Compensador"), values=["Adelanto", "Atraso", "Adelanto-Atraso"], command=self.actualizar_ganancias_visuales)
        self.cmb_modo_comp.set("Adelanto-Atraso") 
        self.cmb_modo_comp.pack(pady=(10, 10), fill="x", padx=20)
        
        self.lbl_info_comp = ctk.CTkLabel(self.tab_control.tab("Compensador"), text="Coeficientes Digitales:", text_color="gray60")
        self.lbl_info_comp.pack(pady=(0, 5))
        
        self.frame_vars_comp = ctk.CTkFrame(self.tab_control.tab("Compensador"), fg_color="transparent")
        self.frame_vars_comp.pack(expand=True, fill="both", pady=(5, 10))
        self.lbls_comp = self.crear_etiquetas_lectura(self.frame_vars_comp, 
                                                      ["A:", "B:", "C:", "D:", "E:"], 
                                                      ["0.000", "0.000", "0.000", "0.000", "0.000"])

        # -------------------------------------------------------------
        # PESTAÑA: AUTOTUNING (Sin PID Suavizado)
        # -------------------------------------------------------------
        self.cmb_modo_auto = ctk.CTkComboBox(self.tab_control.tab("Autotuning"), 
                                             values=["Sintonizar P", "Sintonizar PI", "Sintonizar PD", "Sintonizar PID"], 
                                             command=self.actualizar_ganancias_visuales)
        self.cmb_modo_auto.set("Sintonizar PI")
        self.cmb_modo_auto.pack(pady=(10, 10), fill="x", padx=20)
        
        self.lbl_info_auto = ctk.CTkLabel(self.tab_control.tab("Autotuning"), text="Valores Calculados (Ziegler-Nichols):", text_color="gray60")
        self.lbl_info_auto.pack(pady=(0, 5))

        self.frame_vars_auto = ctk.CTkFrame(self.tab_control.tab("Autotuning"), fg_color="transparent")
        self.frame_vars_auto.pack(expand=True, fill="both", pady=(5, 5))
        self.lbls_auto = self.crear_etiquetas_lectura(self.frame_vars_auto, 
                                                      ["Kp:", "Ki:", "Kd:"], 
                                                      ["0.000", "0.000", "0.000"])

        self.btn_iniciar_auto = ctk.CTkButton(self.tab_control.tab("Autotuning"), text="▶ Ejecutar Relé (Calcular)", fg_color="#D9534F", hover_color="#C9302C", command=self.iniciar_autotuning)
        self.btn_iniciar_auto.pack(pady=(5, 10), padx=40, fill="x")

        # -------------------------------------------------------------
        # BOTÓN MAESTRO DE PRUEBA
        # -------------------------------------------------------------
        self.frame_acciones = ctk.CTkFrame(self.frame_izquierdo, fg_color="transparent")
        self.frame_acciones.pack(fill="x", padx=10, pady=15, side="bottom")

        self.btn_arrancar = ctk.CTkButton(self.frame_acciones, text="▶ Aplicar Configuración y Arrancar", 
                                          fg_color="#5CB85C", hover_color="#4CAE4C", text_color="white",
                                          command=self.toggle_arranque)
        self.btn_arrancar.pack(fill="x", pady=5)

        self.actualizar_puertos()
        self.actualizar_ganancias_visuales()

    def crear_etiquetas_lectura(self, padre, labels, valores_default):
        fuente_grande = ctk.CTkFont(size=14, weight="bold")
        fuente_valores = ctk.CTkFont(size=14)
        etiquetas_creadas = [] 

        for label, valor in zip(labels, valores_default):
            frame = ctk.CTkFrame(padre, fg_color="transparent")
            frame.pack(fill="x", expand=True, padx=40, pady=1)
            
            lbl_nombre = ctk.CTkLabel(frame, text=label, font=fuente_grande)
            lbl_nombre.pack(side="left")
            
            lbl_valor = ctk.CTkLabel(frame, text=valor, font=fuente_valores, text_color="gray70")
            lbl_valor.pack(side="right")
            
            etiquetas_creadas.append((frame, lbl_valor)) 
            
        return etiquetas_creadas 

    # =========================================================================
    # COLUMNA DERECHA: VISUALIZACIÓN Y TELEMETRÍA
    # =========================================================================
    def crear_panel_derecho(self):
        self.frame_derecho = ctk.CTkFrame(self, corner_radius=10, fg_color="transparent")
        self.frame_derecho.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        self.lbl_titulo_grafica = ctk.CTkLabel(self.frame_derecho, text="📈 OSCILOSCOPIO", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_titulo_grafica.pack(pady=(0, 5))

        self.frame_grafica = ctk.CTkFrame(self.frame_derecho, corner_radius=10)
        self.frame_grafica.pack(fill="both", expand=True, padx=5, pady=5)

        self.frame_telemetria = ctk.CTkFrame(self.frame_derecho, fg_color="transparent")
        self.frame_telemetria.pack(fill="x", pady=10)
        self.frame_telemetria.grid_columnconfigure((0,1,2), weight=1)

        self.lbl_val_setpoint = self.crear_tarjeta_telemetria(self.frame_telemetria, "SETPOINT", "0.0", 0)
        self.lbl_val_lectura  = self.crear_tarjeta_telemetria(self.frame_telemetria, "LECTURA REAL", "0.0", 1)
        self.lbl_val_error    = self.crear_tarjeta_telemetria(self.frame_telemetria, "ERROR e(t)", "0.0", 2)

        self.frame_pwm = ctk.CTkFrame(self.frame_derecho, height=60, corner_radius=10)
        self.frame_pwm.pack(fill="x", padx=5, pady=(0,5))

        self.lbl_titulo_pwm = ctk.CTkLabel(self.frame_pwm, text="ESFUERZO DE CONTROL (PWM):", font=ctk.CTkFont(size=12, weight="bold"))
        self.lbl_titulo_pwm.pack(side="left", padx=20)

        self.barra_pwm = ctk.CTkProgressBar(self.frame_pwm, width=300, progress_color="#5CB85C")
        self.barra_pwm.set(0.0)
        self.barra_pwm.pack(side="left", padx=10, expand=True)

        self.lbl_val_pwm = ctk.CTkLabel(self.frame_pwm, text="0 / 1000", font=ctk.CTkFont(size=14, weight="bold"), width=80)
        self.lbl_val_pwm.pack(side="right", padx=20)

        self.configurar_grafica()

    def crear_tarjeta_telemetria(self, padre, titulo, valor_inicial, columna):
        tarjeta = ctk.CTkFrame(padre, corner_radius=8)
        tarjeta.grid(row=0, column=columna, padx=5, sticky="nsew")
        
        lbl_titulo = ctk.CTkLabel(tarjeta, text=titulo, font=ctk.CTkFont(size=12), text_color="gray70")
        lbl_titulo.pack(pady=(10, 0))
        
        lbl_valor = ctk.CTkLabel(tarjeta, text=valor_inicial, font=ctk.CTkFont(size=24, weight="bold"), width=150)
        lbl_valor.pack(pady=(5, 15))
        return lbl_valor
    
    # =========================================================================
    # COMUNICACIÓN SERIAL Y THREADING
    # =========================================================================
    def actualizar_puertos(self):
        puertos_detectados = [puerto.device for puerto in serial.tools.list_ports.comports()]
        if not puertos_detectados:
            puertos_detectados = ["Sin conexión"]
        self.cmb_puertos.configure(values=puertos_detectados)
        self.cmb_puertos.set(puertos_detectados[0])

    def conectar_desconectar(self):
        if self.puerto_serial is None or not self.puerto_serial.is_open:
            puerto = self.cmb_puertos.get()
            baudrate = 115200 
            try:
                self.puerto_serial = serial.Serial(puerto, baudrate, timeout=0.1)
                self.leyendo_serial = True
                
                hilo = threading.Thread(target=self.hilo_leer_uart, daemon=True)
                hilo.start()
                
                self.lbl_estado_conexion.configure(text="🟢")
                self.lbl_texto_estado.configure(text="Conectado", text_color="#5CB85C")
                self.btn_conectar.configure(text="Desconectar", fg_color="#F0AD4E", hover_color="#EEA236", text_color="black")
            except Exception as e:
                print(f"Error al abrir {puerto}: {e}")
        else:
            self.leyendo_serial = False
            self.puerto_serial.close()
            
            self.lbl_estado_conexion.configure(text="🔴")
            self.lbl_texto_estado.configure(text="Desconectado", text_color="gray60")
            self.btn_conectar.configure(text="Conectar", fg_color="#3B8ED0", text_color="white")

    def hilo_leer_uart(self):
        while self.leyendo_serial and self.puerto_serial.is_open:
            try:
                if self.puerto_serial.in_waiting > 0:
                    linea_cruda = self.puerto_serial.readline()
                    try:
                        linea = linea_cruda.decode('utf-8').strip()
                    except UnicodeDecodeError:
                        continue
                    
                    if linea == "<HALT>":
                        print("\n[ALERTA] Señal de paro físico recibida del STM32")
                        self.after(0, self.paro_emergencia)
                        continue
                    
                    if linea.startswith("T,"):
                        datos = linea.split(",")
                        if len(datos) == 4:
                            try:
                                lec = float(datos[1])
                                err = float(datos[2])
                                pwm = float(datos[3])
                                self.ultimos_datos_uart = (lec, err, pwm)
                            except ValueError:
                                print(f"⚠️ ERROR numérico: {datos}")
                        else:
                            print(f"⚠️ Trama rechazada: {linea}")

                    elif linea.startswith("<AUTO,"):
                        datos_auto = linea.replace("<", "").replace(">", "").split(",")
                        if len(datos_auto) == 3:
                            try:
                                ku = float(datos_auto[1])
                                pu = float(datos_auto[2])
                                self.after(0, self.procesar_autotuning, ku, pu)
                            except ValueError:
                                print(f"⚠️ ERROR numérico en Auto: {datos_auto}")
                        else:
                            print(f"⚠️ Trama AUTO rechazada: {linea}")

                else:
                    time.sleep(0.005) 
            except Exception as e:
                print(f"ERROR CRÍTICO UART: {e}")
                time.sleep(0.005)

    def iniciar_autotuning(self):
        """Envía el comando ID 7 para que el MCU ejecute el relevador"""
        if self.puerto_serial is None or not self.puerto_serial.is_open:
            print("ERROR: Conecta el puerto serial primero para iniciar el Autotuning.")
            return

        self.es_autotuning = True 

        modo = "VEL" if self.modo_sistema_actual == "Velocidad" else "POS"
        trama_tx = f"<START,{modo},7,0.000,0.000,0.000,0.000,0.000>\n"

        try:
            self.puerto_serial.write(trama_tx.encode('utf-8'))
            print(f"Comando Autotuning: {trama_tx.strip()}")

            self.btn_iniciar_auto.configure(text="⏳ Sintonizando...", state="disabled")
            
            self.t_data.clear()
            self.r_data.clear()
            self.y_data.clear()
            self.tiempo_actual = 0.0
            self.ax.set_xlim(0, 20)
            self.sistema_corriendo = True 
        except Exception as e:
            print(f"Error UART: {e}")

    def procesar_autotuning(self, ku, pu):
        """Calcula las variantes y las inyecta en los diccionarios (Sin PID Suavizado)"""
        print(f"\n✅ ¡Autotuning Exitoso! Ku = {ku:.3f}, Pu = {pu:.3f} s")

        # 1. Control P Clásico
        kp_p = 0.5 * ku
        
        # 2. Control PI Clásico
        kp_pi = 0.45 * ku
        ki_pi = (1.2 * kp_pi) / pu if pu > 0 else 0.0
        
        # 3. Control PD
        kp_pd = 0.8 * ku
        kd_pd = (kp_pd * pu) / 8.0
        
        # 4. Control PID Clásico
        kp_pid = 0.6 * ku
        ki_pid = (2.0 * kp_pid) / pu if pu > 0 else 0.0
        kd_pid = (kp_pid * pu) / 8.0

        perfiles = self.perfiles_velocidad if self.modo_sistema_actual == "Velocidad" else self.perfiles_posicion

        perfiles["Sintonizar P"]["K1"] = kp_p
        perfiles["Sintonizar P"]["K2"] = 0.0
        perfiles["Sintonizar P"]["K3"] = 0.0

        perfiles["Sintonizar PI"]["K1"] = kp_pi
        perfiles["Sintonizar PI"]["K2"] = ki_pi
        perfiles["Sintonizar PI"]["K3"] = 0.0
        
        perfiles["Sintonizar PD"]["K1"] = kp_pd
        perfiles["Sintonizar PD"]["K2"] = 0.0
        perfiles["Sintonizar PD"]["K3"] = kd_pd

        perfiles["Sintonizar PID"]["K1"] = kp_pid
        perfiles["Sintonizar PID"]["K2"] = ki_pid
        perfiles["Sintonizar PID"]["K3"] = kd_pid

        self.actualizar_ganancias_visuales()

        self.es_autotuning = False 
        self.btn_iniciar_auto.configure(text="▶ Ejecutar Relé (Calcular)", state="normal")
        self.sistema_corriendo = False 
        self.puerto_serial.write(b"<STOP>\n") 
        
        print("💡 Ganancias calculadas. Selecciona tu controlador favorito en el menú y presiona 'Arrancar'.")

    def toggle_arranque(self):
        """Arranca el ensayo de control"""
        if self.puerto_serial is None or not self.puerto_serial.is_open:
            print("ERROR: Conecta el puerto serial primero.")
            return

        if not self.sistema_corriendo:
            self.es_autotuning = False 
            
            modo = "VEL" if self.modo_sistema_actual == "Velocidad" else "POS"
            pestaña_activa = self.tab_control.get()
            
            if pestaña_activa == "Clásico":
                controlador = self.cmb_modo_clasico.get()
            elif pestaña_activa == "Compensador":
                controlador = self.cmb_modo_comp.get()
            else:
                controlador = self.cmb_modo_auto.get()

            perfiles = self.perfiles_velocidad if self.modo_sistema_actual == "Velocidad" else self.perfiles_posicion
            
            if controlador in perfiles:
                g = perfiles[controlador]
                
                # --- ALIAS ACTUALIZADO (Sin PID Suavizado) ---
                alias = {
                    "PI": "PI", "PD": "PD", "PID": "PID", 
                    "Adelanto": "AD", "Atraso": "AT", "Adelanto-Atraso": "AA",
                    "Sintonizar P": "P", "Sintonizar PI": "PI", 
                    "Sintonizar PD": "PD", "Sintonizar PID": "PID"
                }
                
                if "Control" in controlador:
                    nombre_corto = controlador.replace("Control ", "")
                else:
                    nombre_corto = alias.get(controlador, controlador)
                
                mapa_ids = {"P": 0, "PI": 1, "PD": 2, "PID": 3, "AD": 4, "AT": 5, "AA": 6}
                id_controlador = mapa_ids.get(nombre_corto, 0)
                
                trama_tx = f"<START,{modo},{id_controlador},{g['K1']:.3f},{g['K2']:.3f},{g['K3']:.3f},{g['K4']:.3f},{g['K5']:.3f}>\n"
            else:
                trama_tx = f"<START,{modo},99,0.000,0.000,0.000,0.000,0.000>\n"

            self.puerto_serial.write(trama_tx.encode('utf-8'))
            print(f"Transmitido: {trama_tx.strip()}")

            self.t_data.clear()
            self.r_data.clear()
            self.y_data.clear()
            
            self.tiempo_actual = 0.0
            self.ax.set_xlim(0, 20)
            self.linea_ref.set_data([], [])
            self.linea_salida.set_data([], [])
            self.canvas.draw_idle()

            self.btn_arrancar.configure(text="⏹ Detener Ensayo", fg_color="#F0AD4E", hover_color="#EEA236", text_color="black")
            self.seg_sistema.configure(state="disabled")
            self.cmb_modo_clasico.configure(state="disabled")
            self.cmb_modo_comp.configure(state="disabled")
            self.cmb_modo_auto.configure(state="disabled")
            self.btn_iniciar_auto.configure(state="disabled") 
            
            self.sistema_corriendo = True
        else:
            self.puerto_serial.write(b"<STOP>\n")
            print("Transmitido: <STOP>")
            
            self.btn_arrancar.configure(text="▶ Aplicar Configuración y Arrancar", fg_color="#5CB85C", hover_color="#4CAE4C", text_color="white")
            self.seg_sistema.configure(state="normal")
            self.cmb_modo_clasico.configure(state="normal")
            self.cmb_modo_comp.configure(state="normal")
            self.cmb_modo_auto.configure(state="normal")
            self.btn_iniciar_auto.configure(state="normal")
            
            self.sistema_corriendo = False

    def configurar_grafica(self):
        import matplotlib.pyplot as plt
        self.t_data = deque(maxlen=1000)
        self.r_data = deque(maxlen=1000)
        self.y_data = deque(maxlen=1000)
        self.tiempo_actual = 0.0

        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(6, 4), dpi=100)
        self.fig.patch.set_facecolor('#2b2b2b')
        self.ax.set_facecolor('#2b2b2b')
        
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.spines['bottom'].set_color('gray')
        self.ax.spines['left'].set_color('gray')
        self.ax.tick_params(colors='gray')

        self.linea_ref, = self.ax.plot([], [], color='#5BC0DE', linestyle='--', linewidth=1.5, label='Referencia r(t)')
        self.linea_salida, = self.ax.plot([], [], color='#D9534F', linewidth=2, label='Salida y(t)')
        
        self.ax.set_xlim(0, 16)
        self.ax.set_ylim(0, 200) 
        self.ax.set_ylabel("RPM", color='gray')

        self.ax.legend(loc="upper left", frameon=False)
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame_grafica)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.actualizar_grafica_tiempo_real() 

    def cambiar_modo_sistema(self, modo_seleccionado):
        self.modo_sistema_actual = "Velocidad" if "Velocidad" in modo_seleccionado else "Posición"
        
        self.t_data.clear()
        self.r_data.clear()
        self.y_data.clear()
        self.tiempo_actual = 0.0
        self.ax.set_xlim(0, 16)
        
        if self.modo_sistema_actual == "Posición":
            self.ax.set_ylim(0, 360)
            self.ax.set_ylabel("Grados (°)", color='gray')
            self.lbl_val_setpoint.configure(text="0.0°")
            self.lbl_val_lectura.configure(text="0.0°")
            self.lbl_val_error.configure(text="0.0°")
        else:
            self.ax.set_ylim(0, 200)
            self.ax.set_ylabel("RPM", color='gray')
            self.lbl_val_setpoint.configure(text="0.0 RPM")
            self.lbl_val_lectura.configure(text="0.0 RPM")
            self.lbl_val_error.configure(text="0.0 RPM")
            
        self.actualizar_ganancias_visuales()
        self.canvas.draw_idle()

    def actualizar_ganancias_visuales(self, *args):
        perfiles = self.perfiles_velocidad if self.modo_sistema_actual == "Velocidad" else self.perfiles_posicion
        
        # 1. Sincronizar Clásico 
        ctrl_clasico = self.cmb_modo_clasico.get()
        if ctrl_clasico in perfiles:
            g = perfiles[ctrl_clasico]
            self.lbls_clasicos[0][1].configure(text=f"{g['K1']:.3f}")
            self.lbls_clasicos[1][1].configure(text=f"{g['K2']:.3f}")
            self.lbls_clasicos[2][1].configure(text=f"{g['K3']:.3f}")
            self.lbls_clasicos[3][0].pack_forget()
            self.lbls_clasicos[4][0].pack_forget()
            
        # 2. Sincronizar Compensador 
        ctrl_comp = self.cmb_modo_comp.get()
        if ctrl_comp in perfiles:
            g = perfiles[ctrl_comp]
            self.lbls_comp[0][1].configure(text=f"{g['K1']:.3f}")
            self.lbls_comp[1][1].configure(text=f"{g['K2']:.3f}")
            self.lbls_comp[2][1].configure(text=f"{g['K3']:.3f}")
            
            if ctrl_comp == "Adelanto-Atraso":
                self.lbls_comp[3][1].configure(text=f"{g['K4']:.3f}")
                self.lbls_comp[4][1].configure(text=f"{g['K5']:.3f}")
                self.lbls_comp[3][0].pack(fill="x", expand=True, padx=40, pady=1)
                self.lbls_comp[4][0].pack(fill="x", expand=True, padx=40, pady=1)
            else:
                self.lbls_comp[3][0].pack_forget()
                self.lbls_comp[4][0].pack_forget()
                
        # 3. Sincronizar Autotuning 
        ctrl_auto = self.cmb_modo_auto.get()
        if ctrl_auto in perfiles:
            g = perfiles[ctrl_auto]
            self.lbls_auto[0][1].configure(text=f"{g['K1']:.3f}")
            self.lbls_auto[1][1].configure(text=f"{g['K2']:.3f}")
            self.lbls_auto[2][1].configure(text=f"{g['K3']:.3f}")

    def obtener_referencia_combinada(self, t):
        v_max = 180.0 if self.modo_sistema_actual == "Velocidad" else 355.0
        
        if getattr(self, 'es_autotuning', False):
            if t < 0.5:
                return 0.0 
            else:
                return v_max * 0.25 
                
        if t < 0.5:
            return 0.0
        elif t < 5.0:
            return v_max * 0.4
        elif t < 10.0:
            return v_max * (0.4 + 0.08 * (t - 5.0))
        elif t < 15.0:
            return v_max * (0.8 + 0.2 * np.sin(2 * np.pi * 0.5 * (t - 10.0)))
        else:
            return 0.0

    def actualizar_grafica_tiempo_real(self):
        try:
            if self.sistema_corriendo:
                if not hasattr(self, 'contador_tx'):
                    self.contador_tx = 0
                if not hasattr(self, 'pwm_guardado'):
                    self.pwm_guardado = 0.0

                setpoint_actual = self.obtener_referencia_combinada(self.tiempo_actual)
                self.contador_tx += 1
                
                if self.contador_tx % 2 == 0:
                    trama_sp = f"<SP,{setpoint_actual:.1f}>\n"
                    if self.puerto_serial and self.puerto_serial.is_open:
                        try:
                            self.puerto_serial.write(trama_sp.encode('utf-8'))
                        except:
                            pass 

                if self.ultimos_datos_uart is not None:
                    salida_motor, error, pwm_real = self.ultimos_datos_uart
                    self.pwm_guardado = pwm_real 
                    self.ultimos_datos_uart = None
                else:
                    if not (self.puerto_serial and self.puerto_serial.is_open):
                        if len(self.y_data) > 0:
                            salida_motor = self.y_data[-1] + (setpoint_actual - self.y_data[-1]) * 0.08
                        else:
                            salida_motor = 0.0
                        salida_motor += np.random.normal(0, 0.5) 
                        error = abs(setpoint_actual - salida_motor)
                        pwm_real = min(1000, max(0, error * 10.0))
                    else:
                        salida_motor = self.y_data[-1] if len(self.y_data) > 0 else 0.0
                        error = abs(setpoint_actual - salida_motor)
                        pwm_real = self.pwm_guardado

                self.tiempo_actual += 0.02
                self.t_data.append(self.tiempo_actual)
                self.r_data.append(setpoint_actual) 
                self.y_data.append(salida_motor)     

                if self.contador_tx % 2 == 0:
                    self.linea_ref.set_data(list(self.t_data), list(self.r_data))
                    self.linea_salida.set_data(list(self.t_data), list(self.y_data))
                    
                    if self.tiempo_actual > 20:
                        self.ax.set_xlim(self.tiempo_actual - 20, self.tiempo_actual)
                    
                    self.canvas.draw_idle()

                if self.contador_tx % 5 == 0:
                    unidad = "°" if self.modo_sistema_actual == "Posición" else " RPM"
                    self.lbl_val_setpoint.configure(text=f"{setpoint_actual:.1f}{unidad}") 
                    self.lbl_val_lectura.configure(text=f"{salida_motor:.1f}{unidad}")
                    self.lbl_val_error.configure(text=f"{error:.1f}{unidad}")
                    self.barra_pwm.set(pwm_real / 1000.0)
                    self.lbl_val_pwm.configure(text=f"{int(pwm_real)} / 1000")

        except Exception as e:
            print(f"❌ Error interno en graficación: {e}")

        self.after(20, self.actualizar_grafica_tiempo_real)

    def on_closing(self):
        print("Cerrando la aplicación de forma segura...")
        self.leyendo_serial = False 
        if self.puerto_serial and self.puerto_serial.is_open:
            try:
                self.puerto_serial.write(b"<STOP>\n") 
            except:
                pass
            self.puerto_serial.close()
        import matplotlib.pyplot as plt
        plt.close('all') 
        self.quit()      
        self.destroy()   
        sys.exit(0)     

if __name__ == "__main__":
    app = MotorDashboard()
    app.mainloop()