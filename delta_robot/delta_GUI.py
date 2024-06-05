import time
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import threading

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel, QStackedWidget, QHBoxLayout, QFrame, QSizePolicy, QScrollArea, QListWidget, QListWidgetItem, QLineEdit
from PyQt5.QtGui import QIcon, QFontDatabase, QFont, QPixmap
from PyQt5.QtCore import Qt, QSize
import sys
import os

class DeltaRobotController(Node):
    def __init__(self):
        super().__init__('delta_robot_controller')
        self.homing_complete = False
        self.home_subscriber = self.create_subscription(
            Bool,
            'homing_complete',
            self.homing_callback,
            10)
        self.get_logger().info("Waiting for homing to complete.")
        # self.coordinate_input_thread = threading.Thread(target=self.coordinate_input_loop)
        # self.coordinate_input_thread.start()
        # Constants
        self.f =400 # Example value for f
        self.e = 100      # Example value for e
        self.rf = 230    # Example value for rf
        self.re = 400    # Example value for re
        self.pi = math.pi
        self.x_target = None
        self.y_target = None
        self.z_target = None
        # Create a publisher to publish motor angles
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_angles', 10)

        self.trajectory = [
            (0, 0, -300),  # Example coordinates for point 1
            (0, 0, -500),  # Example coordinates for point 2
            (0,0, -300)  # Example coordinates for point 3
        ]

    def homing_callback(self, msg):
        if msg.data == True:
            self.homing_complete = True
            self.get_logger().info("Homing complete. Updating home angles.")
            self.save_theta(-30, -30, -30)

    def coordinate_input_loop(self):
        while not self.homing_complete:
            time.sleep(0.5)
        while True:
            self.current_theta1, self.current_theta2, self.current_theta3 = self.read_theta()
            self.x_target, self.y_target, self.z_target = self.get_target_coordinates()
            if self.x_target is not None and self.y_target is not None and self.z_target is not None:
                self.calculate_and_publish_angles()
    def follow_trajectory(self):
        for point in self.trajectory:
            self.current_theta1, self.current_theta2, self.current_theta3 = self.read_theta()
            self.x_target, self.y_target, self.z_target = point
            self.calculate_and_publish_angles()
            time.sleep(3)  # Adjust sleep time as needed
    def get_target_coordinates(self):
        while True:
            try:
                x_target = float(input("Ingrese la coordenada X del objetivo: "))
                y_target = float(input("Ingrese la coordenada Y del objetivo: "))
                z_target = float(input("Ingrese la coordenada Z del objetivo: "))
                return x_target, y_target, z_target
            except ValueError:
                print("Por favor, ingrese coordenadas válidas.")
                return self.get_target_coordinates()

    def delta_calcAngleYZ(self, x0, y0, z0):
        y1 = -0.5 * 0.57735 * self.f  # f/2 * tg 30
        y0 -= 0.5 * 0.57735 * self.e   # shift center to edge
        a = (x0*x0 + y0*
        y0 + z0*z0 + self.rf*self.rf - self.re*self.re - y1*y1)/(2*z0)
        b = (y1 - y0)/z0
        d = -(a + b*y1)**2 + self.rf * (b*b*self.rf + self.rf)
        if d < 0:
            return -1, 0  # Non-existing point
        yj = (y1 - a*b - math.sqrt(d))/(b*b + 1)  # Choosing outer point
        zj = a + b*yj
        theta = 180.0 * math.atan(-zj/(y1 - yj))/self.pi + (180.0 if yj > y1 else 0.0)
        theta = max(theta,-30)
        return 0, theta

    def delta_calcInverse(self, x0, y0, z0):
        delta_theta1_desired, theta1_desired = self.delta_calcAngleYZ(x0, y0, z0)
        if delta_theta1_desired == -1:
            return -1, 0, 0, 0  # Non-existing position
        
        delta_theta2_desired, theta2_desired = self.delta_calcAngleYZ(x0*math.cos(2*self.pi/3) + y0*math.sin(2*self.pi/3), y0*math.cos(2*self.pi/3) - x0*math.sin(2*self.pi/3), z0)
        if delta_theta2_desired == -1:
            return -1, 0, 0, 0  # Non-existing position
        
        delta_theta3_desired, theta3_desired = self.delta_calcAngleYZ(x0*math.cos(2*self.pi/3) - y0*math.sin(2*self.pi/3), y0*math.cos(2*self.pi/3) + x0*math.sin(2*self.pi/3), z0)
        if delta_theta3_desired == -1:
            return -1, 0, 0, 0  # Non-existing position
        
        # Calculate the difference between current and desired angles
        delta_theta1 = theta1_desired - self.current_theta1
        delta_theta2 = theta2_desired - self.current_theta2
        delta_theta3 = theta3_desired - self.current_theta3
        
        return 0, delta_theta1, delta_theta2, delta_theta3

    def read_theta(self):
        try:
            with open("current_theta.txt", "r") as f:
                lines = f.readlines()
                theta1 = float(lines[0])
                theta2 = float(lines[1])
                theta3 = float(lines[2])
                return theta1, theta2, theta3
        except FileNotFoundError:
            print("Current theta file not found. Using default values.")
            return 0, 0, 0

    def save_theta(self, theta1, theta2, theta3):
        with open("current_theta.txt", "w") as f:
            f.write(f"{theta1}\n{theta2}\n{theta3}")

    def calculate_and_publish_angles(self):
        # Calculate joint angles for the target position
        status, theta1, theta2, theta3 = self.delta_calcInverse(self.x_target, self.y_target, self.z_target)

        if status == 0:
            # Display current angles, changes, and desired angles
            print('Current angles: ', self.current_theta1, self.current_theta2, self.current_theta3)
            print('Changes in angles: ', theta1, theta2, theta3)
            print('Desired angles: ', theta1 + self.current_theta1, theta2 + self.current_theta2,
                  theta3 + self.current_theta3)

            # Save new theta values to file
            self.save_theta(theta1 + self.current_theta1, theta2 + self.current_theta2, theta3 + self.current_theta3)

            # Publish calculated angles
            msg = Float32MultiArray()
            msg.data = [theta1, theta2, theta3]
            self.publisher.publish(msg)
        else:
            print("Non-existing position")


class MainWindow(QMainWindow):
    def __init__(self,nodo):
        super().__init__()
        self.setWindowTitle("Robot Delta")
        self.setGeometry(100, 100, 1000, 800)  # Aumentar tamaño de la ventana
        #ROS
        self.nodo = nodo
        # Load custom font
        QFontDatabase.addApplicationFont("fonts/Roboto-Regular.ttf")
        
        # Initialize history list
        self.history = []

        # Main layout
        main_layout = QVBoxLayout()

        # Coordenadas máximas y mínimas
        self.X_MIN = -300
        self.X_MAX = 300
        self.Y_MIN = -300
        self.Y_MAX = 300
        self.Z_MIN = -600
        self.Z_MAX = -100

        # Top bar
        top_bar = QFrame()
        top_bar.setStyleSheet("background-color: #F0F0F0; height: 50px;")
        top_bar.setFixedHeight(50)
        top_bar_layout = QHBoxLayout()
        top_bar_layout.setContentsMargins(10, 0, 10, 0)  # Añadir márgenes para evitar el corte
        top_bar.setLayout(top_bar_layout)
        
        # Menu button in top bar
        self.menu_button = QPushButton("MENU")
        self.set_button_icon(self.menu_button, "icons/barra-de-menus.png", 24)
        self.menu_button.setStyleSheet("QPushButton { text-align: left; padding: 10px; color: #333333; font-size: 18px; font-family: Roboto; }")
        self.menu_button.clicked.connect(self.toggle_menu)
        
        # Application title
        app_title = QLabel("Robot Delta")
        app_title.setStyleSheet("color: #333333; font-size: 18px; margin-left: 10px; font-family: Roboto;")
        
        # EIA logo
        eia_logo = QLabel()
        pixmap = QPixmap("images/eia.png")  # Asegúrate de que la ruta del icono sea correcta
        pixmap = pixmap.scaled(70, 70, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        eia_logo.setPixmap(pixmap)
        eia_logo.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        top_bar_layout.addWidget(self.menu_button)
        top_bar_layout.addStretch()
        top_bar_layout.addWidget(app_title)
        top_bar_layout.addStretch()
        top_bar_layout.addWidget(eia_logo)
        
        # Main content layout
        content_layout = QHBoxLayout()
        
        # Menu widget
        self.menu_widget = QFrame()
        self.menu_layout = QVBoxLayout()
        self.menu_widget.setLayout(self.menu_layout)
        self.menu_widget.setFixedWidth(200)  # Ajustar el ancho del menú
        self.menu_widget.setStyleSheet("background-color: #E0E0E0;")
        
        # Menu buttons with icons
        self.about_button = QPushButton("Información")
        self.set_button_icon(self.about_button, "icons/informa.png", 24)
        self.about_button.setStyleSheet("QPushButton { text-align: left; padding: 10px; color: #333333; font-family: Roboto; }")
        
        self.usage_button = QPushButton("Coordenadas")
        self.set_button_icon(self.usage_button, "icons/monitor.png", 24)
        self.usage_button.setStyleSheet("QPushButton { text-align: left; padding: 10px; color: #333333; font-family: Roboto; }")
        
        self.history_button = QPushButton("Historial")
        self.set_button_icon(self.history_button, "icons/histori.png", 24)
        self.history_button.setStyleSheet("QPushButton { text-align: left; padding: 10px; color: #333333; font-family: Roboto; }")
        
        self.menu_layout.addWidget(self.about_button)
        self.menu_layout.addWidget(self.usage_button)
        self.menu_layout.addWidget(self.history_button)
        self.menu_layout.addStretch()
        
        # Stack widget for content pages
        self.stack = QStackedWidget()
        self.stack.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Pages
        self.about_page = self.create_about_page()
        self.usage_page = self.create_usage_page()
        self.history_page = self.create_history_page()
        
        self.stack.addWidget(self.about_page)
        self.stack.addWidget(self.usage_page)
        self.stack.addWidget(self.history_page)
        
        # Connect buttons to pages
        self.about_button.clicked.connect(lambda: self.stack.setCurrentWidget(self.about_page))
        self.usage_button.clicked.connect(lambda: self.stack.setCurrentWidget(self.usage_page))
        self.history_button.clicked.connect(lambda: self.stack.setCurrentWidget(self.history_page))
        
        content_layout.addWidget(self.menu_widget)
        content_layout.addWidget(self.stack)
        
        main_layout.addWidget(top_bar)
        main_layout.addLayout(content_layout)
        
        # Central widget
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
        # Style
        self.setStyleSheet("""
            QMainWindow {
                background-color: #F5F5F5;
            }
            QWidget {
                background-color: #F5F5F5;
                color: #333333;
            }
            QScrollBar:vertical {
                background: #E0E0E0;
                width: 16px;
                margin: 16px 0 16px 0;
                border-radius: 8px;
            }
            QScrollBar::handle:vertical {
                background: #C0C0C0;
                min-height: 20px;
                border-radius: 8px;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                background: none;
            }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
                background: none;
            }
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 16px;
                padding: 10px;
                border-radius: 5px;
                border: none;
                margin: 5px;
                font-family: 'Roboto';
            }
            QPushButton::hover {
                background-color: #45a049;
            }
            QLineEdit {
                padding: 10px;
                border: 1px solid #ccc;
                border-radius: 5px;
                margin: 5px 0;
                font-size: 16px;
                font-family: 'Roboto';
            }
            QLabel {
                color: #333333;
                font-family: 'Roboto';
                margin: 5px 0;
            }
            QListWidget {
                padding: 10px;
                border: 1px solid #ccc;
                border-radius: 5px;
                margin: 5px 0;
                font-family: 'Roboto';
            }
        """)

    def set_button_icon(self, button, icon_path, size):
        if os.path.exists(icon_path):
            button.setIcon(QIcon(icon_path))
            button.setIconSize(QSize(size, size))
        else:
            print(f"Icon not found: {icon_path}")

    def create_about_page(self):
        page = QWidget()
        layout = QVBoxLayout()

        # Add a scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        # Title
        title = QLabel("Información sobre el Robot Delta")
        title.setFont(QFont("Roboto", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        scroll_layout.addWidget(title)

        # Sections
        sections = [
            ("¿Qué es un robot delta?", "Un robot delta es un dispositivo robótico que se utiliza principalmente en aplicaciones industriales donde se requiere alta velocidad y precisión. A diferencia de los robots seriales, que tienen una estructura de brazo en serie (uno tras otro), los robots delta están compuestos por tres brazos paralelos conectados a una base fija en un extremo y a una plataforma móvil en el otro. Esta plataforma móvil sostiene el efector final, que es la parte del robot que interactúa directamente con los objetos, como una herramienta de agarre o una boquilla."),
            ("Estructura y Funcionamiento", """
1. Base Fija:
   • La base del robot delta es una estructura rígida donde se encuentran los motores. Esta base permanece fija durante el funcionamiento del robot.
2. Brazos Móviles:
   • Desde la base, tres brazos móviles se extienden hacia abajo. Estos brazos están conectados a la base mediante uniones esféricas, lo que les permite moverse en diferentes direcciones. Cada brazo está compuesto por dos segmentos: uno fijo y otro móvil. El movimiento de estos brazos es controlado por motores en la base.
3. Plataforma Móvil:
   • En el extremo inferior de los brazos, estos se unen a una plataforma móvil. Esta plataforma es donde se encuentra el efector final. A medida que los brazos se mueven, la plataforma también se mueve, permitiendo que el efector final se posicione con precisión en el espacio tridimensional.
            """),
            ("Principio de Operación", "El robot delta opera utilizando principios de cinemática paralela. Los motores en la base mueven los brazos de manera coordinada. La combinación de los movimientos de los tres brazos permite que la plataforma móvil se posicione en cualquier punto dentro del espacio de trabajo del robot. Esto se logra mediante cálculos matemáticos precisos que determinan la posición exacta de la plataforma basada en los ángulos de los brazos."),
            ("Aplicaciones Comunes", """
• Industria Alimentaria: Clasificación y empaquetado de alimentos, como chocolates o galletas.
• Electrónica: Ensamblaje de pequeños componentes electrónicos en dispositivos como teléfonos móviles.
• Medicina: Manipulación de instrumentos quirúrgicos y muestras en laboratorios de análisis.
            """),
            ("Ventajas del Robot Delta", """
• Alta Velocidad: Los robots delta son extremadamente rápidos, lo que los hace ideales para tareas que requieren rapidez y eficiencia.
• Precisión: Pueden realizar movimientos muy precisos, lo cual es crucial en la manipulación de componentes pequeños y delicados.
• Flexibilidad: Son capaces de realizar una amplia variedad de tareas, adaptándose a diferentes aplicaciones industriales.
            """),
            ("Cinemática Directa", """
La cinemática directa se refiere al cálculo de la posición del efector final del robot (la herramienta o agarre que realiza la tarea) en función de los ángulos de las juntas de los brazos del robot. En otras palabras, se trata de determinar dónde está la herramienta del robot en el espacio tridimensional, dado un conjunto de ángulos de las articulaciones de sus brazos.

Cómo Funciona la Cinemática Directa en un Robot Delta:
1. Entrada: Los ángulos de los motores de los brazos del robot (θ1, θ2, θ3).
2. Proceso: Usando las longitudes de los brazos y los ángulos proporcionados, se aplican ecuaciones matemáticas para calcular la posición cartesiana (x, y, z) de la plataforma móvil del robot.
3. Salida: La posición (x, y, z) del efector final.
            """),
            ("Cinemática Inversa", """
La cinemática inversa se ocupa de determinar los ángulos de las juntas necesarias para posicionar el efector final en una ubicación específica en el espacio. Este es un proceso más complejo, ya que debe encontrar los ángulos correctos para cada una de las juntas del robot que permitan alcanzar la posición deseada.

Cómo Funciona la Cinemática Inversa en un Robot Delta:
1. Entrada: La posición deseada del efector final (x, y, z).
2. Proceso: Se aplican ecuaciones matemáticas para resolver los ángulos de las juntas (θ1, θ2, θ3) necesarios para alcanzar esa posición. Este proceso puede involucrar la resolución de sistemas de ecuaciones no lineales.
3. Salida: Los ángulos de las juntas (θ1, θ2, θ3).
            """)
        ]

        for title, content in sections:
            section_title = QLabel(title)
            section_title.setFont(QFont("Roboto", 14, QFont.Bold))
            section_title.setAlignment(Qt.AlignLeft)
            scroll_layout.addWidget(section_title)

            section_content = QLabel(content)
            section_content.setFont(QFont("Roboto", 12))
            section_content.setAlignment(Qt.AlignLeft)
            section_content.setWordWrap(True)
            scroll_layout.addWidget(section_content)

        # Add images
        image_paths = [
            "images/delta_robot_structure.png",
            "images/delta_robot_operation.png",
            "images/applications.jpeg"
        ]

        for image_path in image_paths:
            if os.path.exists(image_path):
                image_label = QLabel()
                pixmap = QPixmap(image_path)
                image_label.setPixmap(pixmap)
                image_label.setAlignment(Qt.AlignCenter)
                scroll_layout.addWidget(image_label)
            else:
                print(f"Image not found: {image_path}")

        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)
        page.setLayout(layout)

        return page
    
    def create_usage_page(self):
        page = QWidget()
        layout = QVBoxLayout()

        # Create a scroll area for the instructions and inputs
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        # Coordenadas inputs
        coord_layout = QVBoxLayout()

        coord_label = QLabel("Ingresar Coordenadas (x, y, z):")
        coord_label.setFont(QFont("Roboto", 14, QFont.Bold))
        coord_layout.addWidget(coord_label)

        instructions = QLabel(
            f"<ol>"
            f"<li>Ingrese las coordenadas en los campos x, y, z para calcular los ángulos de la cinemática inversa.</li>"
            f"<li>Rango de valores:</li>"
            f"<ul>"
            f"<li>x: {self.X_MIN} a {self.X_MAX}</li>"
            f"<li>y: {self.Y_MIN} a {self.Y_MAX}</li>"
            f"<li>z: {self.Z_MIN} a {self.Z_MAX}</li>"
            f"</ul>"
            f"</ol>"
        )
        instructions.setFont(QFont("Roboto", 12))
        instructions.setWordWrap(True)
        coord_layout.addWidget(instructions)

        self.x_input = QLineEdit()
        self.x_input.setPlaceholderText("x")
        coord_layout.addWidget(self.x_input)

        self.y_input = QLineEdit()
        self.y_input.setPlaceholderText("y")
        coord_layout.addWidget(self.y_input)

        self.z_input = QLineEdit()
        self.z_input.setPlaceholderText("z")
        coord_layout.addWidget(self.z_input)

        self.calculate_button = QPushButton("Calcular Ángulos")
        ###ROS
        if True:
            self.nodo.x_target=float(self.x_input)
            self.nodo.y_target=float(self.y_input)
            self.nodo.z_target=float(self.z_input)
            self.nodo.calculate_and_publish_angles()
        # self.calculate_button.clicked.connect(self.calculate_angles)
        coord_layout.addWidget(self.calculate_button)

        self.angle_output = QLabel("Ángulos: ")
        coord_layout.addWidget(self.angle_output)

        scroll_layout.addLayout(coord_layout)

        # Trayectoria inputs
        trajectory_layout = QVBoxLayout()

        trajectory_label = QLabel("Crear Trayectoria:")
        trajectory_label.setFont(QFont("Roboto", 14, QFont.Bold))
        trajectory_layout.addWidget(trajectory_label)

        traj_instructions = QLabel(
            f"<ol>"
            f"<li>Ingrese los puntos (x, y, z) para la trayectoria y haga clic en 'Añadir Punto'.</li>"
            f"<li>Los puntos se numerarán y se agregarán a la lista.</li>"
            f"<li>Haga clic en 'Enviar Trayectoria' para mover el robot a través de todos los puntos.</li>"
            f"<li>Rango de valores:</li>"
            f"<ul>"
            f"<li>x: {self.X_MIN} a {self.X_MAX}</li>"
            f"<li>y: {self.Y_MIN} a {self.Y_MAX}</li>"
            f"<li>z: {self.Z_MIN} a {self.Z_MAX}</li>"
            f"</ul>"
            f"</ol>"
        )
        traj_instructions.setFont(QFont("Roboto", 12))
        traj_instructions.setWordWrap(True)
        trajectory_layout.addWidget(traj_instructions)

        self.traj_x_input = QLineEdit()
        self.traj_x_input.setPlaceholderText("x")
        trajectory_layout.addWidget(self.traj_x_input)

        self.traj_y_input = QLineEdit()
        self.traj_y_input.setPlaceholderText("y")
        trajectory_layout.addWidget(self.traj_y_input)

        self.traj_z_input = QLineEdit()
        self.traj_z_input.setPlaceholderText("z")
        trajectory_layout.addWidget(self.traj_z_input)

        self.add_point_button = QPushButton("Añadir Punto")
        self.add_point_button.clicked.connect(self.add_point_to_trajectory)
        trajectory_layout.addWidget(self.add_point_button)

        self.point_list = QListWidget()
        trajectory_layout.addWidget(self.point_list)

        #self.send_trajectory_button = QPushButton("Enviar Trayectoria")
        #self.send_trajectory_button.clicked.connect(self.send_trajectory)
        #trajectory_layout.addWidget(self.send_trajectory_button)

        scroll_layout.addLayout(trajectory_layout)

        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)
        page.setLayout(layout)
        return page
    
    def create_history_page(self):
        page = QWidget()
        layout = QVBoxLayout()

        # History list
        self.history_list = QListWidget()
        layout.addWidget(self.history_list)

        # Clear history button
        self.clear_history_button = QPushButton("Borrar Historial")
        self.clear_history_button.clicked.connect(self.clear_history)
        layout.addWidget(self.clear_history_button)

        page.setLayout(layout)
        return page
    
    def toggle_menu(self):
        if self.menu_widget.width() == 0:
            self.menu_widget.setFixedWidth(200)
            self.set_button_icon(self.menu_button, "icons/barra-de-menus.png", 24)
        else:
            self.menu_widget.setFixedWidth(0)
            self.set_button_icon(self.menu_button, "icons/barra-de-menus.png", 24)
    
    # def calculate_angles(self):
    #     try:
    #         x = float(self.x_input.text())
    #         y = float(self.y_input.text())
    #         z = float(self.z_input.text())
    #         # result = self.move_to_angles(x, y, z)  # Llamar a la función corregida
    #         self.angle_output.setText(result)
    #         # Agregar al historial
    #         self.add_to_history(x, y, z, result)
    #     except ValueError:
    #         self.angle_output.setText("Por favor ingrese valores válidos para x, y, z.")
    
    # def move_to_angles(self, x, y, z):
    #     status, theta1, theta2, theta3 = delta_calcInverse(x, y, z)
    #     if status == 0:
    #         move_to_angles(x, y, z)
    #         return f"Ángulos: θ1={theta1}, θ2={theta2}, θ3={theta3}"
    #     else:
    #         return "Posición no existente."
    
    def add_point_to_trajectory(self):
        try:
            x = float(self.traj_x_input.text())
            y = float(self.traj_y_input.text())
            z = float(self.traj_z_input.text())
            point_number = self.point_list.count() + 1
            self.point_list.addItem(f"Punto {point_number} agregado: ({x}, {y}, {z})")
        except ValueError:
            self.angle_output.setText("Por favor ingrese valores válidos para x, y, z.")
    
    # def send_trajectory(self):
    #     trajectory = []
    #     for i in range(self.point_list.count()):
    #         point_text = self.point_list.item(i).text()
    #         point = point_text.split(": ")[1].strip("()").split(", ")
    #         x, y, z = map(float, point)
    #         trajectory.append((x, y, z))
        
    #     # Enviar la trayectoria al robot
    #     move_through_trajectory(trajectory)
    #     print("Trayectoria enviada al robot:", trajectory)

    def add_to_history(self, x, y, z, result):
        history_entry = f"Coordenadas: ({x}, {y}, {z}) - Resultado: {result}"
        self.history.append(history_entry)
        self.history_list.addItem(history_entry)

    def clear_history(self):
        self.history.clear()
        self.history_list.clear()

def main(args=None):
    rclpy.init(args=args)
    delta_robot_controller = DeltaRobotController()
    app = QApplication(sys.argv)
    window = MainWindow(delta_robot_controller)
    window.show()
    delta_robot_controller.destroy_node()
    rclpy.shutdown()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
