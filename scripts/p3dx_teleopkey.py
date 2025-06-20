#!/usr/bin/env python

import rospy
import tkinter as tk
import signal
import sys
from geometry_msgs.msg import Twist

pub = None

class KeyDisplayApp:
    def __init__(self, root):
        self.root = root
        self.root.title("P3Dx_teleop_key")
        self.root.geometry("300x200")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)  # Trata o fechamento pelo "X"

        self.label = tk.Label(root, text="Pressione uma tecla", font=("Arial", 14))
        self.label.pack(expand=True)

        # Eventos de teclado
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<Escape>", self.on_escape)  # Fechar com ESC
        root.bind("<KeyRelease>", self.on_key_release)

    def on_key_release(self, event):
        global pub
        twist_msg = Twist()  # Criando uma instância de Twist
        twist_msg.linear.x = 0  # Move forward with a speed of 0.5 m/s
        twist_msg.angular.z = 0  # Rotate with a speed of 0.1 rad/s

        # Publish the message
        pub.publish(twist_msg)

    def on_key_press(self, event):
        global pub
        twist_msg = Twist()  # Criando uma instância de Twist
        twist_msg.linear.x = 0  # Move forward with a speed of 0.5 m/s
        twist_msg.angular.z = 0  # Rotate with a speed of 0.1 rad/s

        tecla = event.keysym
        self.label.config(text=f"Última tecla: {tecla}")
        print(f"Tecla pressionada: {tecla}")

        if tecla == "esc":
            signal_handler(None, None)
        elif tecla == 'w' or tecla == 'Up': # Frente
            twist_msg.linear.x = 0.5  # Move forward with a speed of 0.5 m/s
            twist_msg.angular.z = 0  # Rotate with a speed of 0.1 rad/s
        elif tecla == 's' or tecla == 'Down': # Ré
            twist_msg.linear.x = -0.5  # Move forward with a speed of 0.5 m/s
            twist_msg.angular.z = 0  # Rotate with a speed of 0.1 rad/s
        elif tecla == 'a'  or tecla == 'Left': # Giro
            twist_msg.linear.x = 0  # Move forward with a speed of 0.5 m/s
            twist_msg.angular.z = 0.5  # Rotate with a speed of 0.1 rad/s
        elif tecla == 'd' or tecla == 'Right': # Frente
            twist_msg.linear.x = 0  # Move forward with a speed of 0.5 m/s
            twist_msg.angular.z = -0.5  # Rotate with a speed of 0.1 rad/s

        # Publish the message
        pub.publish(twist_msg)

    def on_escape(self, event):
        print("Fechando via ESC...")
        self.on_close()

    def on_close(self):
        print("Fechando a janela...")
        self.root.destroy()  # Destrói a janela completamente


def signal_handler(sig, frame):
    """Trata o sinal SIGINT (Ctrl+C) para fechar o programa."""
    print("\nCtrl+C pressionado: encerrando o programa...")
    rospy.signal_shutdown("Ctrl+C pressionado")
    sys.exit(0)


def main():
    global pub
    rospy.init_node('p3dx_teleop_key', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)  # Captura Ctrl+C

    # pub = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

    root = tk.Tk()
    app = KeyDisplayApp(root)

    print("A janela do Tkinter está rodando. Pressione ESC ou Ctrl+C para sair.")

    # Loop principal integrado
    try:
        while not rospy.is_shutdown():
            root.update()  # Atualiza a janela do Tkinter
            root.update_idletasks()
            rospy.sleep(0.01)
    except (rospy.ROSInterruptException, tk.TclError):
        pass
    finally:
        if root.winfo_exists():
            root.destroy()  # Garante o fechamento da janela


if __name__ == '__main__':
    main()
