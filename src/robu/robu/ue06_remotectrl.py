#Exercise Title:    Remote Control for the TurtleBot3 MRBurger
#Group:             ?
#Class:             ?
#Date:              ?


# ROS Packages
import rclpy    # ROS Framework/Bibliothek roscontrol library -> twist, Publisher, Subscriber
from geometry_msgs.msg import Twist # Datentyp twist importieren (Standart Geometrie-Datentypen für ROS)
from rclpy.qos import QoSProfile    # Quality of Service -> wie wichtig eine Message in ROS ist (darf was verloren gehen???)

# System Packages
import os       # Betriebsystemabhängige library -> Pfade erstellen, OS Tools
import select   # Threats und Multitasking
import sys      # Systembibliothek -> Ausführen lassen in der Konsole, z.B. vor dem Start Pfad wechseln, übergabe systemargumente (cd, ls), System Events (strg + c) abfangen
import time     # Programm anhalten z.B. time.sleep(1) -> Programm schläft für 1 sek

# Terminal Packages
import termios #Terminal input output
import tty


# Fragen ob das Betriebssystem Windows ist
# if os.name == 'nt':
#     import msvcrt
# else:    

#Start Message
msg = """   
Excercise:  ?
Group:      ?
Class:      ?
Date:       ?
"""

#Error Message
e = """
Communications Failed
"""

#Physikalische Grenzen des Roboters
MAX_LIN_VEL = 0.22          #m/s
MAX_ANG_VEL = 2.84          #rad/s

LIN_VEL_STEP_SIZE = 0.01    #m/s
ANG_VEL_STEP_SIZE = 0.1     #rad/s

# Gibt das einzelne Zeichen der Tastatur gerade gehalten wird zurück 
def get_key():
    old_settings = termios.tcgetattr(sys.stdin)
    ts = time.time()
    key = ''
    try:
        tty.setraw(sys.stdin.fileno())
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key += os.read(sys.stdin.fileno(), 1).decode("utf-8")
            else:
                break
        return key
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# Hauptprogramm
def main():
    
    rclpy.init() # ROS initialisieren -> !!!Wichtig

    # Nachrichtendringlichkeit festlegen für ROS -> Max 10 Nachrichten aufstauen/zwischengespeichert im Fall von Netzwerkproblemen, alle weiteren werden gelöscht(immer der älteste)
    qos = QoSProfile(depth=10) 
    
    # Node mit dem Namen "remotectrl" erstellen -> Später machen wir eine Klasse Node 
    node = rclpy.create_node('remotectrl') 
    
    # Erstellt einen Publisher für den Twist mit dem Namen "cmd_vel" -> erstellt ein Objekt mit dem Namen "pub"
    pub = node.create_publisher(Twist, 'cmd_vel', qos) 

    # Die Variable mit dem Datentypen Twist erstellen -> Datenerhaltungsklasse der Geschwindigkeit
    vel = Twist()
    
    # Anwenden von vel / ins Netzwerk übertragen 
    # pub.publish(vel)

    # Alles in einem Loop über eine while laufen lassen 
    # kann exceptions werfen -> z.B. man drückt strg + c
    try:
        while(1): 
            key = get_key()
            if key != '':
                str = "String: " + key.replace(chr(0x1B), '^') + ", Code:"
                for c in key:
                    str += " %d" % (ord(c))
                print(str)
                
                # Wenn strg+c gedrückt wird soll das Programm beendet werden
                if ord(key[0]) == 0x03: #Unicode für strg+c
                        break
						        
				# Bei Leertaste soll der Roboter stehen bleiben
                elif key == chr(0x1B): #Unicode für Leertaste
                    print("Roboter bleibt stehen")
                    vel.linear.x = 0.0  
                    vel.linear.y = 0.0 
                    vel.linear.z = 0.0 
                    vel.angular.x = 0.0
                    vel.angular.y = 0.0
                    vel.angular.z = 0.0
                            
                # Pfeil nach oben ist nach vorne fahren    
                elif key == "\x1B[A":
                    vel.linear.x += LIN_VEL_STEP_SIZE
                    if vel.linear.x > MAX_LIN_VEL:
                        vel.linear.x = MAX_LIN_VEL
                        node.get_logger().info("MaximalVorwaertsGeschwindigkeit erreicht") #Schreibt die Nachricht ins Terminal und in ein Log File
                        print("Max-V erreicht")
                
                # Pfeiltaste nach unten zum nach hinten fahren
                elif key == "\x1B[B":
                    vel.linear.x -= LIN_VEL_STEP_SIZE
                    node.get_logger().info("MaximalRueckwertsGeschwindigkeit erreicht") #Schreibt die Nachricht ins Terminal und in ein Log File
                    print("Max-V erreicht")
                
                # Daten werden an Subscriber Nodes gesendet 
                pub.publish(vel) 
				          
		             		                               
    except Exception as e:
        print(e) # Exceptionstext drucken

    # Roboter ausschalten -> alles auf 0 setzen
    finally:
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        pub.publish(vel)

        # Node 
        node.destroy_node()
        rclpy.shutdown() 

    if __name__ == '__main__':
        main()
