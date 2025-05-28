#Exercise Title:    "LED-Strip"
#Name:              Rafael Hausch  
#Group:             2
#Class:             4BHME
#Date:              30.01.2024

import rclpy
import rclpy.logging
from rclpy.qos import QoSProfile
from std_msgs.msg import String, UInt8MultiArray

import os
import select
import sys
import time

import termios
import tty

# Fragen:
#--------------------------------
# Frage 1 - Wie kannst du die ROS_DOMAIN_ID in der CLI auf 0 setzen?
# Antwort 1: export ROS_DOMAIN_ID=0 

# Frage 2 - Wofür wird die ROS_DOMAIN_ID verwendet, und was passiert, wenn du sie auf 0 setzt?
# Antwort 2: Man verwendet sie um zwischen Robotern unterscheiden zu können, wenn mehrere in einem netzwerk sind.
#           wenn man sie auf 0 setzt kann man nur mit node kommunizieren die auch ihre ID auf 0 haben
#           0 ist die standart id

# Frage 3 - Warum haben Anwender, die eine virtuelle Maschine nutzen, Probleme damit, ROS-Nachrichten von Remote-Nodes anzuzeigen?
# Antwort 3: Da man in einem virtuellen Netzwerk ist.
#            Die Virtuelle Maschine ist standartmäßig in einem seperaten netzwerk konfiguriert -> NAT

# Frage 4 - Wie kannst du über die CLI herausfinden, welche ROS-Nachrichten gerade verfügbar sind?
# Antwort 4: ros2 topic list

# Frage 5 - Wie lautet der CLI-Befehl, um die USER-LED zu schalten?
# Antwort 5:  ros2 topic pub /user std_msg/msg/String "{data: 'hauran21'}"

# Frage 6 - Wie lautet der CLI-Befehl, um die TEAM-LED zu schalten?
# Antwort 6: ros2 topic pub /team2 std.msgs/msg/UInt8MultiArray '{data: {4, 100, 0, 0, 0}}' -l

# Frage 7 - Was ist ein PWM-Signal, wofür wird es in unserem Beispiel verwendet, und wie bzw. von wem wird es erzeugt?
# Antwort 7: Puls-Weiten-Modulation -> frequenz bleibt gleich man verändert das Verhältnis von an zu aus ; 
#            Wir verwenden es um die helligkeit einer LED zu steueren
#            Vom GPIO Controller 

# Frage 8 - Was musst du tun, um deinen ROS-Node über einen ROS-Befehl in der CLI starten zu können? Wie könnte das Programm alternativ gestartet werden?
# Antwort 8: -> in die setup.py muss das Executable hinterlegt werden.
#               python3 programmname

# Frage 9 - Welche LED-Farbe hat deine individuelle LED?
# Antwort 9: Grün -> [0,255,0,0] -> RGBW

# Frage 10 - Welchen Datentyp müssen die ROS-Nachrichten mit den Topics "user" und "team" haben?
# Antwort 10: user=String ; team=msg:UInt8MultiArray


# Programmieraufgabe 1: 
# --------------------------------
# Programmiere ein Programm zur Steuerung von RGBW-LEDs und speichere diese in dein robu Paket. Ein Subscriber-Node wurde bereits programmiert 
# und auf dem Raspberry gestartet 
# (siehe: https://github.com/mlieschnegg/robu_rpi_ws/blob/main/src/robu_rpi_examples/robu_rpi_examples/ledstrip_sub.py, von LI/SH). 
# Die Topics dieser ROS-Nachrichten lauten "user" sowie "team1" bis "team6".
# 
# Am LED-Streifen sind folgende LEDs reserviert:
#    - Eine individuelle LED (Topic "user") für dich. 
#    - Fünf LEDs für deine ROBU-Gruppe (Topics "team1" bis "team6").
#
# Implementiere:
#    - Je einen Publisher für deine individuelle LED sowie die LEDs deines Teams.
# 
# Anforderungen:
#    - Wenn du die Taste 'u' drückst, soll die individuelle LED ein- bzw. ausgeschaltet werden.
#    - Wenn du die Taste 't' drückst, sollen die LEDs deines Teams mit einer bestimmten Farbe (freie Wahl!) konfiguriert werden.
# 
# Hinweis: 
#    - Der grundlegende Aufbau deines Programms entspricht dem Programm aus Übung 6.
#    - Studiere den Code des Subscribers und finde heraus welchen Datentyp und Nachrichteninhalt 
#      du beim Puschlisher zum Schalten der LEDs verwenden musst.

#Abgabe
#------------------------
#Drucke diese Dokument aus (doppelseitig) und gib es bei LI/SH ab

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


def main():
    rclpy.init() 
    qos = QoSProfile(depth=10) 
    node = rclpy.create_node('led_strip') 
    
    
    pub_user = node.create_publisher(String, 'user', qos) 
    pub_team = node.create_publisher(UInt8MultiArray, 'team1', qos) 

    vel_user = String()
    vel_team = UInt8MultiArray()

    #vel_user = "hauran21"
    #vel_team = [0, 255, 0, 0, 0, 3, 0, 255, 0, 0]

    try:
        while(1): 
            key = get_key()
            if key != '':
                str = "String: " + key.replace(chr(0x1B), '^') + ", Code:"
                for c in key:
                    str += " %d" % (ord(c))
                print(str)

                if ord(key[0]) == 0x03: #Unicode für strg+c
                    break
                            
                elif key == "u":
                    vel_user.data = "hauran21" #input("Username eingeben: ")
                    pub_user.publish(vel_user)

                elif key == "t":
                    vel_team.data = [0, 255, 0, 0, 0, 3, 0, 255, 0, 0]
                    pub_team.publish(vel_team)
            
                # Daten werden an Subscriber Nodes gesendet 

    # Roboter ausschalten -> alles auf 0 setzen
    finally:
        # Node 
        node.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()