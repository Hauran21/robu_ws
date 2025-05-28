import rclpy 
from std_msgs.msg import String, UInt8MultiArray

def callback_user(msg:String):
    global node_ledstrip
    print(f"User: {msg.data}") # f = formated string
    node_ledstrip.get_logger().info(f"User: {msg.data}")

def callback_team(msg:UInt8MultiArray):
    global node_ledstrip
    print(f"team: {msg.data}")
    node_ledstrip.get_logger().info(f"Team-LEDs: {msg.data}")

def main():
    global node_ledstrip
    
    try:
        rclpy.init()
        node_ledstrip = rclpy.create_node('ledstrip_subscriber')

        node_ledstrip.create_subscription(String, "user", callback_user, 10)
        node_ledstrip.create_subscription(UInt8MultiArray, "team1", callback_team, 10)

        # Node wird nun ausgeführt -> hier bleibt das Programm "hängen"
        rclpy.spin(node_ledstrip)

    except KeyboardInterrupt as e:
        print("Sie haben STRG+C gedrückt!")
        #terminal Befehl oder STRG+C: killall /usr/python3
        #Terminal Befehl ob das Programm ganz sicher zu beenden: killall -9 Usr/python3
        
    finally:
        node_ledstrip.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            pass

if __name__ == '__main__':
    main()