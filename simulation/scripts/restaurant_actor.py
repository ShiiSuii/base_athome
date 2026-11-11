#!/usr/bin/env python3
"""Clientes y barman simulados para probar Restaurant en Gazebo (dominio 42). Escucha lo que dice el robot y,
cuando el microfono simulado abre, contesta: el pedido de cada cliente o "ready" al barman. Solo simulacion."""
import sys
import os
import json
import threading

import rclpy
from std_msgs.msg import String

ORDERS = ["coke and water", "juice and tea"]


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        raise SystemExit("Restaurant actor requires isolated ROS_DOMAIN_ID=42")
    rclpy.init()
    node = rclpy.create_node("restaurant_actor")
    pub = node.create_publisher(String, "/sim/speech", 10)
    state = dict(last="", customer=0, answered=0)
    if os.environ.get("ATHOME_SIM_REAL_PERCEPTION") == "1":
        from base_athome_sim.restaurant_humans import reply_for_state
        mission = {}
        def on_state(msg):
            mission.clear()
            mission.update(json.loads(msg.data))
        def on_request(msg):
            request = json.loads(msg.data)
            if request.get("prompt") == "Stop.":
                return
            text = reply_for_state(mission)
            threading.Timer(1.0, lambda: pub.publish(String(data=text))).start()
        node.create_subscription(String, "/restaurant/state", on_state, 10)
        node.create_subscription(String, "/voice/listen_request", on_request, 10)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return


    def heard(msg):
        state["last"] = msg.data.lower()
        print(f"ROBOT: {msg.data}", flush=True)

    def status(msg):
        if msg.data != "LISTENING" or state["answered"] == id(state["last"]):
            return
        said = state["last"]
        if "mission complete" in said or "enjoy your order" in said:
            return
        if "what is your order" in said or "would you like" in said or "what can i" in said:
            # el pedido de este cliente; si el robot pide repetir, se repite el mismo
            text = "my order is " + ORDERS[min(state["customer"], len(ORDERS) - 1)]
        elif "are you the barman" in said:
            text = "robot yes"     # el barman esta de pie detras de la barra (ultimo de ATHOME_SIM_CUSTOMERS en restaurant.sh)
        elif "robot yes" in said or "?" in said:
            # are you the barman / do you have an order / is that correct / tell me when done / did you take it
            text = "robot yes"
            if "is that correct" in said or "your order is" in said:
                state["customer"] += 1
        elif "tray" in said or "please place" in said or "please put" in said:
            text = "robot yes"
        else:
            return
        state["answered"] = id(said)
        threading.Timer(1.0, lambda: (pub.publish(String(data=text)), print(f"PERSONA: {text}", flush=True))).start()

    node.create_subscription(String, "/hri/say_something", heard, 10)
    node.create_subscription(String, "/voice/status", status, 10)
    rclpy.spin(node)


if __name__ == "__main__":
    sys.exit(main())
