#!/usr/bin/env python3
"""Clientes y barman simulados para probar Restaurant en Gazebo (dominio 42). Escucha lo que dice el robot y,
cuando el microfono simulado abre, contesta: el pedido de cada cliente o "ready" al barman. Solo simulacion."""
import sys
import threading

import rclpy
from std_msgs.msg import String

ORDERS = ["I would like a coke and a water please", "a juice and a tea please"]


def main():
    rclpy.init()
    node = rclpy.create_node("restaurant_actor")
    pub = node.create_publisher(String, "/sim/speech", 10)
    state = dict(last="", customer=0, answered=0)

    def heard(msg):
        state["last"] = msg.data.lower()
        print(f"ROBOT: {msg.data}", flush=True)

    def status(msg):
        if msg.data != "LISTENING" or state["answered"] == id(state["last"]):
            return
        said = state["last"]
        if "here is your" in said or "mission complete" in said:     # entrega o fin: el cliente no pide de nuevo
            return
        if "order" in said or "would you like" in said or "what can i" in said:
            text = ORDERS[min(state["customer"], len(ORDERS) - 1)]
            state["customer"] += 1
        elif "tray" in said or "barman" in said or "please place" in said or "please put" in said:
            text = "ready"
        elif "?" in said:
            text = "yes"
        else:
            return
        state["answered"] = id(said)
        threading.Timer(1.0, lambda: (pub.publish(String(data=text)), print(f"PERSONA: {text}", flush=True))).start()

    node.create_subscription(String, "/hri/say_something", heard, 10)
    node.create_subscription(String, "/voice/status", status, 10)
    rclpy.spin(node)


if __name__ == "__main__":
    sys.exit(main())
