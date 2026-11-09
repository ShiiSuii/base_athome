-- Base@Home: el parlante Bluetooth del robot se desconectaba en cada pausa. Al suspender el sink, WirePlumber
-- liberaba el transporte, fallaba ("Failed to release transport") y el parlante cortaba la conexion.
-- No suspender nunca las salidas Bluetooth (2026-11-09).
rule = {
  matches = {
    {
      { "node.name", "matches", "bluez_output.*" },
    },
  },
  apply_properties = {
    ["session.suspend-timeout-seconds"] = 0,
  },
}
table.insert(bluez_monitor.rules, rule)
