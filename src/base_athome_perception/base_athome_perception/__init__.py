import os

def check_and_fix_cyclonedds():
    """Protege contra fallos de CycloneDDS si enp1s0 no tiene conexion fisica."""
    dds_uri = os.environ.get("CYCLONEDDS_URI", "")
    if "enp1s0" in dds_uri or "cyclonedds.xml" in dds_uri:
        carrier_path = "/sys/class/net/enp1s0/carrier"
        carrier_ok = False
        if os.path.exists(carrier_path):
            try:
                with open(carrier_path, "r") as f:
                    carrier_ok = (f.read().strip() == "1")
            except Exception:
                carrier_ok = False
        if not carrier_ok:
            local_xml = "/home/athome/cyclonedds_local.xml"
            if os.path.exists(local_xml):
                os.environ["CYCLONEDDS_URI"] = f"file://{local_xml}"
            else:
                os.environ.pop("CYCLONEDDS_URI", None)

# Ejecutar automáticamente al importar el paquete
check_and_fix_cyclonedds()

from .face_recognizer import FaceRecognizer
from .speaker import RoboCupSpeaker

__all__ = ["FaceRecognizer", "RoboCupSpeaker", "check_and_fix_cyclonedds"]

