#!/usr/bin/env python3
"""Ventana de manipulacion para las poses de Restaurant (alias restaurantgrasping)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from manipulacion_simple import main  # noqa: E402

from base_athome_perception.restaurant_grasping import window_class  # noqa: E402

if __name__ == "__main__":
    main(window_class())
