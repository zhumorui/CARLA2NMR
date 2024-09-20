# main.py
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from open3d.visualization import gui
from src.gui.app import CARLA2NMR_App

def main():
    gui.Application.instance.initialize()
    CARLA2NMR_App()
    gui.Application.instance.run()


if __name__ == "__main__":
    main()