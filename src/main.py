# main.py
import sys
sys.path.append("/home/morin/ZHITAI_SSD/projects/CARLA2NMR")
from open3d.visualization import gui
from src.gui.app import CARLA2NMR_App

def main():
    gui.Application.instance.initialize()
    CARLA2NMR_App()
    gui.Application.instance.run()


if __name__ == "__main__":
    main()