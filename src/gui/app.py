import open3d as o3d
import open3d.visualization.gui as gui # type: ignore
import open3d.visualization.rendering as rendering # type: ignore
import platform
import random
import threading
import time
from src.model.model import Model

isMacOS = (platform.system() == "Darwin")

class CARLA2NMR_App:
    MENU_QUIT = 1
    MENU_LOAD_FILE = 2
    MENU_SHOW_CAMERA = 3
    MENU_SHOW_IMAGE = 4

    def __init__(self):
        self.model = None
        self.window = gui.Application.instance.create_window(
            "CARLA2NMR Viewer", 1024, 768)
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.scene.scene.set_background([1, 1, 1, 1])
        self.scene.scene.scene.set_sun_light(
            [-1, -1, -1],  # direction
            [1, 1, 1],  # color
            100000)  # intensity
        self.scene.scene.scene.enable_sun_light(True)
        self.window.add_child(self.scene)

        # The menu is global (because the macOS menu is global), so only create
        # it once, no matter how many windows are created
        if gui.Application.instance.menubar is None:
            if isMacOS:
                app_menu = gui.Menu()
                app_menu.add_item("Quit", CARLA2NMR_App.MENU_QUIT)
            file_menu = gui.Menu()
            file_menu.add_item("Open Dir", CARLA2NMR_App.MENU_LOAD_FILE)
            view_menu = gui.Menu()
            view_menu.add_item("Show Camera", CARLA2NMR_App.MENU_SHOW_CAMERA)
            if not isMacOS:
                file_menu.add_separator()
                file_menu.add_item("Quit", CARLA2NMR_App.MENU_QUIT)

            menu = gui.Menu()
            if isMacOS:
                # macOS will name the first menu item for the running application
                # (in our case, probably "Python"), regardless of what we call
                # it. This is the application menu, and it is where the
                # About..., Preferences..., and Quit menu items typically go.
                menu.add_menu("Python", app_menu) 
                menu.add_menu("File", file_menu)
                menu.add_menu("View", view_menu)
            else:
                menu.add_menu("File", file_menu)
                menu.add_menu("View", view_menu)
            gui.Application.instance.menubar = menu

        # The menubar is global, but we need to connect the menu items to the
        # window, so that the window can call the appropriate function when the
        # menu item is activated.
        self.window.set_on_menu_item_activated(CARLA2NMR_App.MENU_QUIT,
                                               self._on_menu_quit)
        self.window.set_on_menu_item_activated(CARLA2NMR_App.MENU_LOAD_FILE,
                                               self._on_menu_filedlg)
        self.window.set_on_menu_item_activated(CARLA2NMR_App.MENU_SHOW_CAMERA,
                                                self._on_menu_show_camera)
        # self.window.set_on_menu_item_activated(CARLA2NMR_App.MENU_SHOW_IMAGE,
        #                                         self._on_menu_show_image)

    def _on_menu_quit(self):
        gui.Application.instance.quit()

    def _on_menu_filedlg(self):
        filedlg = gui.FileDialog(gui.FileDialog.OPEN_DIR, "Select dir",
                                 self.window.theme)
        filedlg.set_on_cancel(self._on_filedlg_cancel)
        filedlg.set_on_done(self._on_filedlg_done)
        self.window.show_dialog(filedlg)

    def _on_filedlg_cancel(self):
        self.window.close_dialog()

    def _on_filedlg_done(self, path):
        self.model = Model(path, ext=".txt")
        print(f"Load model successfully!\n{self.model}")
        # self._on_menu_show_camera()
        self.window.close_dialog()

    def _on_menu_show_camera(self):
        frames = self.model.add_cameras()
        self.scene.scene.clear_geometry()
        for id, cam in zip(range(10), frames[:10]):
            self.scene.scene.add_model(f"camera_{id}", cam)
        

    def _on_menu_show_image(self):
        pass