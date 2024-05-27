import open3d as o3d
import open3d.visualization.gui as gui # type: ignore
import open3d.visualization.rendering as rendering # type: ignore
import platform
import random
import threading
import time
from src.model.model import Model
import numpy as np

isMacOS = (platform.system() == "Darwin")

class CARLA2NMR_App:
    MENU_QUIT = 1
    MENU_LOAD_FILE = 2
    MENU_SHOW_CAMERA = 3
    MENU_SHOW_LIDAR = 4
    MENU_SHOW_SETTING = 5

    def __init__(self):
        self.model = None
        self.window = gui.Application.instance.create_window(
            "CARLA2NMR Viewer", 1280, 720)
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)

        # Disable lighting to avoid reflections
        self.scene.scene.scene.set_sun_light([0, 0, 0], [0, 0, 0], 0)
        self.scene.scene.scene.enable_sun_light(False)


        self.window.add_child(self.scene)

        # The menu is global (because the macOS menu is global), so only create
        # it once, no matter how many windows are created
        if gui.Application.instance.menubar is None:
            if isMacOS:
                app_menu = gui.Menu()
                app_menu.add_item("Quit", CARLA2NMR_App.MENU_QUIT)
            file_menu = gui.Menu()
            file_menu.add_item("Open Dir...", CARLA2NMR_App.MENU_LOAD_FILE)
            view_menu = gui.Menu()
            view_menu.add_item("Show Camera", CARLA2NMR_App.MENU_SHOW_CAMERA)
            view_menu.add_item("Show Lidar", CARLA2NMR_App.MENU_SHOW_LIDAR)
            actions_menu = gui.Menu()
            actions_menu.add_item("Show Settings", CARLA2NMR_App.MENU_SHOW_SETTING)
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
        self.window.set_on_menu_item_activated(CARLA2NMR_App.MENU_SHOW_LIDAR,
                                                self._on_menu_show_lidar)
        self.window.set_on_menu_item_activated(CARLA2NMR_App.MENU_SHOW_SETTING,
                                                self._on_menu_show_settings)

        # Set widgets
        em = self.window.theme.font_size
        self._pannel = gui.Vert(0, gui.Margins(0.25*em,0.25*em,0.25*em,0.254*em))

        
        # Set Slider 
        slider = gui.Slider(gui.Slider.INT)
        slider.set_limits(0, 149)
        slider.set_on_value_changed(self._on_slider)
        self._pannel.add_child(slider)


        self._button = gui.Button("Set Default Camera View")
        self._button.set_on_clicked(self._set_default_camera_view)
        self._pannel.add_child(self._button)

        # 布局回调函数
        self.window.set_on_layout(self._on_layout)
        self.window.add_child(self._pannel)

    def _on_layout(self, layout_context):
        #   在on_layout回调函数中应正确设置所有子对象的框架(position + size)，
        #   回调结束之后才会布局孙子对象。

        r = self.window.content_rect
        self.scene.frame = r

        pannel_width = 17*layout_context.theme.font_size
        pannel_height = min(
            r.height, self._pannel.calc_preferred_size(
                layout_context, gui.Widget.Constraints()).height
        )
        self._pannel.frame = gui.Rect(r.get_right()-pannel_width,r.y,pannel_width,pannel_height)

        button_pref = self._button.calc_preferred_size(
            layout_context, gui.Widget.Constraints())
        self._button.frame = gui.Rect(r.x,r.get_bottom()-button_pref.height, button_pref.width,button_pref.height)
        print(r.x,r.y)


    def _on_slider(self, new_val):
        self._on_menu_show_lidar(int(new_val))

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
        cams_axis, cams_mesh, cams_line_set = self.model.add_cameras()
        self.scene.scene.clear_geometry()

        mat_axis = rendering.MaterialRecord()
        mat_axis.shader = "defaultLit"
        mat_axis.base_color = [1, 1, 1, 1]
        for id, axis in enumerate(cams_axis):
            self.scene.scene.add_geometry(f"axis{id}", axis, mat_axis)

        mat_mesh = rendering.MaterialRecord()

        for id, mesh in enumerate(cams_mesh):
            texture = np.asarray(mesh.textures[0]).copy()
            texture = o3d.geometry.Image(texture)
            mat_mesh.albedo_img = texture 

            self.scene.scene.add_geometry(f"mesh{id}", mesh, mat_mesh)

        mat_line_set = rendering.MaterialRecord()
        mat_line_set.shader = "unlitLine"
        mat_line_set.line_width = 1 

        for id, line_set in enumerate(cams_line_set):
            self.scene.scene.add_geometry(f"line_set{id}", line_set, mat_line_set)

    def _on_menu_show_lidar(self, idx=None):
        pcd = self.model.add_lidar(idx)
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        mat.base_color = [1, 1, 1, 1]
        self.scene.scene.remove_geometry("lidar")
        self.scene.scene.add_geometry("lidar", pcd, mat)

    def _on_menu_show_settings(self):
        pass

    def _set_default_camera_view(self):
        try:
            W2C = self.model.get_transform(0)
            K = self.model.get_intrinsics(0)
        except:
            W2C = None
            K = None

        if W2C is not None and K is not None:
            # Extract rotation (R) and translation (t) from the W2C matrix
            R = W2C[:3, :3]
            t = W2C[:3, 3]

            # Create extrinsic matrix for Open3D
            extrinsic = np.eye(4)
            extrinsic[:3, :3] = R
            extrinsic[:3, 3] = t

            # Get intrinsic parameters from K
            fx = K[0, 0]
            fy = K[1, 1]
            cx = K[0, 2]
            cy = K[1, 2]

            # Create intrinsic matrix for Open3D
            intrinsic = o3d.camera.PinholeCameraIntrinsic()
            intrinsic.set_intrinsics(
                width=1280, height=720, fx=fx, fy=fy, cx=cx, cy=cy
            )

            # Get bounding box of the scene
            bounds = self.scene.scene.bounding_box

            # Setup camera with the extrinsic matrix (W2C should be the inverse of the camera pose)
            self.scene.setup_camera(intrinsic, np.linalg.inv(W2C), bounds)
        else:
            # Default behavior if no W2C matrix is provided
            bounds = self.scene.scene.bounding_box
            self.scene.setup_camera(60.0, bounds, bounds.get_center())
            self.scene.look_at(bounds.get_center(), bounds.get_center() + [0, 0, -1], [0, 1, 0])

