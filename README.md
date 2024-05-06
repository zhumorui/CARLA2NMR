# CARLA2NMR


model/model.py -- 加载CARLA2NMR的项目文件,为整个项目的核心。

vis_utils.py -- 负责一些可视化的小插件

gui/main_window.py -- 主要的窗口
gui/widgets.py --窗口中的小部件

utils/colmap/python -- colmap的官方脚本，提供一系列的3D数据处理工具

https://vscode.dev/github/zhumorui/CARLA2NMR/blob/main/src/gui/app.py#L43 : 在这个menu里添加一些基本功能, 包括about, preferences, quit


model里储存的3D文件的路径, 只有在被调用的时候才会被加载, 以节省内存

Model的基本属性:
1. cameras: 相机的内参
2. images: 储存了相机拍摄的图片
3. points3D: 储存了global点云
4. lidar_pcds: 储存了每一帧的lidar点云
5. poses: 储存了每一帧的相机位姿


cameras:

{1: Camera(id=1, model='SIMPLE_PINHOLE', width=1280, height=720, params=array([369.50417228, 640.        , 360.        ]))}


images: 

{1: Image(id=1, qvec=array([ 1.60577656e-02,  7.69582344e-05,  9.99871060e-01, -8.03012091e-05]), tvec=array([-5.25953003,  2.12081622, 32.3448464 ]), camera_id=1, name='0001.png', xys=array([], shape=(0, 2), dtype=float64), point3D_ids=array([], dtype=float64))}

masks: 

['/Users/morin/carla_12_20_rgb_1_1/mask/0001.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0002.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0003.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0004.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0005.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0006.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0007.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0008.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0009.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0010.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0011.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0012.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0013.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0014.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0015.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0016.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0017.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0018.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0019.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0020.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0021.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0022.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0023.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0024.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0025.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0026.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0027.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0028.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0029.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0030.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0031.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0032.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0033.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0034.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0035.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0036.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0037.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0038.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0039.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0040.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0041.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0042.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0043.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0044.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0045.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0046.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0047.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0048.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0049.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0050.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0051.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0052.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0053.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0054.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0055.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0056.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0057.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0058.png.png', '/Users/morin/carla_12_20_rgb_1_1/mask/0059.png.png', ...]


TODO：
增加show_lidar widget, 使其可以通过滑动条的方式选择特定的lidar pcd
Actions:
    Show Settings



Settings:
保留draw当中的
Mouse controls:
    -
    -

Scene:
    - select lidar （滑动条）


修复mesh上图像倒置的问题
计算图像2d 到3D点的投射， filter lidar pcd
