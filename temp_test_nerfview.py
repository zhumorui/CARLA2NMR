import nerfview
import numpy as np
from typing import Tuple, Union, Optional
import viser
import time



def render_fn(camera_state: nerfview.CameraState, resolution: Tuple[int, int]) -> Union[
    np.ndarray,
    Tuple[np.ndarray, Optional[np.ndarray]]
]:
    height, width = resolution
    
    # 使用camera_state.c2w矩阵的值来生成一个随机种子
    seed = int(np.sum(camera_state.c2w) * 1000) % (2**32 - 1)
    np.random.seed(seed)
    
    # 生成随机颜色
    random_color = np.random.randint(0, 256, 3, dtype=np.uint8)
    
    # 创建一个RGB图像，填充随机颜色
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :] = random_color
    
    # 创建一个简单的深度图
    depth_map = np.ones((height, width), dtype=np.float32)  # 假设深度值为1.0

    # 返回图像和深度图
    return image, depth_map


server = viser.ViserServer()

nerfview.Viewer(server, render_fn)


while True:
    time.sleep(10)