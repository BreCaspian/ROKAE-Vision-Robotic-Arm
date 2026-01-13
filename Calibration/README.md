# 标定工具说明

本目录提供一个交互式标定工具，用于估算相机像素与机器人毫米之间的比例关系，并输出
可直接粘贴到 `Workspace/Socket/Socket.py` 的参数片段。

## 依赖环境

- Python 3.x
- 海康 MVS SDK 已安装并可正常调用
- Python 包：

```bash
pip install numpy opencv-python
```

## 运行方式

在项目根目录执行：

```bash
python Workspace/Calibration/calibrate_circle.py
```

## 工具原理

工具会用 OpenCV 的 Hough 圆检测找到圆形目标，要求你在每个轴向上采集两次点
（A/B），并输入实际移动距离（mm），从而计算：

- `_k_mm_per_px`（毫米 / 像素）
- `_sign_x` / `_sign_y`（方向符号）

## 窗口按键

- `c`：采集当前帧
- `q`：退出

窗口显示说明：

- 黄色十字：图像中心
- 绿色圆圈：检测到的圆心

## 使用步骤

1. 将圆形目标放在相机视野内并保持可见。
2. 选择是否标定 X 轴 / Y 轴。
3. 标定 X：
   - 按 `c` 采集点 A。
   - 将目标沿机器人 +X 方向移动已知距离（mm）。
   - 按 `c` 采集点 B。
   - 输入实际移动距离。
4. 标定 Y：按同样流程，沿机器人 +Y 方向移动。
5. 将工具输出的参数片段粘贴到 `Workspace/Socket/Socket.py`。

## 输出示例

```
=== Suggested Socket.py params ===
_k_mm_per_px = 0.250000
_sign_x = +1.0
_sign_y = -1.0
```

如果 X/Y 比例明显不同，建议在代码里改成 `kx/ky` 分开处理。


```
(YOLO_Improved) PS D:\Desktop\RoboMaster\RoboticArm\ROKAE> python Workspace/Calibration/calibrate_circle.py
Found 1 devices, use index 0
HikCamera initialized OK.
Calibration tool started.
Make sure the circle is visible in the camera view.
Calibrate X axis? (y/n): y
Calibrate Y axis? (y/n): y

=== Calibrate X-axis ===
Place the circle near image center.
Press 'c' to capture, 'q' to quit. Click the window to focus.
No circle found, try again...
A: (u, v)=(1435.2, 1668.5)
Move target along robot +X by a known distance (mm).
Press 'c' to capture, 'q' to quit. Click the window to focus.
B: (u, v)=(1494.8, 1134.0)
Enter the moved distance along +X (mm): 50                  
du=59.65 px, kx=0.838159 mm/px, sign_x=+1

=== Calibrate Y-axis ===
Place the circle near image center.
Press 'c' to capture, 'q' to quit. Click the window to focus.
A: (u, v)=(2879.6, 1245.8)
Move target along robot +Y by a known distance (mm).
Press 'c' to capture, 'q' to quit. Click the window to focus.
B: (u, v)=(1811.5, 1171.5)
Enter the moved distance along +Y (mm): 100
dv=-74.26 px, ky=1.346629 mm/px, sign_y=-1

=== Suggested Socket.py params ===
_sign_y = -1.0
_k_mm_per_px = 1.346629
HikCamera closed.
(YOLO_Improved) PS D:\Desktop\RoboMaster\RoboticArm\ROKAE> 
```