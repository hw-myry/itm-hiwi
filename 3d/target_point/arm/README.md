# README - Fusion 360 OBJ 导入 Gazebo Sim

> 环境
>
> - Ubuntu 22.04
> - Gazebo Sim 8 (`gz sim`)
> - Fusion 360
> - Blender（用于检查 OBJ）

---

# 1. 从 Fusion 360 导出 OBJ

在 Fusion 360 中：

```
File
    ↓
Export
    ↓
OBJ
```

导出后会得到：

例如：

```
Traumfaenger_EXPORT_arm.obj
Traumfaenger_EXPORT_arm.mtl
```

或者：

```
Traumfaenger_EXPORT_motor.obj
Traumfaenger_EXPORT_motor.mtl
```

建议先使用 Blender 打开 OBJ 文件进行检查：

- 模型是否完整
- 法向是否正确
- 是否存在缺面
- 坐标方向是否正确
- 尺寸是否正确

确认 Blender 中显示正常后，再导入 Gazebo。

---

# 2. 工程目录结构

推荐目录如下：

```
target_point/

├── arm/
│   ├── model.config
│   ├── model.sdf
│   └── meshes/
│       ├── Traumfaenger_EXPORT_arm.obj
│       └── Traumfaenger_EXPORT_arm.mtl
│
├── motor/
│   ├── model.config
│   ├── model.sdf
│   └── meshes/
│       ├── Traumfaenger_EXPORT_motor.obj
│       └── Traumfaenger_EXPORT_motor.mtl
│
├── world1.sdf
├── world2.sdf
└── world3.sdf
```

其中：

```
arm/
motor/
```

均属于 Gazebo 的一个独立 Model。

后续可以继续增加：

```
base/
camera/
gripper/
link1/
link2/
```

每个零件都建议建立独立的 Model。

---

# 3. Gazebo Model 文件说明

一个 Gazebo Model 推荐采用如下结构：

```
model/

├── model.config
├── model.sdf
└── meshes/
```

其中三个部分分别负责不同功能。

---

## 3.1 model.config

作用：

- 注册模型名称
- 告诉 Gazebo 模型描述文件的位置

例如：

```xml
<?xml version="1.0"?>

<model>

    <name>motor</name>

    <version>1.0</version>

    <sdf version="1.9">model.sdf</sdf>

    <author>
        <name>Jiaqi</name>
    </author>

    <description>
        Motor Model
    </description>

</model>
```

如果是 arm，只需要修改：

```xml
<name>arm</name>
```

即可。

---

## 3.2 model.sdf

作用：

描述模型结构。

例如：

```xml
<?xml version="1.0"?>

<sdf version="1.9">

<model name="motor">

    <static>true</static>

    <link name="motor_link">

        <visual name="motor_visual">

            <geometry>

                <mesh>

                    <uri>meshes/Traumfaenger_EXPORT_motor.obj</uri>

                    <scale>1 1 1</scale>

                </mesh>

            </geometry>

        </visual>

    </link>

</model>

</sdf>
```

Gazebo 模型层级如下：

```
Model
└── Link
    └── Visual
        └── Geometry
            └── Mesh
```

目前仅使用：

```
Visual
```

负责模型显示。

以后可继续添加：

```
Collision
```

实现碰撞检测。

再添加：

```
Inertial
```

实现真实物理仿真。

最后添加：

```
Joint
```

连接多个 Link，实现机器人运动。

---

## 3.3 meshes

作用：

存放模型文件。

例如：

```
meshes/

├── xxx.obj
├── xxx.mtl
├── texture.png
└── texture.jpg
```

通常包括：

- OBJ 模型
- MTL 材质
- PNG/JPG 纹理

---

# 4. World 文件说明

目前工程包含三个测试场景。

---

## world1.sdf

作用：

仅显示 Arm 模型。

启动：

```bash
gz sim world1.sdf
```

Entity Tree：

```
default

ground_plane

arm

sun
```

---

## world2.sdf

作用：

仅显示 Motor 模型。

启动：

```bash
gz sim world2.sdf
```

Entity Tree：

```
default

ground_plane

motor

sun
```

---

## world3.sdf

作用：

同时显示 Arm 与 Motor。

启动：

```bash
gz sim world3.sdf
```

Entity Tree：

```
default

ground_plane

arm

motor

sun
```

以后如果继续导入：

```
base
camera
gripper
```

只需继续增加：

```xml
<include>
    <uri>model://base</uri>
</include>
```

即可。

---

# 5. 设置 Gazebo 模型搜索路径

Gazebo 需要知道模型所在目录。

运行：

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/jwang/itm-hiwi/3d/target_point
```

检查：

```bash
echo $GZ_SIM_RESOURCE_PATH
```

输出应类似：

```bash
/home/itmhiwi/jwang/itm-hiwi/3d/target_point
```

如果已经存在其它模型路径，推荐：

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/jwang/itm-hiwi/3d/target_point:$GZ_SIM_RESOURCE_PATH
```

避免覆盖已有路径。

---

# 6. 启动 Gazebo

进入工程目录：

```bash
cd ~/jwang/itm-hiwi/3d/target_point
```

启动不同场景：

测试 Arm：

```bash
gz sim world1.sdf
```

测试 Motor：

```bash
gz sim world2.sdf
```

测试整体：

```bash
gz sim world3.sdf
```

查看详细日志：

```bash
gz sim -v 4 world3.sdf
```

---

# 7. 关于模型尺寸（Scale）

Fusion 360 默认建模单位通常为：

```
mm
```

Gazebo 使用单位：

```
m
```

不同 OBJ 导出器对于单位的处理方式可能不同，因此：

```xml
<scale>1 1 1</scale>
```

或者：

```xml
<scale>0.001 0.001 0.001</scale>
```

都有可能正确。

建议根据 Gazebo 中模型实际尺寸进行判断。

如果尺寸与 Fusion 一致，则保持：

```xml
<scale>1 1 1</scale>
```

无需修改。

---

# 8. 当前工程完成情况

目前已经完成：

- ✅ Fusion 360 导出 OBJ
- ✅ Blender 检查模型
- ✅ Arm 成功导入 Gazebo
- ✅ Motor 成功导入 Gazebo
- ✅ 多个 Model 同时加载
- ✅ Gazebo 模型搜索路径配置完成

目前已经能够正常加载多个独立模型进行测试。

---

# 9. 后续开发计划

```
Fusion 360 装配体

        │

        ▼

导出各个零件 OBJ

        │

        ▼

建立 Gazebo Model

        │

        ▼

导入 Base

        │

        ▼

导入 Motor

        │

        ▼

导入 Arm

        │

        ▼

导入其它 Link

        │

        ▼

建立 Joint

        │

        ▼

添加 Collision

        │

        ▼

添加 Inertial

        │

        ▼

ROS 2 控制

        │

        ▼

完成机械臂仿真
```

最终目标是将 Fusion 360 的装配体逐步转换为 Gazebo 中可运动、可控制、可进行物理仿真的机器人模型。