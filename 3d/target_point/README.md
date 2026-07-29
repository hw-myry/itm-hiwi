# README - 导入 Fusion 360 OBJ 到 Gazebo Sim

> 环境：
>
> - Ubuntu 22.04
> - Gazebo Sim 8 (`gz sim`)
> - Fusion 360
> - Blender（用于检查 OBJ）

---

# 1. Fusion 导出 OBJ

Fusion 360：

```
File
    ↓
Export
    ↓
OBJ
```

导出后得到：

```
Traumfaenger_EXPORT_arm.obj
Traumfaenger_EXPORT_arm.mtl
```

建议先用 Blender 打开确认：

- 模型正常
- 法向正确
- 没有缺面
- 材质正常

如果 Blender 能正常显示，说明 OBJ 基本没有问题。

---

# 2. 建立 Gazebo 模型目录

推荐目录结构：

```
target_point/

├── world.sdf
└── arm/
    ├── model.config
    ├── model.sdf
    └── meshes/
        ├── Traumfaenger_EXPORT_arm.obj
        └── Traumfaenger_EXPORT_arm.mtl
```

为什么这样？

Gazebo 一个模型(Model)至少需要：

```
model.config
model.sdf
```

mesh 放在：

```
meshes/
```

这是 Gazebo 官方推荐的目录结构。

---

# 3. model.config

内容：

```xml
<?xml version="1.0"?>

<model>

    <name>arm</name>

    <version>1.0</version>

    <sdf version="1.9">model.sdf</sdf>

    <author>
        <name>Jiaqi</name>
    </author>

    <description>
        Arm Model
    </description>

</model>
```

作用：

告诉 Gazebo：

- 模型名称叫 arm
- 真正描述模型的是 model.sdf

---

# 4. model.sdf

最简单版本：

```xml
<?xml version="1.0"?>

<sdf version="1.9">

<model name="arm">

    <static>true</static>

    <link name="link">

        <visual name="visual">

            <geometry>

                <mesh>

                    <uri>meshes/Traumfaenger_EXPORT_arm.obj</uri>

                    <scale>1 1 1</scale>

                </mesh>

            </geometry>

        </visual>

    </link>

</model>

</sdf>
```

为什么这样写？

Gazebo 模型层级：

```
Model
    ↓
Link
    ↓
Visual
    ↓
Geometry
    ↓
Mesh
```

也就是：

```
Model
└── Link
        └── Visual
                └── Mesh
```

其中：

```
<visual>
```

负责显示模型。

目前没有物理碰撞。

以后再添加：

```
<collision>
```

以及

```
<inertial>
```

即可实现真实物理仿真。

---

# 5. world.sdf

例如：

```xml
<?xml version="1.0"?>

<sdf version="1.9">

<world name="default">

    <include>
        <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>

    <include>
        <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

    <include>
        <uri>model://arm</uri>
    </include>

</world>

</sdf>
```

作用：

Ground Plane：

提供地面。

Sun：

提供光照。

```
model://arm
```

表示：

去 Resource Path 中寻找

```
arm/
```

这个模型。

---

# 6. 设置 Gazebo 模型路径

告诉 Gazebo 去哪里寻找模型。

例如：

```bash
export GZ_SIM_RESOURCE_PATH=~/jwang/itm-hiwi/3d/target_point
```

检查：

```bash
echo $GZ_SIM_RESOURCE_PATH
```

应该输出：

```bash
/home/itmhiwi/jwang/itm-hiwi/3d/target_point
```

如果以后有多个模型路径：

推荐：

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/jwang/itm-hiwi/3d/target_point:$GZ_SIM_RESOURCE_PATH
```

不要覆盖已有路径。

---

# 7. 运行 Gazebo

进入工程目录：

```bash
cd ~/jwang/itm-hiwi/3d/target_point
```

运行：

```bash
gz sim world.sdf
```

或者查看详细日志：

```bash
gz sim -v 4 world.sdf
```

---

# 8. 判断是否成功

Entity Tree 中应出现：

```
default

ground_plane

arm

sun
```

说明：

OBJ 已经成功导入 Gazebo。

---

# 9. 关于 Scale

Fusion 当前测量：

```
孔直径 = 10 mm
```

说明：

Fusion 建模单位为 **mm**。

Gazebo 内部单位为 **m**。

不同导出器对 OBJ 单位的处理方式不同，因此：

```
<scale>1 1 1</scale>
```

或者

```
<scale>0.001 0.001 0.001</scale>
```

都可能是正确的。

不要默认认为 Fusion 导出的 OBJ 一定需要乘 0.001。

正确的方法是：

根据 Gazebo 中实际尺寸判断。

如果尺寸与 Fusion 一致，就保持：

```xml
<scale>1 1 1</scale>
```

---

# 10. 为什么先调整尺寸？

最终机械臂结构：

```
Robot

├── Base
│
├── Link1
│
├── Link2
│
├── Link3
│
└── End Effector
```

如果第一个 Link：

- Scale
- Pose
- 坐标系

没有调整正确，

后续所有 Link 都需要重新调整。

因此建议流程：

```
确认 Scale

↓

确认 Pose

↓

确认坐标系

↓

导入 Base

↓

导入其它 Link

↓

建立 Joint

↓

添加 Collision

↓

添加 Inertial

↓

ROS 2 控制
```

这样后续几乎不用返工。

---

# 11. 后续计划

完成单个 OBJ 导入之后：

```
Fusion Assembly

↓

导出每一个零件 OBJ

↓

导入 Base

↓

导入 Link1

↓

导入 Link2

↓

导入 Link3

↓

建立 Joint

↓

添加 Collision

↓

添加 Inertial

↓

ROS 2 + Gazebo 控制

↓

完成机械臂仿真
```

最终目标是把 Fusion 360 的装配体转换为 Gazebo 中可运动、可控制、可仿真的机器人。