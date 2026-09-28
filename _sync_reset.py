import bpy, math, importlib.util

blend = r"d:\Develop\workspace\BlendoBot-6D-main\乐高6轴机械臂\乐高机械臂.blend"
py = r"d:\Develop\workspace\BlendoBot-6D-main\乐高6轴机械臂\IK角度.py"

text = bpy.data.texts["ik角度显示"]
with open(py, "r", encoding="utf-8") as f:
    src = f.read()
text.clear()
text.write(src)
assert "ik.reset_pose" in text.as_string()

spec = importlib.util.spec_from_file_location("ikfix", py)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

arm = bpy.data.objects["骨架"]
target = bpy.data.objects["控制"]
saved_target = target.matrix_world.copy()
saved_pose = []
for pb in arm.pose.bones:
    saved_pose.append((pb.name, pb.location.copy(), pb.rotation_mode, pb.rotation_euler.copy(), pb.rotation_quaternion.copy(), pb.scale.copy()))

print("before target", tuple(round(c, 2) for c in target.location))
moved = mod.reset_armature_to_rest(arm)
print("moved", moved, "after target", tuple(round(c, 2) for c in target.location))

def joint_deg(name, axis):
    deps = bpy.context.evaluated_depsgraph_get()
    ev = arm.evaluated_get(deps).pose.bones[name]
    bone = arm.data.bones[name]
    local = ev.parent.matrix.inverted() @ ev.matrix if ev.parent else ev.matrix.copy()
    rest = bone.parent.matrix_local.inverted() @ bone.matrix_local if bone.parent else bone.matrix_local.copy()
    delta = (rest.inverted() @ local).to_euler("XYZ")
    return round(math.degrees(delta[axis]), 2)

print("j1Z", joint_deg("j1", 2), "j2Z", joint_deg("j2", 2), "j3Z", joint_deg("j3", 2))
print("j4Y", joint_deg("j4", 1), "j5X", joint_deg("j5", 0), "j6Y", joint_deg("j6", 1))

target.matrix_world = saved_target
for name, loc, mode, euler, quat, scale in saved_pose:
    pb = arm.pose.bones[name]
    pb.location = loc
    pb.rotation_mode = mode
    pb.rotation_euler = euler
    pb.rotation_quaternion = quat
    pb.scale = scale
if bpy.context.view_layer:
    bpy.context.view_layer.update()

bpy.ops.wm.save_mainfile(filepath=blend)
print("SAVED")
