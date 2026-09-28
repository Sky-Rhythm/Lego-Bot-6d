import bpy
import math
import bpy.props as bp
from mathutils import Matrix, Euler
from typing import Optional

# ====================== 核心配置：安全注册独立角度属性 ======================
MAX_BONE_ITEMS = 12
ANGLE_PRECISION = 1  # 角度显示精度（小数点后位数）
ANGLE_COMPARE_THRESHOLD = 0.01  # 角度变化判断阈值（更小的阈值减少精度误差）

def safe_register_angle_properties():
    """安全注册角度属性：先删除旧属性，再注册新属性"""
    # 先删除已存在的同名属性
    for i in range(MAX_BONE_ITEMS):
        prop_name = f"ik_angle_{i}"
        if hasattr(bpy.types.Scene, prop_name):
            delattr(bpy.types.Scene, prop_name)
    
    # 重新注册角度属性（移除所有update回调中的refresh标记）
    for i in range(MAX_BONE_ITEMS):
        # 定义更新回调（完全兼容版：无refresh标记）
        def make_angle_update_callback(idx):
            def angle_update_callback(self, context):
                """角度更新时触发刷新（全版本兼容）"""
                self.update_tag()
                if depsgraph := context.evaluated_depsgraph_get():
                    depsgraph.update()
            return angle_update_callback
        
        # 注册属性（保留ANIMATABLE保证驱动器识别）
        setattr(
            bpy.types.Scene, 
            f"ik_angle_{i}", 
            bp.FloatProperty(
                name=f"角度{i+1}",
                default=0.0,
                precision=ANGLE_PRECISION,  # 使用统一的精度配置
                options={'ANIMATABLE', 'SKIP_SAVE'},
                description=f"第{i+1}个骨骼的角度值（稳定版）",
                update=make_angle_update_callback(i)
            )
        )

# ====================== 骨骼配置项 ======================
class IK_BONE_ITEM(bpy.types.PropertyGroup):
    bone_name: bp.StringProperty(
        name="骨骼名称", 
        default="",
        description="选择需要监控的骨骼名称"
    )
    axis: bp.EnumProperty(
        items=[
            ('X','X','绕X轴旋转'), 
            ('Y','Y','绕Y轴旋转'), 
            ('Z','Z','绕Z轴旋转')
        ],
        default='Z',
        name="轴向"
    )

# ====================== 核心工具类 ======================
class ArmatureUtils:
    @staticmethod
    def get_bone_evaluated_angle(armature_obj: bpy.types.Object, bone_name: str, axis: str) -> Optional[float]:
        """
        获取骨骼指定轴的最终计算角度（使用评估后的依赖图，支持动画/关键帧场景）
        解决关键帧后角度不更新的核心修复
        优化精度处理：解决-90度显示为-89.99度的问题
        """
        if not armature_obj or armature_obj.type != 'ARMATURE':
            return None
        
        # 关键修复1：获取评估后的依赖图和物体（动画/关键帧后的最终状态）
        depsgraph = bpy.context.evaluated_depsgraph_get()
        if not depsgraph:
            return None
        
        # 获取评估后的骨架对象（包含所有动画/关键帧/约束的最终结果）
        evaluated_arm = armature_obj.evaluated_get(depsgraph)
        pose_bone = evaluated_arm.pose.bones.get(bone_name)
        
        if not pose_bone:
            # 降级处理：尝试获取原始pose bone
            pose_bone = armature_obj.pose.bones.get(bone_name)
            if not pose_bone:
                return None
        
        try:
            # 关节角 = 相对静止姿态的旋转。直接对父子矩阵做欧拉分解时，
            # 静止姿态里已经有的大角度会混进读数（j4 的 Y 轴静止就是 90°），
            # 并且在越过 ±90° 时发生跳变。
            order = pose_bone.rotation_mode
            if order not in {"XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"}:
                order = "XYZ"
            axis_index = "xyz".index(axis.lower())

            src_bone = armature_obj.data.bones.get(bone_name)
            if pose_bone.parent and src_bone and src_bone.parent:
                local_matrix = pose_bone.parent.matrix.inverted() @ pose_bone.matrix
                rest_local = src_bone.parent.matrix_local.inverted() @ src_bone.matrix_local
            elif src_bone:
                local_matrix = pose_bone.matrix.copy()
                rest_local = src_bone.matrix_local.copy()
            else:
                local_matrix = pose_bone.matrix.copy()
                rest_local = None

            if rest_local is not None and abs(rest_local.to_euler(order)[axis_index]) > math.radians(45.0):
                euler = (rest_local.inverted() @ local_matrix).to_euler(order)
            else:
                euler = local_matrix.to_euler(order)
            angle_rad = euler[axis_index]
            
            # 优化2：角度归一化到 [-180, 180] 范围，减少计算偏差
            angle_rad = math.fmod(angle_rad, 2 * math.pi)
            if angle_rad > math.pi:
                angle_rad -= 2 * math.pi
            elif angle_rad < -math.pi:
                angle_rad += 2 * math.pi
            
            angle_deg = math.degrees(angle_rad)
            
            # 优化3：使用更精确的四舍五入策略，解决-90显示为-89.99的问题
            # 先放大100倍取整再缩小，避免浮点精度问题
            angle_deg = round(angle_deg * 10**ANGLE_PRECISION) / 10**ANGLE_PRECISION
            
            return angle_deg
        except Exception as e:
            print(f"计算骨骼{bone_name}角度失败: {e}")
            return None

# ====================== 全局更新函数（修复刷新层级问题） ======================
_ik_angle_updating = False

def update_all_ik_angles(scene):
    """核心逻辑：使用评估后的依赖图，支持关键帧/动画场景"""
    global _ik_angle_updating
    # 本函数会改场景属性并刷新依赖图，刷新又会再次进入这里。
    # 不挡住的话，j4 这种一次变化几十度的修正会递归到栈溢出，角度停在旧值。
    if _ik_angle_updating:
        return
    _ik_angle_updating = True
    try:
        _update_all_ik_angles_impl(scene)
    finally:
        _ik_angle_updating = False

def _update_all_ik_angles_impl(scene):
    arm = scene.ik_armature
    if not arm or arm.type != 'ARMATURE':
        return
    
    has_angle_change = False
    
    for idx, item in enumerate(scene.ik_bone_configs):
        if idx >= MAX_BONE_ITEMS:
            break
        if not item.bone_name:
            continue
        
        # 关键修复3：使用评估后的角度计算
        new_angle = ArmatureUtils.get_bone_evaluated_angle(arm, item.bone_name, item.axis)
        if new_angle is None:
            continue
        
        current_angle = getattr(scene, f"ik_angle_{idx}", 0.0)
        
        # 优化：使用配置的阈值，并且先统一精度再比较
        current_angle_rounded = round(current_angle * 10**ANGLE_PRECISION) / 10**ANGLE_PRECISION
        new_angle_rounded = round(new_angle * 10**ANGLE_PRECISION) / 10**ANGLE_PRECISION
        
        # 精度阈值判断（使用更小的阈值）
        if abs(new_angle_rounded - current_angle_rounded) > ANGLE_COMPARE_THRESHOLD:
            # 临时关闭更新回调避免递归
            prop = getattr(bpy.types.Scene, f"ik_angle_{idx}", None)
            original_update = getattr(prop, "update", None) if prop else None
            if prop and original_update:
                prop.update = None
            
            # 更新角度值（使用四舍五入后的值）
            setattr(scene, f"ik_angle_{idx}", new_angle_rounded)
            has_angle_change = True
            
            # 恢复更新回调
            if prop and original_update:
                prop.update = original_update
    
    if has_angle_change:
        # 关键修复4：多重刷新保障
        try:
            scene.update_tag()
        except Exception:
            pass
        
        # 强制刷新依赖图（高优先级）
        depsgraph = bpy.context.evaluated_depsgraph_get()
        if depsgraph:
            depsgraph.update()
            
            # 额外刷新UI（解决面板不更新）
            for area in bpy.context.screen.areas:
                if area.type == 'VIEW_3D':
                    area.tag_redraw()

# ====================== 补充定时器刷新（兜底方案） ======================
timer_handle = None

def timer_update_ik_angles():
    """定时器刷新：兜底保障，确保关键帧后仍能更新"""
    scene = bpy.context.scene
    if scene and hasattr(scene, 'ik_armature') and scene.ik_armature:
        update_all_ik_angles(scene)
    return 0.01  # 每0.01秒刷新一次（可根据性能调整）

# ====================== 操作符（无refresh标记） ======================
class IK_OT_AddBoneConfig(bpy.types.Operator):
    bl_idname = "ik.add_bone_config"
    bl_label = "添加骨骼"
    bl_description = f"添加骨骼监控项（最多{MAX_BONE_ITEMS}个）"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        scene = context.scene
        if len(scene.ik_bone_configs) >= MAX_BONE_ITEMS:
            self.report({'WARNING'}, f"最多只能添加{MAX_BONE_ITEMS}个骨骼项！")
            return {'CANCELLED'}
        
        scene.ik_bone_configs.add()
        scene.update_tag()
        
        # 立即更新一次
        update_all_ik_angles(scene)
        
        self.report({'INFO'}, "已添加骨骼监控项")
        return {'FINISHED'}

class IK_OT_RemoveBoneConfig(bpy.types.Operator):
    bl_idname = "ik.remove_bone_config"
    bl_label = "删除最后一项"
    bl_description = "删除最后添加的骨骼监控项"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        scene = context.scene
        if len(scene.ik_bone_configs) > 0:
            scene.ik_bone_configs.remove(len(scene.ik_bone_configs)-1)
            scene.update_tag()
            
            # 立即更新一次
            update_all_ik_angles(scene)
            
            self.report({'INFO'}, "已删除最后一项骨骼配置")
        else:
            self.report({'WARNING'}, "没有可删除的骨骼项！")
        return {'FINISHED'}

def _clear_pose_channels(pose_bone):
    pose_bone.location = (0.0, 0.0, 0.0)
    pose_bone.scale = (1.0, 1.0, 1.0)
    if pose_bone.rotation_mode == "QUATERNION":
        pose_bone.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
    elif pose_bone.rotation_mode == "AXIS_ANGLE":
        pose_bone.rotation_axis_angle = (0.0, 0.0, 1.0, 0.0)
    else:
        pose_bone.rotation_euler = (0.0, 0.0, 0.0)


def _rest_ik_matrix(armature_obj, pose_bone, use_tail):
    """IK 目标在静止姿态下应处的世界矩阵。use_tail 时位置用骨骼尾端。"""
    bone = pose_bone.bone
    rest_world = armature_obj.matrix_world @ bone.matrix_local
    if use_tail:
        rest_world.translation = armature_obj.matrix_world @ bone.tail_local
    return rest_world


def reset_armature_to_rest(armature_obj):
    """清掉姿态旋转，并把 IK 目标放回静止姿态，否则约束会立刻把骨骼再弯回去。"""
    ik_constraints = []
    for pose_bone in armature_obj.pose.bones:
        for constraint in pose_bone.constraints:
            if constraint.type == "IK" and not constraint.mute:
                ik_constraints.append((pose_bone, constraint))
                constraint.mute = True

    for pose_bone in armature_obj.pose.bones:
        _clear_pose_channels(pose_bone)

    if bpy.context.view_layer:
        bpy.context.view_layer.update()

    moved_targets = 0
    for pose_bone, constraint in ik_constraints:
        target = constraint.target
        if target is not None:
            target.matrix_world = _rest_ik_matrix(
                armature_obj, pose_bone, getattr(constraint, "use_tail", True)
            )
            moved_targets += 1
        constraint.mute = False

    if bpy.context.view_layer:
        bpy.context.view_layer.update()
    return moved_targets


# ====================== 骨骼角度归零 ======================
class IK_OT_ResetPose(bpy.types.Operator):
    bl_idname = "ik.reset_pose"
    bl_label = "骨骼角度归零"
    bl_description = "把机械臂骨架回到静止姿态，并把 IK 控制目标放回对应位置"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        arm = context.scene.ik_armature
        if not arm or arm.type != "ARMATURE":
            self.report({"WARNING"}, "请先选择机械臂骨架")
            return {"CANCELLED"}

        moved_targets = reset_armature_to_rest(arm)
        update_all_ik_angles(context.scene)
        if moved_targets:
            self.report({"INFO"}, f"已归零，并复位 {moved_targets} 个 IK 控制目标")
        else:
            self.report({"INFO"}, "已将骨骼姿态归零")
        return {"FINISHED"}


# ====================== 强制刷新操作符（手动兜底） ======================
class IK_OT_ForceRefresh(bpy.types.Operator):
    bl_idname = "ik.force_refresh"
    bl_label = "强制刷新角度"
    bl_description = "手动强制刷新所有骨骼角度（关键帧后使用）"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        update_all_ik_angles(context.scene)
        self.report({'INFO'}, "已强制刷新所有骨骼角度")
        return {'FINISHED'}

# ====================== 主面板 ======================
class IK_PT_RoboticArmPanel(bpy.types.Panel):
    bl_label = "多关节机械臂IK角度监控"
    bl_idname = "IK_PT_RoboticArmPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "IK工具"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        arm = scene.ik_armature

        layout.prop(scene, "ik_armature", text="机械臂骨架")

        if not arm:
            layout.label(text="请先选择骨架对象")
            return

        row_reset = layout.row(align=True)
        row_reset.operator("ik.reset_pose", text="骨骼角度归零", icon="LOOP_BACK")
        row_reset.operator("ik.force_refresh", text="强制刷新角度", icon="FILE_REFRESH")
        
        row = layout.row(align=True)
        row.operator("ik.add_bone_config", text="添加骨骼项")
        row.operator("ik.remove_bone_config", text="删除最后一项")

        box = layout.box()
        col = box.column(align=True)

        row_h = col.row(align=True)
        row_h.alignment = 'CENTER'
        row_h.label(text="序号")
        row_h.label(text="骨骼名称")
        row_h.label(text="旋转轴")
        row_h.label(text=f"实时角度(°, 精度:{ANGLE_PRECISION}位)")

        for idx, item in enumerate(scene.ik_bone_configs):
            row = col.row(align=True)
            row.alignment = 'CENTER'
            row.label(text=str(idx+1))
            row.prop_search(item, "bone_name", arm.data, "bones", text="")
            row.prop(item, "axis", text="")
            row.prop(scene, f"ik_angle_{idx}", text="")

# ====================== 安全注册/注销 ======================
classes = (
    IK_BONE_ITEM,
    IK_OT_AddBoneConfig,
    IK_OT_RemoveBoneConfig,
    IK_OT_ResetPose,
    IK_OT_ForceRefresh,  # 新增
    IK_PT_RoboticArmPanel,
)

update_handler = None

def register():
    global update_handler, timer_handle
    
    # 第一步：安全注册角度属性
    safe_register_angle_properties()
    
    # 第二步：注销已存在的类
    for cls in classes:
        try:
            bpy.utils.unregister_class(cls)
        except:
            pass
    
    # 第三步：重新注册所有类
    for cls in classes:
        bpy.utils.register_class(cls)
    
    # 第四步：注册骨架选择属性
    if hasattr(bpy.types.Scene, "ik_armature"):
        del bpy.types.Scene.ik_armature
    bpy.types.Scene.ik_armature = bp.PointerProperty(
        type=bpy.types.Object,
        poll=lambda s, o: o.type == 'ARMATURE',
        description="选择机械臂骨架对象",
        update=lambda self, ctx: (self.update_tag(), update_all_ik_angles(ctx.scene))
    )
    
    # 第五步：注册配置列表
    if hasattr(bpy.types.Scene, "ik_bone_configs"):
        del bpy.types.Scene.ik_bone_configs
    bpy.types.Scene.ik_bone_configs = bp.CollectionProperty(
        type=IK_BONE_ITEM,
        description="机械臂骨骼监控配置列表"
    )
    
    # 第六步：添加多重更新处理器
    # 1. 依赖图更新前（基础）
    update_handler = update_all_ik_angles
    try:
        bpy.app.handlers.depsgraph_update_pre.remove(update_handler)
    except ValueError:
        pass
    bpy.app.handlers.depsgraph_update_pre.append(update_handler)
    
    # 2. 帧变化时（关键帧/动画播放时）
    try:
        bpy.app.handlers.frame_change_post.remove(update_handler)
    except ValueError:
        pass
    bpy.app.handlers.frame_change_post.append(update_handler)
    
    # 3. 定时器兜底（解决关键帧插入后无响应）
    if timer_handle is None:
        timer_handle = bpy.app.timers.register(timer_update_ik_angles)

def unregister():
    global update_handler, timer_handle
    
    # 移除定时器
    if timer_handle is not None and bpy.app.timers.is_registered(timer_handle):
        bpy.app.timers.unregister(timer_handle)
    timer_handle = None
    
    # 移除处理器
    if update_handler:
        try:
            bpy.app.handlers.depsgraph_update_pre.remove(update_handler)
        except ValueError:
            pass
        try:
            bpy.app.handlers.frame_change_post.remove(update_handler)
        except ValueError:
            pass
    
    # 注销类
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except:
            pass
    
    # 删除属性
    try:
        del bpy.types.Scene.ik_armature
        del bpy.types.Scene.ik_bone_configs 
    except:
        pass
    
    # 删除角度属性
    for i in range(MAX_BONE_ITEMS):
        prop_name = f"ik_angle_{i}"
        if hasattr(bpy.types.Scene, prop_name):
            delattr(bpy.types.Scene, prop_name)

# ====================== 运行入口 ======================
if __name__ == "__main__":
    # 先注销旧版本
    try:
        unregister()
    except Exception as e:
        print(f"注销旧版本（首次运行可忽略）: {e}")
    
    # 注册新版本
    register()
    print(f"多关节机械臂IK工具已加载（支持最多{MAX_BONE_ITEMS}个骨骼项，修复关键帧刷新和精度问题）")