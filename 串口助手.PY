# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
import bpy
import serial
import serial.tools.list_ports
import threading
import time
import math
from bpy.types import Context
import re
import queue
import random


bl_info = {
    "name": "Serial Helper/串口助手",
    "author": "SFY",
    "description": "在blender中使用串口进行通讯/A tool to help with serial communication.",
    "blender": (2, 80, 0),
    "version": (1, 0, 1),
    "location": "View3D > Tool Shelf",
    "warning": "",
    "category": "Generic"
}

# 定义属性组


class SerialConnection:
    def __init__(self, port, baudrate, bytesize, parity, stopbits):
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.serial = serial.Serial(port, baudrate, bytesize, parity, stopbits, timeout=0.1)


class ServoDataSender:
    def __init__(self, serial_connection):
        self.serial_connection = serial_connection

    def pack_servo_data(self, servos):
        # servos 是一个包含多个舵机数据的列表，每个元素是一个 (编号, 角度) 的元组
        data_str = ''
        for servo_id, angle in servos:
            data_str += f's{servo_id:03d}a{angle:.2f}'
        return data_str

    def collect_and_send_servo_data(self):
        arm1_data = math.degrees(bpy.data.objects['Armature'].pose.bones['arm1'].rotation_euler[2]) + 90
        arm2_data = -math.degrees(bpy.data.objects['Armature'].pose.bones['arm2'].rotation_euler[0]) + 90
        arm3_data = math.degrees(bpy.data.objects['Armature'].pose.bones['arm3'].rotation_euler[0]) + 140
        servos = [(1, arm1_data), (2, arm2_data), (3, arm3_data)]
        packed_data = self.pack_servo_data(servos)
        self.serial_connection.serial.write(packed_data.encode())


def extract_value(input_str, var_name):
    # 定义正则表达式模式，用于匹配特定变量名和其后的浮点数（包括负数）
    pattern = fr"{re.escape(var_name)}=([-+]?\d*\.\d+|\d+)"

    # 使用re.search查找第一个匹配项
    match = re.search(pattern, input_str)

    # 检查是否找到匹配项
    if match:
        # 返回匹配的浮点数
        return int(float(match.group(1)))
    else:
        # 如果没有找到匹配项，返回None或合适的提示信息
        return None


data_queue = queue.Queue()


class SerialHelperThread(threading.Thread):
    def __init__(self, serial_connection):
        threading.Thread.__init__(self)
        self.serial_connection = serial_connection
        self.should_terminate = False
        self.data_queue = data_queue
        self.buffer = b''  # 接收缓冲区

    def run(self):
        while not self.should_terminate:
            if not bpy.context.scene.serial_helper.StopReceiving:
                try:
                    # 读取所有可用数据
                    in_waiting = self.serial_connection.serial.in_waiting
                    if in_waiting > 0:
                        chunk = self.serial_connection.serial.read(in_waiting)
                        self.buffer += chunk
                        
                        # 处理缓冲区中的数据
                        self.process_buffer()
                    
                    time.sleep(0.0001)  # 减少等待时间，提高响应速度
                except Exception as e:
                    print(f"数据接收失败: {e}")
                    import traceback
                    traceback.print_exc()
            else:
                time.sleep(0.01)

    def process_buffer(self):
        """处理缓冲区中的数据，提取完整的数据包"""
        debug_mode = bpy.context.scene.serial_helper.debug_mode
        
        while len(self.buffer) >= 4:
            # 查找以校验码 0x6B 结尾的数据包
            # 数据包格式: 地址(1) + 指令(1) + 数据(4-5) + 校验(1) = 7-8字节
            # 或者: 地址(1) + 指令(1) + 校验1(1) + 校验2(1) = 4字节（到位命令）
            
            # 查找下一个可能的起始位置（地址可以是 0x00-0xFF）
            start_idx = -1
            for i in range(min(len(self.buffer), 32)):  # 最多检查前32字节
                # 查找可能的起始位置：后面跟着有效指令的字节
                if i + 1 < len(self.buffer):
                    next_byte = self.buffer[i + 1]
                    # 检查是否是已知指令：0x36(角度位置返回), 0x32(脉冲位置返回), 0xFD(到位命令)
                    if next_byte in [0x36, 0x32, 0xFD]:
                        start_idx = i
                        break
            
            if start_idx == -1:
                # 没有找到有效起始位置，清空缓冲区
                if debug_mode:
                    print(f"[缓冲区] 未找到有效起始位置，清空缓冲区: {self.buffer.hex().upper()}")
                self.buffer = b''
                return
            
            # 移除起始位置之前的数据
            if start_idx > 0:
                if debug_mode:
                    print(f"[缓冲区] 跳过 {start_idx} 字节无效数据: {self.buffer[:start_idx].hex().upper()}")
                self.buffer = self.buffer[start_idx:]
            
            # 尝试提取数据包
            # 检查是否是到位命令（4字节：地址 + FD + 9F + 6B）
            if len(self.buffer) >= 4:
                if (self.buffer[1] == 0xFD and 
                    self.buffer[2] == 0x9F and 
                    self.buffer[3] == 0x6B):
                    # 找到到位命令
                    packet = self.buffer[:4]
                    self.buffer = self.buffer[4:]
                    if debug_mode:
                        print(f"[数据包] 到位命令: {packet.hex().upper()}")
                    self.process_packet(packet)
                    continue
            
            # 检查是否是位置返回命令（8字节：地址 + 36/32 + 符号 + 位置(4) + 6B）
            if len(self.buffer) >= 8:
                cmd_byte = self.buffer[1]  # 先保存指令字节
                if (cmd_byte in [0x36, 0x32] and 
                    self.buffer[7] == 0x6B):
                    # 找到位置返回命令
                    packet = self.buffer[:8]
                    self.buffer = self.buffer[8:]
                    if debug_mode:
                        print(f"[数据包] 位置返回(指令={cmd_byte:02X}): {packet.hex().upper()}")
                    self.process_packet(packet)
                    continue
            
            # 如果当前起始位置不匹配，跳过这个字节
            if debug_mode:
                print(f"[缓冲区] 起始位置不匹配，跳过1字节: {self.buffer[0]:02X} (缓冲区: {self.buffer.hex().upper()})")
            self.buffer = self.buffer[1:]

    def process_packet(self, packet):
        """处理单个数据包"""
        try:
            encoding = bpy.context.scene.serial_helper.Encoding
            if encoding == 'hex':
                # 以16进制格式处理数据
                hex_data = ' '.join([f'{b:02X}' for b in packet])
                self.data_queue.put(f"[HEX] {hex_data}")
                
                # 检测到位命令: 地址 + FD + 9F + 6B
                if len(packet) == 4:
                    addr = packet[0]
                    cmd = packet[1]
                    checksum1 = packet[2]
                    checksum2 = packet[3]
                    if cmd == 0xFD and checksum1 == 0x9F and checksum2 == 0x6B:
                        bpy.context.scene.serial_helper.motor_arrived = True
                
                bpy.app.timers.register(serial_data_update)
        except Exception as e:
            print(f"数据包处理失败: {e}")


def serial_data_update():
    scene = bpy.context.scene
    try:
        data = data_queue.get(block=False)
        if data:
            item = scene.serial_helper.serial_data_list.add()
            item.index = scene.serial_helper.serial_data_count
            item.data_string = data
            
            # 检查是否是电机位置返回数据 (HEX格式: [HEX] 02 32 01 00 00 0B 38 6B)
            if data.startswith("[HEX] "):
                parse_motor_position_from_hex(data)
            else:
                # 原有的文本格式解析
                for mapping_item in scene.serial_helper.serial_data_matching_list:
                    if extract_value(data, mapping_item.matching_data_name) != None:
                        mapping_item.matching_data_value = extract_value(data, mapping_item.matching_data_name)

            # 移除多余的项
            if len(scene.serial_helper.serial_data_list) > scene.serial_helper.serial_data_max_count:
                for i in range(len(scene.serial_helper.serial_data_list) - scene.serial_helper.serial_data_max_count):
                    scene.serial_helper.serial_data_list.remove(i)

            scene.serial_helper.serial_data_count += 1
            scene.frame_set(scene.frame_current)  # 刷新界面
    except queue.Empty:
        pass


def refresh_scene_drivers(scene):
    """统一刷新场景和驱动器"""
    try:
        scene.update_tag()
        depsgraph = bpy.context.evaluated_depsgraph_get()
        if depsgraph:
            depsgraph.update()
        if bpy.context.view_layer:
            bpy.context.view_layer.update()
        for window in bpy.context.window_manager.windows:
            for area in window.screen.areas:
                area.tag_redraw()
        if hasattr(scene.serial_helper, 'driver_update_trigger'):
            scene.serial_helper.driver_update_trigger += 1
    except Exception as e:
        if bpy.context.scene.serial_helper.debug_mode:
            print(f"[刷新] 警告: {e}")


def update_motor_data(addr_str, value, is_arrived=False):
    """统一更新电机数据到匹配列表"""
    scene = bpy.context.scene
    debug_mode = scene.serial_helper.debug_mode
    
    for mapping_item in scene.serial_helper.serial_data_matching_list:
        if str(mapping_item.matching_data_name).strip().upper() == addr_str:
            mapping_item.matching_data_value = float(value)
            if debug_mode:
                status = "到位" if is_arrived else f"{value:.2f}°"
                print(f"[更新] {addr_str} = {status}")
            refresh_scene_drivers(scene)
            return True
    
    if debug_mode:
        print(f"[更新] 未找到匹配 '{addr_str}'")
    return False


def parse_motor_position_from_hex(hex_string):
    """从HEX字符串解析电机实时位置（角度）数据
    格式: [HEX] 01 36 01 00 01 00 00 6B
    """
    try:
        hex_data = hex_string.replace("[HEX] ", "").strip()
        bytes_list = hex_data.split()
        
        if len(bytes_list) < 8:
            return
        
        addr = int(bytes_list[0], 16)
        cmd = int(bytes_list[1], 16)
        addr_str = f"{addr:02X}"
        debug_mode = bpy.context.scene.serial_helper.debug_mode
        
        # 处理到位命令 (0xFD)
        if cmd == 0xFD:
            if debug_mode:
                print(f"[解析] 到位命令: {addr_str}")
            bpy.context.scene.serial_helper.motor_arrived = True
            update_motor_data(addr_str, 1.0, is_arrived=True)
            return
        
        # 只处理位置返回指令 (0x36=角度, 0x32=脉冲)
        if cmd not in [0x36, 0x32]:
            return
        
        symbol = int(bytes_list[2], 16)
        position_raw = (int(bytes_list[3], 16) << 24) | \
                       (int(bytes_list[4], 16) << 16) | \
                       (int(bytes_list[5], 16) << 8) | \
                       int(bytes_list[6], 16)
        
        if symbol == 0x01:
            position_raw = -position_raw
        
        # 统一转换为角度
        angle_value = (position_raw * 360.0) / 65536.0
        
        if debug_mode:
            mode = "角度" if cmd == 0x36 else "脉冲"
            print(f"[解析] {addr_str} {mode}模式: {angle_value:.2f}°")
        
        update_motor_data(addr_str, angle_value)
            
    except Exception as e:
        if bpy.context.scene.serial_helper.debug_mode:
            print(f"[解析] 失败: {e}")


def open_serial_port():
    scence = bpy.context.scene
    if scence.serial_helper.use_input_serial_port:
        port = scence.serial_helper.user_input_serial_port
    else:
        port = scence.serial_helper.serial_ports
    baudrate = scence.serial_helper.baudrate
    # 使用固定的串口参数：8位数据位，无校验，1位停止位
    bytesize = 8
    parity = 'N'
    stopbits = 1
    if not "serial_connection" in bpy.app.driver_namespace:
        bpy.app.driver_namespace["serial_connection"] = SerialConnection(port, baudrate, bytesize, parity, stopbits)
        serial_thread = SerialHelperThread(bpy.app.driver_namespace["serial_connection"])
        serial_thread.start()
        bpy.app.driver_namespace["serial_thread"] = serial_thread
        print(f"成功打开串口{port}")
    else:
        print("串口已经打开")


class SerialHelpPanel(bpy.types.Panel):
    bl_label = "Serial Helper"
    bl_idname = "VIEW3D_PT_serial_help"  # 通常与视图3D面板关联的ID
    bl_space_type = 'VIEW_3D'  # VIEW_3D 是3D视图的上下文
    bl_region_type = 'UI'  # UI 区域类型通常用于侧边的工具面板
    bl_category = "串口助手"  # 面板的类别

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        col = layout.column()
        col.operator("test.operator", text="Test Operator")
        row = col.row(align=True)
        row.scale_y = 2  # 调整按钮高度
        row.label(text="端口名：")
        row.alignment = 'LEFT'
        row1 = row.row()
        row1.scale_x = 2.5
        row1.alignment = 'EXPAND'
        if scene.serial_helper.use_input_serial_port:
            row1.prop(scene.serial_helper, "user_input_serial_port", text="")
        else:
            row1.prop(context.scene.serial_helper, "serial_ports", text="")
        row1.prop(scene.serial_helper, "use_input_serial_port", text="", icon_value=197)
        
        # 波特率放在端口后面，同一排
        row1.prop(context.scene.serial_helper, "baudrate_enum", text="")
        
        row2 = col.row()
        row2.alert = context.scene.serial_helper.serial_is_open  # 设置警告状态，使按钮变红
        row2.scale_y = 2  # 调整按钮高度
        row2.operator("serial.switch_operator", text="打开串口" if context.scene.serial_helper.serial_is_open == False else "关闭串口")


class ReceivingSettingsPanel(bpy.types.Panel):
    bl_label = '接收设置'
    bl_idname = 'VIEW_3D_PT_ReceivingSettings'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_context = 'scene'

    bl_order = 0

    bl_parent_id = 'VIEW3D_PT_serial_help'
    bl_ui_units_x = 0

    @ classmethod
    def poll(cls, context):
        return not (False)

    def draw(self, context):
        layout = self.layout
        box = layout.box()
        row = box.row()
        row.scale_y = 2
        row.prop(context.scene.serial_helper, "Encoding")
        col = row.column()
        col.prop(context.scene.serial_helper, "StopReceiving", text="暂停" if context.scene.serial_helper.StopReceiving else "正在接收", icon_value=498 if context.scene.serial_helper.StopReceiving else 495)
        
        # 添加调试模式开关
        col_debug = row.column()
        col_debug.prop(context.scene.serial_helper, "debug_mode", text="调试", icon='INFO')
        
        # 添加读取电机位置按钮
        col2 = row.column()
        if context.scene.serial_helper.is_reading_positions:
            col2.operator("serial.toggle_read_positions", text="停止读取位置", icon='PAUSE')
        else:
            col2.operator("serial.toggle_read_positions", text="开始读取位置", icon='PLAY')


class SerialDataDisplayPanel(bpy.types.Panel):
    bl_label = "接受数据显示"
    bl_idname = "VIEW_3D_PT_DataDisplayPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_context = "scene"

    bl_parent_id = 'VIEW_3D_PT_ReceivingSettings'
    bl_ui_units_x = 0

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        layout.template_list("SERIAL_UL_DataList", "", scene.serial_helper, "serial_data_list", scene.serial_helper, "serial_data_index")
        row = layout.row()
        # row.operator("serial_data.add_item", text="添加项")
        # row.operator("serial_data.delete_item", text="删除项")
        row.operator("serial_data.clear_items", text="清空数据", icon='TRASH')
        row.prop(scene.serial_helper, "serial_data_max_count", text="显示数量")


class SerialDataItemProperties(bpy.types.PropertyGroup):
    index: bpy.props.IntProperty(default=0)
    data_string: bpy.props.StringProperty()


class SERIAL_UL_DataList(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        # item是列表中的每一项，这里我们假设item有两个属性：index和data_string
        row = layout.row(align=True)
        row.alignment = 'LEFT'.upper()
        row.label(text=f"{item.index}")
        row2 = row.row(align=True)
        row2.alignment = 'Expand'.upper()
        row2.scale_x = 1.5
        row2.prop(item, "data_string", text="")


class SerialDataMatchingProperties(bpy.types.PropertyGroup):
    matching_data_name: bpy.props.StringProperty(name="匹配数据名称", default="匹配数据名称")
    matching_data_value: bpy.props.FloatProperty(name="匹配值", default=0)


class SERIAL_UL_DataMatchingList(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        row = layout.row(align=True)
        row.alignment = 'EXPAND'
        row.prop(item, "matching_data_name", text="数据名称")
        row.prop(item, "matching_data_value", text="匹配值")
        row.operator("serial_data_matching.copy_driver", text="", icon='COPYDOWN', emboss=False).index = index


class SerialHelperDataMatchingPanel(bpy.types.Panel):
    bl_label = "数据匹配"
    bl_idname = "VIEW_3D_PT_DataMatchingPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_context = "scene"

    bl_parent_id = 'VIEW_3D_PT_ReceivingSettings'
    bl_ui_units_x = 0

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        row = layout.row()
        row.template_list("SERIAL_UL_DataMatchingList", "", scene.serial_helper, "serial_data_matching_list", scene.serial_helper, "serial_data_matching_index")
        col = row.column(align=True)
        col.operator("serial_data_matching.add_item", icon_value=31, text="")
        col.operator("serial_data_matching.delete_item", icon_value=32, text="")


class SendDataSerialPanel(bpy.types.Panel):
    bl_label = "发送数据"
    bl_idname = "VIEW_3D_PT_SendDataInSerialPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_context = "scene"

    bl_parent_id = 'VIEW3D_PT_serial_help'
    bl_ui_units_x = 0

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        box = layout.box()
        row = box.row()
        row.scale_y = 2
        row.prop(scene.serial_helper, "serial_send_data", text="")
        row2 = box.row()
        row2.scale_y = 2
        row2.operator("serial.send_data_operator", text="发送数据", icon='FILE_TICK')
        row2.prop(scene.serial_helper, "is_auto_send", text="脉冲变化时自动发送", icon_value=118)
        
        # 添加使能和设置零位按钮
        row3 = box.row()
        row3.scale_y = 2
        # 使能按钮列：使能时显示深色
        col_enable = row3.column()
        if scene.serial_helper.motor_enabled:
            col_enable.alert = True  # 使能时按钮变深色（红色警告样式）
            col_enable.operator("serial.toggle_motor_enable", text="电机不使能", icon='CHECKBOX_HLT')
        else:
            col_enable.alert = False
            col_enable.operator("serial.toggle_motor_enable", text="电机使能", icon='CHECKBOX_DEHLT')
        # 设置零位按钮列：不受使能状态影响
        col_zero = row3.column()
        col_zero.operator("serial.set_zero_position", text="设置当前位置为零位", icon='EMPTY_AXIS')
        
        # 添加自动发送关键帧设置
        box.separator()
        row_settings = box.row()
        row_settings.label(text="自动发送关键帧设置:", icon='SETTINGS')
        
        # 缓冲时间设置
        row_buffer = box.row()
        row_buffer.prop(scene.serial_helper, "keyframe_buffer_ms", text="机械误差缓冲(ms)")
        
        # 起始帧和结束帧设置
        row_frames = box.row()
        row_frames.prop(scene.serial_helper, "keyframe_start_frame", text="起始帧")
        row_frames.prop(scene.serial_helper, "keyframe_end_frame", text="结束帧")
        
        # 添加自动发送关键帧按钮
        row4 = box.row()
        row4.scale_y = 2
        if scene.serial_helper.is_auto_keyframe_sending:
            row4.alert = True
            row4.operator("serial.auto_keyframe_send", text="停止发送关键帧", icon='CANCEL')
            row4.label(text=f"当前帧: {scene.serial_helper.current_keyframe}/{scene.serial_helper.total_keyframes}")
        else:
            row4.operator("serial.auto_keyframe_send", text="自动发送关键帧", icon='PLAY')


class SendDataSerialPanel2(bpy.types.Panel):
    bl_label = "发送数据2"
    bl_idname = "VIEW_3D_PT_SendDataInSerialPanel2"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_context = "scene"

    bl_parent_id = 'VIEW3D_PT_serial_help'
    bl_ui_units_x = 0

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        box = layout.box()
        row = box.row()
        row.scale_y = 2
        row.prop(scene.serial_helper, "serial_send_data2", text="")
        row2 = box.row()
        col2 = row2.column()
        col2.scale_y = 2
        col2.operator("serial.send_data_operator2", text="发送数据", icon='FILE_TICK')
        row3 = box.row()
        row3.prop(scene.serial_helper, "is_auto_send2", text="定时发送", icon_value=118)
        row3.prop(scene.serial_helper, "auto_send_interval2", text="发送间隔(s)")


class SendVariablePathItem(bpy.types.PropertyGroup):
    variable_name: bpy.props.StringProperty(name="地址", default="01")
    data_path: bpy.props.StringProperty(name="脉冲数", default="")
    direction: bpy.props.IntProperty(name="方向", default=0, min=0, max=1, description="00: CW, 01: CCW")
    speed: bpy.props.IntProperty(name="速度(RPM)", default=1500, min=0, max=65535)
    acceleration: bpy.props.IntProperty(name="加速度档位", default=0, min=0, max=255)
    accel_compensation: bpy.props.FloatProperty(name="加速度补偿", default=1.0, min=0.5, max=2.0, description="加速度补偿系数，用于调整不同电机的响应速度")
    backlash_compensation: bpy.props.IntProperty(name="间隙补偿(脉冲)", default=0, min=-5000, max=5000, description="关节间隙补偿脉冲数")
    mode: bpy.props.IntProperty(name="模式", default=0, min=0, max=1, description="00: 相对模式, 01: 绝对模式")
    sync: bpy.props.IntProperty(name="同步", default=0, min=0, max=1, description="00: 不同步, 01: 同步")
    checksum: bpy.props.IntProperty(name="校验", default=0, min=0, max=255)


class SERIAL_UL_SendVariable_list(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        row = layout.row(align=True)
        row.alignment = 'LEFT'
        row.label(text=f"{index} ")
        row.prop(item, "variable_name", text="地址")
        row.prop(item, "direction", text="方向")
        row.prop(item, "speed", text="速度")
        row.prop(item, "acceleration", text="加速度")
        row.prop(item, "backlash_compensation", text="间隙补偿")
        row.prop(item, "data_path", text="脉冲数")
        row.prop(item, "mode", text="模式")
        row.prop(item, "sync", text="同步")
        row.prop(item, "checksum", text="校验")


class SerialHelperSendVariablePanel(bpy.types.Panel):
    bl_label = "变量列表"
    bl_idname = "VIEW_3D_PT_SendVariablePanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_context = "scene"

    bl_parent_id = 'VIEW_3D_PT_SendDataInSerialPanel'

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        row = layout.row()
        row.template_list("SERIAL_UL_SendVariable_list", "", scene.serial_helper, "Send_variable_list", scene.serial_helper, "Send_variable_index")

        col = row.column(align=True)
        col.operator("serial.add_variable_operator", icon='ADD', text="")
        col.operator("serial.remove_variable_operator", icon='REMOVE', text="")


class SerialFastMessagePanle(bpy.types.Panel):
    bl_label = "快速消息"
    bl_idname = "VIEW_3D_PT_FastMessagePanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_context = "scene"

    bl_parent_id = 'VIEW_3D_PT_SendDataInSerialPanel'

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        row = layout.row()
        row.template_list("SERIAL_UL_FastMessage_list", "", scene.serial_helper, "fast_message_list", scene.serial_helper, "fast_message_index")
        col = row.column(align=True)
        col.operator("serial.add_fast_message_operator", icon='ADD', text="")
        col.operator("serial.remove_fast_message_operator", icon='REMOVE', text="")


class SerialFastMessageItem(bpy.types.PropertyGroup):
    message_name: bpy.props.StringProperty(name="名称", default="")
    message: bpy.props.StringProperty(name="消息", default="")


class SERIAL_UL_FastMessage_list(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        row = layout.row(align=True)
        row.alignment = 'LEFT'
        row.scale_x = 2
        row.label(text=f"{index+1}:")
        row.prop(item, "message_name", text="")
        row2 = row.row(align=True)
        row2.scale_x = 1.5
        row2.alignment = 'EXPAND'
        row2.prop(item, "message", text="")
        row2.operator("serial.send_fast_message_operator", text="", icon_value=415, emboss=False).index = index


class ClearSerialDataItemsOperator(bpy.types.Operator):
    bl_idname = "serial_data.clear_items"
    bl_label = "Clear All Serial Data Items"

    def execute(self, context):
        scene = context.scene
        scene.serial_helper.serial_data_list.clear()
        scene.serial_helper.serial_data_count = 1
        return {'FINISHED'}


class AddSerialDataMatchingItemOperator(bpy.types.Operator):

    bl_idname = "serial_data_matching.add_item"
    bl_label = "Add Serial Data Matching Item"

    def execute(self, context):
        scene = context.scene
        item = scene.serial_helper.serial_data_matching_list.add()
        scene.serial_helper.serial_data_matching_index = len(scene.serial_helper.serial_data_matching_list) - 1
        item.matching_data_name = "数据名称"
        item.matching__data_value = 0
        return {'FINISHED'}


class RemoveSerialDataMatchingItemOperator(bpy.types.Operator):

    bl_idname = "serial_data_matching.delete_item"
    bl_label = "Delete Serial Data Matching Item"

    def execute(self, context):
        scene = context.scene
        if scene.serial_helper.serial_data_matching_list:
            scene.serial_helper.serial_data_matching_list.remove(scene.serial_helper.serial_data_matching_index)
            if scene.serial_helper.serial_data_matching_index != 0:
                scene.serial_helper.serial_data_matching_index = scene.serial_helper.serial_data_matching_index-1
            else:
                scene.serial_helper.serial_data_matching_index = 0
        return {'FINISHED'}


class CopyDriverSerialDataMatchingItemOperator(bpy.types.Operator):
    bl_idname = "serial_data_matching.copy_driver"
    bl_label = "复制驱动器路径到剪贴板"
    index: bpy.props.IntProperty()

    def execute(self, context):
        full_path = f"#bpy.context.scene.serial_helper.serial_data_matching_list[{self.index}].matching_data_value"
        bpy.context.window_manager.clipboard = full_path
        self.report({'INFO'}, f"已复制驱动器路径: {full_path}")
        return {'FINISHED'}


class UpdateSerialDriverDataMatchingOperator(bpy.types.Operator):
    bl_idname = "serial_data_matching.update_driver"
    bl_label = "更新驱动器数据"

    def execute(self, context):
        scene = context.scene
        scene.serial_helper.driver_remove("serial_data_matching_update_use")
        driver = scene.serial_helper.driver_add("serial_data_matching_update_use").driver
        var = driver.variables.new()
        var.name = "serial_data_matching_update_use"
        var.type = "SINGLE_PROP"
        var.targets[0].id_type = "SCENE"
        var.targets[0].id = bpy.context.scene
        var.targets[0].data_path = "serial_helper.serial_data_matching_list[0].matching_data_value"
        driver.expression = f"serial_data_matching_update_use"
        return {'FINISHED'}




def format_replace_var_string(self, input_string):
    scene = bpy.context.scene
    eval_globals = {
        'random': random,
        'math': math,
    }
    
    # 电机参数
    MAX_SPEED = 5000  # 最大速度（用户指定为5000）
    PULSES_PER_REV = 3200  # 每转脉冲数
    
    # 第一步：收集所有电机的脉冲数据
    motor_data = []
    
    for item in scene.serial_helper.Send_variable_list:
        try:
            # 处理脉冲数
            pulse_value = None
            
            try:
                # 尝试直接求值表达式
                eval_globals = {
                    'bpy': bpy,
                    'math': math,
                    '__builtins__': {},
                }
                
                value = eval(item.data_path, eval_globals)
                pulse_value = float(value)
            except Exception as e:
                try:
                    value = float(item.data_path)
                    pulse_value = value
                except:
                    pulse_value = 3200
            
            # 处理负角度
            actual_pulse_value = pulse_value if pulse_value is not None else 3200
            direction = item.direction
            
            # 保存原始脉冲值用于计算差值
            original_pulse_value = actual_pulse_value
            
            if actual_pulse_value < 0:
                direction = 1 - item.direction
                actual_pulse_value = abs(actual_pulse_value)
            
            pulses = int(actual_pulse_value)
            address = item.variable_name
            
            # 获取上一次发送的原始脉冲值（带符号）
            last_pulses_key = f"last_pulses_{address}"
            last_original_pulses_key = f"last_original_pulses_{address}"
            
            if last_original_pulses_key in bpy.app.driver_namespace:
                last_original_pulses = bpy.app.driver_namespace[last_original_pulses_key]
            else:
                last_original_pulses = 0
                bpy.app.driver_namespace[last_original_pulses_key] = 0
            
            # 计算原始脉冲值的差值（带符号），然后取绝对值
            pulse_diff = abs(original_pulse_value - last_original_pulses)
            
            # 注意：不在此处更新上一次值，而是在成功发送后更新
            # 这样可以确保只有在真正发送后才记录值
            
            # 记录电机数据
            motor_info = {
                'item': item,
                'pulses': pulses,  # 绝对位置（用于发送）
                'direction': direction,
                'address': address,
                'pulse_diff': pulse_diff,  # 脉冲差值
                'last_original_pulses': last_original_pulses,  # 上一次的原始脉冲值
                'original_pulse_value': original_pulse_value  # 当前原始脉冲值
            }
            motor_data.append(motor_info)
            
        except Exception as e:
            print(f"变量 {item.variable_name} 数据路径 {item.data_path} 获取失败: {e}")
            continue
    
    # 第二步：如果没有电机数据，直接返回
    if not motor_data:
        return input_string
    
    # 第三步：先筛选出有效电机（差值>10），只对有效电机进行加速度分配
    valid_motors_for_calc = []
    for motor_info in motor_data:
        pulse_diff = motor_info['pulse_diff']
        
        # 差值10以内不处理
        if pulse_diff <= 5:
            motor_info['calculated_speed'] = 0
            motor_info['calculated_acceleration'] = 0
            continue
        
        valid_motors_for_calc.append(motor_info)
    
    # 如果没有有效电机，使用默认速度和加速度
    if not valid_motors_for_calc:
        for motor_info in motor_data:
            motor_info['calculated_speed'] = 3000  # 默认速度
            motor_info['calculated_acceleration'] = 140  # 默认加速度
        valid_motors = motor_data
        has_sync_enabled = any(m['item'].sync == 1 for m in motor_data)
        
        for motor_info in motor_data:
            item = motor_info['item']
            pulses = motor_info['pulses']
            direction = motor_info['direction']
            address = motor_info['address']
            
            # 获取间隙补偿值
            backlash_compensation = item.backlash_compensation
            
            # 检查方向是否改变，以确定是否需要间隙补偿
            last_direction_key = f"last_direction_{address}"
            if last_direction_key in bpy.app.driver_namespace:
                last_direction = bpy.app.driver_namespace[last_direction_key]
                direction_changed = (last_direction != direction)
            else:
                last_direction = direction
                direction_changed = False
            
            # 应用间隙补偿
            applied_backlash = 0
            if direction_changed and backlash_compensation > 0:
                # 根据当前方向调整脉冲值
                if direction == 0:  # CW
                    pulses += backlash_compensation
                    applied_backlash = backlash_compensation
                else:  # CCW
                    pulses -= backlash_compensation
                    applied_backlash = backlash_compensation
                # 确保脉冲值为正数
                if pulses < 0:
                    pulses = abs(pulses)
                    direction = 1 - direction
            
            # 保存当前方向
            bpy.app.driver_namespace[last_direction_key] = direction
            
            # 使用默认速度和加速度
            speed = 3000  # 默认速度
            acceleration = 140  # 默认加速度
            
            # 更新UI显示速度和加速度值
            item.speed = speed
            item.acceleration = acceleration
            
            # 使用用户设置的模式和同步值
            mode = 1  # 01: 绝对模式
            sync = item.sync
            
            # 计算校验和
            checksum = item.checksum
            
            # 按照规则格式构建数据
            direction_hex = f"{direction:02X}"
            speed_hex = f"{speed:04X}"
            acceleration_hex = f"{acceleration:02X}"
            pulses_hex = f"{pulses:08X}"
            mode_hex = f"{mode:02X}"
            sync_hex = f"{sync:02X}"
            checksum_hex = f"{checksum:02X}"
            
            # 构建完整的命令
            formatted_data = f"{address} FD {direction_hex} {speed_hex[:2]} {speed_hex[2:]} {acceleration_hex} {pulses_hex[:2]} {pulses_hex[2:4]} {pulses_hex[4:6]} {pulses_hex[6:]} {mode_hex} {sync_hex} {checksum_hex}"
            
            # 替换到输入字符串中
            input_string = input_string.replace(f"{{{address}}}", formatted_data)
            
            # 更新上一次发送的原始脉冲值
            last_original_pulses_key = f"last_original_pulses_{address}"
            bpy.app.driver_namespace[last_original_pulses_key] = motor_info['original_pulse_value']
        
        # 如果有电机启用了同步，发送同步启动命令
        # 这部分在SendDataSerialOperator中处理
        
        return input_string
    
    # 按脉冲差值从大到小排序
    valid_motors_for_calc.sort(key=lambda x: x['pulse_diff'], reverse=True)
    
    # 分配补偿系数
    compensation_factors = [2.0, 0.8, 0.6, 0.4, 0.2, 0.1]
    for i, motor_info in enumerate(valid_motors_for_calc):
        if i < len(compensation_factors):
            motor_info['auto_accel_compensation'] = compensation_factors[i]
        else:
            motor_info['auto_accel_compensation'] = 0.1  # 超过6个电机时使用最小值
    
    # 只基于有效电机计算最大和最小脉冲差值
    max_pulse_diff = valid_motors_for_calc[0]['pulse_diff'] if valid_motors_for_calc else 0
    min_pulse_diff = valid_motors_for_calc[-1]['pulse_diff'] if valid_motors_for_calc else 0
    
    # 以最小脉冲差值作为基准（加速度档位最小，加速最慢）
    ref_pulse_diff = min_pulse_diff
    
    # 基准参数设定
    BASE_ACC = 1  # 基准加速度档位（最小脉冲差值使用此值）
    BASE_SPEED = 1000  # 基准转速 RPM
    
    # 首先计算每个电机的速度（基于脉冲差值比例）
    for motor_info in valid_motors_for_calc:
        pulse_diff = motor_info['pulse_diff']
        
        # 计算目标转速：V = V基准 × (PN / P基准)
        # 脉冲数越多，目标转速越高
        if ref_pulse_diff > 0:
            calculated_speed = BASE_SPEED * (pulse_diff / ref_pulse_diff)
            calculated_speed = max(1, min(MAX_SPEED, int(round(calculated_speed))))
        else:
            calculated_speed = BASE_SPEED
        
        motor_info['calculated_speed'] = calculated_speed
    
    # 计算参考运动时间（基于最大脉冲差值和基础加速度）
    # 使用运动学公式：T = 2 * V / a + (P - V²/a) / V
    # 其中 V 是速度（RPM），a 是加速度档位，P 是脉冲差值
    ref_pulse = max_pulse_diff
    ref_speed = BASE_SPEED
    
    # 动态计算加速度，确保所有电机同时到达目标
    # 设 T_max 为参考运动时间（最大脉冲差值电机的时间）
    # 根据梯形速度曲线公式反推加速度
    # T = 2 * V / acc + (P - V²/acc) / V
    # 简化：T = 2V/acc + P/V - V/acc = P/V + V/acc
    # 所以：acc = V² / (T*V - P) = V / (T - P/V)
    
    # 先用基础加速度计算参考时间
    ref_acc = BASE_ACC
    ref_time = ref_pulse / ref_speed + ref_speed / ref_acc  # 简化公式
    print(f"[同步计算] 参考运动时间: {ref_time:.2f} (基于最大脉冲差值 {ref_pulse})")
    
    # 为每个电机动态计算加速度，使运动时间相同
    for motor_info in valid_motors_for_calc:
        pulse_diff = motor_info['pulse_diff']
        item = motor_info['item']
        speed = motor_info['calculated_speed']
        
        # 使用自动分配的补偿系数
        accel_compensation = motor_info.get('auto_accel_compensation', 1.0)
        
        # 动态计算加速度：使所有电机运动时间相同
        # T = P/V + V/acc，所以 acc = V / (T - P/V)
        # 对于小脉冲，需要确保加速度时间足够长
        min_accel_time_ms = 800  # 最小加速度时间800ms
        
        # 计算使所有电机同步的加速度
        # 使用简化公式：acc = speed² / (pulse_diff * speed / 1000 - min_accel_time_ms)
        # 转换为加速度档位（256对应最大加速度）
        if pulse_diff > 0 and speed > 0:
            # 计算所需加速度
            time_per_pulse = 1000 / speed  # 每毫秒脉冲数
            required_acc = speed / (ref_time - pulse_diff / speed) if ref_time > pulse_diff / speed else 255
            
            # 将加速度转换为档位（256 - acc）
            calculated_acc = 256 - required_acc
            # 应用加速度补偿系数
            calculated_acc = calculated_acc * accel_compensation
            calculated_acc = max(1, min(210, int(round(calculated_acc))))
        else:
            calculated_acc = BASE_ACC
        
        motor_info['calculated_acceleration'] = calculated_acc
    
    # 检查是否有电机启用了同步
    has_sync_enabled = any(m['item'].sync == 1 for m in motor_data)
    
    # 第四步：为每个电机构建命令
    valid_motors = []
    for motor_info in motor_data:
        # 跳过速度为0的电机（差值<=10或差值=1）
        if motor_info['calculated_speed'] == 0:
            continue
        
        valid_motors.append(motor_info)
    
    # 计算最长运动时间（用于同步）
    max_motion_time_ms = 0
    
    for motor_info in valid_motors:
        item = motor_info['item']
        pulses = motor_info['pulses']  # 绝对位置（用于发送）
        direction = motor_info['direction']
        address = motor_info['address']
        
        # 获取间隙补偿值
        backlash_compensation = item.backlash_compensation
        
        # 检查方向是否改变，以确定是否需要间隙补偿
        last_direction_key = f"last_direction_{address}"
        if last_direction_key in bpy.app.driver_namespace:
            last_direction = bpy.app.driver_namespace[last_direction_key]
            direction_changed = (last_direction != direction)
        else:
            last_direction = direction
            direction_changed = False
        
        # 应用间隙补偿
        applied_backlash = 0
        if direction_changed and backlash_compensation > 0:
            print(f"[间隙补偿] 电机 {address}: 方向改变，应用间隙补偿 {backlash_compensation} 脉冲")
            # 根据当前方向调整脉冲值
            if direction == 0:  # CW
                pulses += backlash_compensation
                applied_backlash = backlash_compensation
            else:  # CCW
                pulses -= backlash_compensation
                applied_backlash = backlash_compensation
            # 确保脉冲值为正数
            if pulses < 0:
                pulses = abs(pulses)
                direction = 1 - direction
        
        # 保存当前方向
        bpy.app.driver_namespace[last_direction_key] = direction
        
        # 使用计算出的速度值
        speed = motor_info['calculated_speed']
        
        # 使用计算出的加速度值
        acceleration = motor_info['calculated_acceleration']
        
        # 更新UI显示计算后的速度和加速度值
        item.speed = speed
        item.acceleration = acceleration
        
        # 使用用户设置的模式和同步值
        mode = 1  # 01: 绝对模式（保持绝对模式）
        sync = item.sync  # 使用用户设置的同步值
        
        # 计算校验和
        checksum = item.checksum
        
        # 按照规则格式构建数据
        direction_hex = f"{direction:02X}"
        speed_hex = f"{speed:04X}"  # 2字节
        acceleration_hex = f"{acceleration:02X}"  # 1字节
        pulses_hex = f"{pulses:08X}"  # 4字节（绝对位置）
        mode_hex = f"{mode:02X}"
        sync_hex = f"{sync:02X}"
        checksum_hex = f"{checksum:02X}"
        
        # 构建完整的命令
        formatted_data = f"{address} FD {direction_hex} {speed_hex[:2]} {speed_hex[2:]} {acceleration_hex} {pulses_hex[:2]} {pulses_hex[2:4]} {pulses_hex[4:6]} {pulses_hex[6:]} {mode_hex} {sync_hex} {checksum_hex}"
        
        # 替换到输入字符串中
        input_string = input_string.replace(f"{{{address}}}", formatted_data)
        
        # 更新上一次发送的原始脉冲值
        last_original_pulses_key = f"last_original_pulses_{address}"
        bpy.app.driver_namespace[last_original_pulses_key] = motor_info['original_pulse_value']
        
        # 计算该电机的运动时间
        # 基于脉冲差值、速度和加速度计算实际运动时间
        pulse_diff = motor_info['pulse_diff']
        
        # 如果应用了间隙补偿，需要将补偿脉冲数加入到总脉冲差中
        if applied_backlash > 0:
            pulse_diff += applied_backlash
        
        if speed > 0 and acceleration < 256 and pulse_diff > 0:
            # 加速度档位转换为加速度值 (RPM/s)
            # acc=0: 不使用曲线加减速，直接以最大速度运行
            # acc=1-255: 加速度档位，值越大加速越快
            
            if acceleration == 0:
                # 不使用曲线加减速，直接计算匀速运动时间
                # 假设电机响应速度约为 1000脉冲/秒 (需要根据实际电机调整)
                pulses_per_second = speed * 10  # 粗略估计：RPM * 10 = 脉冲/秒
                motion_time_ms = (pulse_diff / pulses_per_second) * 1000
                # 根据脉冲差值调整最小时间
                if pulse_diff < 100:
                    motion_time_ms = max(100, motion_time_ms)  # 小脉冲最少200ms
                elif pulse_diff < 300:
                    motion_time_ms = max(200, motion_time_ms)  # 中等脉冲最少300ms
                else:
                    motion_time_ms = max(300, motion_time_ms)  # 大脉冲最少400ms
            else:
                # 使用曲线加减速
                # 单RPM耗时：T = (256 - acc) * 50 μs
                time_per_rpm_us = (256 - acceleration) * 50
                
                # 加速到目标速度所需时间 (ms)
                accel_time_ms = (speed * time_per_rpm_us) / 1000
                
                # 估算加速阶段走的脉冲数 (简化为平均速度的一半)
                # 平均速度 = speed / 2 (加速阶段的平均RPM)
                # 脉冲数 = 平均速度 * 时间 / 60 * 每转脉冲数
                # 简化计算：假设 100脉冲/RPM
                pulses_during_accel = (speed / 2) * (accel_time_ms / 1000) * 100 / 60
                
                if pulse_diff <= pulses_during_accel * 2:
                    # 脉冲数很少，可能还没加速到最大速度就已经到达目标
                    # 简化为三角形速度曲线
                    motion_time_ms = 2 * math.sqrt(pulse_diff / (speed * 100 / 60 / accel_time_ms * 1000))
                    # 根据脉冲差值调整最小时间
                    if pulse_diff < 100:
                        motion_time_ms = max(300, motion_time_ms)  # 小脉冲最少300ms
                    elif pulse_diff < 500:
                        motion_time_ms = max(500, motion_time_ms)  # 中等脉冲最少500ms
                    else:
                        motion_time_ms = max(800, motion_time_ms)  # 大脉冲最少800ms
                else:
                    # 完整的加速-匀速-减速过程
                    # 加速+减速时间
                    accel_decel_time_ms = accel_time_ms * 2
                    
                    # 匀速阶段的脉冲数和时间
                    pulses_at_constant_speed = pulse_diff - pulses_during_accel * 2
                    constant_speed_time_ms = (pulses_at_constant_speed / (speed * 100 / 60)) * 1000
                    
                    motion_time_ms = accel_decel_time_ms + constant_speed_time_ms
                
                # 不再添加额外缓冲，只使用用户设置的缓冲
            
            print(f"  电机 {address} 运动时间计算: 脉冲差值={pulse_diff:.0f}, 速度={speed}RPM, 加速度={acceleration}, 运动时间={motion_time_ms:.1f}ms")
            
            # 取所有电机中最长的运动时间
            if motion_time_ms > max_motion_time_ms:
                max_motion_time_ms = motion_time_ms
    
    # 保存计算出的运动时间到全局命名空间，供自动发送关键帧使用
    # 获取用户设置的缓冲时间
    scene = bpy.context.scene
    buffer_ms = scene.serial_helper.keyframe_buffer_ms
    
    if max_motion_time_ms > 0:
        # 添加用户设置的机械误差缓冲时间
        max_motion_time_ms += buffer_ms
        # 限制最大等待时间为10秒（避免过长等待）
        max_motion_time_ms = min(max_motion_time_ms, 10000)
        bpy.app.driver_namespace["last_calculated_motion_time"] = max_motion_time_ms / 1000.0  # 转换为秒
        print(f"[运动时间] 本帧最长运动时间: {max_motion_time_ms:.1f}ms ({max_motion_time_ms/1000.0:.3f}s，含{buffer_ms}ms机械误差缓冲)")
    else:
        # 无有效电机时使用默认时间（含缓冲）
        default_time_ms = 500 + buffer_ms
        bpy.app.driver_namespace["last_calculated_motion_time"] = default_time_ms / 1000.0
    
    return input_string



class SendDataSerialOperator(bpy.types.Operator):
    bl_idname = "serial.send_data_operator"
    bl_label = "发送数据"

    def execute(self, context):
        scene = context.scene
        SerialConnection = bpy.app.driver_namespace.get("serial_connection")
        
        if not SerialConnection or not SerialConnection.serial:
            self.report({'ERROR'}, "串口未连接")
            return {'CANCELLED'}
        
        data_to_send = scene.serial_helper.serial_send_data
        
        if not data_to_send or data_to_send.strip() == "":
            self.report({'ERROR'}, "发送数据为空")
            return {'CANCELLED'}
        
        var_replace_str = format_replace_var_string(self, data_to_send)
        
        if not var_replace_str or var_replace_str.strip() == "":
            self.report({'ERROR'}, "格式化后的数据为空")
            return {'CANCELLED'}
        
        # 按逗号分割多个命令
        commands = var_replace_str.split(',')
        
        # 检查是否有电机启用了同步
        has_sync_enabled = False
        for item in scene.serial_helper.Send_variable_list:
            if item.sync == 1:
                has_sync_enabled = True
                break
        
        # 逐条发送每个命令
        for i, cmd in enumerate(commands):
            cmd = cmd.strip()
            if cmd:
                # 清理命令：移除花括号等非十六进制字符，只保留十六进制字符和空格
                cleaned_cmd = re.sub(r'[^0-9A-Fa-f\s]', '', cmd)
                
                if not cleaned_cmd:
                    continue
                
                try:
                    # 将字符串转换为16进制格式
                    hex_data = bytes.fromhex(cleaned_cmd)
                    
                    # 逐条发送每个命令
                    SerialConnection.serial.write(hex_data)
                    
                    # 如果启用了换行，在每个命令后添加换行符
                    if scene.serial_helper.is_newline:
                        SerialConnection.serial.write(b'\r\n')
                except Exception as e:
                    self.report({'ERROR'}, f"发送命令{i+1}失败: {e}")
                
                # 在命令之间添加小延迟
                time.sleep(0.005)
        
        # 如果有电机启用了同步，发送同步启动命令
        if has_sync_enabled:
            try:
                sync_cmd = "00 FF 66 6B"
                hex_data = bytes.fromhex(sync_cmd)
                SerialConnection.serial.write(hex_data)
                
                if scene.serial_helper.is_newline:
                    SerialConnection.serial.write(b'\r\n')
            except Exception as e:
                self.report({'WARNING'}, f"发送同步启动命令失败: {e}")
                self.report({'ERROR'}, f"发送同步启动命令失败: {e}")

        return {'FINISHED'}


class SendDataSerialOperator2(bpy.types.Operator):
    bl_idname = "serial.send_data_operator2"
    bl_label = "发送数据2"

    def execute(self, context):
        scene = context.scene
        SerialConnection = bpy.app.driver_namespace["serial_connection"]
        data_to_send = scene.serial_helper.serial_send_data2
        
        print(f"原始输入字符串(2): {data_to_send}")
        
        var_replace_str = format_replace_var_string(self, data_to_send)
        print(f"替换变量后的字符串(2): {var_replace_str}")
        
        # 按逗号分割多个命令
        commands = var_replace_str.split(',')
        
        for i, cmd in enumerate(commands):
            cmd = cmd.strip()
            if cmd:
                # 清理命令：移除花括号等非十六进制字符，只保留十六进制字符和空格
                cleaned_cmd = re.sub(r'[^0-9A-Fa-f\s]', '', cmd)
                print(f"清理后的命令{i+1}(2): {cleaned_cmd}")
                
                # 将字符串转换为16进制格式
                hex_data = bytes.fromhex(cleaned_cmd)
                print(f"命令{i+1}的十六进制数据(2): {hex_data}")
                print(f"命令{i+1}的十六进制字符串(2): {' '.join([f'{b:02X}' for b in hex_data])}")
                
                # 逐条发送每个命令
                SerialConnection.serial.write(hex_data)
                
                # 如果启用了换行，在每个命令后添加换行符
                if scene.serial_helper.is_newline2:
                    SerialConnection.serial.write(b'\r\n')
                
                print(f"已发送命令{i+1}(2)")
                
                # 在命令之间添加小延迟
                time.sleep(0.01)

        return {'FINISHED'}








class AddSerialHelperSendVariableOperator(bpy.types.Operator):
    bl_idname = "serial.add_variable_operator"
    bl_label = "Add Variable"

    def execute(self, context):
        item = context.scene.serial_helper.Send_variable_list.add()
        # 设置默认值为命令示例 01 FD 01 05 DC 00 00 00 7D 00 00 00 6B 中的值
        item.variable_name = "01"  # 地址
        item.direction = 1  # 方向 (01 in hex)
        item.speed = 1500  # 速度 (05 DC = 1500 RPM)
        item.acceleration = 0  # 加速度档位 (00 in hex)
        item.data_path = "32000"  # 脉冲数 (00 00 7D 00 = 32000)
        item.mode = 0  # 模式 (00 in hex, 相对位置模式)
        item.sync = 0  # 同步 (00 in hex, 不同步)
        item.checksum = 107  # 校验 (6B in hex)
        context.scene.serial_helper.Send_variable_index = len(context.scene.serial_helper.Send_variable_list) - 1
        return {'FINISHED'}


class RemoveSerialHelperSendVariableOperator(bpy.types.Operator):
    bl_idname = "serial.remove_variable_operator"
    bl_label = "Remove Variable"

    def execute(self, context):
        variable_list = context.scene.serial_helper.Send_variable_list
        index = context.scene.serial_helper.Send_variable_index
        variable_list.remove(index)
        if index != 0:
            context.scene.serial_helper.Send_variable_index = context.scene.serial_helper.Send_variable_index - 1
        return {'FINISHED'}


class AddSerialFastMessageListOperator(bpy.types.Operator):
    bl_idname = "serial.add_fast_message_operator"
    bl_label = "Add Fast Message"

    def execute(self, context):
        item = context.scene.serial_helper.fast_message_list.add()
        item.message_name = "name_" + str(len(context.scene.serial_helper.fast_message_list))
        item.message = ""
        context.scene.serial_helper.fast_message_index = len(context.scene.serial_helper.fast_message_list) - 1
        return {'FINISHED'}


class RemoveSerialFastMessageListOperator(bpy.types.Operator):
    bl_idname = "serial.remove_fast_message_operator"
    bl_label = "Remove Fast Message"

    def execute(self, context):
        list = context.scene.serial_helper.fast_message_list
        index = context.scene.serial_helper.fast_message_index
        list.remove(index)
        if index != 0:
            context.scene.serial_helper.fast_message_index = context.scene.serial_helper.fast_message_index - 1
        return {'FINISHED'}


class SendFastMessageOperator(bpy.types.Operator):
    bl_idname = "serial.send_fast_message_operator"
    bl_label = "发送快速消息"
    index: bpy.props.IntProperty()

    def execute(self, context):
        scene = context.scene
        message = scene.serial_helper.fast_message_list[self.index].message
        scene.serial_helper.serial_send_data = message
        bpy.ops.serial.send_data_operator()
        return {'FINISHED'}


def send_data_periodically():
    scene = bpy.context.scene
    if scene.serial_helper.is_auto_send:
        # 检查电机是否到位
        if scene.serial_helper.motor_arrived:
            print(f"[自动发送] 电机已到位，重置到位状态")
            scene.serial_helper.motor_arrived = False
            return 0.01  # 等待下一次检查
        
        # 获取当前要发送的数据字符串
        data_to_send = scene.serial_helper.serial_send_data
        
        if not data_to_send or data_to_send.strip() == "":
            return 0.01
        
        # 使用 driver_namespace 存储上一次发送的数据
        last_data_key = "last_sent_data_auto_send"
        if last_data_key not in bpy.app.driver_namespace:
            bpy.app.driver_namespace[last_data_key] = ""
        
        last_sent_data = bpy.app.driver_namespace[last_data_key]
        
        # 格式化当前数据（计算变量值）
        try:
            current_formatted_data = format_replace_var_string(None, data_to_send)
        except Exception as e:
            print(f"格式化数据失败: {e}")
            return 0.01
        
        # 检查数据是否发生变化
        if current_formatted_data != last_sent_data:
            print(f"检测到数据变化，准备发送")
            print(f"上次数据: {last_sent_data}")
            print(f"当前数据: {current_formatted_data}")
            
            # 重置到位状态，等待新的到位信号
            scene.serial_helper.motor_arrived = False
            
            # 调用发送函数（在后台线程中执行）
            send_thread = threading.Thread(
                target=send_data_async_simple,
                args=(current_formatted_data,),
                daemon=True
            )
            send_thread.start()
            
            # 更新上一次发送的数据
            bpy.app.driver_namespace[last_data_key] = current_formatted_data
        
        return 0.01  # 每0.01秒检查一次
    else:
        return None  # Stop the timer if sending is disabled


def send_data_async_simple(formatted_data):
    """简化的异步发送函数 - 直接发送格式化后的数据"""
    try:
        import bpy
        import re
        import time
        
        scene = bpy.context.scene
        SerialConnection = bpy.app.driver_namespace.get("serial_connection")
        if not SerialConnection or not SerialConnection.serial:
            print("串口未连接")
            return
        
        is_newline = scene.serial_helper.is_newline
        
        # 按逗号分割多个命令
        commands = formatted_data.split(',')
        print(f"分割后的命令数量: {len(commands)}")
        
        # 检查是否有电机启用了同步
        has_sync_enabled = False
        for item in scene.serial_helper.Send_variable_list:
            if item.sync == 1:
                has_sync_enabled = True
                break
        
        print(f"同步模式: {has_sync_enabled}")
        
        # 逐条发送每个命令
        for i, cmd in enumerate(commands):
            cmd = cmd.strip()
            if cmd:
                print(f"处理命令{i+1}: {cmd}")
                
                # 清理命令：移除花括号等非十六进制字符
                cleaned_cmd = re.sub(r'[^0-9A-Fa-f\s]', '', cmd)
                print(f"清理后的命令{i+1}: {cleaned_cmd}")
                
                if not cleaned_cmd:
                    print(f"命令{i+1}清理后为空，跳过")
                    continue
                
                try:
                    hex_data = bytes.fromhex(cleaned_cmd)
                    print(f"命令{i+1}的十六进制数据: {' '.join([f'{b:02X}' for b in hex_data])}")
                    
                    SerialConnection.serial.write(hex_data)
                    
                    if is_newline:
                        SerialConnection.serial.write(b'\r\n')
                    
                    print(f"已发送命令{i+1}")
                except Exception as e:
                    print(f"发送命令{i+1}失败: {e}")
                
                time.sleep(0.01)
        
        # 如果有电机启用了同步，发送同步启动命令
        if has_sync_enabled:
            try:
                sync_cmd = "00 FF 66 6B"
                print(f"发送同步启动命令: {sync_cmd}")
                hex_data = bytes.fromhex(sync_cmd)
                SerialConnection.serial.write(hex_data)
                
                if is_newline:
                    SerialConnection.serial.write(b'\r\n')
                
                print("已发送同步启动命令")
            except Exception as e:
                print(f"发送同步启动命令失败: {e}")
                
    except Exception as e:
        print(f"异步发送数据失败: {e}")
        import traceback
        traceback.print_exc()


def send_data_periodically2():
    scene = bpy.context.scene
    if scene.serial_helper.is_auto_send2:
        bpy.ops.serial.send_data_operator2()
        auto_send_interval = scene.serial_helper.auto_send_interval2
        return auto_send_interval  # Repeat every second
    else:
        return None  # Stop the timer if sending is disabled


def update_sending_state(self, context):
    # timer_handle = bpy.app.timers.is_registered(send_data_periodically)
    # print(timer_handle)
    if bpy.context.scene.serial_helper.is_auto_send:
        # 重置上一次发送的数据
        bpy.app.driver_namespace["last_sent_data_auto_send"] = ""
        bpy.app.timers.register(send_data_periodically, first_interval=0.01)
    else:
        bpy.app.timers.unregister(send_data_periodically)


def update_sending_state2(self, context):
    if bpy.context.scene.serial_helper.is_auto_send2:
        bpy.app.timers.register(send_data_periodically2, first_interval=1.0)
    else:
        bpy.app.timers.unregister(send_data_periodically2)


# ============ 读取电机位置功能 ============

def read_motor_positions_periodically():
    """定时读取电机位置的函数 - 使用后台线程避免卡顿"""
    scene = bpy.context.scene
    if scene.serial_helper.is_reading_positions:
        # 在后台线程中执行读取操作
        read_thread = threading.Thread(target=read_all_motor_positions, daemon=True)
        read_thread.start()
        return 0.05  # 每0.05秒触发一次读取，提高读取频率
    else:
        return None


def read_all_motor_positions():
    """在后台线程中读取所有电机位置 - 优化版本"""
    try:
        scene = bpy.context.scene
        SerialConnection = bpy.app.driver_namespace.get("serial_connection")
        if not SerialConnection or not SerialConnection.serial:
            print("串口未连接，无法读取电机位置")
            return
        
        # 批量发送所有读取命令，减少等待时间
        motor_list = list(scene.serial_helper.Send_variable_list)
        if not motor_list:
            return
        
        # 快速连续发送所有命令
        for item in motor_list:
            try:
                address = str(item.variable_name).strip()
                cmd = f"{address} 36 6B"
                hex_data = bytes.fromhex(cmd.replace(" ", ""))
                SerialConnection.serial.write(hex_data)
                time.sleep(0.005)  # 极短间隔，快速发送
            except Exception as e:
                print(f"[读取线程] 发送命令失败: {e}")
        
        # 发送完成后，给主接收线程时间处理所有返回数据
        # 不需要在这里等待，让主线程异步处理
                
    except Exception as e:
        print(f"读取电机位置线程失败: {e}")





class ToggleReadMotorPositionsOperator(bpy.types.Operator):
    """切换读取电机位置状态"""
    bl_idname = "serial.toggle_read_positions"
    bl_label = "实时读取位置"
    bl_description = "点击开始/停止实时读取电机位置"
    
    def execute(self, context):
        scene = context.scene
        scene.serial_helper.is_reading_positions = not scene.serial_helper.is_reading_positions
        
        if scene.serial_helper.is_reading_positions:
            # 启动定时读取
            bpy.app.timers.register(read_motor_positions_periodically, first_interval=0.1)
            self.report({'INFO'}, "开始读取电机位置")
            print("开始读取电机位置")
        else:
            # 停止定时读取
            bpy.app.timers.unregister(read_motor_positions_periodically)
            self.report({'INFO'}, "停止读取电机位置")
            print("停止读取电机位置")
        
        return {'FINISHED'}


class ToggleMotorEnableOperator(bpy.types.Operator):
    """切换电机使能状态"""
    bl_idname = "serial.toggle_motor_enable"
    bl_label = "电机使能"
    bl_description = "点击使能/不使能电机"
    
    def execute(self, context):
        scene = context.scene
        SerialConnection = bpy.app.driver_namespace.get("serial_connection")
        
        if not SerialConnection or not SerialConnection.serial:
            self.report({'ERROR'}, "串口未连接")
            return {'CANCELLED'}
        
        # 切换使能状态
        scene.serial_helper.motor_enabled = not scene.serial_helper.motor_enabled
        
        if scene.serial_helper.motor_enabled:
            # 使能：发送 00 F3 AB 01 00 6B
            cmd = "00 F3 AB 01 00 6B"
            self.report({'INFO'}, "电机使能")
            print("电机使能")
        else:
            # 不使能：发送 00 F3 AB 00 00 6B
            cmd = "00 F3 AB 00 00 6B"
            self.report({'INFO'}, "电机不使能")
            print("电机不使能")
        
        # 发送命令
        try:
            hex_data = bytes.fromhex(cmd.replace(" ", ""))
            SerialConnection.serial.write(hex_data)
            print(f"已发送: {cmd}")
        except Exception as e:
            self.report({'ERROR'}, f"发送失败: {e}")
            return {'CANCELLED'}
        
        return {'FINISHED'}


class SetZeroPositionOperator(bpy.types.Operator):
    """设置当前位置为零位"""
    bl_idname = "serial.set_zero_position"
    bl_label = "设置零位"
    bl_description = "将当前位置设置为零位"
    
    def execute(self, context):
        scene = context.scene
        SerialConnection = bpy.app.driver_namespace.get("serial_connection")
        
        if not SerialConnection or not SerialConnection.serial:
            self.report({'ERROR'}, "串口未连接")
            return {'CANCELLED'}
        
        # 发送 00 0A 6D 6B
        cmd = "00 0A 6D 6B"
        
        try:
            hex_data = bytes.fromhex(cmd.replace(" ", ""))
            SerialConnection.serial.write(hex_data)
            print(f"已发送设置零位命令: {cmd}")
            self.report({'INFO'}, "已设置当前位置为零位")
        except Exception as e:
            self.report({'ERROR'}, f"发送失败: {e}")
            return {'CANCELLED'}
        
        return {'FINISHED'}


class AutoKeyframeSendOperator(bpy.types.Operator):
    """自动发送关键帧"""
    bl_idname = "serial.auto_keyframe_send"
    bl_label = "自动发送关键帧"
    bl_description = "自动从第一个关键帧开始发送，等待到位后发送下一个"
    
    def execute(self, context):
        scene = context.scene
        
        if scene.serial_helper.is_auto_keyframe_sending:
            # 停止自动发送
            scene.serial_helper.is_auto_keyframe_sending = False
            self.report({'INFO'}, "已停止自动发送关键帧")
            return {'FINISHED'}
        
        # 启动自动发送
        scene.serial_helper.is_auto_keyframe_sending = True
        
        # 使用用户设置的起始帧和结束帧
        start_frame = scene.serial_helper.keyframe_start_frame
        end_frame = scene.serial_helper.keyframe_end_frame
        
        scene.serial_helper.current_keyframe = start_frame
        scene.serial_helper.total_keyframes = end_frame
        scene.serial_helper.sent_addresses = ""
        scene.serial_helper.last_arrival_time = 0
        
        # 初始化下一帧发送时间到 driver_namespace
        bpy.app.driver_namespace["auto_keyframe_next_time"] = 0
        
        self.report({'INFO'}, f"开始自动发送关键帧，从第{start_frame}帧到第{end_frame}帧")
        
        # 启动定时器
        bpy.app.timers.register(auto_keyframe_send_handler, first_interval=0.1)
        
        return {'FINISHED'}


def auto_keyframe_send_handler():
    """自动发送关键帧的处理函数 - 基于计算的运动时间"""
    scene = bpy.context.scene
    
    if not scene.serial_helper.is_auto_keyframe_sending:
        return None
    
    # 检查是否已完成所有关键帧
    if scene.serial_helper.current_keyframe > scene.serial_helper.total_keyframes:
        scene.serial_helper.is_auto_keyframe_sending = False
        print("所有关键帧发送完成")
        return None
    
    # 获取当前时间
    current_time = time.time()
    
    # 从 driver_namespace 获取 next_frame_time
    next_frame_time_key = "auto_keyframe_next_time"
    if next_frame_time_key not in bpy.app.driver_namespace:
        bpy.app.driver_namespace[next_frame_time_key] = 0
    
    next_frame_time = bpy.app.driver_namespace[next_frame_time_key]
    
    # 检查是否需要发送下一帧（基于运动时间）
    if current_time >= next_frame_time:
        # 运动时间已到，移动到下一个关键帧
        scene.serial_helper.current_keyframe += 1
        
        if scene.serial_helper.current_keyframe <= scene.serial_helper.total_keyframes:
            # 发送下一个关键帧，并获取运动时间
            motion_time = send_keyframe_data(scene.serial_helper.current_keyframe)
            # 设置下一帧的发送时间
            bpy.app.driver_namespace[next_frame_time_key] = current_time + motion_time
            print(f"[自动发送] 帧 {scene.serial_helper.current_keyframe} 已发送，运动时间 {motion_time:.3f}s，下次发送时间: {current_time + motion_time:.3f}")
        
        return 0.1
    
    # 如果是第一个关键帧且尚未发送，直接发送
    if scene.serial_helper.current_keyframe == 1 and next_frame_time == 0:
        motion_time = send_keyframe_data(1)
        bpy.app.driver_namespace[next_frame_time_key] = current_time + motion_time
        print(f"[自动发送] 第一帧已发送，运动时间 {motion_time:.3f}s，下次发送时间: {current_time + motion_time:.3f}")
    
    return 0.1  # 每0.1秒检查一次


def send_keyframe_data(frame_number):
    """发送指定关键帧的数据，返回计算出的运动时间（秒）"""
    scene = bpy.context.scene
    
    # 设置到指定关键帧
    scene.frame_set(frame_number)
    
    # 构建发送数据
    data_to_send = scene.serial_helper.serial_send_data
    
    if not data_to_send or data_to_send.strip() == "":
        print(f"关键帧 {frame_number}: 发送数据为空，跳过")
        return 0.5  # 返回默认时间
    
    # 格式化数据（这会计算并保存运动时间到 driver_namespace）
    try:
        formatted_data = format_replace_var_string(None, data_to_send)
    except Exception as e:
        print(f"格式化数据失败: {e}")
        return 0.5  # 返回默认时间
    
    # 提取发送的地址
    sent_addresses = []
    for item in scene.serial_helper.Send_variable_list:
        if item.data_path and item.data_path != "":
            sent_addresses.append(item.variable_name)
    
    # 记录已发送的地址
    scene.serial_helper.sent_addresses = ",".join(sent_addresses)
    
    # 发送数据
    try:
        SerialConnection = bpy.app.driver_namespace.get("serial_connection")
        if not SerialConnection or not SerialConnection.serial:
            print("串口未连接")
            return 0.5  # 返回默认时间
        
        # 按逗号分割多个命令
        commands = formatted_data.split(',')
        
        # 检查是否有电机启用了同步
        has_sync_enabled = False
        for item in scene.serial_helper.Send_variable_list:
            if item.sync == 1:
                has_sync_enabled = True
                break
        
        # 逐条发送每个命令
        for i, cmd in enumerate(commands):
            cmd = cmd.strip()
            if cmd:
                # 清理命令
                cleaned_cmd = re.sub(r'[^0-9A-Fa-f\s]', '', cmd)
                if not cleaned_cmd:
                    continue
                
                hex_data = bytes.fromhex(cleaned_cmd)
                SerialConnection.serial.write(hex_data)
                
                # 如果启用了换行，在每个命令后添加换行符
                if scene.serial_helper.is_newline:
                    SerialConnection.serial.write(b'\r\n')
                
                # 在命令之间添加小延迟
                time.sleep(0.005)
        
        # 如果有电机启用了同步，发送同步启动命令
        if has_sync_enabled:
            sync_cmd = "00 FF 66 6B"
            hex_data = bytes.fromhex(sync_cmd)
            SerialConnection.serial.write(hex_data)
            
            if scene.serial_helper.is_newline:
                SerialConnection.serial.write(b'\r\n')
        
        print(f"已发送关键帧 {frame_number} 数据")
        
    except Exception as e:
        print(f"发送数据失败: {e}")
        return 0.5  # 返回默认时间
    
    # 从 driver_namespace 获取计算出的运动时间
    motion_time = bpy.app.driver_namespace.get("last_calculated_motion_time", 0.5)
    return motion_time


class testOperator(bpy.types.Operator):
    bl_idname = "test.operator"
    bl_label = "Test Operator"

    def execute(self, context):
        print(context.scene.serial_helper.bytesize)
        return {'FINISHED'}


class stopReceivingOperator(bpy.types.Operator):
    bl_idname = "serial.stop_receiving_operator"
    bl_label = "暂停接收"
    bl_options = {'REGISTER', 'UNDO'}  # 选项，注册到操作列表中，提供撤销功能

    def execute(self, context):
        bpy.context.scene.serial_helper.StopReceiving = not bpy.context.scene.serial_helper.StopReceiving
        return {'FINISHED'}


class switchTheSerialPortOperator(bpy.types.Operator):
    bl_idname = "serial.switch_operator"  # 操作符 ID
    bl_label = "打开串口"  # 操作符显示名称
    # bl_options = {'REGISTER', 'UNDO'}  # 选项，注册到操作列表中，提供撤销功能

    def execute(self, context):
        # 在这里执行按钮操作
        context.scene.serial_helper.serial_is_open = not context.scene.serial_helper.serial_is_open
        if context.scene.serial_helper.serial_is_open:

            if bpy.context.scene.serial_helper.use_input_serial_port:
                port = bpy.context.scene.serial_helper.user_input_serial_port
            else:
                port = bpy.context.scene.serial_helper.serial_ports
            print(context.scene.serial_helper.serial_is_open)
            print(port)
            print(context.scene.serial_helper.baudrate)
            try:
                open_serial_port()
                self.report(
                    {'INFO'}, f"串口打开{port}")
            except Exception as e:
                print(f"An error occurred: {e}")
                context.scene.serial_helper.serial_is_open = False
                self.report(
                    {'ERROR'}, f"串口打开失败,{e}")

        else:
            if "serial_connection" in bpy.app.driver_namespace:
                serial_thread = bpy.app.driver_namespace["serial_thread"]
                serial_thread.should_terminate = True
                serial_SerialConnection = bpy.app.driver_namespace["serial_connection"]
                serial_SerialConnection.serial.close()

                del bpy.app.driver_namespace["serial_connection"]
                del bpy.app.driver_namespace["serial_thread"]
                print("成功关闭串口")
            else:
                print("无可关闭的串口")
            self.report(
                {'INFO'}, "串口关闭.")

        return {'FINISHED'}


def update_serial_ports(self, context):
    ports = [port.device for port in serial.tools.list_ports.comports()]
    return [(port, port, "") for port in ports]


class SerialHelperProperties(bpy.types.PropertyGroup):
    use_input_serial_port: bpy.props.BoolProperty(
        name="是否手动输入端口",
        description="是否手动输入端口",
        default=False
    )
    serial_ports: bpy.props.EnumProperty(
        name="串口端口",
        description="串口端口",
        items=update_serial_ports,
    )
    user_input_serial_port: bpy.props.StringProperty(
        name="手动端口",
        description="手动输入的端口",
        default="COM3"
    )

    serial_is_open: bpy.props.BoolProperty(
        name="serial_is_open_name",
        description="串口是否打开",
        default=False
    )
    def update_baudrate_enum(self, context):
        """当选择常用波特率时更新数值"""
        if self.baudrate_enum != 'CUSTOM':
            self.baudrate = int(self.baudrate_enum)
    
    baudrate_enum: bpy.props.EnumProperty(
        name="波特率选择",
        description="选择常用波特率",
        items=[
            ('9600', "9600", "9600 bps"),
            ('19200', "19200", "19200 bps"),
            ('38400', "38400", "38400 bps"),
            ('57600', "57600", "57600 bps"),
            ('115200', "115200", "115200 bps"),
            ('256000', "256000", "256000 bps"),
            ('512000', "512000", "512000 bps"),
            ('921600', "921600", "921600 bps"),
            ('CUSTOM', "自定义", "自定义波特率"),
        ],
        default='115200',
        update=update_baudrate_enum
    )
    
    baudrate: bpy.props.IntProperty(
        name="波特率",
        description="波特率",
        default=115200
    )
    # FIVEBITS、SIXBITS、SEVENBITS、EIGHTBITS
    bytesize: bpy.props.EnumProperty(
        name="数据位",
        description="数据位",
        items=[
            ('5', "5位", "使用7位数据位"),
            ('6', "6位", "使用8位数据位"),
            ('7', "7位", "使用7位数据位"),
            ('8', "8位", "使用8位数据位"),
        ],
        default='8'
    )
    # STOPBITS_ONE、STOPBITS_ONE_POINT_FIVE、STOPBITS_TWO
    stopbits: bpy.props.EnumProperty(
        name="停止位",
        description="停止位",
        items=[
            ('1', "1", "1位停止位"),
            ('1.5', "1.5", "1.5位停止位"),
            ('2', "2", "2位停止位"),
            # 有些系统可能还支持其他类型的校验位，如：
            # 'MARK', "标记校验", "标记校验位"),
            # 'SPACE', "空格校验", "空格校验位"),
        ],
        default='1'
    )
    # PARITY_NONE、PARITY_EVEN、PARITY_ODD PARITY_MARK、PARITY_SPACE
    parity: bpy.props.EnumProperty(
        name="校验位",
        description="校验位",
        items=[
            ('N', "NONE", "无校验位"),
            ('E', "偶校验", "偶数校验位"),
            ('O', "奇校验", "奇数校验位"),
            ('M', "MARK", "标记校验位"),
            ('S', "SPACE", "空格校验位"),
        ],
        default='N'
    )
    Encoding: bpy.props.EnumProperty(
        name="编码格式",
        description="编码格式",
        items=[
            ('utf-8', "utf-8", "utf-8"),
            ('ascii', "ascii", "ascii"),
            ('gbk', "gbk", "gbk"),
            ('utf-16', "utf-16", "utf-16"),
            ('gb2312', "gb2312", "gb2312"),
            ('hex', "16进制", "以16进制格式显示数据"),
        ],
        default='utf-8'
    )
    StopReceiving: bpy.props.BoolProperty(
        name="StopReceiving",
        description="暂停接收",
        default=False
    )
    serial_data_list: bpy.props.CollectionProperty(type=SerialDataItemProperties)
    serial_data_index: bpy.props.IntProperty()
    serial_data_count: bpy.props.IntProperty(default=1)
    serial_data_max_count: bpy.props.IntProperty(default=5)
    serial_data_matching_list: bpy.props.CollectionProperty(type=SerialDataMatchingProperties)
    serial_data_matching_index: bpy.props.IntProperty()
    serial_data_matching_update_use: bpy.props.FloatProperty(default=0)
    serial_send_data: bpy.props.StringProperty(default="")
    is_newline: bpy.props.BoolProperty(default=True)
    Send_variable_list: bpy.props.CollectionProperty(type=SendVariablePathItem)
    Send_variable_index: bpy.props.IntProperty()
    is_auto_send: bpy.props.BoolProperty(default=False, update=update_sending_state)
    auto_send_interval: bpy.props.FloatProperty(default=1, min=0.01)
    fast_message_list: bpy.props.CollectionProperty(type=SerialFastMessageItem)
    fast_message_index: bpy.props.IntProperty()
    
    # 存储上一次的脉冲值用于变化检测
    last_pulse_value: bpy.props.FloatProperty(default=0.0)
    
    # 新的发送数据功能
    serial_send_data2: bpy.props.StringProperty(default="")
    is_newline2: bpy.props.BoolProperty(default=True)
    is_auto_send2: bpy.props.BoolProperty(default=False, update=update_sending_state2)
    auto_send_interval2: bpy.props.FloatProperty(default=1, min=0.01)
    
    # 读取电机位置功能
    is_reading_positions: bpy.props.BoolProperty(default=False, description="是否正在读取电机位置")
    
    # 电机使能状态
    motor_enabled: bpy.props.BoolProperty(default=False, description="电机使能状态")
    
    # 到位命令检测 - 用于自动发送控制
    motor_arrived: bpy.props.BoolProperty(default=False, description="电机是否到位")
    
    # 自动发送关键帧功能
    is_auto_keyframe_sending: bpy.props.BoolProperty(default=False, description="是否正在自动发送关键帧")
    current_keyframe: bpy.props.IntProperty(default=1, description="当前处理的关键帧")
    total_keyframes: bpy.props.IntProperty(default=0, description="总关键帧数")
    sent_addresses: bpy.props.StringProperty(default="", description="已发送的地址列表")
    last_arrival_time: bpy.props.FloatProperty(default=0, description="上次收到到位指令的时间")
    
    # 自动发送关键帧设置
    keyframe_buffer_ms: bpy.props.IntProperty(
        name="机械误差缓冲时间(ms)",
        description="机械误差缓冲时间，用于应对机械臂的实际运动误差",
        default=500,
        min=0,
        max=5000
    )
    keyframe_start_frame: bpy.props.IntProperty(
        name="起始帧",
        description="自动发送关键帧的起始帧号",
        default=1,
        min=1
    )
    keyframe_end_frame: bpy.props.IntProperty(
        name="结束帧",
        description="自动发送关键帧的结束帧号",
        default=250,
        min=1
    )
    
    # 驱动器更新触发器 - 用于强制刷新驱动器
    driver_update_trigger: bpy.props.IntProperty(default=0, description="驱动器更新触发器")
    
    # 调试模式 - 用于显示详细的接收和解析日志
    debug_mode: bpy.props.BoolProperty(default=False, description="调试模式：显示详细的接收和解析日志")


property_Class = [
    SerialDataItemProperties,
    SerialDataMatchingProperties,
    SendVariablePathItem,
    SerialFastMessageItem,
    SerialHelperProperties,



]

Panel_Class = [
    SerialHelpPanel,
    ReceivingSettingsPanel,
    SerialDataDisplayPanel,
    SERIAL_UL_DataList,
    SerialHelperDataMatchingPanel,
    SERIAL_UL_DataMatchingList,
    SendDataSerialPanel,
    SendDataSerialPanel2,
    SERIAL_UL_SendVariable_list,
    SERIAL_UL_FastMessage_list,
    SerialHelperSendVariablePanel,
    SerialFastMessagePanle,
]

Operator_Class = [
    testOperator,
    switchTheSerialPortOperator,
    stopReceivingOperator,
    ClearSerialDataItemsOperator,
    AddSerialDataMatchingItemOperator,
    RemoveSerialDataMatchingItemOperator,
    CopyDriverSerialDataMatchingItemOperator,
    UpdateSerialDriverDataMatchingOperator,
    SendDataSerialOperator,
    SendDataSerialOperator2,
    AddSerialHelperSendVariableOperator,
    RemoveSerialHelperSendVariableOperator,
    AddSerialFastMessageListOperator,
    RemoveSerialFastMessageListOperator,
    SendFastMessageOperator,
    ToggleReadMotorPositionsOperator,
    ToggleMotorEnableOperator,
    SetZeroPositionOperator,
    AutoKeyframeSendOperator
]

    
def register():

    for cls in property_Class:
        bpy.utils.register_class(cls)

    for cls in Panel_Class:
        bpy.utils.register_class(cls)

    for cls in Operator_Class:
        bpy.utils.register_class(cls)

    # 将 serial_helper 属性添加到 Scene 中
    bpy.types.Scene.serial_helper = bpy.props.PointerProperty(type=SerialHelperProperties)


def unregister():

    for cls in property_Class:
        bpy.utils.unregister_class(cls)

    for cls in Panel_Class:
        bpy.utils.unregister_class(cls)

    for cls in Operator_Class:
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.serial_helper


if __name__ == "__main__":
    register()
