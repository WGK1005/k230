K230 核心UI类
简化架构 - 仅支持真实触摸硬件
"""

from .touch_manager import TouchManager
from .components import Button, Slider, StaticText, Panel

# 🔧 全局调试开关
DEBUG_ENABLED = False

def set_debug(enabled):
   """设置调试信息开关"""
   global DEBUG_ENABLED
   DEBUG_ENABLED = enabled
   print(f"[UI调试] 调试信息已{'开启' if enabled else '关闭'}")

def debug_print(message):
   """调试信息输出函数"""
   if DEBUG_ENABLED:
       print(message)

class TouchUI:
   """核心UI类 - 仅支持真实触摸"""

   def __init__(self, width, height):
       self.width = width
       self.height = height
       self.components = []
       self.touch_manager = TouchManager()

       debug_print(f"[TouchUI] 初始化完成 - 尺寸: {width}x{height}")
       debug_print(f"[TouchUI] 触摸设备: {'可用' if self.touch_manager.is_available() else '不可用'}")

   def add_button(self, x, y, width, height, text, callback=None):
       """添加按钮"""
       button = Button(x, y, width, height, text, callback)
       self.components.append(button)
       return button

   def add_slider(self, x, y, width, height, min_val, max_val, value,
                  orientation="horizontal", callback=None):
       """添加滑块"""
       slider = Slider(x, y, width, height, min_val, max_val, value, orientation, callback)
       self.components.append(slider)
       return slider

   def add_static_text(self, x, y, font_size, text, color=(255, 255, 255)):
       """添加静态文本"""
       static_text = StaticText(x, y, font_size, text, color)
       self.components.append(static_text)
       return static_text

   def add_panel(self, x, y, width, height, background_color=(50, 50, 50)):
       """添加面板"""
       panel = Panel(x, y, width, height, background_color)
       self.components.append(panel)
       return panel

   def update(self, img):
       """更新UI - 处理触摸事件并绘制所有组件"""
       # 处理触摸事件
       touch_points = self.touch_manager.read_touch()

       # 处理触摸事件
       if touch_points:
           for point in touch_points:
               self._handle_touch_event(point)
       else:
           # 没有触摸时，重置所有组件的拖拽状态
           # 🎯 修正：由于无法获得真正的释放事件(1)，我们在这里直接重置状态
           for component in self.components:
               if hasattr(component, 'pressed') and component.pressed:
                   component.pressed = False
               if hasattr(component, 'dragging') and component.dragging:
                   # 直接重置拖拽状态，不发送假的事件
                   debug_print(f"[UI核心] 检测到触摸结束，重置 {getattr(component, 'name', '组件')} 的拖拽状态")
                   component.dragging = False

       # 🔧 新增：更新所有按钮的状态（处理超时重置）
       for component in self.components:
           if hasattr(component, 'update_state'):
               component.update_state()

       # 绘制所有组件
       for component in self.components:
           component.draw(img)

   def _handle_touch_event(self, touch_point):
       """处理触摸事件"""
       debug_print(f"[UI调试] 处理触摸事件: ({touch_point.x},{touch_point.y}) 事件:{touch_point.event}")

       # 检查哪个组件被触摸
       touched_component = None
       for component in reversed(self.components):  # 后添加的组件优先
           if component.visible and component.contains_point(touch_point.x, touch_point.y):
               component_name = component.name if hasattr(component, 'name') else component.__class__.__name__
               debug_print(f"[UI调试] 命中组件: {component_name} 区域:({component.x},{component.y},{component.x+component.width},{component.y+component.height})")
               component.handle_touch(touch_point)
               touched_component = component
               break  # 只处理最上层的组件

       if not touched_component:
           debug_print(f"[UI调试] 未命中任何组件")
           # 如果没有命中组件，重置所有组件的拖拽状态
           for component in self.components:
               if hasattr(component, 'dragging') and component.dragging:
                   component.dragging = False

   def clear(self):
       """清空所有组件"""
       self.components.clear()

   def get_component_count(self):
       """获取组件数量"""
       return len(self.components)

   def is_touch_available(self):
       """检查触摸是否可用"""
       return self.touch_manager.is_available()
