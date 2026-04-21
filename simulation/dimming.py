import tkinter as tk
from tkinter import ttk
import math

# ================= 算法参数（与C代码完全一致） =================
ADC_MAX = 4095.0
PWM_MAX = 600.0

DEAD_ZONE = 0.0
FILTER_FAST = 0.6
FILTER_SLOW = 0.2

GAMMA = 1.8
CCT_CURVE = 1.0

SMOOTH_STEP = 0.06
MIN_BRIGHT = 0.00

# ================= 灯光控制器类（封装状态与算法） =================
class LightController:
    def __init__(self):
        # 滤波状态变量
        self.cct_fast = 0.0
        self.cct_slow = 0.0
        self.dim_fast = 0.0
        self.dim_slow = 0.0

        # 输出状态变量（用于平滑过渡）
        self.cw_out = 0.0
        self.ww_out = 0.0

    def low_pass(self, input_val, state, alpha):
        """一阶低通滤波器"""
        new_state = state * (1.0 - alpha) + input_val * alpha
        return new_state, new_state

    def dual_filter(self, input_val, fast_state, slow_state):
        """双滤波器：快速+慢速滤波后加权平均"""
        f, new_fast = self.low_pass(input_val, fast_state, FILTER_FAST)
        s, new_slow = self.low_pass(input_val, slow_state, FILTER_SLOW)
        filtered = 0.7 * f + 0.3 * s
        return filtered, new_fast, new_slow

    def apply_deadzone(self, adc):
        """死区处理（当前死区为0，保留逻辑）"""
        if adc < DEAD_ZONE:
            return 0.0
        adc = adc - DEAD_ZONE
        range_val = ADC_MAX - DEAD_ZONE
        if range_val <= 0.0:
            return 0.0
        return adc / range_val

    def smooth_transition(self, current, target):
        """平滑过渡（一阶滞后）"""
        return current + (target - current) * SMOOTH_STEP

    def update(self, cct_adc_raw, dim_adc_raw):
        """
        主控制逻辑：输入原始ADC值（0～4095），计算并更新CW/WW输出
        返回：(cw_pwm, ww_pwm) 两个PWM值（整数）
        """
        # 1. 转换为浮点数
        cct_adc = float(cct_adc_raw)
        dim_adc = float(dim_adc_raw)

        # 2. 双滤波
        cct_adc, self.cct_fast, self.cct_slow = self.dual_filter(
            cct_adc, self.cct_fast, self.cct_slow
        )
        dim_adc, self.dim_fast, self.dim_slow = self.dual_filter(
            dim_adc, self.dim_fast, self.dim_slow
        )

        # 3. 归一化
        ratio = cct_adc / ADC_MAX          # 色温比例 (0~1)
        brightness = self.apply_deadzone(dim_adc)  # 亮度比例 (0~1)

        # 4. 亮度保护
        if brightness < MIN_BRIGHT:
            self.cw_out = 0.0
            self.ww_out = 0.0
            return 0, 0

        # 5. Gamma校正（亮度）
        brightness = math.pow(brightness, GAMMA)

        # 6. 色温曲线修正
        ratio = math.pow(ratio, CCT_CURVE)

        # 7. 功率归一化（保持总光通量恒定）
        cw_ratio = ratio
        ww_ratio = 1.0 - ratio
        norm = math.sqrt(cw_ratio * cw_ratio + ww_ratio * ww_ratio)
        if norm < 1e-6:
            norm = 1.0
        cw_ratio /= norm
        ww_ratio /= norm

        # 8. 目标值
        cw_target = brightness * cw_ratio
        ww_target = brightness * ww_ratio

        # 9. 平滑过渡（模拟控制周期的连续变化）
        self.cw_out = self.smooth_transition(self.cw_out, cw_target)
        self.ww_out = self.smooth_transition(self.ww_out, ww_target)

        # 10. 转换为PWM整数
        cw_pwm = int(self.cw_out * PWM_MAX)
        ww_pwm = int(self.ww_out * PWM_MAX)

        # 限幅保护（理论上不会超出，但以防浮点误差）
        cw_pwm = max(0, min(int(PWM_MAX), cw_pwm))
        ww_pwm = max(0, min(int(PWM_MAX), ww_pwm))

        return cw_pwm, ww_pwm

# ================= GUI应用程序 =================
class LightControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("LED双通道混光可视化 - 色温/亮度控制")
        self.root.geometry("500x400")
        self.root.resizable(False, False)

        # 创建控制器实例
        self.controller = LightController()

        # 当前ADC输入值（滑块值）
        self.cct_adc = tk.IntVar(value=2048)   # 初始中间值
        self.dim_adc = tk.IntVar(value=2048)

        # 输出显示变量
        self.cw_pwm = tk.IntVar(value=0)
        self.ww_pwm = tk.IntVar(value=0)

        # 创建界面控件
        self.create_widgets()

        # 启动定时器，每50ms更新一次（模拟控制周期）
        self.update_loop()

    def create_widgets(self):
        # 样式
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TLabel', font=('微软雅黑', 10))
        style.configure('TScale', length=300)

        # ----- 色温控制区 -----
        frame_cct = ttk.LabelFrame(self.root, text="色温 (CCT)  ADC 输入", padding=10)
        frame_cct.pack(fill='x', padx=20, pady=10)

        self.cct_slider = ttk.Scale(
            frame_cct, from_=0, to=ADC_MAX, variable=self.cct_adc,
            orient='horizontal', length=350
        )
        self.cct_slider.pack(pady=5)

        self.cct_label = ttk.Label(frame_cct, text=f"当前值: {self.cct_adc.get():.0f}")
        self.cct_label.pack()

        # ----- 亮度控制区 -----
        frame_dim = ttk.LabelFrame(self.root, text="亮度 (DIM)  ADC 输入", padding=10)
        frame_dim.pack(fill='x', padx=20, pady=10)

        self.dim_slider = ttk.Scale(
            frame_dim, from_=0, to=ADC_MAX, variable=self.dim_adc,
            orient='horizontal', length=350
        )
        self.dim_slider.pack(pady=5)

        self.dim_label = ttk.Label(frame_dim, text=f"当前值: {self.dim_adc.get():.0f}")
        self.dim_label.pack()

        # ----- 输出显示区 -----
        frame_out = ttk.LabelFrame(self.root, text="PWM 输出 (0 ~ 666)", padding=10)
        frame_out.pack(fill='both', expand=True, padx=20, pady=10)

        # 冷白通道
        cw_frame = ttk.Frame(frame_out)
        cw_frame.pack(fill='x', pady=5)
        ttk.Label(cw_frame, text="冷白 (CW):", width=10).pack(side='left')
        self.cw_bar = ttk.Progressbar(cw_frame, length=250, mode='determinate', maximum=PWM_MAX)
        self.cw_bar.pack(side='left', padx=5)
        self.cw_value = ttk.Label(cw_frame, text="0", width=6)
        self.cw_value.pack(side='left')

        # 暖白通道
        ww_frame = ttk.Frame(frame_out)
        ww_frame.pack(fill='x', pady=5)
        ttk.Label(ww_frame, text="暖白 (WW):", width=10).pack(side='left')
        self.ww_bar = ttk.Progressbar(ww_frame, length=250, mode='determinate', maximum=PWM_MAX)
        self.ww_bar.pack(side='left', padx=5)
        self.ww_value = ttk.Label(ww_frame, text="0", width=6)
        self.ww_value.pack(side='left')

        # 说明标签
        info = ttk.Label(
            self.root,
            text="算法包含：双滤波、死区、Gamma校正、色温曲线修正、功率归一化、平滑过渡",
            font=('微软雅黑', 8), foreground='gray'
        )
        info.pack(side='bottom', pady=10)

        # 绑定滑块值变化事件（仅更新显示数值，不直接触发控制循环）
        self.cct_adc.trace_add('write', lambda *args: self.update_slider_label())
        self.dim_adc.trace_add('write', lambda *args: self.update_slider_label())

    def update_slider_label(self):
        """更新滑块旁边的数值显示"""
        self.cct_label.config(text=f"当前值: {self.cct_adc.get():.0f}")
        self.dim_label.config(text=f"当前值: {self.dim_adc.get():.0f}")

    def update_loop(self):
        """定时器回调：执行一次控制算法并刷新UI"""
        # 获取当前ADC输入（滑块值）
        cct_raw = self.cct_adc.get()
        dim_raw = self.dim_adc.get()

        # 调用控制器计算输出PWM
        cw, ww = self.controller.update(cct_raw, dim_raw)

        # 更新界面
        self.cw_pwm.set(cw)
        self.ww_pwm.set(ww)

        self.cw_bar['value'] = cw
        self.ww_bar['value'] = ww
        self.cw_value.config(text=str(cw))
        self.ww_value.config(text=str(ww))

        # 继续循环（约50ms一次，模拟连续控制）
        self.root.after(50, self.update_loop)

# ================= 主程序入口 =================
if __name__ == "__main__":
    root = tk.Tk()
    app = LightControlApp(root)
    root.mainloop()
