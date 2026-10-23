"""可平移、旋转并被多只蚂蚁协同牵引的矩形食物刚体。"""

import math


class Food:
    """维护食物刚体运动状态及周边搬运槽位。

    由 ``ObstacleTransportSimulation`` 创建；仿真负责安排槽位并汇总蚂蚁施力，
    本类负责几何变换、平动/转动积分和向可视化提供角点。
    """

    def __init__(self, x, y, width=16.0, height=12.0, mass=5.0):
        """创建食物刚体并初始化动力学量；由主仿真初始化调用。

        矩形绕质心的转动惯量为 ``m(w²+h²)/12``；槽位局部坐标固定在食物本体上。
        """
        self.x = x
        self.y = y
        self.theta = 0.0          # 旋转角度 (弧度)
        self.width = width
        self.height = height
        self.mass = mass
        
        self.inertia = (1.0 / 12.0) * mass * (width**2 + height**2)
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        
        self.linear_damping = 1.5 #0.4
        self.angular_damping = 2.0 #0.6

        # 沿矩形四边布置可供蚂蚁占据的局部坐标槽位。
        self.slots = self._generate_perimeter_slots(width, height, spacing=3.0)

    def _generate_perimeter_slots(self, w, h, spacing=3.5):
        """按间距生成矩形四边的局部槽位。

        仅由 ``__init__`` 调用；边上坐标以矩形中心为原点，稍后由
        ``claim_slot`` 和仿真施力计算旋转到世界坐标。
        """
        slots = []
        hw, hh = w / 2.0, h / 2.0
        
        # 沿上、下两条边布点，槽位以矩形中心为原点。
        x = -hw
        while x <= hw:
            slots.append({'local_pos': (x, hh), 'occupied_by': None})
            slots.append({'local_pos': (x, -hh), 'occupied_by': None})
            x += spacing
            
        # 沿左、右两条边补点，避开已添加的角点位置。
        y = -hh + spacing
        while y < hh:
            slots.append({'local_pos': (-hw, y), 'occupied_by': None})
            slots.append({'local_pos': (hw, y), 'occupied_by': None})
            y += spacing
            
        return slots

    def reset_slots(self):
        """清空槽位占用标记。

        每帧由 ``ObstacleTransportSimulation.update`` 调用，再按蚂蚁顺序重分配。
        """
        for slot in self.slots:
            slot['occupied_by'] = None

    def claim_slot(self, ant_x, ant_y, ant_id):
        """将最近的可用槽位分配给指定蚂蚁，过远时返回 ``None``。

        由主仿真在搬运蚂蚁续占和自由蚂蚁加入队伍时调用。局部坐标先按食物角度
        旋转，再平移到世界坐标；距离用欧氏范数计算，最大认领距离为 10。
        """
        best_slot = None
        min_dist = float('inf')
        c, s = math.cos(self.theta), math.sin(self.theta)
        
        # 将每个空槽位从食物局部坐标旋转、平移到世界坐标后测距。
        for slot in self.slots:
            if slot['occupied_by'] is None:
                lx, ly = slot['local_pos']
                wx = self.x + (lx * c - ly * s)
                wy = self.y + (lx * s + ly * c)
                dist = math.hypot(wx - ant_x, wy - ant_y)
                if dist < min_dist:
                    min_dist = dist
                    best_slot = slot

        # 只允许在 10 个坐标单位内认领，防止远处蚂蚁占位。
        if best_slot and min_dist <= 10.0:
            best_slot['occupied_by'] = ant_id
            return best_slot
        return None

    def update_dynamics(self, forces, torques, dt=0.05, width=100, height=100):
        """用显式 Euler 积分更新食物刚体状态。

        由 ``ObstacleTransportSimulation.update`` 每帧调用。先合计各蚂蚁的力与转矩，
        再用 ``(F - c v)/m`` 和 ``(τ - cω)/I`` 更新速度/角速度，最后积分位置和角度。
        """
        # 将所有搬运蚂蚁贡献的力和转矩合并。
        total_fx = sum(f[0] for f in forces)
        total_fy = sum(f[1] for f in forces)
        total_torque = sum(torques)
        
        # 根据线性阻尼计算加速度，再依次更新速度和位置。
        ax = (total_fx - self.linear_damping * self.vx) / self.mass
        ay = (total_fy - self.linear_damping * self.vy) / self.mass
        
        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # 根据转矩、角阻尼和转动惯量更新角速度及旋转角。
        alpha = (total_torque - self.angular_damping * self.omega) / self.inertia
        self.omega += alpha * dt
        self.theta += self.omega * dt
        
        # 根据食物尺寸限制中心位置，避免刚体整体越出给定场地。
        margin = max(self.width, self.height)
        self.x = max(margin, min(width - margin, self.x))
        self.y = max(margin, min(height - margin, self.y))

    def get_corners(self):
        """将局部矩形角点旋转并平移，返回世界坐标。

        由 ``TransportRenderer.draw`` 和障碍物的 SAT 接触/碰撞计算调用。
        """
        hw, hh = self.width / 2.0, self.height / 2.0
        local_corners = [
            (-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)
        ]
        corners = []
        c, s = math.cos(self.theta), math.sin(self.theta)
        # 逐角点应用二维旋转，再平移到食物中心所在位置。
        for lx, ly in local_corners:
            rx = lx * c - ly * s
            ry = lx * s + ly * c
            corners.append((self.x + rx, self.y + ry))
        return corners