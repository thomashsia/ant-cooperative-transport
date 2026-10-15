"""用于协作搬运仿真的蚂蚁模型，包含 CRW、信息素感知和 Bug2 状态数据。"""

import math
import random


class Ant:
    """保存蚂蚁角色、行为状态、运动状态和避障状态。"""

    def __init__(self, ant_id, x, y, is_navigator=False, width=100, height=100, speed=2.0):
        """创建导航蚁或搬运工蚁，并初始化其状态机和运动参数。"""
        self.id = ant_id
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.speed = speed
        self.role = 'navigator' if is_navigator else 'transporter'

        # 按角色创建状态字段：导航蚁负责搜索/返巢，搬运工负责搬运/休息。
        if self.role == 'navigator':
            self.state = 'free'      # 'free' (盲搜) 或 'returning' (发现食物后回巢铺路)
        else:
            self.state = 'free'      # 'free', 'transporting', 或 'escaping'
            self.transport_time = 0.0
            self.max_transport_time = random.uniform(6.0, 12.0)
            self.escape_timer = 0.0

        # Bug2 避障状态：记录正在绕行的障碍、当前阶段和沿边界的方向。
        self.bug_obstacle = None      # 当前正在绕的障碍物
        self.bug_phase = 'go'         # 'go'（朝目标）或 'wall'（沿墙）
        self.bug_direction = 1        # +1: 逆时针, -1: 顺时针（进入 wall 时随机决定）

        # 朝向以弧度表示，初始方向随机。
        self.heading = random.uniform(0, 2 * math.pi)

    def constrain_position(self, obstacles):
        """依次将蚂蚁从所有障碍物内部投影到最近的外侧边缘。"""
        if not obstacles:
            return
        for obs in obstacles:
            self.x, self.y = obs.clamp_position(self.x, self.y)

    def move_random(self, obstacles=None):
        """执行相关随机游走（CRW）：在上一帧朝向附近随机改变方向。"""
        turn_angle = random.gauss(0.0, 0.35)
        self.heading += turn_angle

        # 按当前朝向和速度计算下一位置，并限制在环境边界内。
        nx = self.x + self.speed * math.cos(self.heading)
        ny = self.y + self.speed * math.sin(self.heading)

        nx = max(0, min(self.width, nx))
        ny = max(0, min(self.height, ny))

        # 若预测位置落入障碍物则视为碰撞，否则正常提交新位置。
        collided = False
        if obstacles:
            for obs in obstacles:
                if obs.check_collision_point(nx, ny):
                    collided = True
                    break

        if not collided:
            self.x, self.y = nx, ny
        else:
            # 碰撞后大致反向并加入少量扰动，再尝试移动一步。
            self.heading += math.pi + random.uniform(-0.8, 0.8)
            self.x = max(0, min(self.width, self.x + self.speed * math.cos(self.heading)))
            self.y = max(0, min(self.height, self.y + self.speed * math.sin(self.heading)))

    def move_towards(self, target_x, target_y, obstacles=None):
        """朝给定目标移动，并在目标方向上加入少量随机偏角。"""
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)
        if dist > 0:
            # 目标尚有距离时，将目标方向设为基础朝向。
            target_angle = math.atan2(dy, dx)
            self.heading = target_angle + random.uniform(-0.1, 0.1)

        # 根据朝向预测下一步位置并约束场地边界。
        nx = self.x + self.speed * math.cos(self.heading)
        ny = self.y + self.speed * math.sin(self.heading)

        nx = max(0, min(self.width, nx))
        ny = max(0, min(self.height, ny))

        # 碰撞时反向尝试；无碰撞时提交预测位置。
        collided = False
        if obstacles:
            for obs in obstacles:
                if obs.check_collision_point(nx, ny):
                    collided = True
                    break

        if not collided:
            self.x, self.y = nx, ny
        else:
            self.heading += math.pi + random.uniform(-0.8, 0.8)
            self.x = max(0, min(self.width, self.x + self.speed * math.cos(self.heading)))
            self.y = max(0, min(self.height, self.y + self.speed * math.sin(self.heading)))

    def compute_pheromone_heading(self, pheromone_particles, food_pos,
                                  food_odor_radius=20.0, perception_radius=60.0,
                                  signal_threshold=0.02):
        """
        只计算期望朝向，不移动，也不处理碰撞。

        行为按优先级互斥：先跟随有效信息素，再追踪气味范围内的食物，
        两者都不可用时则沿当前朝向执行相关随机游走。
        """
        # 汇总感知半径内信息素粒子的方向和距离/强度加权值。
        p_attract_x, p_attract_y = 0.0, 0.0
        p_weight_sum = 0.0

        for p in pheromone_particles:
            dist = math.hypot(p['x'] - self.x, p['y'] - self.y)
            if 0 < dist <= perception_radius:
                w = p['intensity'] / (dist ** 1.5 + 0.1)
                p_attract_x += (p['x'] - self.x) / dist * w
                p_attract_y += (p['y'] - self.y) / dist * w
                p_weight_sum += w

        # 当累计权重大于阈值时，沿信息素合成方向前进。
        if p_weight_sum > signal_threshold:
            target_angle = math.atan2(p_attract_y, p_attract_x)
            return target_angle + random.uniform(-0.2, 0.2)

        # 没有有效信息素时，只有食物在气味半径内才直接朝食物前进。
        dist_to_food = math.hypot(food_pos[0] - self.x, food_pos[1] - self.y)
        if 0 < dist_to_food <= food_odor_radius:
            target_angle = math.atan2(food_pos[1] - self.y, food_pos[0] - self.x)
            return target_angle + random.uniform(-0.2, 0.2)

        # 两类信号都不可用时，在当前朝向上叠加高斯转向噪声。
        return self.heading + random.gauss(0.0, 0.35)

    def move_with_pheromone(self, pheromone_particles, food_pos, obstacles=None,
                            food_odor_radius=20.0, perception_radius=60.0,
                            ignore_signals=False):
        """计算信息素/食物引导朝向并移动一步；可选忽略信号直接随机游走。"""
        if ignore_signals:
            self.move_random(obstacles)
            return

        # 朝向计算与实际位移分开，方便其他仿真复用朝向决策。
        self.heading = self.compute_pheromone_heading(
            pheromone_particles, food_pos,
            food_odor_radius=food_odor_radius,
            perception_radius=perception_radius
        )

        # 根据朝向计算下一位置，并防止越出场地。
        nx = self.x + self.speed * math.cos(self.heading)
        ny = self.y + self.speed * math.sin(self.heading)

        nx = max(0, min(self.width, nx))
        ny = max(0, min(self.height, ny))

        # 检查障碍物碰撞；无碰撞则前进，碰撞则反向偏转后尝试脱离。
        collided = False
        if obstacles:
            for obs in obstacles:
                if obs.check_collision_point(nx, ny):
                    collided = True
                    break

        if not collided:
            self.x, self.y = nx, ny
        else:
            self.heading += math.pi + random.uniform(-0.8, 0.8)
            self.x = max(0, min(self.width, self.x + self.speed * math.cos(self.heading)))
            self.y = max(0, min(self.height, self.y + self.speed * math.sin(self.heading)))