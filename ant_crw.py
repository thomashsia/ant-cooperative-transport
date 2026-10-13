# ant.py
import math
import random


class Ant:
    def __init__(self, ant_id, x, y, is_navigator=False, width=100, height=100, speed=2.0):
        self.id = ant_id
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.speed = speed
        self.role = 'navigator' if is_navigator else 'transporter'

        # 状态机初始化
        if self.role == 'navigator':
            self.state = 'free'      # 'free' (盲搜) 或 'returning' (发现食物后回巢铺路)
        else:
            self.state = 'free'      # 'free', 'transporting', 或 'escaping'
            self.transport_time = 0.0
            self.max_transport_time = random.uniform(6.0, 12.0)
            self.escape_timer = 0.0

        # 用于 Bug2 绕障（navigator 和 transporter 共用）
        self.bug_obstacle = None      # 当前正在绕的障碍物
        self.bug_phase = 'go'         # 'go'（朝目标）或 'wall'（沿墙）
        self.bug_direction = 1        # +1: 逆时针, -1: 顺时针（进入 wall 时随机决定）

        self.heading = random.uniform(0, 2 * math.pi)

    def constrain_position(self, obstacles):
        if not obstacles:
            return
        for obs in obstacles:
            self.x, self.y = obs.clamp_position(self.x, self.y)

    def move_random(self, obstacles=None):
        """纯粹的相关随机游走 (CRW)"""
        turn_angle = random.gauss(0.0, 0.35)
        self.heading += turn_angle

        nx = self.x + self.speed * math.cos(self.heading)
        ny = self.y + self.speed * math.sin(self.heading)

        nx = max(0, min(self.width, nx))
        ny = max(0, min(self.height, ny))

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

    def move_towards(self, target_x, target_y, obstacles=None):
        """定向朝目标移动（用于导航蚁回巢）"""
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)
        if dist > 0:
            target_angle = math.atan2(dy, dx)
            self.heading = target_angle + random.uniform(-0.1, 0.1)

        nx = self.x + self.speed * math.cos(self.heading)
        ny = self.y + self.speed * math.sin(self.heading)

        nx = max(0, min(self.width, nx))
        ny = max(0, min(self.height, ny))

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
        只计算期望朝向，不移动、不处理碰撞。
        优先级互斥：
          1. 有信息素信号 → 循迹（沿信息素梯度）
          2. 无信息素，但闻到食物 → 追踪食物
          3. 都没有 → CRW
        """
        # 1. 计算信息素吸引
        p_attract_x, p_attract_y = 0.0, 0.0
        p_weight_sum = 0.0

        for p in pheromone_particles:
            dist = math.hypot(p['x'] - self.x, p['y'] - self.y)
            if 0 < dist <= perception_radius:
                w = p['intensity'] / (dist ** 1.5 + 0.1)
                p_attract_x += (p['x'] - self.x) / dist * w
                p_attract_y += (p['y'] - self.y) / dist * w
                p_weight_sum += w

        # 2. 有信息素 → 循迹
        if p_weight_sum > signal_threshold:
            target_angle = math.atan2(p_attract_y, p_attract_x)
            return target_angle + random.uniform(-0.2, 0.2)

        # 3. 无信息素，但闻到食物 → 追踪食物
        dist_to_food = math.hypot(food_pos[0] - self.x, food_pos[1] - self.y)
        if 0 < dist_to_food <= food_odor_radius:
            target_angle = math.atan2(food_pos[1] - self.y, food_pos[0] - self.x)
            return target_angle + random.uniform(-0.2, 0.2)

        # 4. 都没有 → CRW
        return self.heading + random.gauss(0.0, 0.35)

    def move_with_pheromone(self, pheromone_particles, food_pos, obstacles=None,
                            food_odor_radius=20.0, perception_radius=60.0,
                            ignore_signals=False):
        """保留给不套 Bug2 的场景使用；内部调用 compute_pheromone_heading"""
        if ignore_signals:
            self.move_random(obstacles)
            return

        self.heading = self.compute_pheromone_heading(
            pheromone_particles, food_pos,
            food_odor_radius=food_odor_radius,
            perception_radius=perception_radius
        )

        nx = self.x + self.speed * math.cos(self.heading)
        ny = self.y + self.speed * math.sin(self.heading)

        nx = max(0, min(self.width, nx))
        ny = max(0, min(self.height, ny))

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