"""早期独立示例使用的基础蚂蚁模型。

本模块由 ``swarm.py`` 导入；主协同搬运仿真改用 ``ant_crw.py`` 中的 ``Ant``。
"""

import math
import random


class Ant:
    """表示基础蚁群示例中的单只蚂蚁。

    由 ``swarm.AntSwarmSimulation`` 创建。此类与主仿真中的同名类用途不同，
    维护简单位置/速度状态，并以移动方法供独立示例或外部代码调用。
    """

    def __init__(self, ant_id, x, y, width=100, height=100, speed=2.0):
        """初始化编号、位置、场地尺寸和每步移动距离；由群体示例调用。"""
        self.id = ant_id
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.speed = speed
        self.state = 'free'          # 'free' 或 'transporting'
        self.transport_time = 0.0

    def constrain_position(self, obstacles):
        """用各障碍物的位置约束接口修正自身坐标。

        当前 ``swarm.py`` 不调用该辅助方法；可供带障碍物的独立模型复用。
        """
        if not obstacles:
            return
        for obs in obstacles:
            self.x, self.y = obs.clamp_position(self.x, self.y)

    def move_random(self, obstacles=None):
        """随机移动一步，撞到障碍时尝试沿侧向滑移。

        由本类的 ``move_with_pheromone(ignore_signals=True)`` 调用，也可独立调用。
        """
        angle = random.uniform(0, 2 * math.pi)
        nx = self.x + self.speed * math.cos(angle)
        ny = self.y + self.speed * math.sin(angle)
        
        # 将预测位置限制在场地范围内。
        nx = max(0, min(self.width, nx))
        ny = max(0, min(self.height, ny))
        
        # 检查预测位置是否落入任意障碍物。
        collided = False
        if obstacles:
            for obs in obstacles:
                if obs.check_collision_point(nx, ny):
                    collided = True
                    break
        
        if not collided:
            self.x, self.y = nx, ny
        else:
            # 碰撞时改沿随机方向的侧向移动半步。
            fallback_angle = angle + math.pi / 2
            self.x = max(0, min(self.width, self.x + 0.5 * self.speed * math.cos(fallback_angle)))
            self.y = max(0, min(self.height, self.y + 0.5 * self.speed * math.sin(fallback_angle)))

    def move_with_pheromone(self, pheromone_particles, food_pos, obstacles=None, food_odor_radius=50.0, perception_radius=30.0, ignore_signals=False):
        """综合食物气味和无遮挡的信息素决定朝向，再移动一步。

        当前仓库没有主流程调用方；这是较早版本的独立移动接口。``ignore_signals``
        为真时委托给 ``move_random``。否则将各信号单位方向按距离/强度加权求和，
        通过 ``atan2`` 取得移动角，再执行一次点碰撞检查。
        """
        if ignore_signals:
            self.move_random(obstacles)
            return

        # 将食物和周围信息素对蚂蚁的吸引合成为一个加权方向向量。
        attract_x, attract_y = 0.0, 0.0
        total_weight = 0.0

        dist_to_food = math.hypot(food_pos[0] - self.x, food_pos[1] - self.y)
        if dist_to_food <= food_odor_radius:
            # 食物气味只在指定半径内生效，距离越近吸引权重越大。
            weight = 2.5 / (dist_to_food + 0.1)
            attract_x += (food_pos[0] - self.x) / dist_to_food * weight
            attract_y += (food_pos[1] - self.y) / dist_to_food * weight
            total_weight += weight

        # 感知半径内的信息素按强度和距离衰减后参与方向计算。
        for p in pheromone_particles:
            dist = math.hypot(p['x'] - self.x, p['y'] - self.y)
            if 0 < dist <= perception_radius:
                if obstacles and any(
                    obstacle.segment_intersects(
                        self.x, self.y, p['x'], p['y']
                    )
                    for obstacle in obstacles
                ):
                    continue
                weight = p['intensity'] / (dist ** 1.5 + 0.1)
                attract_x += (p['x'] - self.x) / dist * weight
                attract_y += (p['y'] - self.y) / dist * weight
                total_weight += weight

        if total_weight > 0:
            # 有吸引信号时沿合成方向移动，并加入小幅角度扰动。
            target_angle = math.atan2(attract_y, attract_x)
            angle = target_angle + random.uniform(-0.3, 0.3)
        else:
            # 未感知到有效信号时使用均匀随机方向。
            angle = random.uniform(0, 2 * math.pi)

        # 根据选定朝向预测位置，并约束在环境范围内。
        nx = self.x + self.speed * math.cos(angle)
        ny = self.y + self.speed * math.sin(angle)
        
        nx = max(0, min(self.width, nx))
        ny = max(0, min(self.height, ny))
        
        # 检查预测位置是否与障碍物相交。
        collided = False
        if obstacles:
            for obs in obstacles:
                if obs.check_collision_point(nx, ny):
                    collided = True
                    break

        if not collided:
            self.x, self.y = nx, ny
        else:
            # 碰撞后沿当前方向的侧向尝试移动半步。
            fallback_angle = angle + math.pi / 2
            self.x = max(0, min(self.width, self.x + 0.5 * self.speed * math.cos(fallback_angle)))
            self.y = max(0, min(self.height, self.y + 0.5 * self.speed * math.sin(fallback_angle)))