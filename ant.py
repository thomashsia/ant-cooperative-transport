# ant.py
import math
import random

class Ant:
    def __init__(self, ant_id, x, y, width=100, height=100, speed=2.0):
        self.id = ant_id
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.speed = speed
        self.state = 'free'          # 'free' 或 'transporting'
        self.transport_time = 0.0

    def constrain_position(self, obstacles):
        """【核心新增】：强制将穿墙的蚂蚁纠正到障碍物外部"""
        if not obstacles:
            return
        for obs in obstacles:
            self.x, self.y = obs.clamp_position(self.x, self.y)

    def move_random(self, obstacles=None):
        angle = random.uniform(0, 2 * math.pi)
        nx = self.x + self.speed * math.cos(angle)
        ny = self.y + self.speed * math.sin(angle)
        
        # 边界限制
        nx = max(0, min(self.width, nx))
        ny = max(0, min(self.height, ny))
        
        # 障碍物碰撞检测
        collided = False
        if obstacles:
            for obs in obstacles:
                if obs.check_collision_point(nx, ny):
                    collided = True
                    break
        
        if not collided:
            self.x, self.y = nx, ny
        else:
            # 撞墙时切线滑移
            fallback_angle = angle + math.pi / 2
            self.x = max(0, min(self.width, self.x + 0.5 * self.speed * math.cos(fallback_angle)))
            self.y = max(0, min(self.height, self.y + 0.5 * self.speed * math.sin(fallback_angle)))

    def move_with_pheromone(self, pheromone_particles, food_pos, obstacles=None, food_odor_radius=50.0, perception_radius=30.0, ignore_signals=False):
        if ignore_signals:
            self.move_random(obstacles)
            return

        attract_x, attract_y = 0.0, 0.0
        total_weight = 0.0

        dist_to_food = math.hypot(food_pos[0] - self.x, food_pos[1] - self.y)
        if dist_to_food <= food_odor_radius:
            weight = 2.5 / (dist_to_food + 0.1)
            attract_x += (food_pos[0] - self.x) / dist_to_food * weight
            attract_y += (food_pos[1] - self.y) / dist_to_food * weight
            total_weight += weight

        for p in pheromone_particles:
            dist = math.hypot(p['x'] - self.x, p['y'] - self.y)
            if 0 < dist <= perception_radius:
                weight = p['intensity'] / (dist ** 1.5 + 0.1)
                attract_x += (p['x'] - self.x) / dist * weight
                attract_y += (p['y'] - self.y) / dist * weight
                total_weight += weight

        if total_weight > 0:
            target_angle = math.atan2(attract_y, attract_x)
            angle = target_angle + random.uniform(-0.3, 0.3)
        else:
            angle = random.uniform(0, 2 * math.pi)

        nx = self.x + self.speed * math.cos(angle)
        ny = self.y + self.speed * math.sin(angle)
        
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
            fallback_angle = angle + math.pi / 2
            self.x = max(0, min(self.width, self.x + 0.5 * self.speed * math.cos(fallback_angle)))
            self.y = max(0, min(self.height, self.y + 0.5 * self.speed * math.sin(fallback_angle)))