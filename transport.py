# transport.py
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon, Rectangle
from ant_crw import Ant
from food import Food
from pheromone import PheromoneManager
from obstacle import Obstacle

# check change tracking
# second change track

class ObstacleTransportSimulation:
    def __init__(self, num_ants=30, width=150, height=150):
        self.width = width
        self.height = height
        self.nest = (70.0, 130.0)

        self.food = Food(x=90.0, y=30.0, width=6, height=6, mass=5.0)

        self.obstacles = [
            Obstacle(x=40.0, y=60.0, width=40.0, height=6.0),
            Obstacle(x=120.0, y=60.0, width=40.0, height=6.0)
        ]

        num_navigators = 5
        self.ants = []
        for i in range(num_ants):
            is_nav = i < num_navigators
            self.ants.append(
                Ant(
                    ant_id=i,
                    x=random.uniform(width * 0.1, width * 0.9),
                    y=random.uniform(height * 0.1, height * 0.9),
                    is_navigator=is_nav,
                    width=width,
                    height=height,
                    speed=2.0
                )
            )

        self.pheromone_manager = PheromoneManager(evaporation_rate=0.003)

    def _bug2_step(self, ant, target_x, target_y):
        """
        通用 Bug2 单步移动：
        - 阶段 go：朝目标走，撞障碍则切 wall
        - 阶段 wall：沿障碍边界绕，视线通则切回 go
        """
        if ant.bug_phase == 'go':
            dx = target_x - ant.x
            dy = target_y - ant.y
            if math.hypot(dx, dy) > 1e-6:
                ant.heading = math.atan2(dy, dx) + random.uniform(-0.2, 0.2)

            nx = ant.x + ant.speed * math.cos(ant.heading)
            ny = ant.y + ant.speed * math.sin(ant.heading)
            nx = max(0, min(self.width, nx))
            ny = max(0, min(self.height, ny))

            hit_obs = None
            for obs in self.obstacles:
                if obs.check_collision_point(nx, ny):
                    hit_obs = obs
                    break

            if hit_obs is None:
                ant.x, ant.y = nx, ny
            else:
                ant.bug_obstacle = hit_obs
                ant.bug_phase = 'wall'
                ant.bug_direction = random.choice([1, -1])
                ant.x, ant.y = hit_obs.clamp_position(ant.x, ant.y)

        elif ant.bug_phase == 'wall':
            obs = ant.bug_obstacle

            if obs is None:
                ant.bug_phase = 'go'
                return

            if not obs.segment_intersects(ant.x, ant.y, target_x, target_y):
                ant.bug_phase = 'go'
                ant.bug_obstacle = None
                return

            tx, ty = obs.boundary_tangent(ant.x, ant.y, direction=ant.bug_direction)
            ant.heading = math.atan2(ty, tx) + random.uniform(-0.15, 0.15)

            nx = ant.x + ant.speed * math.cos(ant.heading)
            ny = ant.y + ant.speed * math.sin(ant.heading)
            nx = max(0, min(self.width, nx))
            ny = max(0, min(self.height, ny))

            if not obs.check_collision_point(nx, ny):
                ant.x, ant.y = nx, ny
            else:
                ant.heading += random.uniform(-0.5, 0.5)
                ant.x = max(0, min(self.width, ant.x + ant.speed * math.cos(ant.heading)))
                ant.y = max(0, min(self.height, ant.y + ant.speed * math.sin(ant.heading)))

            ant.x, ant.y = obs.clamp_position(ant.x, ant.y)

    def _free_ant_step_no_pheromone(self, ant, target_x, target_y,
                                    food_odor_radius=30.0):
        """
        free 阶段（navigator 和 transporter 共用）：
        只走“食物气味 + CRW”，不读信息素；套 Bug2 绕障。
        """
        if ant.bug_phase == 'go':
            ant.heading = ant.compute_pheromone_heading(
                [],                              # 不读信息素
                food_pos=(target_x, target_y),
                food_odor_radius=food_odor_radius,
                perception_radius=60.0
            )

            nx = ant.x + ant.speed * math.cos(ant.heading)
            ny = ant.y + ant.speed * math.sin(ant.heading)
            nx = max(0, min(self.width, nx))
            ny = max(0, min(self.height, ny))

            hit_obs = None
            for obs in self.obstacles:
                if obs.check_collision_point(nx, ny):
                    hit_obs = obs
                    break

            if hit_obs is None:
                ant.x, ant.y = nx, ny
            else:
                ant.bug_obstacle = hit_obs
                ant.bug_phase = 'wall'
                ant.bug_direction = random.choice([1, -1])
                ant.x, ant.y = hit_obs.clamp_position(ant.x, ant.y)

        elif ant.bug_phase == 'wall':
            self._bug2_step(ant, target_x, target_y)

    def update(self):
        dt = 0.05

        # 1. 导航蚁 (Navigator) 行为逻辑
        for ant in self.ants:
            if ant.role == 'navigator':
                if ant.state == 'free':
                    # 追踪食物：食物气味 + CRW，不读信息素；套 Bug2
                    self._free_ant_step_no_pheromone(
                        ant,
                        target_x=self.food.x,
                        target_y=self.food.y,
                        food_odor_radius=30.0
                    )

                    dist_to_food = math.hypot(self.food.x - ant.x, self.food.y - ant.y)
                    if dist_to_food <= 15.0:
                        ant.state = 'returning'
                        ant.bug_phase = 'go'
                        ant.bug_obstacle = None
                        ant.bug_direction = 1

                elif ant.state == 'returning':
                    # 指引/返巢：Bug2 朝 nest，沿途释放信息素
                    self._bug2_step(ant, self.nest[0], self.nest[1])
                    self.pheromone_manager.add_pheromone(ant.x, ant.y, intensity=1.0)

                    dist_to_nest = math.hypot(self.nest[0] - ant.x, self.nest[1] - ant.y)
                    if dist_to_nest <= 15.0:
                        ant.state = 'free'
                        ant.bug_phase = 'go'
                        ant.bug_obstacle = None
                        ant.bug_direction = 1

        # 2. 搬运工蚁 (Transporter) 状态管理与疲劳释放
        for ant in self.ants:
            if ant.role == 'transporter':
                if ant.state == 'transporting':
                    ant.transport_time += dt
                    dist = math.hypot(self.food.x - ant.x, self.food.y - ant.y)

                    if dist > 35.0 or ant.transport_time >= ant.max_transport_time:
                        ant.state = 'escaping'
                        ant.transport_time = 0.0
                        ant.escape_timer = 4.0
                        angle_away = math.atan2(ant.y - self.food.y, ant.x - self.food.x)
                        ant.heading = angle_away + random.uniform(-0.5, 0.5)

                elif ant.state == 'escaping':
                    ant.escape_timer -= dt
                    ant.move_random(self.obstacles)
                    if ant.escape_timer <= 0:
                        ant.state = 'free'
                        ant.bug_phase = 'go'
                        ant.bug_obstacle = None
                        ant.bug_direction = 1

        self.food.reset_slots()

        # 3. 自由状态的搬运工蚁：只走“食物气味 + CRW”，不读信息素；套 Bug2
        for ant in self.ants:
            if ant.role == 'transporter' and ant.state == 'free':
                self._free_ant_step_no_pheromone(
                    ant,
                    target_x=self.food.x,
                    target_y=self.food.y,
                    food_odor_radius=30.0
                )

        # 4. 槽位认领
        for ant in self.ants:
            if ant.role == 'transporter':
                if ant.state == 'transporting':
                    slot = self.food.claim_slot(ant.x, ant.y, ant.id)
                    if slot is None:
                        ant.state = 'escaping'
                        ant.escape_timer = 2.0

        for ant in self.ants:
            if ant.role == 'transporter' and ant.state == 'free':
                claimed_slot = self.food.claim_slot(ant.x, ant.y, ant.id)
                if claimed_slot is not None:
                    ant.state = 'transporting'
                    ant.transport_time = 0.0
                    ant.max_transport_time = random.uniform(6.0, 12.0)
                    ant.bug_phase = 'go'
                    ant.bug_obstacle = None
                    ant.bug_direction = 1

        # 5. 计算搬运拉力与转矩
        transporters = [
            ant for ant in self.ants
            if ant.role == 'transporter' and ant.state == 'transporting'
        ]
        num_transporters = len(transporters)

        forces = []
        torques = []

        if num_transporters > 0:
            force_magnitude = 2.0
            ant_intentions = []

            perception_radius = 60.0
            signal_threshold = 0.02

            for ant in transporters:
                weight = 1.0 + 0.3 * ant.transport_time

                vx = math.cos(ant.heading)
                vy = math.sin(ant.heading)

                p_attract_x, p_attract_y = 0.0, 0.0
                p_weight_sum = 0.0

                for p in self.pheromone_manager.particles:
                    dist = math.hypot(p['x'] - ant.x, p['y'] - ant.y)
                    if 0 < dist <= perception_radius:
                        w = p['intensity'] / (dist ** 1.2 + 0.1)
                        p_attract_x += (p['x'] - ant.x) / dist * w
                        p_attract_y += (p['y'] - ant.y) / dist * w
                        p_weight_sum += w

                has_pheromone_signal = p_weight_sum > signal_threshold
                if has_pheromone_signal:
                    p_norm = math.hypot(p_attract_x, p_attract_y)
                    if p_norm > 0:
                        chem_x = p_attract_x / p_norm
                        chem_y = p_attract_y / p_norm
                    else:
                        chem_x, chem_y = vx, vy
                else:
                    chem_x, chem_y = vx, vy

                ant_intentions.append({
                    'ant': ant,
                    'vx': vx,
                    'vy': vy,
                    'chem_x': chem_x,
                    'chem_y': chem_y,
                    'has_signal': has_pheromone_signal,
                    'weight': weight
                })

            total_wx = sum(item['vx'] * item['weight'] for item in ant_intentions)
            total_wy = sum(item['vy'] * item['weight'] for item in ant_intentions)
            social_norm = math.hypot(total_wx, total_wy)

            if social_norm > 0:
                avg_social_x = total_wx / social_norm
                avg_social_y = total_wy / social_norm
            else:
                avg_social_x, avg_social_y = 1.0, 0.0

            for item in ant_intentions:
                ant = item['ant']

                if item['has_signal']:
                    blended_x = item['chem_x']
                    blended_y = item['chem_y']
                else:
                    crw_weight = 0.4
                    social_weight = 0.6
                    blended_x = crw_weight * item['vx'] + social_weight * avg_social_x
                    blended_y = crw_weight * item['vy'] + social_weight * avg_social_y

                b_norm = math.hypot(blended_x, blended_y)
                if b_norm > 0:
                    fx = (blended_x / b_norm) * force_magnitude
                    fy = (blended_y / b_norm) * force_magnitude
                else:
                    fx, fy = force_magnitude, 0.0

                forces.append((fx, fy))

                for slot in self.food.slots:
                    if slot['occupied_by'] == ant.id:
                        lx, ly = slot['local_pos']
                        c, s = math.cos(self.food.theta), math.sin(self.food.theta)
                        rx = lx * c - ly * s
                        ry = lx * s + ly * c
                        torque = rx * fy - ry * fx
                        torques.append(torque)
                        break

        # 6. 更新食物刚体与障碍物碰撞
        self.food.update_dynamics(forces, torques, dt=dt, width=self.width, height=self.height)
        for obs in self.obstacles:
            obs.resolve_food_collision(self.food)

        # 7. 坐标绑定与防穿墙校验
        for ant in transporters:
            for slot in self.food.slots:
                if slot['occupied_by'] == ant.id:
                    lx, ly = slot['local_pos']
                    c, s = math.cos(self.food.theta), math.sin(self.food.theta)
                    target_x = self.food.x + (lx * c - ly * s)
                    target_y = self.food.y + (lx * s + ly * c)
                    ant.x += (target_x - ant.x) * 0.4
                    ant.y += (target_y - ant.y) * 0.4
                    break

        for ant in self.ants:
            ant.constrain_position(self.obstacles)

        self.pheromone_manager.update()


# 运行与可视化
if __name__ == "__main__":
    env_width, env_height = 150, 150
    sim = ObstacleTransportSimulation(num_ants=35, width=env_width, height=env_height)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, env_width)
    ax.set_ylim(0, env_height)

    for obs in sim.obstacles:
        rect = Rectangle(
            (obs.x - obs.width / 2, obs.y - obs.height / 2),
            obs.width, obs.height, color='gray', alpha=0.7
        )
        ax.add_patch(rect)

    pheromone_scatter = ax.scatter([], [], s=8, c='red', alpha=0.5, label='Pheromone (Navigator Ants)')
    ant_scatter = ax.scatter([], [], s=30, c='black', label='Transporter Ants')
    food_patch = Polygon(np.empty((4, 2)), closed=True, color='purple', alpha=0.8, label='Food Block')
    ax.add_patch(food_patch)

    ax.plot(sim.nest[0], sim.nest[1], marker='*', markersize=15, color='green', label='Nest')
    ax.legend(loc='upper right')

    def init():
        pheromone_scatter.set_offsets(np.empty((0, 2)))
        ant_scatter.set_offsets(np.empty((0, 2)))
        food_patch.set_xy(np.empty((4, 2)))
        return pheromone_scatter, ant_scatter, food_patch

    def update_frame(frame):
        sim.update()

        p_particles = sim.pheromone_manager.particles
        if len(p_particles) > 0:
            p_coords = np.array([[p['x'], p['y']] for p in p_particles])
            p_alphas = [min(1.0, max(0.1, p['intensity'])) for p in p_particles]
            pheromone_scatter.set_offsets(p_coords)
            pheromone_scatter.set_alpha(p_alphas)
        else:
            pheromone_scatter.set_offsets(np.empty((0, 2)))

        ant_coords = np.array([[ant.x, ant.y] for ant in sim.ants])

        ant_colors = []
        for ant in sim.ants:
            if ant.role == 'navigator':
                ant_colors.append('orange')
            elif ant.state == 'transporting':
                ant_colors.append('blue')
            elif ant.state == 'escaping':
                ant_colors.append('magenta')
            else:
                ant_colors.append('black')

        ant_scatter.set_offsets(ant_coords)
        ant_scatter.set_facecolors(ant_colors)

        food_corners = sim.food.get_corners()
        food_patch.set_xy(food_corners)

        trans_count = sum(1 for ant in sim.ants if ant.role == 'transporter' and ant.state == 'transporting')
        nav_count = sum(1 for ant in sim.ants if ant.role == 'navigator' and ant.state == 'returning')
        ax.set_title(
            f"Division of Labor: Navigators & Transporters\n"
            f"Active Transporters: {trans_count} | Returning Navigators: {nav_count}",
            fontsize=10
        )

        return pheromone_scatter, ant_scatter, food_patch

    ani = animation.FuncAnimation(fig, update_frame, init_func=init, frames=600, interval=50, blit=False)
    plt.show()