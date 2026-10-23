"""带障碍物的蚂蚁协作搬运仿真核心，不包含绘图依赖。

运行入口 ``run_animation.py`` 和 ``run_experiments.py`` 均创建本模块的
``ObstacleTransportSimulation``，并通过 ``update`` 推进模型；显示由
``transport_visualization.py`` 单独负责。
"""

import math
import random
from dataclasses import replace
from ant_crw import Ant
from food import Food
from pheromone import PheromoneManager
from obstacle import Obstacle
from simulation_config import SimulationConfig

class ObstacleTransportSimulation:
    """管理蚂蚁、食物、障碍物和信息素，并逐帧推进仿真状态。

    每帧由 ``update`` 更新导航/搬运状态，合并个体牵引和转矩，积分食物运动、
    解算障碍碰撞，最后更新诊断数据及信息素。
    """

    def __init__(self, num_ants=None, width=None, height=None,
                 diagnostics_enabled=False, config=None):
        """依据共享配置创建场景与对象。

        由动画入口、批量实验入口和单元测试调用。未传入配置时使用默认参数；
        显式尺寸或蚂蚁数通过 ``dataclasses.replace`` 覆盖配置而不修改原对象。
        """
        if config is None:
            default_navigators = min(5, num_ants) if num_ants is not None else 5
            base_config = SimulationConfig(num_navigators=default_navigators)
        else:
            base_config = config
        overrides = {}
        if num_ants is not None:
            overrides['num_ants'] = num_ants
        if width is not None:
            overrides['width'] = width
        if height is not None:
            overrides['height'] = height
        self.config = replace(base_config, **overrides) if overrides else base_config

        # 场地坐标范围为 [0, width] × [0, height]；巢穴和食物使用场地坐标。
        self.width = self.config.width
        self.height = self.config.height
        self.nest = self.config.nest_position
        self.diagnostics_enabled = diagnostics_enabled
        self.last_diagnostics = []
        self.step_count = 0

        # 食物包含刚体运动状态，以及供搬运工蚁认领的周边槽位。
        self.food = Food(
            x=self.config.food_position[0],
            y=self.config.food_position[1],
            width=self.config.food_width,
            height=self.config.food_height,
            mass=self.config.food_mass
        )

        # 矩形继续使用中心和宽高定义；菱形使用按边界顺序排列的凸多边形顶点。
        self.obstacles = [
            Obstacle(x=55.0, y=60.0, width=50.0, height=6.0),
            Obstacle(x=80.0, y=20.0, width=6.0, height=50.0),
            Obstacle.from_vertices([
                (75.0, 84.0),
                (83.0, 92.0),
                (75.0, 100.0),
                (67.0, 92.0),
            ]),
        ]
        # 同一障碍物上的返巢导航蚁复用首次选定的绕行方向，减少分叉信息素轨迹。
        self._return_wall_directions = {}

        # 前五只蚂蚁担任导航蚁，其余蚂蚁担任搬运工。
        self.ants = []
        for i in range(self.config.num_ants):
            is_nav = i < self.config.num_navigators
            self.ants.append(
                Ant(
                    ant_id=i,
                    x=random.uniform(self.width * 0.1, self.width * 0.9),
                    y=random.uniform(self.height * 0.1, self.height * 0.9),
                    is_navigator=is_nav,
                    width=self.width,
                    height=self.height,
                    speed=self.config.ant_speed
                )
            )

        # 导航蚁返巢时沿途布置信息素，信息素管理器负责挥发和清理。
        self.pheromone_manager = PheromoneManager(
            evaporation_rate=self.config.pheromone_evaporation_rate
        )

    @staticmethod
    def _normalize_angle(angle):
        """将弧度角归一化到 [-pi, pi]，便于选取最短转向方向。

        供 ``_angle_lerp`` 调用；取模运算将任意角差折回一个完整圆周内。
        """
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _angle_lerp(current, target, alpha):
        """沿最短角度差从当前朝向平滑转向目标朝向。

        Bug2 wall 阶段调用；``alpha`` 控制本步完成的角度差比例，避免朝向突变。
        """
        diff = ObstacleTransportSimulation._normalize_angle(target - current)
        return current + diff * alpha

    def _blend_ambiguous_pheromone_direction(
        self, memory_x, memory_y, social_x, social_y,
        signal_confidence, social_confidence
    ):
        """按信息素模糊度和同伴一致性，将群体方向有限融入个体记忆。

        ``update`` 在搬运蚂蚁的信号方向不确定时调用。社会权重是上限、
        信息素不确定度和群体一致性的乘积；归一化加权向量得到最终方向，
        向量抵消时保留个体记忆方向。
        """
        signal_confidence = max(0.0, min(1.0, signal_confidence))
        social_confidence = max(0.0, min(1.0, social_confidence))
        social_weight = (
            self.config.ambiguous_pheromone_max_social_weight
            * (1.0 - signal_confidence)
            * social_confidence
        )

        direction_x = (1.0 - social_weight) * memory_x + social_weight * social_x
        direction_y = (1.0 - social_weight) * memory_y + social_weight * social_y
        direction_length = math.hypot(direction_x, direction_y)
        if direction_length > 1e-12:
            direction_x /= direction_length
            direction_y /= direction_length
        else:
            direction_x, direction_y = memory_x, memory_y
        return direction_x, direction_y, social_weight

    def _visible_pheromone_particles(self, ant, particles, perception_radius):
        """仅返回感知范围内且与蚂蚁之间无遮挡的信息素粒子。

        ``update`` 为搬运蚂蚁筛选信号，返巢导航逻辑也会调用；
        超出半径或任一障碍与蚂蚁—粒子线段相交的粒子都会被排除。
        """
        visible_particles = []
        for particle in particles:
            distance = math.hypot(particle['x'] - ant.x, particle['y'] - ant.y)
            if distance > perception_radius:
                continue
            if any(
                obstacle.segment_intersects(
                    ant.x, ant.y, particle['x'], particle['y']
                )
                for obstacle in self.obstacles
            ):
                continue
            visible_particles.append(particle)
        return visible_particles

    def _estimate_pheromone_tangent(self, ant, particles, radius=20.0):
        """以局部加权 PCA 估计信息素轨迹切线及其方向置信度。

        由 transporter 方向决策和 navigator 返巢跟踪调用。强度/距离权重形成
        加权质心及二维协方差；主轴给出切线，特征值差相对迹值衡量线性置信度。
        返回切线、置信度和参与估计的粒子数；粒子不足或分布近似各向同性时返回
        ``None`` 切线，避免从模糊点云猜测方向。
        """
        local_particles = []
        total_weight = 0.0
        centroid_x = 0.0
        centroid_y = 0.0

        for particle in particles:
            dx = particle['x'] - ant.x
            dy = particle['y'] - ant.y
            distance = math.hypot(dx, dy)
            if distance > radius:
                continue

            weight = particle['intensity'] / (distance ** 1.2 + 0.1)
            local_particles.append((particle['x'], particle['y'], weight))
            total_weight += weight
            centroid_x += particle['x'] * weight
            centroid_y += particle['y'] * weight

        if (
            len(local_particles) < self.config.tangent_min_particle_count
            or total_weight <= 0
        ):
            return None, 0.0, len(local_particles)

        centroid_x /= total_weight
        centroid_y /= total_weight
        covariance_xx = 0.0
        covariance_xy = 0.0
        covariance_yy = 0.0
        for px, py, weight in local_particles:
            dx = px - centroid_x
            dy = py - centroid_y
            covariance_xx += weight * dx * dx
            covariance_xy += weight * dx * dy
            covariance_yy += weight * dy * dy

        covariance_xx /= total_weight
        covariance_xy /= total_weight
        covariance_yy /= total_weight
        trace = covariance_xx + covariance_yy
        if trace <= 1e-9:
            return None, 0.0, len(local_particles)

        discriminant = math.hypot(
            covariance_xx - covariance_yy,
            2.0 * covariance_xy
        )
        # 对称 2x2 协方差矩阵的特征值差为 discriminant；除以迹值作尺度无关置信度。
        confidence = discriminant / trace
        if confidence < self.config.tangent_min_confidence:
            return None, confidence, len(local_particles)

        tangent_angle = 0.5 * math.atan2(
            2.0 * covariance_xy,
            covariance_xx - covariance_yy
        )
        return (
            (math.cos(tangent_angle), math.sin(tangent_angle)),
            confidence,
            len(local_particles)
        )

    def _navigator_return_trail_direction(self, ant, target_x, target_y):
        """估计其他导航蚁留下、且有助于返巢的局部信息素切线。

        仅由返巢阶段的 ``_bug2_step`` 调用。先排除自身粒子，再应用可见性和局部
        PCA；切线符号按与巢穴方向点积校正，并拒绝缺乏最低返巢进展的方向。
        """
        particles_from_others = [
            particle for particle in self.pheromone_manager.particles
            if particle.get('source_ant_id') != ant.id
        ]
        visible_particles = self._visible_pheromone_particles(
            ant,
            particles_from_others,
            self.config.navigator_return_pheromone_radius
        )
        tangent, confidence, _ = self._estimate_pheromone_tangent(
            ant,
            visible_particles,
            radius=self.config.navigator_return_pheromone_radius
        )
        if tangent is None:
            return None

        goal_x = target_x - ant.x
        goal_y = target_y - ant.y
        goal_length = math.hypot(goal_x, goal_y)
        if goal_length <= 1e-9:
            return None
        goal_x /= goal_length
        goal_y /= goal_length

        progress = tangent[0] * goal_x + tangent[1] * goal_y
        if progress < 0:
            tangent = (-tangent[0], -tangent[1])
            progress = -progress
        if progress < self.config.navigator_min_nest_progress:
            return None

        return tangent, confidence

    def _select_wall_direction(self, ant, target_x, obs,
                               share_return_direction=False):
        """根据蚂蚁、目标与障碍物相对位置选择并可共享绕障方向。

        由两个 Bug2 移动器在进入 wall 状态时调用；返巢 navigator 可将首次方向
        记录到 ``_return_wall_directions``，让同一障碍上的后续导航蚁复用该决定。
        """
        if share_return_direction and obs in self._return_wall_directions:
            return self._return_wall_directions[obs]

        to_target_x = target_x - obs.x
        obstacle_to_ant_y = ant.y - obs.y

        # 目标位于障碍右侧时，结合蚂蚁所在一侧选择绕行方向。
        if to_target_x >= 0:
            direction = 1 if obstacle_to_ant_y >= 0 else -1
        else:
            direction = -1 if obstacle_to_ant_y >= 0 else 1

        if share_return_direction:
            self._return_wall_directions[obs] = direction
        return direction

    def _bug2_step(self, ant, target_x, target_y, allow_wobble=False,
                   follow_return_pheromone=False):
        """
        推进一只蚂蚁的 Bug2 风格避障状态机一步。

        go 阶段朝目标移动，预测下一位置碰到障碍时切换到 wall；
        wall 阶段沿障碍切线绕行，连续两步视线畅通后再恢复 go。
        allow_wobble 仅允许在 go 阶段加入微弱航向变化，wall 阶段始终确定性移动。
        由 ``update`` 驱动返巢 navigator；其 wall 子状态也由自由搜索移动器委托。
        """
        if ant.bug_phase == 'go':
            # 计算当前蚂蚁指向目标的期望朝向。
            dx = target_x - ant.x
            dy = target_y - ant.y
            if math.hypot(dx, dy) > 1e-6:
                target_x_direction = dx / math.hypot(dx, dy)
                target_y_direction = dy / math.hypot(dx, dy)
                if follow_return_pheromone:
                    trail = self._navigator_return_trail_direction(
                        ant, target_x, target_y
                    )
                    if trail is not None:
                        tangent, confidence = trail
                        blend = self.config.navigator_trail_blend * confidence
                        target_x_direction = (
                            (1.0 - blend) * target_x_direction
                            + blend * tangent[0]
                        )
                        target_y_direction = (
                            (1.0 - blend) * target_y_direction
                            + blend * tangent[1]
                        )
                        direction_length = math.hypot(
                            target_x_direction, target_y_direction
                        )
                        target_x_direction /= direction_length
                        target_y_direction /= direction_length
                target_heading = math.atan2(
                    target_y_direction, target_x_direction
                )
                if allow_wobble:
                    # 用障碍物包围盒估算最近距离；对凸多边形这是偏保守的距离。
                    nearest_obstacle_distance = min(
                        math.hypot(
                            max(abs(ant.x - obs.x) - obs.width / 2.0, 0.0),
                            max(abs(ant.y - obs.y) - obs.height / 2.0, 0.0)
                        )
                        for obs in self.obstacles
                    ) if self.obstacles else float('inf')
                    # 返巢也带部分 CRW：把"上一步朝向 + 高斯转角"按权重混入目标方向；
                    # 靠近障碍时关闭，避免随机转向干扰绕障。
                    crw_weight = self.config.navigator_return_crw_weight
                    if (crw_weight > 0
                            and nearest_obstacle_distance > max(4.0, ant.speed * 3.0)):
                        crw_heading = ant.heading + random.gauss(
                            0.0, self.config.navigator_return_turn_sigma
                        )
                        mixed_x = ((1.0 - crw_weight) * math.cos(target_heading)
                                   + crw_weight * math.cos(crw_heading))
                        mixed_y = ((1.0 - crw_weight) * math.sin(target_heading)
                                   + crw_weight * math.sin(crw_heading))
                        target_heading = math.atan2(mixed_y, mixed_x)
                    # 将小偏角保存在蚂蚁身上，使相邻帧的扰动平滑连续而非逐帧跳变。
                    wobble = getattr(ant, 'return_heading_wobble', 0.0)
                    if nearest_obstacle_distance > max(4.0, ant.speed * 3.0):
                        wobble = max(
                            -0.08,
                            min(0.08, wobble + random.gauss(0.0, 0.008))
                        )
                    else:
                        wobble *= 0.5
                    ant.return_heading_wobble = wobble
                    target_heading += wobble
                ant.heading = target_heading

            # 按当前朝向预测一步，并将坐标限制在场地边界内。
            nx = ant.x + ant.speed * math.cos(ant.heading)
            ny = ant.y + ant.speed * math.sin(ant.heading)
            nx = max(0, min(self.width, nx))
            ny = max(0, min(self.height, ny))

            # 检查预测位置是否落入某个障碍物。
            hit_obs = None
            for obs in self.obstacles:
                if obs.check_collision_point(nx, ny):
                    hit_obs = obs
                    break

            if hit_obs is None:
                ant.x, ant.y = nx, ny
            else:
                # 记录正在绕行的障碍，并将蚂蚁推到障碍外侧，避免陷在障碍内部。
                ant.bug_obstacle = hit_obs
                ant.bug_phase = 'wall'
                ant.bug_direction = self._select_wall_direction(
                    ant,
                    target_x,
                    hit_obs,
                    share_return_direction=allow_wobble
                )
                ant.wall_exit_count = 0
                ant.x, ant.y = hit_obs.clamp_position(ant.x, ant.y)

        elif ant.bug_phase == 'wall':
            # wall 状态需要依赖原障碍；若状态丢失则回退到 go。
            obs = ant.bug_obstacle

            if obs is None:
                ant.bug_phase = 'go'
                ant.wall_exit_count = 0
                return

            # 使用连续帧计数作为滞回，避免视线判断临界变化时在 go/wall 间反复切换。
            line_of_sight_clear = not obs.segment_intersects(ant.x, ant.y, target_x, target_y)
            if line_of_sight_clear:
                ant.wall_exit_count = getattr(ant, 'wall_exit_count', 0) + 1
            else:
                ant.wall_exit_count = 0

            if ant.wall_exit_count >= 2:
                ant.bug_phase = 'go'
                ant.bug_obstacle = None
                ant.wall_exit_count = 0
                return

            # 以障碍边界切线为期望朝向，并通过角度插值限制瞬间转向幅度。
            tx, ty = obs.boundary_tangent(ant.x, ant.y, direction=ant.bug_direction)
            target_angle = math.atan2(ty, tx)
            ant.heading = self._angle_lerp(ant.heading, target_angle, 0.35)

            # 沿切线方向试走一步；位置仍需受场地边界约束。
            nx = ant.x + ant.speed * math.cos(ant.heading)
            ny = ant.y + ant.speed * math.sin(ant.heading)
            nx = max(0, min(self.width, nx))
            ny = max(0, min(self.height, ny))

            if not obs.check_collision_point(nx, ny):
                ant.x, ant.y = nx, ny
            else:
                # 平滑航向可能切入凸障碍；改沿选定边界切线移动，避免径向回退造成掉头。
                ant.heading = target_angle
                for step_fraction in (1.0, 0.5, 0.25, 0.125, 0.0625):
                    tangent_x = max(
                        0,
                        min(
                            self.width,
                            ant.x + ant.speed * step_fraction * math.cos(target_angle)
                        )
                    )
                    tangent_y = max(
                        0,
                        min(
                            self.height,
                            ant.y + ant.speed * step_fraction * math.sin(target_angle)
                        )
                    )
                    if not obs.check_collision_point(tangent_x, tangent_y):
                        ant.x, ant.y = tangent_x, tangent_y
                        break

            # 作为最后一道保护，将可能进入矩形内部的位置投影回障碍外缘。
            ant.x, ant.y = obs.clamp_position(ant.x, ant.y)

    def _free_ant_step_no_pheromone(self, ant, target_x, target_y,
                                    food_odor_radius=30.0):
        """
        推进自由状态蚂蚁一步，不读取信息素。

        期望朝向由食物气味和相关随机游走（CRW）决定；
        前进遇到障碍时进入 wall 状态，后续交给 Bug2 风格的绕障逻辑。
        由 ``update`` 驱动自由搜索中的 navigator 和未加入搬运队伍的 transporter。
        """
        if ant.bug_phase == 'go':
            # 传入空信息素列表，确保自由搜索阶段只响应食物气味或 CRW。
            ant.heading = ant.compute_pheromone_heading(
                [],                              # 不读信息素
                food_pos=(target_x, target_y),
                food_odor_radius=food_odor_radius,
                perception_radius=60.0
            )

            # 先预测下一位置，再决定正常前进或进入避障状态。
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
                # 保存障碍与绕行方向，后续帧沿障碍边界绕行。
                ant.bug_obstacle = hit_obs
                ant.bug_phase = 'wall'
                ant.bug_direction = self._select_wall_direction(
                    ant, target_x, hit_obs
                )
                ant.wall_exit_count = 0
                ant.x, ant.y = hit_obs.clamp_position(ant.x, ant.y)

        elif ant.bug_phase == 'wall':
            self._bug2_step(ant, target_x, target_y)

    def _can_navigator_detect_food(self, ant):
        """仅当导航蚁足够接近且与食物中心之间无遮挡时，才确认发现食物。

        由 ``update`` 在自由搜索移动后调用；距离阈值读取配置，障碍线段相交表示
        视线被挡，蚂蚁不会因此进入返巢状态。
        """
        distance = math.hypot(self.food.x - ant.x, self.food.y - ant.y)
        if distance > self.config.navigator_food_detection_radius:
            return False

        return not any(
            obstacle.segment_intersects(ant.x, ant.y, self.food.x, self.food.y)
            for obstacle in self.obstacles
        )

    def update(self):
        """推进一帧仿真并刷新状态、诊断结果。

        动画回调、实验循环及回归测试调用本方法。执行顺序是导航蚁状态、搬运蚁
        状态/槽位、信息素和同伴方向合成、合力/转矩及障碍约束、食物积分与碰撞、
        诊断记录、蚂蚁位置约束、信息素挥发；顺序决定同一帧内的状态更新时间点。
        """
        dt = 0.05

        # 1. 更新导航蚁：搜索食物、携带信息素返回巢穴。
        for ant in self.ants:
            if ant.role == 'navigator':
                if ant.state == 'free':
                    # 搜索阶段使用食物气味和 CRW，不读信息素。
                    self._free_ant_step_no_pheromone(
                        ant,
                        target_x=self.food.x,
                        target_y=self.food.y,
                        food_odor_radius=60.0
                    )

                    # 近距离且视线未被任一障碍挡住，才切换为返巢任务。
                    if self._can_navigator_detect_food(ant):
                        ant.state = 'returning'
                        ant.bug_phase = 'go'
                        ant.bug_obstacle = None
                        ant.bug_direction = 1

                elif ant.state == 'returning':
                    # 返巢时轻度跟随其他导航蚁的路线提示，并沿途留下信息素。
                    self._bug2_step(
                        ant,
                        self.nest[0],
                        self.nest[1],
                        allow_wobble=True,
                        follow_return_pheromone=True
                    )
                    self.pheromone_manager.add_pheromone(
                        ant.x, ant.y, intensity=1.0, source_ant_id=ant.id
                    )

                    # 回到巢穴附近后重新开始搜索，以完成下一轮导航循环。
                    dist_to_nest = math.hypot(self.nest[0] - ant.x, self.nest[1] - ant.y)
                    if dist_to_nest <= self.config.navigator_nest_arrival_radius:
                        ant.state = 'free'
                        ant.bug_phase = 'go'
                        ant.bug_obstacle = None
                        ant.bug_direction = 1

        # 2. 更新搬运工状态：搬运超时或远离食物时暂时离开，之后恢复搜索。
        for ant in self.ants:
            if ant.role == 'transporter':
                if ant.state == 'transporting':
                    ant.transport_time += dt
                    dist = math.hypot(self.food.x - ant.x, self.food.y - ant.y)

                    # 搬运距离或持续时间超出限制，模拟疲劳并设置逃离方向。
                    if dist > 35.0 or ant.transport_time >= ant.max_transport_time:
                        ant.state = 'escaping'
                        ant.transport_time = 0.0
                        ant.escape_timer = 4.0
                        angle_away = math.atan2(ant.y - self.food.y, ant.x - self.food.x)
                        ant.heading = angle_away + random.uniform(-0.5, 0.5)

                elif ant.state == 'escaping':
                    # 逃离计时期间随机游走；计时结束后恢复自由搜索。
                    ant.escape_timer -= dt
                    ant.move_random(self.obstacles)
                    if ant.escape_timer <= 0:
                        ant.state = 'free'
                        ant.bug_phase = 'go'
                        ant.bug_obstacle = None
                        ant.bug_direction = 1

        # 每帧重新认领槽位，避免已离开或失效的蚂蚁继续占用搬运位置。
        self.food.reset_slots()

        # 3. 让自由状态的搬运工向食物移动；此阶段同样不跟随信息素。
        for ant in self.ants:
            if ant.role == 'transporter' and ant.state == 'free':
                self._free_ant_step_no_pheromone(
                    ant,
                    target_x=self.food.x,
                    target_y=self.food.y,
                    food_odor_radius=60.0
                )

        # 4. 已在搬运的蚂蚁重新占回原有空位；若无法占位，则退出并进入逃离状态。
        for ant in self.ants:
            if ant.role == 'transporter':
                if ant.state == 'transporting':
                    slot = self.food.claim_slot(ant.x, ant.y, ant.id)
                    if slot is None:
                        ant.state = 'escaping'
                        ant.escape_timer = 2.0

        # 自由蚂蚁若足够靠近食物且抢到空槽位，就加入搬运队伍。
        for ant in self.ants:
            if ant.role == 'transporter' and ant.state == 'free':
                claimed_slot = self.food.claim_slot(ant.x, ant.y, ant.id)
                if claimed_slot is not None:
                    ant.state = 'transporting'
                    ant.transport_time = 0.0
                    ant.max_transport_time = random.uniform(6.0, 12.0)
                    ant.pheromone_heading = ant.heading
                    ant.signal_lost_time = 0.0
                    ant.bug_phase = 'go'
                    ant.bug_obstacle = None
                    ant.bug_direction = 1

        # 5. 根据搬运蚂蚁朝向及其认领位置，计算施加到食物上的合力和转矩。
        transporters = [
            ant for ant in self.ants
            if ant.role == 'transporter' and ant.state == 'transporting'
        ]
        num_transporters = len(transporters)

        forces = []
        torques = []
        diagnostic_rows = []
        force_magnitude = 2.0
        obstacle_contact_clearance = 1.0

        if num_transporters > 0:
            # 单只蚂蚁施力大小；更长的持续搬运时间会提高其群体方向权重。
            ant_intentions = []

            perception_radius = self.config.transporter_pheromone_radius
            signal_threshold = self.config.pheromone_signal_threshold

            for ant in transporters:
                # 记录每只蚂蚁的当前前进方向和随搬运时间增长的权重。
                weight = 1.0 + 0.3 * ant.transport_time

                # 个体与群体共识都使用方向记忆：搬运期间 heading 不更新，信号消失后
                # 记忆仍保留最后一次可信的跟随方向，避免方向突然跳回附着前的朝向。
                held_heading = getattr(ant, 'pheromone_heading', ant.heading)
                vx = math.cos(held_heading)
                vy = math.sin(held_heading)

                visible_particles = self._visible_pheromone_particles(
                    ant,
                    self.pheromone_manager.particles,
                    perception_radius
                )

                # 汇总感知范围内的信息素方向，距离越近、强度越高的粒子贡献越大。
                p_attract_x, p_attract_y = 0.0, 0.0
                p_weight_sum = 0.0

                for p in visible_particles:
                    dist = math.hypot(p['x'] - ant.x, p['y'] - ant.y)
                    if 0 < dist <= perception_radius:
                        # 信息素贡献 = 指向粒子的单位向量 × 强度/距离衰减权重。
                        w = p['intensity'] / (dist ** 1.2 + 0.1)
                        p_attract_x += (p['x'] - ant.x) / dist * w
                        p_attract_y += (p['y'] - ant.y) / dist * w
                        p_weight_sum += w

                # 有有效信息素时按信息素方向行动；否则保留自身方向供后续社会混合使用。
                has_pheromone_signal = p_weight_sum > signal_threshold
                # 记录连续无信号的时长，供无信号分支逐步提高社会权重。
                if has_pheromone_signal:
                    ant.signal_lost_time = 0.0
                else:
                    ant.signal_lost_time = getattr(ant, 'signal_lost_time', 0.0) + dt
                p_norm = math.hypot(p_attract_x, p_attract_y)
                # 合向量长度/总权重衡量方向一致性；相反轨迹互相抵消时该值会变小。
                pheromone_direction_confidence = (
                    p_norm / p_weight_sum if p_weight_sum > 0 else 0.0
                )
                direction_source = 'social'
                if has_pheromone_signal:
                    if (
                        p_norm > 1e-12
                        and pheromone_direction_confidence
                        >= self.config.resultant_min_confidence
                    ):
                        chem_x = p_attract_x / p_norm
                        chem_y = p_attract_y / p_norm
                        ant.pheromone_heading = math.atan2(chem_y, chem_x)
                        direction_source = 'pheromone_resultant'
                    else:
                        previous_heading = getattr(
                            ant, 'pheromone_heading', ant.heading
                        )
                        chem_x = math.cos(previous_heading)
                        chem_y = math.sin(previous_heading)
                        direction_source = 'memory'
                else:
                    chem_x, chem_y = vx, vy

                tangent, tangent_confidence, tangent_particle_count = (
                    self._estimate_pheromone_tangent(
                        ant,
                        visible_particles,
                        radius=self.config.transporter_pheromone_radius
                    )
                )
                if has_pheromone_signal:
                    if tangent is not None:
                        previous_heading = getattr(
                            ant, 'pheromone_heading', ant.heading
                        )
                        previous_x = math.cos(previous_heading)
                        previous_y = math.sin(previous_heading)
                        if tangent[0] * previous_x + tangent[1] * previous_y < 0:
                            tangent = (-tangent[0], -tangent[1])
                        chem_x, chem_y = tangent
                        ant.pheromone_heading = math.atan2(chem_y, chem_x)
                        direction_source = 'tangent'
                diagnostic_row = None
                if self.diagnostics_enabled:
                    p_vector_magnitude = math.hypot(p_attract_x, p_attract_y)
                    diagnostic_row = {
                        'step': self.step_count + 1,
                        'ant_id': ant.id,
                        'ant_x': ant.x,
                        'ant_y': ant.y,
                        'ant_heading': ant.heading,
                        'bug_phase': ant.bug_phase,
                        'bug_direction': ant.bug_direction,
                        'pheromone_weight_sum': p_weight_sum,
                        'pheromone_vector_x': p_attract_x,
                        'pheromone_vector_y': p_attract_y,
                        'pheromone_vector_magnitude': p_vector_magnitude,
                        'pheromone_direction_confidence':
                            pheromone_direction_confidence,
                        'pheromone_direction': (
                            math.atan2(p_attract_y, p_attract_x)
                            if p_vector_magnitude > 1e-12 else None
                        ),
                        'pheromone_tangent_x': tangent[0] if tangent else None,
                        'pheromone_tangent_y': tangent[1] if tangent else None,
                        'pheromone_tangent_confidence': tangent_confidence,
                        'pheromone_tangent_particle_count': tangent_particle_count,
                        'direction_source': direction_source,
                        'group_direction_confidence': None,
                        'fallback_social_weight': 0.0,
                        'has_pheromone_signal': has_pheromone_signal,
                        'individual_force_x': None,
                        'individual_force_y': None,
                    }
                    diagnostic_rows.append(diagnostic_row)

                ant_intentions.append({
                    'ant': ant,
                    'vx': vx,
                    'vy': vy,
                    'chem_x': chem_x,
                    'chem_y': chem_y,
                    'has_signal': has_pheromone_signal,
                    'direction_source': direction_source,
                    'signal_confidence': max(
                        pheromone_direction_confidence,
                        tangent_confidence
                    ),
                    'weight': weight,
                    'diagnostic_row': diagnostic_row,
                })

            # 以所有搬运蚂蚁的加权朝向计算群体共识方向。
            total_wx = sum(item['vx'] * item['weight'] for item in ant_intentions)
            total_wy = sum(item['vy'] * item['weight'] for item in ant_intentions)
            social_norm = math.hypot(total_wx, total_wy)

            if social_norm > 0:
                avg_social_x = total_wx / social_norm
                avg_social_y = total_wy / social_norm
            else:
                avg_social_x, avg_social_y = 1.0, 0.0

            # 将意图方向归一化为固定大小的力，并通过槽位相对食物中心的位置计算转矩。
            for item in ant_intentions:
                ant = item['ant']

                if item['has_signal']:
                    blended_x = item['chem_x']
                    blended_y = item['chem_y']
                    if item['direction_source'] == 'memory':
                        peer_wx = sum(
                            peer['vx'] * peer['weight']
                            for peer in ant_intentions if peer is not item
                        )
                        peer_wy = sum(
                            peer['vy'] * peer['weight']
                            for peer in ant_intentions if peer is not item
                        )
                        peer_weight_sum = sum(
                            peer['weight']
                            for peer in ant_intentions if peer is not item
                        )
                        peer_norm = math.hypot(peer_wx, peer_wy)
                        group_direction_confidence = (
                            peer_norm / peer_weight_sum
                            if peer_weight_sum > 0 else 0.0
                        )
                        if peer_norm > 1e-12:
                            peer_wx /= peer_norm
                            peer_wy /= peer_norm
                        else:
                            peer_wx, peer_wy = item['chem_x'], item['chem_y']

                        (
                            blended_x,
                            blended_y,
                            fallback_social_weight
                        ) = self._blend_ambiguous_pheromone_direction(
                            item['chem_x'],
                            item['chem_y'],
                            peer_wx,
                            peer_wy,
                            item['signal_confidence'],
                            group_direction_confidence
                        )
                        if fallback_social_weight > 0:
                            item['direction_source'] = 'memory_social'
                        if self.diagnostics_enabled:
                            item['diagnostic_row'][
                                'group_direction_confidence'
                            ] = group_direction_confidence
                            item['diagnostic_row'][
                                'fallback_social_weight'
                            ] = fallback_social_weight
                else:
                    # 信号刚消失时保持记忆方向；随丢失时间增长，社会权重由 0 升到 0.6。
                    fade = min(
                        1.0,
                        getattr(ant, 'signal_lost_time', 0.0)
                        / self.config.signal_loss_fade_time
                    )
                    social_weight = 0.6 * fade
                    crw_weight = 1.0 - social_weight
                    blended_x = crw_weight * item['vx'] + social_weight * avg_social_x
                    blended_y = crw_weight * item['vy'] + social_weight * avg_social_y

                b_norm = math.hypot(blended_x, blended_y)
                if b_norm > 0:
                    fx = (blended_x / b_norm) * force_magnitude
                    fy = (blended_y / b_norm) * force_magnitude
                else:
                    fx, fy = force_magnitude, 0.0

                forces.append((fx, fy))
                # 保存障碍投影前的个体拉力，供可视化显示每只搬运蚁的追踪方向。
                ant.individual_force = (fx, fy)
                ant.force_direction_source = item['direction_source']
                if self.diagnostics_enabled:
                    item['diagnostic_row']['direction_source'] = (
                        item['direction_source']
                    )
                    item['diagnostic_row']['individual_force_x'] = fx
                    item['diagnostic_row']['individual_force_y'] = fy

                for slot in self.food.slots:
                    if slot['occupied_by'] == ant.id:
                        # 将槽位局部坐标旋转到世界坐标；二维叉积 r × F 即为转矩。
                        lx, ly = slot['local_pos']
                        c, s = math.cos(self.food.theta), math.sin(self.food.theta)
                        rx = lx * c - ly * s
                        ry = lx * s + ly * c
                        torque = rx * fy - ry * fx
                        torques.append(torque)
                        break

        # 接近障碍时移除指向障碍内部的合力分量，保留沿边界的可行牵引。
        total_fx = sum(force[0] for force in forces)
        total_fy = sum(force[1] for force in forces)
        requested_force_x = total_fx
        requested_force_y = total_fy
        requested_force_magnitude = math.hypot(total_fx, total_fy)
        requested_torque = sum(torques)
        contact_normals = []
        for obs in self.obstacles:
            contact_normal = obs.food_contact_normal(
                self.food, clearance=obstacle_contact_clearance
            )
            if contact_normal is None:
                continue

            contact_normals.append(contact_normal)
            # 点积是合力在接触法线上的分量；负值表示力指向障碍内部，故投影掉。
            normal_force = total_fx * contact_normal[0] + total_fy * contact_normal[1]
            if normal_force < 0:
                total_fx -= normal_force * contact_normal[0]
                total_fy -= normal_force * contact_normal[1]

        # 正面顶住障碍、切向合力不足时，选择更接近巢穴方向的可行切线继续绕行。
        projected_force_magnitude = math.hypot(total_fx, total_fy)
        blocked_force_threshold = 0.25 * max(
            requested_force_magnitude, force_magnitude
        )
        if (
            contact_normals
            and projected_force_magnitude < blocked_force_threshold
        ):
            goal_x = self.nest[0] - self.food.x
            goal_y = self.nest[1] - self.food.y
            desired_magnitude = max(requested_force_magnitude, force_magnitude)
            best_direction = None
            best_progress = float('-inf')

            for normal_x, normal_y in contact_normals:
                for sign in (-1.0, 1.0):
                    candidate_x = -normal_y * sign
                    candidate_y = normal_x * sign

                    # 多障碍接触时，候选切向力也必须满足其他接触面的非穿透约束。
                    for other_normal_x, other_normal_y in contact_normals:
                        inward_force = (
                            candidate_x * other_normal_x
                            + candidate_y * other_normal_y
                        )
                        if inward_force < 0:
                            candidate_x -= inward_force * other_normal_x
                            candidate_y -= inward_force * other_normal_y

                    candidate_magnitude = math.hypot(candidate_x, candidate_y)
                    if candidate_magnitude <= 1e-9:
                        continue
                    progress = (
                        candidate_x * goal_x + candidate_y * goal_y
                    ) / candidate_magnitude
                    if progress > best_progress:
                        best_progress = progress
                        best_direction = (
                            candidate_x / candidate_magnitude,
                            candidate_y / candidate_magnitude,
                        )

            if best_direction is not None:
                total_fx = best_direction[0] * desired_magnitude
                total_fy = best_direction[1] * desired_magnitude
            # 受阻时避免偏心拉力继续驱动食物原地自转。
            torques = []

        projected_force_x = total_fx
        projected_force_y = total_fy
        applied_torque = sum(torques)
        forces = [(total_fx, total_fy)]

        # 6. 积分更新食物的平移和旋转，再处理它与各障碍物的碰撞响应。
        previous_food_x = self.food.x
        previous_food_y = self.food.y
        self.food.update_dynamics(forces, torques, dt=dt, width=self.width, height=self.height)
        for obs in self.obstacles:
            obs.resolve_food_collision(self.food)

        if self.diagnostics_enabled:
            shared_diagnostics = {
                'step': self.step_count + 1,
                'food_x': self.food.x,
                'food_y': self.food.y,
                'food_theta': self.food.theta,
                'food_step_dx': self.food.x - previous_food_x,
                'food_step_dy': self.food.y - previous_food_y,
                'food_speed': math.hypot(self.food.vx, self.food.vy),
                'food_omega': self.food.omega,
                'contact_obstacle_count': len(contact_normals),
                'requested_force_x': requested_force_x,
                'requested_force_y': requested_force_y,
                'requested_force_magnitude': requested_force_magnitude,
                'projected_force_x': projected_force_x,
                'projected_force_y': projected_force_y,
                'projected_force_magnitude': math.hypot(
                    projected_force_x, projected_force_y
                ),
                'requested_torque': requested_torque,
                'applied_torque': applied_torque,
            }
            if diagnostic_rows:
                for row in diagnostic_rows:
                    row.update(shared_diagnostics)
            else:
                diagnostic_rows.append({
                    **shared_diagnostics,
                    'ant_id': '',
                    'ant_x': '',
                    'ant_y': '',
                    'ant_heading': '',
                    'bug_phase': '',
                    'bug_direction': '',
                    'pheromone_weight_sum': '',
                    'pheromone_vector_x': '',
                    'pheromone_vector_y': '',
                    'pheromone_vector_magnitude': '',
                    'pheromone_direction_confidence': '',
                    'pheromone_direction': '',
                    'pheromone_tangent_x': '',
                    'pheromone_tangent_y': '',
                    'pheromone_tangent_confidence': '',
                    'pheromone_tangent_particle_count': '',
                    'direction_source': '',
                    'has_pheromone_signal': '',
                    'individual_force_x': '',
                    'individual_force_y': '',
                })
            self.last_diagnostics = diagnostic_rows
        else:
            self.last_diagnostics = []

        # 7. 将搬运蚂蚁拉向各自槽位，再将所有蚂蚁限制在场地及障碍物允许的位置。
        for ant in transporters:
            for slot in self.food.slots:
                if slot['occupied_by'] == ant.id:
                    # 槽位坐标随食物旋转和平移；用渐进插值避免蚂蚁瞬移。
                    lx, ly = slot['local_pos']
                    c, s = math.cos(self.food.theta), math.sin(self.food.theta)
                    target_x = self.food.x + (lx * c - ly * s)
                    target_y = self.food.y + (lx * s + ly * c)
                    ant.x += (target_x - ant.x) * 0.4
                    ant.y += (target_y - ant.y) * 0.4
                    break

        for ant in self.ants:
            ant.constrain_position(self.obstacles)

        # 帧末挥发信息素，并移除强度归零的粒子。
        self.pheromone_manager.update()
        self.step_count += 1
