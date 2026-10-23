"""协同搬运核心与批量实验入口的行为回归测试。"""

import csv
import json
import math
import random
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from obstacle import Obstacle
from run_experiments import main as run_experiments
from simulation_config import SimulationConfig
from transport_simulation import ObstacleTransportSimulation


class SimulationConfigTests(unittest.TestCase):
    """覆盖配置参数校验及旧式构造参数的兼容行为。"""

    def test_rejects_invalid_population_and_perception_settings(self):
        """非法导航蚁数量和非正感知半径均应在配置阶段拒绝。"""
        with self.assertRaises(ValueError):
            SimulationConfig(num_ants=4, num_navigators=5)
        with self.assertRaises(ValueError):
            SimulationConfig(transporter_pheromone_radius=0)

    def test_legacy_constructor_overrides_configured_scene_values(self):
        """显式传入的旧式构造参数只覆盖对应配置，不影响其他场景值。"""
        simulation = ObstacleTransportSimulation(
            num_ants=7,
            width=120,
            config=SimulationConfig(num_ants=10, width=200, height=180)
        )
        self.assertEqual(len(simulation.ants), 7)
        self.assertEqual((simulation.width, simulation.height), (120, 180))


class PheromonePerceptionTests(unittest.TestCase):
    """验证角色感知、导航状态转换及模糊信息素方向的回归行为。"""

    def setUp(self):
        """建立固定随机种子且无障碍的最小测试仿真。"""
        random.seed(12)
        self.simulation = ObstacleTransportSimulation(
            config=SimulationConfig(
                num_ants=1, num_navigators=1,
                nest_position=(70.0, 130.0), food_position=(60.0, 30.0)
            )
        )
        self.simulation.obstacles = []

    def test_navigator_food_detection_uses_two_unit_radius_and_line_of_sight(self):
        """食物发现须同时满足 2 单位阈值和无障碍视线。"""
        ant = SimpleNamespace(
            x=self.simulation.food.x + 1.9,
            y=self.simulation.food.y
        )
        self.assertTrue(self.simulation._can_navigator_detect_food(ant))

        ant.x = self.simulation.food.x + 2.1
        self.assertFalse(self.simulation._can_navigator_detect_food(ant))

        ant.x = self.simulation.food.x + 1.0
        self.simulation.obstacles = [
            Obstacle(
                self.simulation.food.x + 0.5,
                self.simulation.food.y,
                0.2,
                0.8
            )
        ]
        self.assertFalse(self.simulation._can_navigator_detect_food(ant))

    def test_navigator_returns_to_search_within_two_units_of_nest(self):
        """返巢 navigator 到达巢穴阈值后应恢复自由搜索状态。"""
        simulation = ObstacleTransportSimulation(
            config=SimulationConfig(num_ants=1, num_navigators=1)
        )
        simulation.obstacles = []
        ant = simulation.ants[0]
        ant.state = 'returning'
        ant.x = simulation.nest[0] + 3.0
        ant.y = simulation.nest[1]

        simulation.update()

        self.assertEqual(ant.state, 'free')
        self.assertLessEqual(
            math.hypot(ant.x - simulation.nest[0], ant.y - simulation.nest[1]),
            simulation.config.navigator_nest_arrival_radius
        )

    def test_ambiguous_signal_blends_memory_with_coherent_peer_direction(self):
        """信息素方向模糊且同伴一致时，方向融合应受权重上限约束。"""
        memory = (1.0, 0.0)
        peers = (0.0, 1.0)

        direction_x, direction_y, social_weight = (
            self.simulation._blend_ambiguous_pheromone_direction(
                *memory, *peers, signal_confidence=0.0, social_confidence=1.0
            )
        )

        self.assertAlmostEqual(social_weight, 0.35)
        self.assertGreater(direction_y, 0.0)
        self.assertGreater(direction_x, direction_y)

    def test_ambiguous_signal_uses_less_social_input_when_evidence_is_clearer(self):
        """信息素证据越清晰或同伴越不一致，社会方向权重应越低。"""
        _, _, low_confidence_weight = (
            self.simulation._blend_ambiguous_pheromone_direction(
                1.0, 0.0, 0.0, 1.0,
                signal_confidence=0.1,
                social_confidence=0.8
            )
        )
        _, _, high_confidence_weight = (
            self.simulation._blend_ambiguous_pheromone_direction(
                1.0, 0.0, 0.0, 1.0,
                signal_confidence=0.5,
                social_confidence=0.8
            )
        )
        _, _, incoherent_group_weight = (
            self.simulation._blend_ambiguous_pheromone_direction(
                1.0, 0.0, 0.0, 1.0,
                signal_confidence=0.1,
                social_confidence=0.0
            )
        )

        self.assertGreater(low_confidence_weight, high_confidence_weight)
        self.assertEqual(incoherent_group_weight, 0.0)

    def test_update_uses_peer_heading_for_ambiguous_pheromone_signal(self):
        """主更新流程应为模糊信号记录个体记忆与同伴融合方向。"""
        simulation = ObstacleTransportSimulation(
            diagnostics_enabled=True,
            config=SimulationConfig(
                num_ants=7, num_navigators=5,
                nest_position=(70.0, 130.0), food_position=(60.0, 30.0)
            )
        )
        simulation.obstacles = []
        transporters = simulation.ants[-2:]
        simulation.ants = transporters
        for ant, heading in zip(transporters, (0.0, math.pi / 2.0)):
            ant.state = 'transporting'
            ant.x = simulation.food.x
            ant.y = simulation.food.y
            ant.heading = heading
            ant.pheromone_heading = 0.0
            ant.transport_time = 1.0

        simulation.pheromone_manager.particles = [
            {'x': 55.0, 'y': 30.0, 'intensity': 1.0},
            {'x': 65.0, 'y': 30.0, 'intensity': 1.0},
            {'x': 60.0, 'y': 25.0, 'intensity': 1.0},
            {'x': 60.0, 'y': 35.0, 'intensity': 1.0},
        ]

        simulation.update()

        transporter_rows = [
            row for row in simulation.last_diagnostics if row['ant_id'] != ''
        ]
        self.assertEqual(len(transporter_rows), 2)
        self.assertTrue(
            all(row['direction_source'] == 'memory_social'
                for row in transporter_rows)
        )
        self.assertTrue(
            all(row['fallback_social_weight'] > 0
                for row in transporter_rows)
        )

    def test_signal_loss_keeps_memory_direction_then_fades_to_group(self):
        """信息素消失后应先保持记忆方向，再随时间逐步回到群体共识。"""
        simulation = ObstacleTransportSimulation(
            diagnostics_enabled=True,
            config=SimulationConfig(
                num_ants=7, num_navigators=5,
                nest_position=(70.0, 130.0), food_position=(60.0, 30.0),
                signal_loss_fade_time=1.0
            )
        )
        simulation.obstacles = []
        follower, peer = simulation.ants[-2:]
        simulation.ants = [follower, peer]
        for ant, memory in ((follower, 0.0), (peer, math.pi / 2.0)):
            ant.state = 'transporting'
            ant.x = simulation.food.x
            ant.y = simulation.food.y
            ant.heading = math.pi  # 附着前的旧朝向，不应再影响方向
            ant.pheromone_heading = memory
            ant.transport_time = 1.0
        simulation.pheromone_manager.particles = []

        simulation.update()
        first_force = follower.individual_force
        first_angle = math.atan2(first_force[1], first_force[0])
        self.assertAlmostEqual(first_angle, 0.0, delta=0.1)

        for _ in range(30):
            simulation.update()
        self.assertGreater(follower.individual_force[1], 0.5)

    def test_obstacle_occludes_pheromone_particle(self):
        """障碍物遮挡的粒子不可见，无遮挡粒子仍应保留。"""
        ant = SimpleNamespace(x=40.0, y=50.0)
        hidden = {'x': 52.0, 'y': 62.0, 'intensity': 1.0}
        visible = {'x': 40.0, 'y': 60.0, 'intensity': 1.0}
        self.simulation.obstacles = [Obstacle(46.0, 54.0, 2.0, 12.0)]

        result = self.simulation._visible_pheromone_particles(
            ant, [hidden, visible], 60.0
        )

        self.assertEqual(result, [visible])

    def test_navigator_uses_visible_other_ant_trail_with_configured_radius(self):
        """返巢方向估计应使用半径内其他 navigator 留下的信息素切线。"""
        ant = SimpleNamespace(x=10.0, y=10.0, id=1)
        target_x, target_y = self.simulation.nest
        distance = math.hypot(target_x - ant.x, target_y - ant.y)
        direction = (
            (target_x - ant.x) / distance,
            (target_y - ant.y) / distance
        )
        particles = [
            {
                'x': ant.x + direction[0] * offset,
                'y': ant.y + direction[1] * offset,
                'intensity': 1.0,
                'source_ant_id': 2
            }
            for offset in (30.0, 36.0, 42.0, 48.0, 54.0)
        ]
        self.simulation.pheromone_manager.particles = particles

        trail = self.simulation._navigator_return_trail_direction(
            ant, target_x, target_y
        )

        self.assertIsNotNone(trail)
        self.assertAlmostEqual(trail[0][0], direction[0])
        self.assertAlmostEqual(trail[0][1], direction[1])
        self.assertEqual(
            self.simulation.config.navigator_return_pheromone_radius,
            60.0
        )

    def test_transporter_radius_excludes_distant_particle(self):
        """transporter 感知筛选应按配置半径排除远处粒子。"""
        ant = SimpleNamespace(x=10.0, y=10.0)
        near_particle = {'x': 25.0, 'y': 10.0, 'intensity': 1.0}
        far_particle = {'x': 31.0, 'y': 10.0, 'intensity': 1.0}

        visible = self.simulation._visible_pheromone_particles(
            ant,
            [near_particle, far_particle],
            self.simulation.config.transporter_pheromone_radius
        )

        self.assertEqual(visible, [near_particle])

    def test_navigator_does_not_follow_its_own_trail(self):
        """返巢 navigator 不应把自身释放的轨迹作为跟随信号。"""
        ant = SimpleNamespace(x=10.0, y=10.0, id=1)
        particles = [
            {
                'x': 10.0 + offset,
                'y': 10.0 + offset * 2.0,
                'intensity': 1.0,
                'source_ant_id': ant.id
            }
            for offset in (3.0, 6.0, 9.0, 12.0, 15.0)
        ]
        self.simulation.pheromone_manager.particles = particles

        self.assertIsNone(
            self.simulation._navigator_return_trail_direction(
                ant, *self.simulation.nest
            )
        )

    def test_returning_navigator_keeps_wall_tangent_at_rotated_obstacle(self):
        """返巢时 +45 度障碍的平滑转向若穿入障碍，应沿边界切线通过。"""
        obstacle = Obstacle.from_vertices([
            (75.0, 84.0),
            (83.0, 92.0),
            (75.0, 100.0),
            (67.0, 92.0),
        ])
        self.simulation.obstacles = [obstacle]
        ant = self.simulation.ants[0]
        ant.x, ant.y = 72.5, 85.5
        ant.heading = math.atan2(
            self.simulation.nest[1] - ant.y,
            self.simulation.nest[0] - ant.x
        )

        self.simulation._bug2_step(
            ant, *self.simulation.nest,
            allow_wobble=True,
            follow_return_pheromone=True
        )
        self.assertEqual(ant.bug_phase, 'wall')

        heading_before_wall_step = ant.heading
        self.simulation._bug2_step(
            ant, *self.simulation.nest,
            allow_wobble=True,
            follow_return_pheromone=True
        )
        heading_change = abs(
            (ant.heading - heading_before_wall_step + math.pi)
            % (2.0 * math.pi) - math.pi
        )

        self.assertLess(heading_change, math.radians(60.0))
        self.assertFalse(obstacle.check_collision_point(ant.x, ant.y))


class ExperimentOutputTests(unittest.TestCase):
    """验证批量实验输出记录了实际使用的配置元数据。"""

    def test_summary_records_configuration_used_by_run(self):
        """摘要 CSV 应包含随机种子及 CLI 覆盖后的配置 JSON。"""
        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / 'summary.csv'
            run_experiments([
                '--runs', '1',
                '--steps', '2',
                '--seed', '14',
                '--ants', '8',
                '--navigators', '2',
                '--navigator-pheromone-radius', '45',
                '--transporter-pheromone-radius', '18',
                '--output', str(output_path)
            ])

            with output_path.open(newline='', encoding='utf-8') as output_file:
                row = next(csv.DictReader(output_file))

            config = json.loads(row['configuration_json'])
            self.assertEqual(row['seed'], '14')
            self.assertEqual(config['num_ants'], 8)
            self.assertEqual(config['num_navigators'], 2)
            self.assertEqual(config['navigator_return_pheromone_radius'], 45.0)
            self.assertEqual(config['transporter_pheromone_radius'], 18.0)


if __name__ == '__main__':
    unittest.main()
