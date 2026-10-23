"""Matplotlib renderer 的状态绘制回归测试。"""

import unittest

import matplotlib
import matplotlib.pyplot as plt

matplotlib.use('Agg')

from transport_simulation import ObstacleTransportSimulation
from transport_visualization import TransportRenderer


class FoodTrajectoryTests(unittest.TestCase):
    """验证 renderer 绘制的是食物质心平移轨迹，而非旋转轨迹。"""

    def test_trajectory_tracks_centroid_translation_not_rotation(self):
        """只在质心位置变化时追加轨迹点，纯旋转不应新增轨迹点。"""
        simulation = ObstacleTransportSimulation(num_ants=8)
        renderer = TransportRenderer(simulation)
        self.addCleanup(plt.close, renderer.fig)
        initial_position = (simulation.food.x, simulation.food.y)

        simulation.food.x += 3.0
        simulation.food.y -= 2.0
        renderer.draw()

        simulation.food.theta = 0.5
        renderer.draw()
        trajectory_x, trajectory_y = renderer.food_trajectory_line.get_data()

        self.assertEqual(list(trajectory_x), [initial_position[0], simulation.food.x])
        self.assertEqual(list(trajectory_y), [initial_position[1], simulation.food.y])
        self.assertEqual(len(renderer.food_trajectory_x), 2)

    def test_force_arrows_show_each_transporting_ant_individual_force(self):
        """每只搬运蚁应有一个按方向来源分组的个体拉力箭头。"""
        simulation = ObstacleTransportSimulation(num_ants=8)
        renderer = TransportRenderer(simulation)
        self.addCleanup(plt.close, renderer.fig)
        transporters = [ant for ant in simulation.ants if ant.role == 'transporter']
        transporters[0].state = 'transporting'
        transporters[0].individual_force = (0.0, 2.0)
        transporters[0].force_direction_source = 'tangent'
        transporters[1].state = 'transporting'
        transporters[1].individual_force = (2.0, 0.0)
        transporters[1].force_direction_source = 'memory_social'

        renderer.draw()

        pheromone_arrows = renderer.force_quivers['pheromone']
        memory_arrows = renderer.force_quivers['memory']
        self.assertEqual(len(pheromone_arrows.get_offsets()), 1)
        self.assertEqual(list(pheromone_arrows.U), [0.0])
        self.assertEqual(list(pheromone_arrows.V), [2.0])
        self.assertEqual(len(memory_arrows.get_offsets()), 1)
        self.assertEqual(len(renderer.force_quivers['social'].get_offsets()), 0)


if __name__ == '__main__':
    unittest.main()
