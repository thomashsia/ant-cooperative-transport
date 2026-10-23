"""协同搬运仿真与实验入口共用的参数配置及校验。"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SimulationConfig:
    """集中保存场景、种群、物理和信息素行为参数。

    ``transport_simulation.py`` 消费配置创建仿真；动画及批量实验入口可传入
    配置覆盖运行参数。不可变数据类保证创建后不会被仿真悄悄改写。
    """

    # 场景与蚁群规模：巢穴/食物坐标均使用场地坐标系。
    width: int = 150
    height: int = 150
    num_ants: int = 30
    num_navigators: int = 5
    ant_speed: float = 2.0
    nest_position: tuple[float, float] = (40.0, 40.0)
    food_position: tuple[float, float] = (110.0, 140.0)

    # 导航蚁任务切换阈值：发现食物及返巢完成时分别按中心距离判定。
    navigator_food_detection_radius: float = 2.0
    navigator_nest_arrival_radius: float = 2.0

    # 食物刚体的外形尺寸及质量。
    food_width: float = 6.0
    food_height: float = 6.0
    food_mass: float = 3.0

    # 信息素挥发和不同角色的感知距离。
    pheromone_evaporation_rate: float = 0.05
    navigator_return_pheromone_radius: float = 60.0
    transporter_pheromone_radius: float = 20.0
    pheromone_signal_threshold: float = 0.02

    # 局部 PCA 和合向量方向判定的最小数据量/置信度。
    tangent_min_particle_count: int = 4
    tangent_min_confidence: float = 0.55
    resultant_min_confidence: float = 0.15

    # 导航轨迹融合、返巢进展及模糊信号时的社会方向上限。
    navigator_trail_blend: float = 0.35
    navigator_min_nest_progress: float = 0.2
    ambiguous_pheromone_max_social_weight: float = 0.35

    # 信息素信号完全消失后，搬运蚁先保持最后的跟随方向，并在该时长（仿真秒）内
    # 逐步把社会共识权重提高到无信号时的全额（0.6）。
    signal_loss_fade_time: float = 3.0

    # 返巢导航蚁的 CRW 权重（0 表示关闭）及转角标准差（弧度）。
    navigator_return_crw_weight: float = 0.2
    navigator_return_turn_sigma: float = 0.35

    def __post_init__(self):
        """由 dataclass 初始化自动调用，拒绝越界或不合法的参数组合。"""
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Simulation width and height must be positive.")
        if self.num_ants < 1:
            raise ValueError("Simulation must contain at least one ant.")
        if not 0 <= self.num_navigators <= self.num_ants:
            raise ValueError("Navigator count must be between zero and ant count.")
        if self.ant_speed <= 0:
            raise ValueError("Ant speed must be positive.")
        if (
            self.navigator_food_detection_radius <= 0
            or self.navigator_nest_arrival_radius <= 0
        ):
            raise ValueError("Navigator food and nest radii must be positive.")
        if self.food_width <= 0 or self.food_height <= 0 or self.food_mass <= 0:
            raise ValueError("Food dimensions and mass must be positive.")
        if self.pheromone_evaporation_rate < 0:
            raise ValueError("Pheromone evaporation rate cannot be negative.")
        if (
            self.navigator_return_pheromone_radius <= 0
            or self.transporter_pheromone_radius <= 0
        ):
            raise ValueError("Pheromone perception radii must be positive.")
        if self.pheromone_signal_threshold < 0:
            raise ValueError("Pheromone signal threshold cannot be negative.")
        if self.tangent_min_particle_count < 1:
            raise ValueError("Tangent estimation requires at least one particle.")
        if not 0 <= self.tangent_min_confidence <= 1:
            raise ValueError("Tangent confidence must be between zero and one.")
        if not 0 <= self.resultant_min_confidence <= 1:
            raise ValueError("Resultant confidence must be between zero and one.")
        if not 0 <= self.navigator_trail_blend <= 1:
            raise ValueError("Navigator trail blend must be between zero and one.")
        if not 0 <= self.navigator_min_nest_progress <= 1:
            raise ValueError("Minimum nest progress must be between zero and one.")
        if not 0 <= self.ambiguous_pheromone_max_social_weight <= 1:
            raise ValueError(
                "Ambiguous-pheromone social weight must be between zero and one."
            )
        if not 0 <= self.navigator_return_crw_weight < 1:
            raise ValueError("Return CRW weight must be in [0, 1).")
        if self.navigator_return_turn_sigma < 0:
            raise ValueError("Return CRW turn sigma cannot be negative.")
        if self.signal_loss_fade_time <= 0:
            raise ValueError("Signal-loss fade time must be positive.")
        if len(self.nest_position) != 2 or len(self.food_position) != 2:
            raise ValueError("Nest and food positions must each have two coordinates.")
        coordinates = (*self.nest_position, *self.food_position)
        if any(not math.isfinite(value) for value in coordinates):
            raise ValueError("Nest and food coordinates must be finite.")
