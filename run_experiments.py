"""无窗口批量运行协同搬运仿真并将摘要/诊断写入 CSV。

命令行入口创建 ``ObstacleTransportSimulation`` 并重复调用其 ``update``；
不导入绘图组件，可用于自动化实验和回归验证。
"""

import argparse
import csv
from dataclasses import asdict
import json
import math
import os
import random
from contextlib import ExitStack
from pathlib import Path

from simulation_config import SimulationConfig
from transport_simulation import ObstacleTransportSimulation


DIAGNOSTIC_FIELDS = [
    "run",
    "seed",
    "step",
    "ant_id",
    "ant_x",
    "ant_y",
    "ant_heading",
    "bug_phase",
    "bug_direction",
    "pheromone_weight_sum",
    "pheromone_vector_x",
    "pheromone_vector_y",
    "pheromone_vector_magnitude",
    "pheromone_direction_confidence",
    "pheromone_direction",
    "pheromone_tangent_x",
    "pheromone_tangent_y",
    "pheromone_tangent_confidence",
    "pheromone_tangent_particle_count",
    "direction_source",
    "group_direction_confidence",
    "fallback_social_weight",
    "has_pheromone_signal",
    "individual_force_x",
    "individual_force_y",
    "food_x",
    "food_y",
    "food_theta",
    "food_step_dx",
    "food_step_dy",
    "food_speed",
    "food_omega",
    "contact_obstacle_count",
    "requested_force_x",
    "requested_force_y",
    "requested_force_magnitude",
    "projected_force_x",
    "projected_force_y",
    "projected_force_magnitude",
    "requested_torque",
    "applied_torque",
]


def main(argv=None):
    """解析实验参数，执行多组仿真并流式写出 CSV 结果。

    由命令行 ``__main__`` 调用，也可由 ``tests/test_simulation.py`` 传入参数列表。
    每组使用 ``base_seed + run_index`` 固定随机数种子；摘要 CSV 每次运行写一行，
    可选诊断 CSV 则每帧每只搬运蚂蚁记录一行。食物位移由初末质心的欧氏距离计算。
    """
    parser = argparse.ArgumentParser(
        description="Run headless ant cooperative-transport simulations."
    )
    parser.add_argument("--runs", type=int, default=1, help="number of simulation runs")
    parser.add_argument("--steps", type=int, default=600, help="updates per run")
    parser.add_argument("--seed", type=int, default=0, help="base random seed")
    parser.add_argument("--ants", type=int, default=35, help="total ant count")
    parser.add_argument("--navigators", type=int, default=5, help="navigator ant count")
    parser.add_argument("--width", type=int, default=150, help="simulation width")
    parser.add_argument("--height", type=int, default=150, help="simulation height")
    parser.add_argument(
        "--navigator-pheromone-radius",
        type=float,
        default=60.0,
        help="returning navigator pheromone perception radius"
    )
    parser.add_argument(
        "--transporter-pheromone-radius",
        type=float,
        default=20.0,
        help="transporter pheromone perception radius"
    )
    parser.add_argument(
        "--navigator-trail-blend",
        type=float,
        default=0.35,
        help="maximum returning-navigator trail heading blend"
    )
    parser.add_argument(
        "--ambiguous-pheromone-max-social-weight",
        type=float,
        default=0.35,
        help="maximum peer-heading weight when transporter pheromone direction is ambiguous"
    )
    parser.add_argument(
        "--output",
        default="results/experiment_results.csv",
        help="CSV output path (default: results/experiment_results.csv)"
    )
    parser.add_argument(
        "--diagnostics-output",
        help="optional per-step transporter and food diagnostics CSV"
    )
    args = parser.parse_args(argv)

    if args.runs < 1:
        parser.error("--runs must be at least 1")
    if args.steps < 1:
        parser.error("--steps must be at least 1")
    try:
        config = SimulationConfig(
            width=args.width,
            height=args.height,
            num_ants=args.ants,
            num_navigators=args.navigators,
            navigator_return_pheromone_radius=args.navigator_pheromone_radius,
            transporter_pheromone_radius=args.transporter_pheromone_radius,
            navigator_trail_blend=args.navigator_trail_blend,
            ambiguous_pheromone_max_social_weight=(
                args.ambiguous_pheromone_max_social_weight
            )
        )
    except ValueError as error:
        parser.error(str(error))
    configuration_json = json.dumps(
        asdict(config), sort_keys=True, separators=(",", ":")
    )
    if (
        args.diagnostics_output
        and os.path.normcase(os.path.abspath(args.output))
        == os.path.normcase(os.path.abspath(args.diagnostics_output))
    ):
        parser.error("--diagnostics-output must differ from --output")

    fieldnames = [
        "run",
        "seed",
        "steps",
        "configuration_json",
        "initial_food_x",
        "initial_food_y",
        "final_food_x",
        "final_food_y",
        "food_displacement",
        "final_food_theta",
        "active_transporters",
        "returning_navigators",
        "pheromone_particles"
    ]

    # 摘要与可选诊断均采用流式写入，长批次运行不会累积全部记录。
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        csv_file = stack.enter_context(
            open(args.output, "w", newline="", encoding="utf-8")
        )
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        diagnostics_writer = None
        if args.diagnostics_output:
            Path(args.diagnostics_output).parent.mkdir(
                parents=True, exist_ok=True
            )
            diagnostics_file = stack.enter_context(
                open(args.diagnostics_output, "w", newline="", encoding="utf-8")
            )
            diagnostics_writer = csv.DictWriter(
                diagnostics_file, fieldnames=DIAGNOSTIC_FIELDS
            )
            diagnostics_writer.writeheader()

        for run_index in range(args.runs):
            run_seed = args.seed + run_index
            random.seed(run_seed)
            simulation = ObstacleTransportSimulation(
                config=config,
                diagnostics_enabled=diagnostics_writer is not None
            )

            initial_food_x = simulation.food.x
            initial_food_y = simulation.food.y
            for step in range(args.steps):
                simulation.update()
                if diagnostics_writer:
                    for diagnostic in simulation.last_diagnostics:
                        diagnostics_writer.writerow({
                            "run": run_index + 1,
                            "seed": run_seed,
                            "step": step + 1,
                            **diagnostic,
                        })

            food_displacement = math.hypot(
                simulation.food.x - initial_food_x,
                simulation.food.y - initial_food_y
            )
            writer.writerow({
                "run": run_index + 1,
                "seed": run_seed,
                "steps": args.steps,
                "configuration_json": configuration_json,
                "initial_food_x": initial_food_x,
                "initial_food_y": initial_food_y,
                "final_food_x": simulation.food.x,
                "final_food_y": simulation.food.y,
                "food_displacement": food_displacement,
                "final_food_theta": simulation.food.theta,
                "active_transporters": sum(
                    1 for ant in simulation.ants
                    if ant.role == "transporter" and ant.state == "transporting"
                ),
                "returning_navigators": sum(
                    1 for ant in simulation.ants
                    if ant.role == "navigator" and ant.state == "returning"
                ),
                "pheromone_particles": len(simulation.pheromone_manager.particles)
            })

            print(
                f"Completed run {run_index + 1}/{args.runs} "
                f"(seed={run_seed}, steps={args.steps})"
            )

    print(f"Results written to {args.output}")
    if args.diagnostics_output:
        print(f"Diagnostics written to {args.diagnostics_output}")


if __name__ == "__main__":
    main()
