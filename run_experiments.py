"""无窗口运行协同搬运仿真，并将每次运行的摘要写入 CSV。"""

import argparse
import csv
import math
import os
import random
from contextlib import ExitStack

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
    """运行指定数量的独立仿真，并输出每组运行的最终状态。"""
    parser = argparse.ArgumentParser(
        description="Run headless ant cooperative-transport simulations."
    )
    parser.add_argument("--runs", type=int, default=1, help="number of simulation runs")
    parser.add_argument("--steps", type=int, default=600, help="updates per run")
    parser.add_argument("--seed", type=int, default=0, help="base random seed")
    parser.add_argument(
        "--output",
        default="experiment_results.csv",
        help="CSV output path (default: experiment_results.csv)"
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
    with ExitStack() as stack:
        csv_file = stack.enter_context(
            open(args.output, "w", newline="", encoding="utf-8")
        )
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        diagnostics_writer = None
        if args.diagnostics_output:
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
                num_ants=35,
                width=150,
                height=150,
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
