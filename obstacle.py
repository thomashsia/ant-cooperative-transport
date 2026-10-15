"""凸多边形障碍物的碰撞检测、位置修正和刚体碰撞响应。"""

import math


class Obstacle:
    """表示实体凸多边形障碍物；默认构造器仍创建轴对齐矩形。"""

    def __init__(self, x, y, width, height):
        """使用中心坐标及宽高创建矩形障碍物。"""
        half_width = width / 2.0
        half_height = height / 2.0
        self._set_vertices([
            (x - half_width, y - half_height),
            (x + half_width, y - half_height),
            (x + half_width, y + half_height),
            (x - half_width, y + half_height),
        ])

    @classmethod
    def from_vertices(cls, vertices):
        """使用按边界顺序给出的顶点创建凸多边形障碍物。"""
        obstacle = cls.__new__(cls)
        obstacle._set_vertices(vertices)
        return obstacle

    def _set_vertices(self, vertices):
        """验证凸多边形顶点，并计算兼容旧接口的包围盒和中心。"""
        points = [(float(x), float(y)) for x, y in vertices]
        if len(points) < 3:
            raise ValueError("A polygon obstacle requires at least three vertices.")
        if any(not math.isfinite(value) for point in points for value in point):
            raise ValueError("Polygon vertices must contain only finite coordinates.")
        if any(
            self._distance_squared(points[i], points[(i + 1) % len(points)]) <= 1e-18
            for i in range(len(points))
        ):
            raise ValueError("Polygon vertices must not repeat adjacent points.")

        signed_area_twice = sum(
            points[i][0] * points[(i + 1) % len(points)][1]
            - points[(i + 1) % len(points)][0] * points[i][1]
            for i in range(len(points))
        )
        if abs(signed_area_twice) <= 1e-12:
            raise ValueError("Polygon vertices must enclose a non-zero area.")

        turn_sign = 0
        for i in range(len(points)):
            a = points[i]
            b = points[(i + 1) % len(points)]
            c = points[(i + 2) % len(points)]
            cross = self._cross(a, b, c)
            if abs(cross) <= 1e-10:
                continue
            current_sign = 1 if cross > 0 else -1
            if turn_sign and current_sign != turn_sign:
                raise ValueError("Polygon obstacle vertices must be convex and ordered.")
            turn_sign = current_sign

        if turn_sign == 0:
            raise ValueError("Polygon vertices must enclose a non-zero area.")

        for i, start in enumerate(points):
            end = points[(i + 1) % len(points)]
            if any(
                self._cross(start, end, point) * turn_sign < -1e-10
                for point in points
            ):
                raise ValueError("Polygon obstacle vertices must be convex and ordered.")

        self.vertices = points
        self._orientation = 1 if signed_area_twice > 0 else -1
        min_x = min(point[0] for point in points)
        max_x = max(point[0] for point in points)
        min_y = min(point[1] for point in points)
        max_y = max(point[1] for point in points)
        self.x = sum(point[0] for point in points) / len(points)
        self.y = sum(point[1] for point in points) / len(points)
        self.width = max_x - min_x
        self.height = max_y - min_y

    @staticmethod
    def _distance_squared(a, b):
        """返回两个二维点之间的距离平方。"""
        return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

    @staticmethod
    def _cross(a, b, c):
        """返回向量 AB 与 AC 的二维叉积。"""
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    @staticmethod
    def _closest_point_on_segment(px, py, a, b):
        """计算点在有限线段上的最近投影点。"""
        dx, dy = b[0] - a[0], b[1] - a[1]
        length_squared = dx * dx + dy * dy
        t = ((px - a[0]) * dx + (py - a[1]) * dy) / length_squared
        t = max(0.0, min(1.0, t))
        return a[0] + t * dx, a[1] + t * dy

    def _point_inside(self, px, py):
        """使用凸多边形各边的半平面关系判断点是否在内部或边界。"""
        point = (px, py)
        for i, start in enumerate(self.vertices):
            end = self.vertices[(i + 1) % len(self.vertices)]
            side = self._cross(start, end, point) * self._orientation
            if side < -1e-9:
                return False
        return True

    def check_collision_point(self, px, py):
        """检查点是否位于凸多边形内部或边界。"""
        return self._point_inside(px, py)

    def clamp_position(self, px, py):
        """若点位于障碍物内，将其沿最近边的外法线推出多边形。"""
        if not self._point_inside(px, py):
            return px, py

        nearest_point = None
        nearest_distance_squared = float('inf')
        nearest_edge = None
        for i, start in enumerate(self.vertices):
            end = self.vertices[(i + 1) % len(self.vertices)]
            projected = self._closest_point_on_segment(px, py, start, end)
            distance_squared = self._distance_squared((px, py), projected)
            if distance_squared < nearest_distance_squared:
                nearest_distance_squared = distance_squared
                nearest_point = projected
                nearest_edge = (start, end)

        start, end = nearest_edge
        dx, dy = end[0] - start[0], end[1] - start[1]
        edge_length = math.hypot(dx, dy)
        outward_x = self._orientation * dy / edge_length
        outward_y = -self._orientation * dx / edge_length
        return (
            nearest_point[0] + outward_x * 0.1,
            nearest_point[1] + outward_y * 0.1,
        )

    @staticmethod
    def _segments_intersect(a, b, c, d):
        """判断两条线段是否相交，包括端点接触和共线重叠。"""
        epsilon = 1e-9

        def on_segment(p, q, r):
            return (
                min(p[0], r[0]) - epsilon <= q[0] <= max(p[0], r[0]) + epsilon
                and min(p[1], r[1]) - epsilon <= q[1] <= max(p[1], r[1]) + epsilon
            )

        o1 = Obstacle._cross(a, b, c)
        o2 = Obstacle._cross(a, b, d)
        o3 = Obstacle._cross(c, d, a)
        o4 = Obstacle._cross(c, d, b)

        if ((o1 > epsilon and o2 < -epsilon) or (o1 < -epsilon and o2 > epsilon)) \
                and ((o3 > epsilon and o4 < -epsilon) or (o3 < -epsilon and o4 > epsilon)):
            return True
        return (
            (abs(o1) <= epsilon and on_segment(a, c, b))
            or (abs(o2) <= epsilon and on_segment(a, d, b))
            or (abs(o3) <= epsilon and on_segment(c, a, d))
            or (abs(o4) <= epsilon and on_segment(c, b, d))
        )

    def segment_intersects(self, x1, y1, x2, y2):
        """判断线段是否穿过或接触障碍物，可用于视线遮挡检测。"""
        start, end = (x1, y1), (x2, y2)
        if self._point_inside(x1, y1) or self._point_inside(x2, y2):
            return True

        for i, edge_start in enumerate(self.vertices):
            edge_end = self.vertices[(i + 1) % len(self.vertices)]
            if self._segments_intersect(start, end, edge_start, edge_end):
                return True
        return False

    def boundary_tangent(self, px, py, direction=1):
        """返回最近边界的单位切线；正方向表示沿顺时针方向行进。"""
        nearest_edge = None
        nearest_distance_squared = float('inf')
        for i, start in enumerate(self.vertices):
            end = self.vertices[(i + 1) % len(self.vertices)]
            projected = self._closest_point_on_segment(px, py, start, end)
            distance_squared = self._distance_squared((px, py), projected)
            if distance_squared < nearest_distance_squared:
                nearest_distance_squared = distance_squared
                nearest_edge = (start, end)

        start, end = nearest_edge
        dx, dy = end[0] - start[0], end[1] - start[1]
        edge_length = math.hypot(dx, dy)
        ccw_sign = self._orientation
        clockwise_sign = -ccw_sign
        sign = clockwise_sign if direction >= 0 else -clockwise_sign
        return sign * dx / edge_length, sign * dy / edge_length

    def food_contact_normal(self, food, clearance=0.0):
        """返回障碍物指向食物的接触法线；分离轴间隙超过 clearance 时返回 None。"""
        food_vertices = food.get_corners()
        axes = []
        for polygon in (self.vertices, food_vertices):
            for i, start in enumerate(polygon):
                end = polygon[(i + 1) % len(polygon)]
                dx, dy = end[0] - start[0], end[1] - start[1]
                edge_length = math.hypot(dx, dy)
                axes.append((-dy / edge_length, dx / edge_length))

        best_gap = float('-inf')
        contact_normal = None
        for axis_x, axis_y in axes:
            obstacle_projection = [
                point[0] * axis_x + point[1] * axis_y
                for point in self.vertices
            ]
            food_projection = [
                point[0] * axis_x + point[1] * axis_y
                for point in food_vertices
            ]
            obstacle_min = min(obstacle_projection)
            obstacle_max = max(obstacle_projection)
            food_min = min(food_projection)
            food_max = max(food_projection)

            positive_gap = food_min - obstacle_max
            negative_gap = obstacle_min - food_max
            gap = max(positive_gap, negative_gap)

            if gap > 0:
                normal = (axis_x, axis_y) if positive_gap >= negative_gap \
                    else (-axis_x, -axis_y)
            elif obstacle_max - food_min <= food_max - obstacle_min:
                normal = (axis_x, axis_y)
            else:
                normal = (-axis_x, -axis_y)

            if gap > best_gap:
                best_gap = gap
                contact_normal = normal

        if best_gap <= clearance:
            return contact_normal
        return None

    def resolve_food_collision(self, food):
        """用分离轴定理检测旋转矩形食物，并沿最小穿透方向推出。"""
        food_vertices = food.get_corners()
        axes = []
        for polygon in (self.vertices, food_vertices):
            for i, start in enumerate(polygon):
                end = polygon[(i + 1) % len(polygon)]
                dx, dy = end[0] - start[0], end[1] - start[1]
                length = math.hypot(dx, dy)
                axes.append((-dy / length, dx / length))

        minimum_overlap = float('inf')
        collision_normal = None
        for axis_x, axis_y in axes:
            obstacle_projection = [
                point[0] * axis_x + point[1] * axis_y
                for point in self.vertices
            ]
            food_projection = [
                point[0] * axis_x + point[1] * axis_y
                for point in food_vertices
            ]
            positive_overlap = max(obstacle_projection) - min(food_projection)
            negative_overlap = max(food_projection) - min(obstacle_projection)
            if positive_overlap <= 0 or negative_overlap <= 0:
                return
            if positive_overlap < negative_overlap:
                overlap = positive_overlap
            else:
                overlap = negative_overlap
                axis_x, axis_y = -axis_x, -axis_y
            if overlap < minimum_overlap:
                minimum_overlap = overlap
                collision_normal = (axis_x, axis_y)

        normal_x, normal_y = collision_normal
        food.x += normal_x * (minimum_overlap + 0.1)
        food.y += normal_y * (minimum_overlap + 0.1)

        velocity_into_obstacle = food.vx * normal_x + food.vy * normal_y
        if velocity_into_obstacle < 0:
            restitution = 0.2
            food.vx -= (1.0 + restitution) * velocity_into_obstacle * normal_x
            food.vy -= (1.0 + restitution) * velocity_into_obstacle * normal_y
