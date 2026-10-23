"""凸多边形障碍物的几何查询与食物碰撞响应。

主仿真在 ``ObstacleTransportSimulation`` 中创建障碍，并将这里的点碰撞、
视线相交、边界切线、接触法线及食物推出接口用于导航和搬运。
"""

import math


class Obstacle:
    """表示实体凸多边形障碍物；默认构造器仍创建轴对齐矩形。

    顶点顺序可以为顺时针或逆时针，但必须构成凸多边形；内部统一保存方向符号，
    供半平面点测和外法线计算使用。
    """

    def __init__(self, x, y, width, height):
        """使用中心坐标及宽高创建矩形；由主仿真初始化调用。"""
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
        """使用按边界顺序给出的顶点创建凸多边形。

        主仿真用该工厂创建非矩形障碍；委托 ``_set_vertices`` 做顶点校验和派生量计算。
        """
        obstacle = cls.__new__(cls)
        obstacle._set_vertices(vertices)
        return obstacle

    def _set_vertices(self, vertices):
        """验证凸多边形顶点并计算包围盒、顶点均值中心和绕序。

        由两个构造入口调用。多边形面积用鞋带公式的两倍判断是否退化；
        相邻边叉积的符号一致且所有点位于每条有向边同侧，构成凸性校验。
        """
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
        """返回两点欧氏距离的平方；供边长退化和最近边比较复用。"""
        return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

    @staticmethod
    def _cross(a, b, c):
        """返回 ``AB × AC`` 的标量值；由凸性、点内测试和线段相交计算复用。"""
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    @staticmethod
    def _closest_point_on_segment(px, py, a, b):
        """计算点在线段上的最近投影点；由 ``clamp_position`` 和 ``boundary_tangent`` 调用。

        参数 ``t`` 是投影在边向量上的归一化参数，限制到 [0, 1] 后即排除线段延长线。
        """
        dx, dy = b[0] - a[0], b[1] - a[1]
        length_squared = dx * dx + dy * dy
        t = ((px - a[0]) * dx + (py - a[1]) * dy) / length_squared
        t = max(0.0, min(1.0, t))
        return a[0] + t * dx, a[1] + t * dy

    def _point_inside(self, px, py):
        """使用凸多边形各边的半平面关系判断点是否在内部或边界。

        由公开点碰撞、位置推出以及线段遮挡接口复用；乘以绕序符号后，
        顺/逆时针顶点都统一为同一侧判定。
        """
        point = (px, py)
        for i, start in enumerate(self.vertices):
            end = self.vertices[(i + 1) % len(self.vertices)]
            side = self._cross(start, end, point) * self._orientation
            if side < -1e-9:
                return False
        return True

    def check_collision_point(self, px, py):
        """检查点是否位于凸多边形内部或边界。

        由蚂蚁单步移动和 ``ObstacleTransportSimulation._bug2_step`` 检查碰撞。
        """
        return self._point_inside(px, py)

    def clamp_position(self, px, py):
        """若点在障碍内，将其沿最近边外法线推出多边形。

        由主仿真 Bug2 入障碍时及蚂蚁位置约束逻辑调用。边向量旋转 90 度并结合
        多边形绕序得到外法线，额外偏移 0.1 以避免结果仍落在边界容差内。
        """
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
        """判断两条线段是否相交，包括端点接触和共线重叠。

        由 ``segment_intersects`` 调用。叉积符号用于严格交叉判断，包围区间测试
        补充处理端点落在线段上及共线情形。
        """
        epsilon = 1e-9

        def on_segment(p, q, r):
            """在叉积近似为零时，检查 q 是否落在 p-r 的包围范围内。"""
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
        """判断线段是否穿过或接触障碍物，可用于视线遮挡检测。

        主仿真用它检查导航视线和信息素可见性；内部先判端点在内，再检查线段与各边。
        """
        start, end = (x1, y1), (x2, y2)
        if self._point_inside(x1, y1) or self._point_inside(x2, y2):
            return True

        for i, edge_start in enumerate(self.vertices):
            edge_end = self.vertices[(i + 1) % len(self.vertices)]
            if self._segments_intersect(start, end, edge_start, edge_end):
                return True
        return False

    def boundary_tangent(self, px, py, direction=1):
        """返回最近边界的单位切线；正方向表示顺时针行进。

        由主仿真 Bug2 wall 阶段调用。最近点距离用于选择边，边向量归一化后按
        多边形绕序及 ``direction`` 确定切线朝向。
        """
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
        """返回障碍物指向食物的接触法线，间隙超过阈值则返回 ``None``。

        由主仿真搬运更新调用。依据分离轴定理，在两多边形各边法线轴上投影顶点；
        最大分离间隙不超过 ``clearance`` 时视为接触，并返回将食物推离障碍的方向。
        """
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
        """用分离轴定理检测食物与障碍的碰撞，并沿最小穿透方向推出。

        由主仿真每帧食物动力学积分后调用。投影区间任一轴分离即无碰撞；
        否则取最小重叠轴修正位置，并仅反射指向障碍内部的速度分量。
        """
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
