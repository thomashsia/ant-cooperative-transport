# obstacle.py

class Obstacle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def check_collision_point(self, px, py):
        """检查某个点（如蚂蚁坐标）是否在障碍物内部"""
        half_w = self.width / 2.0
        half_h = self.height / 2.0
        return (self.x - half_w <= px <= self.x + half_w) and \
               (self.y - half_h <= py <= self.y + half_h)

    def clamp_position(self, px, py):
        """
        如果点 (px, py) 在障碍物内部，将其投影到离它最近的障碍物外部边缘
        """
        half_w = self.width / 2.0
        half_h = self.height / 2.0

        min_x, max_x = self.x - half_w, self.x + half_w
        min_y, max_y = self.y - half_h, self.y + half_h

        if min_x <= px <= max_x and min_y <= py <= max_y:
            d_left = px - min_x
            d_right = max_x - px
            d_bottom = py - min_y
            d_top = max_y - py

            min_d = min(d_left, d_right, d_bottom, d_top)
            if min_d == d_left:
                return min_x - 0.1, py
            elif min_d == d_right:
                return max_x + 0.1, py
            elif min_d == d_bottom:
                return px, min_y - 0.1
            else:
                return px, max_y + 0.1
        return px, py

    def resolve_food_collision(self, food):
        """食物刚体与矩形障碍物的碰撞及切线滑移响应"""
        half_w = self.width / 2.0
        half_h = self.height / 2.0

        fx, fy = food.x, food.y
        fw, fh = food.width / 2.0, food.height / 2.0

        overlap_x = (fw + half_w) - abs(fx - self.x)
        overlap_y = (fh + half_h) - abs(fy - self.y)

        if overlap_x > 0 and overlap_y > 0:
            if overlap_x < overlap_y:
                if fx < self.x:
                    food.x -= overlap_x
                else:
                    food.x += overlap_x
                food.vx = -food.vx * 0.2
            else:
                if fy < self.y:
                    food.y -= overlap_y
                else:
                    food.y += overlap_y
                food.vy = -food.vy * 0.2

    def segment_intersects(self, x1, y1, x2, y2):
        """判断线段 (x1,y1)-(x2,y2) 是否与矩形相交（用于 Bug2 的“视线”检测）"""
        half_w = self.width / 2.0
        half_h = self.height / 2.0
        min_x, max_x = self.x - half_w, self.x + half_w
        min_y, max_y = self.y - half_h, self.y + half_h

        if max(x1, x2) < min_x or min(x1, x2) > max_x:
            return False
        if max(y1, y2) < min_y or min(y1, y2) > max_y:
            return False

        if min_x <= x1 <= max_x and min_y <= y1 <= max_y:
            return True
        if min_x <= x2 <= max_x and min_y <= y2 <= max_y:
            return True

        def seg_seg_intersect(ax, ay, bx, by, cx, cy, dx, dy):
            def cross(ox, oy, px, py, qx, qy):
                return (px - ox) * (qy - oy) - (py - oy) * (qx - ox)
            d1 = cross(cx, cy, dx, dy, ax, ay)
            d2 = cross(cx, cy, dx, dy, bx, by)
            d3 = cross(ax, ay, bx, by, cx, cy)
            d4 = cross(ax, ay, bx, by, dx, dy)
            if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
               ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
                return True
            return False

        edges = [
            (min_x, min_y, max_x, min_y),
            (max_x, min_y, max_x, max_y),
            (max_x, max_y, min_x, max_y),
            (min_x, max_y, min_x, min_y),
        ]
        for ex1, ey1, ex2, ey2 in edges:
            if seg_seg_intersect(x1, y1, x2, y2, ex1, ey1, ex2, ey2):
                return True
        return False

    def boundary_tangent(self, px, py, direction=1):
        """
        返回点 (px,py) 在矩形边界上应沿哪个方向绕行。
        direction = +1 → 逆时针
        direction = -1 → 顺时针
        """
        half_w = self.width / 2.0
        half_h = self.height / 2.0
        min_x, max_x = self.x - half_w, self.x + half_w
        min_y, max_y = self.y - half_h, self.y + half_h

        d_left = abs(px - min_x)
        d_right = abs(px - max_x)
        d_bottom = abs(py - min_y)
        d_top = abs(py - max_y)
        nearest = min(d_left, d_right, d_bottom, d_top)

        if nearest == d_left:
            base = (0, 1)
        elif nearest == d_right:
            base = (0, -1)
        elif nearest == d_bottom:
            base = (-1, 0)
        else:
            base = (1, 0)

        if direction >= 0:
            return base
        else:
            return (-base[0], -base[1])