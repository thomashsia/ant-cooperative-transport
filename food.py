# food.py
import math

class Food:
    def __init__(self, x, y, width=16.0, height=12.0, mass=5.0):
        self.x = x
        self.y = y
        self.theta = 0.0          # 旋转角度 (弧度)
        self.width = width
        self.height = height
        self.mass = mass
        
        self.inertia = (1.0 / 12.0) * mass * (width**2 + height**2)
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        
        self.linear_damping = 1.5 #0.4
        self.angular_damping = 2.0 #0.6

        # 仅保留基于周长生成的密集内层搬运槽位
        self.slots = self._generate_perimeter_slots(width, height, spacing=3.0)

    def _generate_perimeter_slots(self, w, h, spacing=3.5):
        slots = []
        hw, hh = w / 2.0, h / 2.0
        
        x = -hw
        while x <= hw:
            slots.append({'local_pos': (x, hh), 'occupied_by': None})
            slots.append({'local_pos': (x, -hh), 'occupied_by': None})
            x += spacing
            
        y = -hh + spacing
        while y < hh:
            slots.append({'local_pos': (-hw, y), 'occupied_by': None})
            slots.append({'local_pos': (hw, y), 'occupied_by': None})
            y += spacing
            
        return slots

    def reset_slots(self):
        """每帧清空槽位占用"""
        for slot in self.slots:
            slot['occupied_by'] = None

    def claim_slot(self, ant_x, ant_y, ant_id):
        """尝试抢占搬运槽位"""
        best_slot = None
        min_dist = float('inf')
        c, s = math.cos(self.theta), math.sin(self.theta)
        
        for slot in self.slots:
            if slot['occupied_by'] is None:
                lx, ly = slot['local_pos']
                wx = self.x + (lx * c - ly * s)
                wy = self.y + (lx * s + ly * c)
                dist = math.hypot(wx - ant_x, wy - ant_y)
                if dist < min_dist:
                    min_dist = dist
                    best_slot = slot

        if best_slot and min_dist <= 10.0:
            best_slot['occupied_by'] = ant_id
            return best_slot
        return None

    def update_dynamics(self, forces, torques, dt=0.05, width=100, height=100):
        total_fx = sum(f[0] for f in forces)
        total_fy = sum(f[1] for f in forces)
        total_torque = sum(torques)
        
        ax = (total_fx - self.linear_damping * self.vx) / self.mass
        ay = (total_fy - self.linear_damping * self.vy) / self.mass
        
        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        alpha = (total_torque - self.angular_damping * self.omega) / self.inertia
        self.omega += alpha * dt
        self.theta += self.omega * dt
        
        margin = max(self.width, self.height)
        self.x = max(margin, min(width - margin, self.x))
        self.y = max(margin, min(height - margin, self.y))

    def get_corners(self):
        hw, hh = self.width / 2.0, self.height / 2.0
        local_corners = [
            (-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)
        ]
        corners = []
        c, s = math.cos(self.theta), math.sin(self.theta)
        for lx, ly in local_corners:
            rx = lx * c - ly * s
            ry = lx * s + ly * c
            corners.append((self.x + rx, self.y + ry))
        return corners