from dataclasses import dataclass

@dataclass
class Point:
    x: float
    y: float

def get_point(x, y):
    return Point(x, y)

class CurveCalculator:
    def __init__(self):
        self.ab_curve_scaling = 0.5

    def get_target_k(self, prev_point, target_point, next_point):
        k = 0
        if ((prev_point.y < target_point.y < next_point.y) or
            (prev_point.y > target_point.y > next_point.y)):
            v1 = get_point(prev_point.x - target_point.x, prev_point.y - target_point.y)
            v2 = get_point(next_point.x - target_point.x, next_point.y - target_point.y)
            
            if v2.x != 0 and abs(v1.x / v2.x - v1.y / v2.y) < 1e-6:  # Use epsilon for float comparison
                k = v1.y / v1.x if v1.x != 0 else 0
            else:
                tmp_point_x = (prev_point.x + target_point.x) / 2.0 - (target_point.x + next_point.x) / 2.0
                tmp_point_y = (prev_point.y + target_point.y) / 2.0 - (target_point.y + next_point.y) / 2.0
                k = tmp_point_y / tmp_point_x if tmp_point_x != 0 else 0
        return k

    def get_control_point(self, prev_point, target_point, next_point):
        left_cp = [0, 0]
        right_cp = [0, 0]
        k = 0
        if prev_point and next_point:
            k = self.get_target_k(prev_point, target_point, next_point)
        
        if prev_point:
            ab_interval = -(target_point.x - prev_point.x)
            left_cp = [
                ab_interval * self.ab_curve_scaling,
                k * (ab_interval * self.ab_curve_scaling)
            ]
        
        if next_point:
            ab_interval = next_point.x - target_point.x
            right_cp = [
                ab_interval * self.ab_curve_scaling,
                k * (ab_interval * self.ab_curve_scaling)
            ]
        
        return [
            [round(left_cp[0], 1), round(left_cp[1], 1)],
            [round(right_cp[0], 1), round(right_cp[1], 1)]
        ]