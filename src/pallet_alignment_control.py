"""Fixed-target pallet alignment geometry. No ROS publishers or hardware access.

The pallet is observed while stopped, fixed in odom, and approached by
turn-drive-turn moves on odometry; each stop is followed by a new observation.
"""
import math
from statistics import median


def validate_fixed_target_config(cfg):
    for key in ('staging_target_gap', 'backoff_gap', 'max_move_distance', 'insertion_depth', 'target_position_spread',
                'target_yaw_spread_deg', 'face_lateral_tolerance', 'completion_face_lateral_tolerance',
                'completion_yaw_tolerance_deg', 'completion_gap_tolerance'):
        value = cfg.get(key)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if type(cfg.get('max_observations')) is not int or not 2 <= cfg['max_observations'] <= 10:
        raise ValueError('max_observations must be an integer from 2 to 10')
    if not (max(cfg['min_body_gap'], cfg['fork_projection'])
            < cfg['target_body_gap']-cfg['completion_gap_tolerance']
            < cfg['target_body_gap']+cfg['completion_gap_tolerance']
            < cfg['staging_target_gap'] < cfg['backoff_gap']):
        raise ValueError('Required: min gap/fork < completion band < staging gap < backoff gap')
    for key in ('face_lateral_tolerance', 'yaw_tolerance_deg', 'gap_tolerance'):
        if cfg[key] > cfg['completion_'+key]:
            raise ValueError(f'{key} must not exceed completion_{key}')
    # Aiming at the opening center from the centerline band must not leave a heading error to re-plan.
    nearest = cfg['target_body_gap']-cfg['completion_gap_tolerance']+cfg['body_front_x']
    if math.degrees(math.atan2(cfg['lateral_tolerance'], nearest)) >= cfg['yaw_tolerance_deg']:
        raise ValueError('lateral_tolerance is too large for yaw_tolerance_deg')
    if not cfg['backoff_gap']-cfg['target_body_gap'] < cfg['max_move_distance'] <= .30:
        raise ValueError('max_move_distance must cover backoff and be <= 0.30m')


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pallet_in_odom(target, odom, cfg):
    """(x, y, yaw) of the opening center and inward normal in odom."""
    x, y, yaw, _ = target
    distance = cfg['body_front_x']+cfg['target_body_gap']
    cx, cy = x+distance*math.cos(yaw), y+distance*math.sin(yaw)
    ox, oy, heading = odom[:3]
    c, s = math.cos(heading), math.sin(heading)
    return ox+c*cx-s*cy, oy+s*cx+c*cy, wrap_angle(heading+yaw)


def fixed_pallet(samples, cfg):
    """Median of stationary samples; None unless every sample agrees."""
    px, py = median(v[0] for v in samples), median(v[1] for v in samples)
    anchor = samples[-1][2]
    yaw = wrap_angle(anchor+median(wrap_angle(v[2]-anchor) for v in samples))
    if any(math.hypot(v[0]-px, v[1]-py) > cfg['target_position_spread']
           or abs(wrap_angle(v[2]-yaw)) > math.radians(cfg['target_yaw_spread_deg']) for v in samples):
        return None
    return px, py, yaw


def alignment_errors(pallet, odom, cfg):
    """(body gap, base_link offset from the entry centerline, heading error, face offset).

    Face offset: where the robot's forward axis (fork line) meets the pallet face,
    measured from the opening center. Positive values are to the left.
    """
    px, py, yaw = pallet
    nx, ny = math.cos(yaw), math.sin(yaw)
    dx, dy = odom[0]-px, odom[1]-py
    gap, cross, heading = -(dx*nx+dy*ny)-cfg['body_front_x'], dy*nx-dx*ny, wrap_angle(odom[2]-yaw)
    return gap, cross, heading, cross+(gap+cfg['body_front_x'])*math.tan(heading)


def aligned(errors, cfg):
    gap, _, heading, face = errors
    return (abs(gap-cfg['target_body_gap']) <= cfg['completion_gap_tolerance']
            and abs(face) <= cfg['completion_face_lateral_tolerance']
            and abs(heading) <= math.radians(cfg['completion_yaw_tolerance_deg']))


def plan_moves(pallet, odom, cfg):
    """[('turn', odom heading) | ('drive', signed meters along the current heading)]."""
    gap, cross, heading, face = alignment_errors(pallet, odom, cfg)
    px, py, yaw = pallet
    if abs(cross) > cfg['lateral_tolerance']:
        lead = gap-cfg['staging_target_gap']
        if lead < abs(cross) and gap < cfg['backoff_gap']:
            # No room to reach the centerline at <= 45deg: face the pallet, back straight off.
            return [('turn', yaw), ('drive', gap-cfg['backoff_gap'])]
        distance = cfg['body_front_x']+cfg['staging_target_gap']
        wx, wy = px-distance*math.cos(yaw), py-distance*math.sin(yaw)
        return [('turn', math.atan2(wy-odom[1], wx-odom[0])),
                ('drive', math.hypot(wx-odom[0], wy-odom[1])), ('turn', yaw)]
    moves = []
    drive = gap-cfg['target_body_gap']
    if abs(face) > cfg['face_lateral_tolerance'] or abs(heading) > math.radians(cfg['yaw_tolerance_deg']):
        # Point the fork line at the opening center; a straight drive keeps it there.
        moves.append(('turn', math.atan2(py-odom[1], px-odom[0])))
    if abs(drive) > cfg['gap_tolerance']:
        moves.append(('drive', drive))
    return moves
