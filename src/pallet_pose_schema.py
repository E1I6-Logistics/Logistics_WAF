"""Shared semantic order for LabelMe, YOLO Pose and the PnP opening model."""
import math

KEYPOINT_NAMES = (
    'left_top_left', 'left_top_right', 'left_bottom_right', 'left_bottom_left',
    'right_top_left', 'right_top_right', 'right_bottom_right', 'right_bottom_left',
)
FLIP_INDEX = [5, 4, 7, 6, 1, 0, 3, 2]


def labelme_rows(document):
    """Strict conversion. Missing points stay invisible; never invent corners.

    Empty annotations require flags.background=true. An occluded point must be
    explicitly marked flags.occluded=true. Unknown shapes/duplicate groups fail.
    """
    w, h = document['imageWidth'], document['imageHeight']
    if not all(isinstance(v, int) and v > 0 for v in (w, h)):
        raise ValueError('Invalid image dimensions')
    shapes = document['shapes']
    if not shapes:
        if document.get('flags', {}).get('background') is not True:
            raise ValueError('Empty annotation requires explicit flags.background=true')
        return []
    if document.get('flags', {}).get('background'):
        raise ValueError('Background annotation contains objects')
    groups = {}
    for shape in shapes:
        label = shape['label']
        if label not in ('pallet', *KEYPOINT_NAMES):
            raise ValueError(f'Unknown label: {label}')
        group = shape.get('group_id')
        if group is None:
            raise ValueError('Every object and keypoint requires a group_id')
        entry = groups.setdefault(group, {})
        if label in entry:
            raise ValueError(f'Duplicate {label} in group {group}')
        points = shape['points']
        expected = ('polygon', 'rectangle') if label == 'pallet' else ('point',)
        if shape['shape_type'] not in expected:
            raise ValueError(f'Wrong shape type: {label}')
        if (label != 'pallet' and len(points) != 1 or
                label == 'pallet' and len(points) < (3 if shape['shape_type'] == 'polygon' else 2)):
            raise ValueError(f'Wrong point count: {label}')
        for xy in points:
            if len(xy) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in xy):
                raise ValueError(f'Non-finite coordinate: {label}')
            if not (0 <= xy[0] < w and 0 <= xy[1] < h):
                raise ValueError(f'Out-of-image coordinate: {label}')
        entry[label] = shape
    rows = []
    for group, entry in groups.items():
        if 'pallet' not in entry:
            raise ValueError(f'Group {group} has no pallet outline')
        polygon = entry['pallet']['points']
        x0, y0 = min(p[0] for p in polygon), min(p[1] for p in polygon)
        x1, y1 = max(p[0] for p in polygon), max(p[1] for p in polygon)
        if x1 <= x0 or y1 <= y0:
            raise ValueError('Degenerate pallet outline')
        row = [0, (x0+x1)/2/w, (y0+y1)/2/h, (x1-x0)/w, (y1-y0)/h]
        for name in KEYPOINT_NAMES:
            if name not in entry:
                row.extend([0, 0, 0])
                continue
            x, y = entry[name]['points'][0]
            if not (x0-1 <= x <= x1+1 and y0-1 <= y <= y1+1):
                raise ValueError(f'Keypoint outside pallet bounding box: {name}')
            visibility = 1 if entry[name].get('flags', {}).get('occluded') is True else 2
            row.extend([x/w, y/h, visibility])
        rows.append(row)
    return rows
