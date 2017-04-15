PATH_THRESHOLD = 0.00001
DISTANCE_THRESHOLD = 0.0001

def bisect_line(pt1, pt2):
    """
    Finds the middle point of an n-dimensional line defined by its start and
    end points.

    Parameters
    ----------
    pt1 : list of float
    pt2 : list of float

    Returns
    -------
    mid_pt : list of float

    Raises
    ------
    ValueError
        If the dimensions of pt1 do not match pt2 (e.g a 2d point and a 3d
        point)
    """
    if len(pt1) != len(pt2):
        raise ValueError("Dimensions of pt1 must match dimensions of pt2.")

    return [(pt1[i] + pt2[i]) / 2.0 for i in range(len(pt1))]

def euclid_distance(pt1, pt2):
    """
    Finds the euclidian distance between two n-dimensional points.

    Parameters
    ----------
    pt1 : list of float
    pt2 : list of float

    Returns
    -------
    distance : float

    Raises
    ------
    ValueError
        If the dimensions of pt1 do not match pt2 (e.g a 2d point and a 3d
        point)
    """
    if len(pt1) != len(pt2):
        raise ValueError("Dimensions of pt1 must match dimensions of pt2.")

    return sum([(pt1[i] - pt2[i]) ** 2.0 for i in range(len(pt1))]) ** 0.5

def check_if_pt_on_line(pt, line_start, line_end):
    """
    Determines if a given point lies along a line defined by its start and end
    points.

    Parameters
    ----------
    pt : list of float
    line_start : list of float
    line_end : list of float

    Returns
    -------
    is_on_line : bool

    Raises
    ------
    ValueError
        If the dimensions of pt, line_start, and end_point do not match
    """
    if len(pt) != len(line_start) or len(pt) != len(line_end):
        raise ValueError("Dimensions of pt, line_start, and line_end must match")

    if abs((euclid_distance(line_start, pt) + euclid_distance(pt, line_end)) -
            euclid_distance(line_start, line_end)) < PATH_THRESHOLD:
        return True
    else:
        return False

def find_closest_point(pt, pt_set):
    """
    Finds the minimum euclidian distance between a given point and a set of
    points.

    Parameters
    ----------
    pt : list of float
    pt_set : list of list of points

    Returns
    -------
    index : int
        Index of closest point
    closest_point : list of float
        Closest point in set

    Raises
    ------
    ValueError
        If the dimensions of pt, and any point in pt_set do not match.
    """
    for p in pt_set:
        if len(pt) != len(p):
            raise ValueError("Dimensions of pt must match dimensions of all points in pt_set.")

    index = 0
    closest_point = pt_set[0]
    min_d = euclid_distance(pt, pt_set[0])

    for i, p in enumerate(pt_set):
        d = euclid_distance(pt, p)
        if d < min_d:
            min_d = d
            index = i
            closest_point = p

    return index, closest_point

def recursive_closest_on_segments(pt, left, mid, right):
    """
    Finds the closest point along left and right line segments to a given point
    by recursively interpolating along the closer line segment.

    Parameters
    ----------
    pt : list of float
    left : list of float
    mid : list of float
    right : list of float

    Returns
    -------
    closest_point : closest point

    Raises
    ------
    ValueError
        If the dimensions of pt, left, mid, and right do not all match.
    """
    for p in [left, mid, right]:
        if len(pt) != len(p):
            raise ValueError("Dimensions of pt, left, mid, and right do not match.")

    if check_if_pt_on_line(pt, left, mid):
        return pt
    if check_if_pt_on_line(pt, mid, right):
        return pt

    left_mid = bisect_line(left, mid)
    right_mid = bisect_line(mid, right)

    left_mid_distance = euclid_distance(pt, left_mid)
    mid_distance = euclid_distance(pt, mid)
    right_mid_distance = euclid_distance(pt, right_mid)

    if abs(mid_distance - left_mid_distance) < DISTANCE_THRESHOLD and \
    abs(mid_distance - right_mid_distance) < DISTANCE_THRESHOLD:
        return bisect_line(left_mid, right_mid)

    if mid_distance < left_mid_distance and mid_distance < right_mid_distance:
        return recursive_closest_on_segments(pt, bisect_line(left_mid, mid),
                mid, bisect_line(mid, right_mid))

    if left_mid_distance <= right_mid_distance:
        return recursive_closest_on_segments(pt, left, left_mid, mid)

    if right_mid_distance < left_mid_distance:
        return recursive_closest_on_segments(pt, mid, right_mid, right)

def get_closest_point_and_distance_from_path(pt, path):
    """
    Uses recursive_closest_on_segments to find the closest point along an
    entire path to a given point.

    Parameters
    ----------
    pt : list of float
    path : list of list of float

    Returns
    -------
    closest_on_path : list of float
    distance : float

    Raises
    ------
    ValueError
        If the dimensions of pt, and any point in path do not match.
    """
    for p in path:
        if len(pt) != len(p):
            raise ValueError("Dimensions of pt must match dimensions of all points in path.")

    idx, closest_point = find_closest_point(pt, path)

    left_idx = (idx - 1) % len(path)
    right_idx = (idx + 1) % len(path)

    closest_on_path = recursive_closest_on_segments(pt, path[left_idx],
            closest_point, path[right_idx])
    distance = euclid_distance(pt, closest_on_path)

    return closest_on_path, distance
