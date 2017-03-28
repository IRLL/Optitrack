path_threshold = 0.00001
threshold = 0.0001

def bisect_line(p1, p2):
    return [(p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0]

def euclid_distance(p1, p2):
    return (((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2)) ** 0.5

def check_if_pt_on_line(pt, l1, l2):
    if abs( (euclid_distance(l1, pt) + euclid_distance(pt, l2)) - euclid_distance(l1, l2)) < path_threshold:
        return True
    return False

def find_closest_point(pt, path):
    path_index = 0
    path_point = path[0]
    min_d = euclid_distance(pt, path[0])

    for i, p in enumerate(path):
        d = euclid_distance(pt, p)
        if d < min_d:
            min_d = d
            path_index = i
            path_point = p

    return path_index, path_point

def recursive_closest_on_path(pt, left, mid, right, iters):
    if check_if_pt_on_line(pt, left, mid):
        return pt
    if check_if_pt_on_line(pt, mid, right):
        return pt

    left_mid = bisect_line(left, mid)
    right_mid = bisect_line(mid, right)

    left_mid_distance = euclid_distance(pt, left_mid)
    mid_distance = euclid_distance(pt, mid)
    right_mid_distance = euclid_distance(pt, right_mid)

    """
    print "left_mid_distance: ",
    print left_mid_distance
    print "mid_distance: ",
    print mid_distance
    print "right_mid_distance: ",
    print right_mid_distance
    print
    """

    if abs(mid_distance - left_mid_distance) < threshold and abs(mid_distance - right_mid_distance) < threshold:
        return bisect_line(left_mid, right_mid)

    if mid_distance < left_mid_distance and mid_distance < right_mid_distance:
        return recursive_closest_on_path(pt, bisect_line(left_mid, mid), mid, bisect_line(mid, right_mid), iters + 1)

    if left_mid_distance <= right_mid_distance:
        return recursive_closest_on_path(pt, left, left_mid, mid, iters + 1)

    if right_mid_distance < left_mid_distance:
        return recursive_closest_on_path(pt, mid, right_mid, right, iters + 1)

def distance_from_path(location, path):
    """
    print "path: ",
    print path
    print "location: ",
    print location
    """

    idx, closest_point = find_closest_point(location, path)
    """
    print "idx: ",
    print idx
    print "closest_point: ",
    print closest_point
    """

    left_idx = (idx - 1) % len(path)
    right_idx = (idx + 1) % len(path)
    left_pt = path[left_idx]
    right_pt = path[right_idx]
    """
    print "mid_pt: ",
    print closest_point
    print "left_pt: ",
    print left_pt
    print "right_pt: ",
    print right_pt
    print
    """

    closest_on_path = recursive_closest_on_path(location, path[left_idx], closest_point, path[right_idx], 0)
    distance = euclid_distance(location, closest_on_path)

    """
    print "closest_on_path: ",
    print closest_on_path
    print "distance_from_path: ",
    print distance
    """

    return closest_on_path, distance
