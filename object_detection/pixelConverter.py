def pixel_to_cm(dist_px, px_to_cm_leq_25, px_to_cm_bet_25_30, px_to_cm_gt_30):
    dist_cm1 = min(dist_px, 25) / px_to_cm_leq_25
    dist_cm2 = min(max(dist_px - 25, 0), 5) / px_to_cm_bet_25_30
    dist_cm3 = max(dist_px - 30, 0) / px_to_cm_gt_30
    return dist_cm1 + dist_cm2 + dist_cm3


def sign(position):
    return position / abs(position)