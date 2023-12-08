def pixel_to_cm(dist_px, px_to_cm_leq_25, px_to_cm_bet_25_30, px_to_cm_gt_30):
    # print(px_to_cm_leq_25, px_to_cm_bet_25_30, px_to_cm_gt_30)
    pixels_25 = 25 / px_to_cm_leq_25
    pixels_30 = 5 /px_to_cm_bet_25_30 + pixels_25
    pixels_35 = 5 / px_to_cm_gt_30 + pixels_30

    # print(pixels_25)
    # print(pixels_30)

    # print(5/px_to_cm_bet_25_30)
    dist_cm1 = min(dist_px, pixels_25) * px_to_cm_leq_25
    dist_cm2 = min(max(dist_px - pixels_25, 0), 5/px_to_cm_bet_25_30) * px_to_cm_bet_25_30
    # print(min(max(dist_px - pixels_25, 0), 5/px_to_cm_bet_25_30))
    dist_cm3 = max(dist_px - pixels_30, 0) * px_to_cm_gt_30
    print(dist_cm1 + dist_cm2 + dist_cm3
)
    return dist_cm1 + dist_cm2 + dist_cm3



def sign(position):
    return position / abs(position)
