"""
This file contains functions to convert pixel distances to centimeters based off environment and measurements
"""
def pixel_to_cm(dist_px, px_to_cm_leq_25, px_to_cm_bet_25_30, px_to_cm_gt_30):
    """
    Converts bottle's distance in pixels from cup to centimeters in real setting and return.

    Parameters:
    dist_px (int): distance in pixels
    px_to_cm_leq_25 (float): conversion factor for distances less than 25 cm
    px_to_cm_bet_25_30 (float): conversion factor for distances between 25 and 30 cm
    px_to_cm_gt_30 (float): conversion factor for distances greater than 30 cm
    """
    pixels_25 = 25 / px_to_cm_leq_25
    pixels_30 = 5 /px_to_cm_bet_25_30 + pixels_25

    dist_cm1 = min(dist_px, pixels_25) * px_to_cm_leq_25
    dist_cm2 = min(max(dist_px - pixels_25, 0), 5/px_to_cm_bet_25_30) * px_to_cm_bet_25_30
    dist_cm3 = max(dist_px - pixels_30, 0) * px_to_cm_gt_30
    return dist_cm1 + dist_cm2 + dist_cm3


def sign(position):
    """
    Returns the sign of the position
    """
    return position / abs(position)
