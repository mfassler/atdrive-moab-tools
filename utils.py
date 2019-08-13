
import numpy as np


def get_new_gps_coords(lat, lon, dlat_meters, dlon_meters):
    earthRadiusMeters = 6371.0 * 1000.0
    lat1 = lat + np.degrees(dlat_meters / earthRadiusMeters)
    lon1 = lon + np.degrees(dlon_meters / earthRadiusMeters) / np.cos(np.radians(lat))
    return (lat1, lon1)

