
import numpy as np



def get_new_gps_coords(lat, lon, dlat_meters, dlon_meters):
    earthRadiusMeters = 6371.0 * 1000.0
    lat1 = lat + np.degrees(dlat_meters / earthRadiusMeters)
    lon1 = lon + np.degrees(dlon_meters / earthRadiusMeters) / np.cos(np.radians(lat))
    return (lat1, lon1)


def deadReckon(lat, lon, dist, hdg_radians):
    # 0 points North.  And the direction is reverse
    dLat = dist * np.cos(hdg_radians)
    dLon = dist * np.sin(hdg_radians)
    return get_new_gps_coords(lat, lon, dLat, dLon)


