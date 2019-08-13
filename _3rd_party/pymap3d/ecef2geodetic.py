# this code is from:
#        https://github.com/scivision/pymap3d
#
#  The LICENSE there says:
'''
Copyright (c) 2014-2018 Michael Hirsch, Ph.D.
Copyright (c) 2013, Felipe Geremia Nievinski
Copyright (c) 2004-2007 Michael Kleder

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''


from numpy import radians, sin, cos, tan, arctan, hypot, degrees, arctan2, sqrt, pi
from typing import Tuple
import numpy as np


class Ellipsoid:
    """
    generate reference ellipsoid parameters

    https://en.wikibooks.org/wiki/PROJ.4#Spheroid

    https://nssdc.gsfc.nasa.gov/planetary/factsheet/index.html

    as everywhere else in this program, distance units are METERS
    """

    # def __init__(self, model: str = 'wgs84'):
    def __init__(self, model='wgs84'):
        """
        feel free to suggest additional ellipsoids

        Parameters
        ----------
        model : str
                name of ellipsoid
        """
        if model == 'wgs84':
            """https://en.wikipedia.org/wiki/World_Geodetic_System#WGS84"""
            self.semimajor_axis = 6378137.
            self.flattening = 1 / 298.2572235630
            self.semiminor_axis = self.semimajor_axis * (1 - self.flattening)
        elif model == 'wgs72':
            self.semimajor_axis = 6378135.
            self.flattening = 1 / 298.26
            self.semiminor_axis = self.semimajor_axis * (1 - self.flattening)
        elif model == 'grs80':
            """https://en.wikipedia.org/wiki/GRS_80"""
            self.semimajor_axis = 6378137.
            self.flattening = 1 / 298.257222100882711243
            self.semiminor_axis = self.semimajor_axis * (1 - self.flattening)
        elif model == 'clarke1866':
            self.semimajor_axis = 6378206.4
            self.semiminor_axis = 6356583.8
            self.flattening = -(self.semiminor_axis / self.semimajor_axis - 1)
        elif model == 'mars':
            """
            https://tharsis.gsfc.nasa.gov/geodesy.html
            """
            self.semimajor_axis = 3396900.
            self.semiminor_axis = 3376097.80585952
            self.flattening = 1 / 163.295274386012
        elif model == 'moon':
            self.semimajor_axis = 1738000.
            self.semiminor_axis = self.semimajor_axis
            self.flattening = 0.
        elif model == 'venus':
            self.semimajor_axis = 6051000.
            self.semiminor_axis = self.semimajor_axis
            self.flattening = 0.
        elif model == 'jupiter':
            self.semimajor_axis = 71492000.
            self.flattening = 1 / 15.415446277169725
            self.flattening = -(self.semiminor_axis / self.semimajor_axis - 1)
        elif model == 'io':
            """
            https://doi.org/10.1006/icar.1998.5987
            """
            self.semimajor_axis = 1829.7
            self.flattening = 1 / 131.633093525179
            self.semiminor_axis = self.semimajor_axis * (1 - self.flattening)
        elif model == 'pluto':
            self.semimajor_axis = 1187000.
            self.semiminor_axis = self.semimajor_axis
            self.flattening = 0.
        else:
            raise NotImplementedError('{} model not implemented, let us know and we will add it (or make a pull request)'.format(model))




def ecef2geodetic(x: float, y: float, z: float,
                  ell: Ellipsoid = None, deg: bool = True) -> Tuple[float, float, float]:
    """
    convert ECEF (meters) to geodetic coordinates

    Parameters
    ----------
    x : float or numpy.ndarray of float
        target x ECEF coordinate (meters)
    y : float or numpy.ndarray of float
        target y ECEF coordinate (meters)
    z : float or numpy.ndarray of float
        target z ECEF coordinate (meters)
    ell : Ellipsoid, optional
          reference ellipsoid
    deg : bool, optional
          degrees input/output  (False: radians in/out)

    Returns
    -------
    lat : float or numpy.ndarray of float
           target geodetic latitude
    lon : float or numpy.ndarray of float
           target geodetic longitude
    h : float or numpy.ndarray of float
         target altitude above geodetic ellipsoid (meters)

    based on:
    You, Rey-Jer. (2000). Transformation of Cartesian to Geodetic Coordinates without Iterations.
    Journal of Surveying Engineering. doi: 10.1061/(ASCE)0733-9453
    """
    if ell is None:
        ell = Ellipsoid()

    x = np.asarray(x)
    y = np.asarray(y)
    z = np.asarray(z)

    r = sqrt(x**2 + y**2 + z**2)

    E = sqrt(ell.semimajor_axis**2 - ell.semiminor_axis**2)

    # eqn. 4a
    u = sqrt(0.5 * (r**2 - E**2) + 0.5 * sqrt((r**2 - E**2)**2 + 4 * E**2 * z**2))

    Q = hypot(x, y)

    huE = hypot(u, E)

    # eqn. 4b
    with np.errstate(divide='ignore'):
        Beta = arctan(huE / u * z / hypot(x, y))

    # eqn. 13
    eps = ((ell.semiminor_axis * u - ell.semimajor_axis * huE + E**2) * sin(Beta)) / (ell.semimajor_axis * huE * 1 / cos(Beta) - E**2 * cos(Beta))

    Beta += eps
# %% final output
    lat = arctan(ell.semimajor_axis / ell.semiminor_axis * tan(Beta))

    lon = arctan2(y, x)

    # eqn. 7
    alt = hypot(z - ell.semiminor_axis * sin(Beta),
                Q - ell.semimajor_axis * cos(Beta))

    # inside ellipsoid?
    with np.errstate(invalid='ignore'):
        inside = x**2 / ell.semimajor_axis**2 + y**2 / ell.semimajor_axis**2 + z**2 / ell.semiminor_axis**2 < 1
    if isinstance(inside, np.ndarray):
        alt[inside] = -alt[inside]
    elif inside:
        alt = -alt

    if deg:
        lat = degrees(lat)
        lon = degrees(lon)

    return lat, lon, alt

