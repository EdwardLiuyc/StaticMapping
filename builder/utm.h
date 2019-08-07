// MIT License

// Copyright (c) 2019 Edward Liu

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Original Javascript by Chuck Taylor
// Port to C++ by Alex Hajnal
//
// *** THIS CODE USES 32-BIT FLOATS BY DEFAULT ***
// *** For 64-bit double-precision edit this file: undefine FLOAT_32 and define
// FLOAT_64 (see below)
//
// This is a simple port of the code on the Geographic/UTM Coordinate Converter
// (1) page from Javascript to C++. Using this you can easily convert between
// UTM and WGS84 (latitude and longitude). Accuracy seems to be around 50cm (I
// suspect rounding errors are limiting precision). This code is provided as-is
// and has been minimally tested; enjoy but use at your own risk! The license
// for UTM.cpp and UTM.h is the same as the original Javascript: "The C++ source
// code in UTM.cpp and UTM.h may be copied and reused without restriction."
//
// 1) http://home.hiwaay.net/~taylorc/toolbox/geography/geoutm.html

#ifndef BUILDER_UTM_H_
#define BUILDER_UTM_H_

#include <math.h>

namespace static_map {
namespace utm {

/* Ellipsoid model constants (actual values here are for WGS84) */
constexpr double sm_a = 6378137.0;
constexpr double sm_b = 6356752.314;
constexpr double sm_EccSquared = 6.69437999013e-03;
constexpr double UTMScaleFactor = 0.9996;

// ArcLengthOfMeridian
// Computes the ellipsoidal distance from the equator to a point at a
// given latitude.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
// GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//     phi - Latitude of the point, in radians.
//
// Globals:
//     sm_a - Ellipsoid model major axis.
//     sm_b - Ellipsoid model minor axis.
//
// Returns:
//     The ellipsoidal distance of the point from the equator, in meters.
double ArcLengthOfMeridian(double phi);

// UTMCentralMeridian
// Determines the central meridian for the given UTM zone.
//
// Inputs:
//     zone - An integer value designating the UTM zone, range [1,60].
//
// Returns:
//   The central meridian for the given UTM zone, in radians
//   Range of the central meridian is the radian equivalent of [-177,+177].
double UTMCentralMeridian(int zone);

// FootpointLatitude
//
// Computes the footpoint latitude for use in converting transverse
// Mercator coordinates to ellipsoidal coordinates.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
//   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//   y - The UTM northing coordinate, in meters.
//
// Returns:
//   The footpoint latitude, in radians.
double FootpointLatitude(double y);

// MapLatLonToXY
// Converts a latitude/longitude pair to x and y coordinates in the
// Transverse Mercator projection.  Note that Transverse Mercator is not
// the same as UTM; a scale factor is required to convert between them.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
// GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//    phi - Latitude of the point, in radians.
//    lambda - Longitude of the point, in radians.
//    lambda0 - Longitude of the central meridian to be used, in radians.
//
// Outputs:
//    x - The x coordinate of the computed point.
//    y - The y coordinate of the computed point.
//
// Returns:
//    The function does not return a value.
void MapLatLonToXY(double phi, double lambda, double lambda0, double& x,
                   double& y);

// MapXYToLatLon
// Converts x and y coordinates in the Transverse Mercator projection to
// a latitude/longitude pair.  Note that Transverse Mercator is not
// the same as UTM; a scale factor is required to convert between them.
//
// Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
//   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
//
// Inputs:
//   x - The easting of the point, in meters.
//   y - The northing of the point, in meters.
//   lambda0 - Longitude of the central meridian to be used, in radians.
//
// Outputs:
//   phi    - Latitude in radians.
//   lambda - Longitude in radians.
//
// Returns:
//   The function does not return a value.
//
// Remarks:
//   The local variables Nf, nuf2, tf, and tf2 serve the same purpose as
//   N, nu2, t, and t2 in MapLatLonToXY, but they are computed with respect
//   to the footpoint latitude phif.
//
//   x1frac, x2frac, x2poly, x3poly, etc. are to enhance readability and
//   to optimize computations.
void MapXYToLatLon(double x, double y, double lambda0, double& phi,
                   double& lambda);

// LatLonToUTMXY
// Converts a latitude/longitude pair to x and y coordinates in the
// Universal Transverse Mercator projection.
//
// Inputs:
//   lat - Latitude of the point, in radians.
//   lon - Longitude of the point, in radians.
//   zone - UTM zone to be used for calculating values for x and y.
//          If zone is less than 1 or greater than 60, the routine
//          will determine the appropriate zone from the value of lon.
//
// Outputs:
//   x - The x coordinate (easting) of the computed point. (in meters)
//   y - The y coordinate (northing) of the computed point. (in meters)
//
// Returns:
//   The UTM zone used for calculating the values of x and y.
int LatLonToUTMXY(double lat, double lon, int zone, double& x, double& y);

// UTMXYToLatLon
//
// Converts x and y coordinates in the Universal Transverse Mercator//   The UTM
// zone parameter should be in the range [1,60].

// projection to a latitude/longitude pair.
//
// Inputs:
// x - The easting of the point, in meters.
// y - The northing of the point, in meters.
// zone - The UTM zone in which the point lies.
// southhemi - True if the point is in the southern hemisphere;
//               false otherwise.
//
// Outputs:
// lat - The latitude of the point, in radians.
// lon - The longitude of the point, in radians.
//
// Returns:
// The function does not return a value.
void UTMXYToLatLon(double x, double y, int zone, bool southhemi, double& lat,
                   double& lon);

}  // namespace utm
}  // namespace static_map

#endif  // BUILDER_UTM_H_
