using System;
using NETGeographicLib;

namespace example_Geodesic
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                Geodesic geod = new Geodesic( Constants.WGS84.EquatorialRadius,
                                              Constants.WGS84.Flattening );
                // Alternatively: Geodesic geod = new Geodesic();
                {
                    // Sample direct calculation, travelling about NE from JFK
                    double lat1 = 40.6, lon1 = -73.8, s12 = 5.5e6, azi1 = 51;
                    double lat2, lon2;
                    geod.Direct(lat1, lon1, azi1, s12, out lat2, out lon2);
                    Console.WriteLine(String.Format("Latitude: {0} Longitude: {1}", lat2, lon2));
                }
                {
                    // Sample inverse calculation, JFK to LHR
                    double
                    lat1 = 40.6, lon1 = -73.8, // JFK Airport
                    lat2 = 51.6, lon2 = -0.5;  // LHR Airport
                    double s12;
                    geod.Inverse(lat1, lon1, lat2, lon2, out s12);
                    Console.WriteLine( s12 );
                }
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
