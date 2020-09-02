Imports NETGeographicLib

Module example_Geodesic
    Sub Main()
        Try
            Dim geod As Geodesic = New Geodesic(Constants.WGS84.EquatorialRadius,
                                                Constants.WGS84.Flattening)
            ' Alternatively: Dim geod As Geodesic = new Geodesic()
            ' Sample direct calculation, travelling about NE from JFK
            Dim lat1 As Double = 40.6, lon1 = -73.8, s12 = 5500000.0, azi1 = 51
            Dim lat2, lon2 As Double
            geod.Direct(lat1, lon1, azi1, s12, lat2, lon2)
            Console.WriteLine(String.Format("Latitude: {0} Longitude: {1}", lat2, lon2))
            ' Sample inverse calculation, JFK to LHR
            lat1 = 40.6 : lon1 = -73.8 ' JFK Airport
            lat2 = 51.6 : lon2 = -0.5  ' LHR Airport
            geod.Inverse(lat1, lon1, lat2, lon2, s12)
            Console.WriteLine(s12)
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
