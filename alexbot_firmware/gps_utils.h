
// radius of Earth in m
#define R 6371000

// lat/lon and result in radians
double compute_bearing(double i_lat, double i_lon, double f_lat, double f_lon) {
    double y = sin(f_lon-i_lon) * cos(f_lat);
    double x = cos(i_lat)*sin(f_lat) - sin(i_lat)*cos(f_lat)*cos(f_lon-i_lon);
    return atan2(y, x); 
}

// lat/lon in radians. returns distance in meters
double compute_distance(double i_lat, double i_lon, double f_lat, double f_lon) {
    double a = sin((f_lat - i_lat)/2) * sin((f_lat - i_lat)/2) +
                        cos(i_lat) * cos(f_lat) *
                        sin((f_lon - i_lon)/2) * sin((f_lon - i_lon)/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return R * c;
}

double to_circle(double value)
{
    if (value > 2.0 * PI)
        return value - (2.0 * PI);
    if (value < 0)
        return value + 2.0 * PI;

    return value;
}