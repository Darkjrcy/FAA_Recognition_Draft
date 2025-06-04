#ifndef SATELLITE_PREDICTOR_H_
#define SATELLITE_PREDICTOR_H_

#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include <gnss_multipath_plugin/geodetic_conv.hpp>
#include <gnss_multipath_plugin/multipath_sensor.hpp>

namespace satellite_predictor {

class Satellites
{
 public:  
  Satellites()
  {
  }

  ~Satellites()
  {
  }

  void parseTLE()
  {
    num_sat_ = 8;
    // Initialization of data structures for satellite prediction 
    tle_lines_ = (const char**) calloc(2*num_sat_, sizeof(const char*));
    sat_orbit_elements_ = (predict_orbital_elements_t**) calloc(num_sat_, sizeof(predict_orbital_elements_t*));
    sat_position_ = (struct predict_position*) calloc(num_sat_, sizeof(struct predict_position));
    for ( int i = 0; i < 2*num_sat_; i++ )
    {
        tle_lines_[i] = (const char*) calloc(100, sizeof(const char));
    }
    for ( int i = 0; i < num_sat_; i++ )
    {
        sat_orbit_elements_[i] = (predict_orbital_elements_t*) calloc(1, sizeof( predict_orbital_elements_t));
    }
    //G01
    tle_lines_[0] = "1 37753U 11036A   23033.10118579 -.00000081  00000+0  00000+0 0  9997";
    tle_lines_[1] = "2 37753  56.6933  19.7748 0121088  52.9316 129.8426  2.00566862 84583";

    //G04
    tle_lines_[2] = "1 43873U 18109A   23032.57137684 -.00000044  00000+0  00000+0 0  9992";
    tle_lines_[3] = "2 43873  55.1375 140.8489 0023229 190.4825 204.0790  2.00558671 30373";

    //G07
    tle_lines_[4] = "1 32711U 08012A   23032.64592938  .00000056  00000+0  00000+0 0  9990";
    tle_lines_[5] = "2 32711  54.4610 199.0507 0168701 232.8355 125.5954  2.00572261109059";

    //G08
    tle_lines_[6] = "1 40730U 15033A   23033.72582779 -.00000076  00000+0  00000+0 0  9996";
    tle_lines_[7] = "2 40730  55.0217 317.3848 0082059  10.2474 349.9421  2.00566329 55333";

    //G09
    tle_lines_[8] = "1 40105U 14045A   23032.56334991 -.00000048  00000+0  00000+0 0  9995";
    tle_lines_[9] = "2 40105  54.7481 137.6538 0023243 114.6825 245.5467  2.00562646 61376";

    //G16
    tle_lines_[10] = "1 27663U 03005A   23033.62997381  .00000047  00000+0  00000+0 0  9997";
    tle_lines_[11] = "2 27663  55.3642 263.9131 0131905  42.8180 326.5456  2.00551168146626";

    //G21
    tle_lines_[12] = "1 27704U 03010A   23033.55723828 -.00000091  00000+0  00000+0 0  9994";
    tle_lines_[13] = "2 27704  55.0899  13.9998 0249685 312.4590 223.4527  2.00565951145429";

    //G27
    tle_lines_[14] = "1 39166U 13023A   23033.18918996 -.00000082  00000+0  00000+0 0  9995";
    tle_lines_[15] = "2 39166  55.5293 318.6885 0111059  38.9232 321.8895  2.00565200 71193";
    
    for ( int i = 0; i < num_sat_; i++ )
    {
        // Create orbit object
        sat_orbit_elements_[i] = predict_parse_tle(tle_lines_[2*i], tle_lines_[2*i+1]);
    }
  }
  int getCount()
  {
    return num_sat_;
  }
  void getECEF(struct tm *_timeinfo, std::vector<std::vector<double>> &_sat_ecef)
  {
    _sat_ecef.resize(num_sat_, std::vector<double>(3));
    // Predict the julian time using the current utc time
    predict_julian_date_t curr_time = predict_to_julian(makeTimeUTC(_timeinfo));
    for ( int i = 0; i < num_sat_; i++ )
    {
      // Predict satellite's position based on the current time.
      predict_orbit(sat_orbit_elements_[i], &sat_position_[i], curr_time);
      g_geodetic_converter.geodetic2Ecef(sat_position_[i].latitude*180.0/M_PI, sat_position_[i].longitude*180.0/M_PI , 
                    sat_position_[i].altitude*1000 , &_sat_ecef[i][0], &_sat_ecef[i][1], &_sat_ecef[i][2]);
    }
  }

  void getRange(std::vector<double> _rec_lla , std::vector<double> &_true_range)  
  {
    for ( int i = 0; i < num_sat_; i++ )
    {
      predict_observer_t *obs = predict_create_observer("obs", _rec_lla[0]*M_PI/180.0, _rec_lla[1]*M_PI/180.0, _rec_lla[2]);
      struct predict_observation reciever_obs;
      predict_observe_orbit(obs, &sat_position_[i], &reciever_obs);
      _true_range.push_back(reciever_obs.range*1000); //From Km to m
    }

  }
  void getAngles(std::vector<double> _rec_lla , std::vector<double> &_azimuth, std::vector<double> &_elevation )  
  {
    for ( int i = 0; i < num_sat_; i++ )
    {
      predict_observer_t *obs = predict_create_observer("obs", _rec_lla[0]*M_PI/180.0, _rec_lla[1]*M_PI/180.0, _rec_lla[2]);
      struct predict_observation reciever_obs;
      predict_observe_orbit(obs, &sat_position_[i], &reciever_obs);
      // Setting the azimuth and elevation angles for the satellite ray and the reflected ray for each satellite.
      _azimuth.push_back(reciever_obs.azimuth);
      _elevation.push_back(reciever_obs.elevation); // assume reflection has the same elevation as the direct ray
    }

  }
  time_t makeTimeUTC(const struct tm* timeinfo_utc)
  {
    time_t curr_time = time(NULL);
    int timezone_diff = 0; //deviation of the current timezone from UTC in seconds

    //get UTC time, interpret resulting tm as a localtime
    struct tm timeinfo_gmt;
    gmtime_r(&curr_time, &timeinfo_gmt);
    time_t time_gmt = mktime(&timeinfo_gmt);

    //get localtime, interpret resulting tm as localtime
    struct tm timeinfo_local;
    localtime_r(&curr_time, &timeinfo_local);
    time_t time_local = mktime(&timeinfo_local);

    //find the time difference between the two interpretations
    timezone_diff += difftime(time_local, time_gmt);

    //hack for preventing mktime from assuming localtime: add timezone difference to the input struct.
    struct tm ret_timeinfo;
    ret_timeinfo.tm_sec = timeinfo_utc->tm_sec + timezone_diff;
    ret_timeinfo.tm_min = timeinfo_utc->tm_min;
    ret_timeinfo.tm_hour = timeinfo_utc->tm_hour;
    ret_timeinfo.tm_mday = timeinfo_utc->tm_mday;
    ret_timeinfo.tm_mon = timeinfo_utc->tm_mon;
    ret_timeinfo.tm_year = timeinfo_utc->tm_year;
    ret_timeinfo.tm_isdst = timeinfo_utc->tm_isdst;
    return mktime(&ret_timeinfo);
  }
  int num_sat_;
  // TlE parameters to store the satellite's orbital parameters
  const char **tle_lines_;

  // Satellite orbital parameters to predict satellite's position
  predict_orbital_elements_t **sat_orbit_elements_;
  
  // Satellite position information at a given time
  struct predict_position *sat_position_;
  
  geodetic_converter::GeodeticConverter g_geodetic_converter;
}; // class Satellites
}; // namespace satellite_predictor

#endif // SATELLITE_PREDICTOR_H_
