#include <iostream>
#include <math.h>

class trackingAntenna
{
  private:
    const float CURVE_ADJ_FACTOR = 0.0023544;
    const int16_t EARTH_RADIUS = 6371;

    float lat_diff = 0;  //all calculated variables
    float long_diff = 0;
    float alt_diff = 0;
    float lat_seg = 0;
    float long_seg = 0;
    float theta = 0;
    float alpha = 0;
    float vector = 0;

    double pitch_rot = 0; //final values
    double yaw_rot = 0;
    
    std::string yaw_and_pitch_rot_string = ""; //returned string "yaw,pitch"

    static trackingAntenna *instance_ptr;

    trackingAntenna()
    {
    }

    double degToRadian(double degree) 
    {
        return degree * M_PI / 180.0;
    }
    double radianToDeg(double radian) 
    {
        return radian * 180.0 / M_PI;
    }
    void getDifference(double lat_1, double long_1, float alt_1, double lat_2, double long_2, float alt_2)
    {   //differences between the two positions in each "dimension"
      lat_diff = degToRadian(lat_1 - lat_2);
      long_diff = degToRadian(long_1 - long_2);
      alt_diff = alt_2 - alt_1;
    }
    void getTriangleLengthsAndAngles(double lat_2)
    {
      lat_seg = 2*EARTH_RADIUS*sin(lat_diff/2)*cos(lat_diff/2);   //latitude segment
      long_seg = 2*EARTH_RADIUS*sin(long_diff/2)*cos(degToRadian(lat_2))*cos(long_diff/2);  //longitude segment
      theta = degToRadian(90) - long_diff/2.0;   //approximate angle between the two segments
      vector = sqrt(pow(lat_seg, 2) + pow(long_seg, 2) - 2*lat_seg*long_seg*cos(theta));  //vector joining the ground station and drone (ignoring altitude)
      alpha = asin(vector/(2*EARTH_RADIUS));  //angle between the two radius lines that form a triangle with "vector"
    }

  public:
    trackingAntenna(const trackingAntenna &obj) = delete;

    static trackingAntenna *getInstance()
    {
      return instance_ptr;
    }
    //variables with _1 at the end belong to the ground station, those with _2 at the end belong to the drone
    std::string getYawAndPitch(double lat_1, double long_1, float alt_1, double lat_2, double long_2, float alt_2) //returns "yaw,pitch"
    {
      getDifference(lat_1, long_1, alt_1, lat_2, long_2, alt_2);  
      getTriangleLengthsAndAngles(lat_2); //calculates the lengths of straight segments that are located in a plane
    
      if (long_1>long_2) //calculates final value depending on which direction the vector faces
          yaw_rot = 180 - radianToDeg(acos(((pow(lat_seg, 2) + pow(vector, 2) - pow(long_seg, 2))/(2*lat_seg*vector))));
      else
          yaw_rot = 180 + radianToDeg(acos(((pow(lat_seg, 2) + pow(vector, 2) - pow(long_seg, 2))/(2*lat_seg*vector))));

      pitch_rot = radianToDeg(atan((cos(alpha)*alt_diff - CURVE_ADJ_FACTOR*vector)/vector)); 

      yaw_and_pitch_rot_string = std::to_string(yaw_rot) + ",";
      yaw_and_pitch_rot_string += std::to_string(pitch_rot);

      return yaw_and_pitch_rot_string;
    }
};

trackingAntenna *trackingAntenna ::instance_ptr = new trackingAntenna();

int main()
{
  double lat_1 = 54.9967051; //just for testing - the values currently returned by the program are 305.916116,0.038041. the real values are 305.885743,0.0053246
  double lat_2 = 55.2051573;
  double long_1 = -61.415482;
  double long_2 = -60.9080316;
  double alt_1 = 0;
  double alt_2 = 0.12;
  trackingAntenna *antenna
      = trackingAntenna ::getInstance();
  std::cout << antenna->getYawAndPitch(lat_1, long_1, alt_1, lat_2, long_2, alt_2);
}