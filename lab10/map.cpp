#include "Map.h"
#include <math.h>
float none[2] = {-1.0,-1.0};
float *res = new float(2);

float maps[8][2][2] = { { {0,0},{0,3} } ,{ {0,3},{3,3} },{ {3,3},{3,0} },{{3,0},{0,0} },{ {0,1},{1,1} },{ {1,1},{1,2} },{ {2,1},{2,2} },{ {2,1.5},{3,1.5}} };
Map::Map(){
    
 
}


float Map::closest_distance(float*origin,float theta){
  float result=9999.0;
  for(int i=0;i<8;i++){
    float point1[2] = {maps[i][0][0],maps[i][0][1]};
    float point2[2] = {maps[i][1][0],maps[i][1][1]};
    float *p = ray_line_intersection(origin, theta,point1 , point2);    
    if( p[0] != -1){
        float dist = sqrt(pow(p[0]-origin[0],2)+pow(p[1]-origin[1],2));
        if(dist<result)
          result = dist;
    }      
  }
  return result;
}

float* Map::ray_line_intersection(float ray_origin[2], float theta, float point1[2], float point2[2]){
  float v1[2] = {ray_origin[0] - point1[0] , ray_origin[1] - point1[1]} ;
  float v2[2] = {point2[0] - point1[0] , point2[1] - point1[1]} ;
  float v3[2] = {-sin(theta) , cos(theta)};
  float denominator =  v2[0] * v3[0] + v2[1] * v3[1] ;

  if(denominator == 0)
    return none;
  float t1 = (v2[0] * v1[1] - v2[1] * v1[0])/denominator;
  float t2 = (v1[0] * v3[0] + v1[1] * v3[1])/denominator;
  if (t1 >= 0.0 && 0.0 <= t2 && t2 <= 1.0){
    res[0] = ray_origin[0] + t1 * cos(theta);
    res[1] = ray_origin[1] + t1 * sin(theta);
    return res;
  }

  return none;
  
}
