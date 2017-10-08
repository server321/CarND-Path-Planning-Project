#ifndef SMOOTHER
#define SMOOTHER

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include "spline.h"

using namespace std;

// Interpolation of points connecting a number of x and y values

vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y, 
                                  double interval, int output_size) {


  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
    return { 0 };
  }

  tk::spline s;
  s.set_points(pts_x,pts_y);   
  vector<double> output;
  for (int i = 0; i < output_size; i++) {
    output.push_back(s(pts_x[0] + i * interval));
  }
	
  // return output_size number of y values beginning at y[0] with specified fixed interval
  return output;
}


vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y, 
                                  vector<double> eval_at_x) {



  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
    return { 0 };
  }

  tk::spline s;
  s.set_points(pts_x,pts_y);   
  vector<double> output;
  for (double x: eval_at_x) {
    output.push_back(s(x));
  }
  // return a spline evaluated at each eval_at_x point
  return output;
}

#endif