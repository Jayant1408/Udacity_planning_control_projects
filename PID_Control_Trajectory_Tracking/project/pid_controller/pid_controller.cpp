/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::init_controller(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  this->k_p = Kpi;
  this->k_i = Kii;
  this->k_d = Kdi;
  this->lim_max_output = output_lim_maxi;
  this->lim_min_output = output_lim_mini;
  this->delta_t = 0.0;
  this->error_p = 0.0;
  this->error_i = 0.0;
  this->error_d = 0.0;
}


void PID::update_error(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   this->error_d = (this->delta_t > 0.0) ? (cte - this->error_p) / this->delta_t : 0.0;
   this->error_p = cte;  // Set current error after calculating derivative
   this->error_i += cte * this->delta_t;
 }

double PID::total_error() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control = ((this->k_p * this->error_p)
                     + (this->k_d * this->error_d)
                     + (this->k_i * this->error_i));

   if(control > this->lim_max_output)
   {
      return this->lim_max_output;
   }
   else if (control < this->lim_min_output)
   {
      return this->lim_min_output;
   }
   else
   {
      return control;
   }
                     
   // double control;
   // return control;
}

double PID::update_delta_time(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  this->delta_t = new_delta_time;
  return this->delta_t;
}