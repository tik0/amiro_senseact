#ifndef AMIRO_CONSTANTS_H_
#define AMIRO_CONSTANTS_H_

/*! \brief Constants regarding the AMiRo platform
 *
 *  This header contains constant variables
 *  regarding the AMiRo platform, which means that
 *  these values do not change during runtime.
 *  Constants are e.g. physical ones like seconds per minute
 *  or geometrical ones like the circumference of wheel.
 *  All physical constants (therefore all values with a
 *  physical unit) are implicitly in µ iff the variable 
 *  is of type integer, unless it is explicitly named in
 *  the variable.
 *  All physical constants (therefore all values with a
 *  physical unit) are implicitly without prefix (e.g. µ)
 *  iff the variable is of type float, unless it is
 *  explicitly named in the variable. The SI prefix is
 *  used, iff the variable is of type float and therefor
 *  in SI units.
 */

#include <math.h>
#define M_PI 3.14159265358979323846

namespace amiro {
namespace constants {

  /** \brief Amount of seconds per minute */
  const uint32_t secondsPerMinute = 60;
  
  /** \brief Amount of minutes per hour */
  const uint32_t minutesPerHour = 60;
  
  /** \brief Amount of milliseconds per second */
  const uint32_t millisecondsPerSecond = 1000;
  
  /** \brief Distance between wheels in meter */
  const float wheelDistanceSI = 0.08f;
  
  /** \brief Distance between wheels in micrometer */
  const uint32_t wheelDistance = wheelDistanceSI * 1e6;
  
  /** \brief Wheel diameter in meter */
  const float wheelDiameterSI = 0.05571f;
  
  /** \brief Wheel diameter */
  const uint32_t wheelDiameter = wheelDiameterSI * 1e6;
  
  /** \brief Wheel circumference in meter */
  const float wheelCircumferenceSI = M_PI * wheelDiameterSI;
  
  /** \brief Wheel circumference in micrometer */
  const uint32_t wheelCircumference = wheelCircumferenceSI * 1e6;
  
  /** \brief Wheel error in meter (topview left:0, right:1) */
  const float wheelErrorSI[2] = {0.1, 0.1};
  
  /** \brief Wheel error in meter (topview left:0, right:1) */
  const int32_t wheelError[2] = {(int32_t) (wheelErrorSI[0] * 1e6), (int32_t) (wheelErrorSI[1] * 1e6)};
  
  /** \brief Motor increments per revolution
   *
   *  The increments are produced by 2 channels á 16
   *  pulses per revolution with respect to the rising
   *  and falling signal => 2*2*16 pulses/revolution.
   *  The gearbox is 22:1 => 2*2*16*22 pulses/revolution
   */
  const int32_t incrementsPerRevolution = 2 * 2 * 16 * 22;
  
  /** \brief Index of the proximity sensors
   * 
   * Bottom view of the AMiRo sensors and their indices (F:Front, B:Back):
   *  _____
   * / 0F3 \
   * |1   2|
   * \__B__/
   */
  enum sensorIdx {PROX_WL = 1, PROX_WR = 2, PROX_FL = 0, PROX_FR = 3};
  
  /** \brief Index of the wheels
   * 
   * Top view of the AMiRo wheels and their indices (F:Front, B:Back):
   *  _____
   * /| F |\
   * |0   1|
   * \|_B_|/
   */
  enum wheelIdx {LEFT_WHEEL = 0, RIGHT_WHEEL = 1};
}
}

#endif /* AMIRO_CONSTANTS_H_ */
