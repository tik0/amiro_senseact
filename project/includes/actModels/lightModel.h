#ifndef LightModel
#define LightModel

#include <Color.h>

namespace LightModel {

	/** \brief Initial colors */
	static std::vector<Color> initColors = {amiro::Color(amiro::Color::RED),
						amiro::Color(amiro::Color::GREEN),
						amiro::Color(amiro::Color::BLUE),
						amiro::Color(amiro::Color::WHITE),
						amiro::Color(amiro::Color::RED),
						amiro::Color(amiro::Color::GREEN),
						amiro::Color(amiro::Color::BLUE),
						amiro::Color(amiro::Color::WHITE)};

	/** \brief Type of lighting of the LEDs
	  * INIT: Sets the initial colors (see vector "initColors").
	  * SHINE: Sets the given color(s) and lets it just shine.
	  * BLINK: Sets the given color(s) and blinks with all LEDs turned on or off.
	  * WARNING: Sets the given color(s) and blinks with always the left or right half turned on or off.
	  * CIRCLELEFT: Sets the given color(s) and lets circle two LEDs turned on to the left (counter clockwise).
	  * CIRCLERIGHT: Sets the given color(s) and lets circle two LEDs turned on to the right (clockwise).
	  */
	enum LightType {CMD_INIT, CMD_SHINE, CMD_BLINK, CMD_WARNING, CMD_CIRCLELEFT, CMD_CIRCLERIGHT};

	/** \brief Minimal time for blinking period in ms. */
	static const int MIN_TIME_BLINK = 200;
	/** \brief Minimal time for warning blinking period in ms. */
	static const int MIN_TIME_WARNING = 400;
	/** \brief Minimal time for circling period in ms. */
	static const int MIN_TIME_CIRCLED = 800;
}

#endif // LightModel
