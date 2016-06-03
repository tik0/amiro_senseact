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
	  * CROSSED: Sets the given color(s) and blinks with only 4 crossed LEDs at once turned on.
	  * CIRCLELEFT: Sets the given color(s) and lets circle two LEDs turned on to the left (counter clockwise).
	  * CIRCLERIGHT: Sets the given color(s) and lets circle two LEDs turned on to the right (clockwise).
	  */
	enum LightType {CMD_INIT, CMD_SHINE, CMD_BLINK, CMD_WARNING, CMD_CROSSED, CMD_CIRCLELEFT, CMD_CIRCLERIGHT};

	/** \brief Minimal time for blinking period in ms. */
	static const int MIN_TIME_BLINK = 200;
	/** \brief Minimal time for warning blinking period in ms. */
	static const int MIN_TIME_WARNING = 400;
	/** \brief Minimal time for crossed blinking period in ms. */
	static const int MIN_TIME_CROSSED = 400;
	/** \brief Minimal time for circling period in ms. */
	static const int MIN_TIME_CIRCLED = 800;


	/** \brief Creates integer vector containing lighting type, period time and only one color for the setLights tool. */
	std::vector<int> setLight2Vec(int lightingType, amiro::Color color, int periodTime) {
		std::vector<int> commandVector(5,0);
		commandVector[0] = lightingType;
		commandVector[1] = (int)(color.getRed());
		commandVector[2] = (int)(color.getGreen());
		commandVector[3] = (int)(color.getBlue());
		commandVector[4] = periodTime;
		return commandVector;
	}

	/** \brief Creates integer vector containing lighting type, period time and 8 colors for the setLights tool (if there is a problem, the first entry is -1). */
	std::vector<int> setLights2Vec(int lightingType, std::vector<amiro::Color> colors, int periodTime) {
		std::vector<int> commandVector(26,0);
		if (colors.size() == 8) {
			commandVector[0] = lightingType;
			for (int led=0; led<8; led++) {
				commandVector[led*3+1] = (int)(colors[led].getRed());
				commandVector[led*3+2] = (int)(colors[led].getGreen());
				commandVector[led*3+3] = (int)(colors[led].getBlue());
			}
			commandVector[25] = periodTime;
		} else {
			commandVector[0] = -1;
		}
		return commandVector;
	}
}

#endif // LightModel
