#ifndef VCNL4020Models
#define VCNL4020Models

namespace VCNL4020Models {
	#define VCNL4020MODELS_OBSTACLEMODEL_ALPHA 0.942693757414292
	#define VCNL4020MODELS_OBSTACLEMODEL_BETA -16.252241893638708
	#define VCNL4020MODELS_OBSTACLEMODEL_DELTA 0
	#define VCNL4020MODELS_OBSTACLEMODEL_XI 1.236518540376969
	#define VCNL4020MODELS_OBSTACLEMODEL_MEAS_VARIANCE 20.886268537074187 //7.283035954107597

	#define VCNL4020MODELS_EDGEMODEL_M 6.359946153158588
	#define VCNL4020MODELS_EDGEMODEL_B 0.401918238192352

	// calculates the distance [m] to an obstacle given the obstacles angle [rad] relative to the sensor and the sensor value [ticks]
	float obstacleModel(float angle, float sensorValue) {
		float cosxi = cos(VCNL4020MODELS_OBSTACLEMODEL_XI * angle);
		if (cosxi < 0) {
			cosxi *= -1;
		}
		float divi = sensorValue - VCNL4020MODELS_OBSTACLEMODEL_BETA;
		if (divi <= 0) {
			divi = 1;
		}
		return sqrt(VCNL4020MODELS_OBSTACLEMODEL_ALPHA * cosxi / divi + VCNL4020MODELS_OBSTACLEMODEL_DELTA * cosxi);
	}

	// calculates the distance [m] to the next edge given the normalized edge value factored by 10000
	float edgeModel(int senValue) {
		return VCNL4020MODELS_EDGEMODEL_M * ((float)senValue)/10000.0 + VCNL4020MODELS_EDGEMODEL_B;
	}
}

#endif // VCNL4020Models
