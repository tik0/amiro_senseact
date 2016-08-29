#ifndef TwbTracking
#define TwbTracking

#include <boost/shared_ptr.hpp>
#include <rsc/threading/SynchronizedQueue.h>

// types
#include <types/enum.pb.h>
#include <types/loc.pb.h>
#include <types/pose.pb.h>
#include <types/rotation.pb.h>
#include <types/shapes.pb.h>
#include <types/vertex.pb.h>

namespace twbTracking {

	/** \brief Tracking Position (float) with the following order
	  *  0: x-coordinate
	  *  1: y-coordinate
	  *  2: angle
	  */
	typedef std::array<float,3> TrackingPos;

	/** \brief Tracking Object, including
	  *  - Marker ID (id)
	  *  - Tracking Position (pos)
	  */
	typedef struct {
		int id;
		TrackingPos pos;
	} TrackingObject;



	/** \brief Constant for tracking timeout in seconds */
	static const uint32_t TRACKING_TIMEOUT = 5;



	/** \brief Checks, if an Tracking Object is an error */
	static bool isErrorTracking(TrackingObject obj) {
		return obj.id < 0;
	}

	/** \brief Creates an tracking error object */
	static TrackingObject createErrorTracking() {
		TrackingObject obj;
		obj.id = -1;
		obj.pos[0] = 0.0;
		obj.pos[1] = 0.0;
		obj.pos[2] = 0.0;
		return obj;
	}

	/** \brief Returns the Tracking Object for the given marker ID in the given ObjectList */
	static TrackingObject readTracking(boost::shared_ptr<twbTracking::proto::ObjectList> data, int trackingMarkerID) {
		twbTracking::proto::Pose pose2D;
		bool found = false;
		for (int i = 0; i < data->object_size(); i++) {
			if (trackingMarkerID == data->object(i).id()) {
				pose2D = data->object(i).position();
				found = true;
				break;
			}
		}
		if (found) {
			TrackingObject obj;
			obj.id = trackingMarkerID;
			obj.pos[0] = pose2D.translation().x();
			obj.pos[1] = pose2D.translation().y();
			float angle = pose2D.rotation().z()*M_PI/180.0;
			angle = fmod(angle, 2.0*M_PI);
			if (angle < 0) angle += 2.0*M_PI;
			obj.pos[2] = angle;
			return obj;
		} else {
			return createErrorTracking();
		}
	}

	/** \brief Checks, if the tracking data contains the data of the given marker */
	static bool isBeingTracked(boost::shared_ptr<twbTracking::proto::ObjectList> data, int trackingMarkerID) {
		bool isIn = false;
		for (int i = 0; i < data->object_size(); i++) {
			if (trackingMarkerID == data->object(i).id()) {
				if (data->object(i).position().translation().x() != 0 && data->object(i).position().translation().y() != 0) {
					isIn = true;
				}
				break;
			}
		}
		return isIn;
	}

	/** \brief Waits with the given synchronized queue for new tracking data for the given marker. It returns an tracking error object, if the TRACKING_TIMEOUT has been reached. */
	static TrackingObject getNextTrackingObject(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>> trackingQueue, int trackingMarkerID) {
		boost::shared_ptr<twbTracking::proto::ObjectList> data;
		int restTime = TRACKING_TIMEOUT * 1000; // ms
		do {
			if (!trackingQueue->empty()) {
				data = boost::static_pointer_cast<twbTracking::proto::ObjectList>(trackingQueue->pop());
				if (isBeingTracked(data, trackingMarkerID)) {
					break;
				}
			} else if (restTime <= 0) {
				return createErrorTracking();
			} else {
				// sleep for 10 ms
				usleep(10000);
				restTime -= 10;
			}
		} while (true);
		return readTracking(data, trackingMarkerID);
	}

	/** \brief Waits with the given synchronized queue for any new tracking data. It returns an tracking error object in the vector, if the TRACKING_TIMEOUT has been reached. */
	static std::vector<TrackingObject> getNextTrackingObjects(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>> trackingQueue) {
		boost::shared_ptr<twbTracking::proto::ObjectList> data;
		int restTime = TRACKING_TIMEOUT * 1000; // ms
		do {
			if (!trackingQueue->empty()) {
				data = boost::static_pointer_cast<twbTracking::proto::ObjectList>(trackingQueue->pop());
				break;
			} else if (restTime <= 0) {
				std::vector<TrackingObject> positions;
				TrackingObject obj = createErrorTracking();
				positions.push_back(obj);
				return positions;
			} else {
				// sleep for 10 ms
				usleep(10000);
				restTime -= 10;
			}
		} while (true);
		std::vector<TrackingObject> positions;
		for (int i = 0; i < data->object_size(); i++) {
			TrackingObject tracking = readTracking(data, data->object(i).id());
			positions.push_back(tracking);
		}
		return positions;
	}
}

#endif // TwbTracking
