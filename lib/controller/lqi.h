#ifndef LQI_H
#define LQI_H

#include <Arduino.h>

namespace controller {

	class lqi {
		public:
			struct car_model {
				float r;
				float base_height;
				float leg_max_height;
				float leg_min_height;
			};

			struct speed_limit {
				float max_linear_vel;
				float max_steer_vel;
			};

			struct feedback_state {
				float pitch_angle;
				float pitch_rate;
				float avg_linear_pos;
				float avg_linear_vel;
				float yaw_angle;
				float yaw_rate;
			};

			struct reference_state {
				float linear_vel;
				float yaw_rate;
			};

			struct integral_state {
				float linear_vel_error;
				float yaw_rate_error;
			};

			struct integral_limit {
				float linear_vel_error;
				float yaw_rate_error;
			};

		public:
			lqi();

		public:
			car_model car;
			speed_limit limit;
			feedback_state state;
			reference_state ref;
			integral_state integral;
			integral_limit integral_clamp;
			float gain_poly[12][4];
			float feedback_gain[2][6];
	};

}

#endif
