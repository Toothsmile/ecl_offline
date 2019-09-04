#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>

#include <ekf.h>

#define ECL_DEBUG(...)     {fprintf(stdout,"[ECL_DEBUG]");fprintf(stdout,__VA_ARGS__);fprintf(stdout,"\n");}// by sjj
#define ECL_INFO(...)      {fprintf(stdout,"[ECL_INFO]");fprintf(stdout,__VA_ARGS__);fprintf(stdout,"\n");}
#define ECL_WARN(...)      {fprintf(stdout,"[ECL_WARN]");fprintf(stdout,__VA_ARGS__);fprintf(stdout,"\n");}
#define ECL_ERRO(...)      {fprintf(stdout,"[ECL_ERRO]");fprintf(stdout,__VA_ARGS__);fprintf(stdout,"\n");}

#define PX4_ISFINITE(x) std::isfinite(x) //ISFINITE判断



class Ekf2 
{
public:
	/**
	 * Constructor
	 */
	Ekf2();

	/**
	 * Destructor, also kills task.
	 */
	~Ekf2();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	//int	start();

	void task_main();

	void print_status();

    void exit() { _task_should_exit = true;}

    const Vector3f get_vel_body_wind();

    //char* mat2Str(float *x);


private:
	static constexpr float _dt_max = 0.02;
	bool	_task_should_exit = false;
	int	_control_task = -1;		// task handle for task
	bool 	_replay_mode;			// should we use replay data from a log
	int 	_publish_replay_mode;		// defines if we should publish replay messages
	float	_default_ev_pos_noise = 0.05f;	// external vision position noise used when an invalid value is supplied
	float	_default_ev_ang_noise = 0.05f;	// external vision angle noise used when an invalid value is supplied

    // Used to filter velocity innovations during pre-flight checks
    bool _preflt_horiz_fail = false;	///< true if preflight horizontal innovation checks are failed
    bool _preflt_vert_fail = false;		///< true if preflight vertical innovation checks are failed
    bool _preflt_fail = false;		///< true if any preflight innovation checks are failed
    Vector2f _vel_ne_innov_lpf = {};	///< Preflight low pass filtered NE axis velocity innovations (m/sec)
    float _vel_d_innov_lpf = {};		///< Preflight low pass filtered D axis velocity innovations (m/sec)
    float _hgt_innov_lpf = 0.0f;		///< Preflight low pass filtered height innovation (m)
    float _yaw_innov_magnitude_lpf = 0.0f;	///< Preflight low pass filtered yaw innovation magntitude (rad)

    float _rng_gnd_clearance{0};	        ///< minimum valid value for range when on ground (m)


    // Used to check, save and use learned magnetometer biases
    uint64_t _last_magcal_us = 0;	///< last time the EKF was operating a mode that estimates magnetomer biases (uSec)
    uint64_t _total_cal_time_us = 0;	///< accumulated calibration time since the last save

    float _last_valid_mag_cal[3] = {};	///< last valid XYZ magnetometer bias estimates (mGauss)
    bool _valid_cal_available[3] = {};	///< true when an unsaved valid calibration for the XYZ magnetometer bias is available
    float _last_valid_variance[3] = {};	///< variances for the last valid magnetometer XYZ bias estimates (mGauss**2)

	// Initialise time stamps used to send sensor data to the EKF and for logging
	uint64_t _timestamp_mag_us = 0;
	uint64_t _timestamp_balt_us = 0;

    // time slip monitoring by sjj
    uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
    uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
    int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	// Used to down sample magnetometer data
	matrix::Vector <float, 4> mag_from_API;
	float _mag_data_sum[3] = {0};			// summed magnetometer readings (Ga)
	uint64_t _mag_time_sum_ms = 0;		// summed magnetoemter time stamps (msec)
	// uint64_t _mag_time_sum_ms;		// summed magnetoemter time stamps (msec)
    int _mag_sample_count = 0;		// number of magnetometer measurements summed
	uint32_t _mag_time_ms_last_used = 0;	// time stamp in msec of the last averaged magnetometer measurement used by the EKF

	// Used to down sample barometer data
	matrix::Vector2f baro_from_API;
	float _balt_data_sum;			// summed barometric altitude readings (m)
	uint64_t _balt_time_sum_ms;		// summed barometric altitude time stamps (msec)
    int _balt_sample_count = 0;		// number of barometric altitude measurements summed
	uint32_t _balt_time_ms_last_used = 0;	// time stamp in msec of the last averaged barometric altitude measurement used by the EKF

	bool	_prev_landed = true;	// landed status from the previous frame

	float _acc_hor_filt = 0.0f; 	// low-pass filtered horizontal acceleration

	matrix::Vector <float, 4> gps_from_API;
	Ekf _ekf;

	parameters *_params;	// pointer to ekf parameter struct (located in _ekf class instance)
    struct vehicle_attitude_s {

        uint64_t timestamp; // required for logger
        float rollspeed;
        float pitchspeed;
        float yawspeed;
        float q[4];
        float delta_q_reset[4];
        uint8_t quat_reset_counter;
        uint8_t _padding0[3]; // required for logger
    };
    struct vehicle_local_position_s {
        uint64_t timestamp; // required for logger
        uint64_t ref_timestamp;
        double ref_lat;
        double ref_lon;
        float x;
        float y;
        float z;
        float delta_xy[2];
        float delta_z;
        float vx;
        float vy;
        float vz;
        float z_deriv;
        float delta_vxy[2];
        float delta_vz;
        float ax;
        float ay;
        float az;
        float yaw;
        float ref_alt;
        float dist_bottom;
        float dist_bottom_rate;
        float eph;
        float epv;
        float evh;
        float evv;
        float vxy_max;
        float vz_max;
        float hagl_min;
        float hagl_max;
        bool xy_valid;
        bool z_valid;
        bool v_xy_valid;
        bool v_z_valid;
        uint8_t xy_reset_counter;
        uint8_t z_reset_counter;
        uint8_t vxy_reset_counter;
        uint8_t vz_reset_counter;
        bool xy_global;
        bool z_global;
        bool dist_bottom_valid;
        uint8_t _padding0[5]; // required for logger
    };
    struct vehicle_global_position_s {

        uint64_t timestamp; // required for logger
        double lat;
        double lon;
        float alt;
        float delta_alt;
        float vel_n;
        float vel_e;
        float vel_d;
        float yaw;
        float eph;
        float epv;
        float terrain_alt;
        uint8_t lat_lon_reset_counter;
        uint8_t alt_reset_counter;
        bool terrain_alt_valid;
        bool dead_reckoning;

    };
    struct sensor_bias_s {

        uint64_t timestamp; // required for logger
        float accel_x;
        float accel_y;
        float accel_z;
        float gyro_x_bias;
        float gyro_y_bias;
        float gyro_z_bias;
        float accel_x_bias;
        float accel_y_bias;
        float accel_z_bias;
        float mag_x_bias;
        float mag_y_bias;
        float mag_z_bias;

    };

    struct estimator_status_s {
        uint64_t timestamp; // required for logger
        float states[24];
        float n_states;
        float vibe[3];
        float covariances[24];
        uint32_t control_mode_flags;
        float pos_horiz_accuracy;
        float pos_vert_accuracy;
        float mag_test_ratio;
        float vel_test_ratio;
        float pos_test_ratio;
        float hgt_test_ratio;
        float tas_test_ratio;
        float hagl_test_ratio;
        float beta_test_ratio;
        float time_slip;
        uint16_t gps_check_fail_flags;
        uint16_t filter_fault_flags;
        uint16_t innovation_check_flags;
        uint16_t solution_status_flags;
        uint8_t nan_flags;
        uint8_t health_flags;
        uint8_t timeout_flags;
        bool pre_flt_fail;

        static constexpr uint8_t GPS_CHECK_FAIL_GPS_FIX = 0;
        static constexpr uint8_t GPS_CHECK_FAIL_MIN_SAT_COUNT = 1;
        static constexpr uint8_t GPS_CHECK_FAIL_MIN_GDOP = 2;
        static constexpr uint8_t GPS_CHECK_FAIL_MAX_HORZ_ERR = 3;
        static constexpr uint8_t GPS_CHECK_FAIL_MAX_VERT_ERR = 4;
        static constexpr uint8_t GPS_CHECK_FAIL_MAX_SPD_ERR = 5;
        static constexpr uint8_t GPS_CHECK_FAIL_MAX_HORZ_DRIFT = 6;
        static constexpr uint8_t GPS_CHECK_FAIL_MAX_VERT_DRIFT = 7;
        static constexpr uint8_t GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR = 8;
        static constexpr uint8_t GPS_CHECK_FAIL_MAX_VERT_SPD_ERR = 9;
        static constexpr uint8_t CS_TILT_ALIGN = 0;
        static constexpr uint8_t CS_YAW_ALIGN = 1;
        static constexpr uint8_t CS_GPS = 2;
        static constexpr uint8_t CS_OPT_FLOW = 3;
        static constexpr uint8_t CS_MAG_HDG = 4;
        static constexpr uint8_t CS_MAG_3D = 5;
        static constexpr uint8_t CS_MAG_DEC = 6;
        static constexpr uint8_t CS_IN_AIR = 7;
        static constexpr uint8_t CS_WIND = 8;
        static constexpr uint8_t CS_BARO_HGT = 9;
        static constexpr uint8_t CS_RNG_HGT = 10;
        static constexpr uint8_t CS_GPS_HGT = 11;
        static constexpr uint8_t CS_EV_POS = 12;
        static constexpr uint8_t CS_EV_YAW = 13;
        static constexpr uint8_t CS_EV_HGT = 14;
        static constexpr uint8_t CS_BETA = 15;
        static constexpr uint8_t CS_MAG_FIELD = 16;
        static constexpr uint8_t CS_FIXED_WING = 17;
        static constexpr uint8_t CS_MAG_FAULT = 18;
        static constexpr uint8_t CS_ASPD = 19;
        static constexpr uint8_t CS_GND_EFFECT = 20;
        static constexpr uint8_t CS_RNG_STUCK = 21;
    };
    struct ekf2_innovations_s {
        uint64_t timestamp; // required for logger
        float vel_pos_innov[6];
        float mag_innov[3];
        float heading_innov;
        float airspeed_innov;
        float beta_innov;
        float flow_innov[2];
        float hagl_innov;
        float vel_pos_innov_var[6];
        float mag_innov_var[3];
        float heading_innov_var;
        float airspeed_innov_var;
        float beta_innov_var;
        float flow_innov_var[2];
        float hagl_innov_var;
        float output_tracking_error[3];
        float drag_innov[2];
        float drag_innov_var[2];
        float aux_vel_innov[2];
        uint8_t _padding0[4]; // required for logger
    };

};
