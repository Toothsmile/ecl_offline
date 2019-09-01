#include "Myekf2.h"
//#include <mathlib/mathlib.h>
//#include <ekf.h>
#include <fstream>
#include <time.h>
#include <stdio.h>
#define ECL_STANDALONE

class Ekf2;
std::ifstream imuread("../data/rtk_vision/sensor_combined_0.txt");
std::ifstream gpsread("../data/rtk_vision/gps_position_0.txt");
std::ifstream magread("../data/rtk_vision/magnetometer_0.txt");
std::ifstream airread("../data/rtk_vision/air_data_0.txt");
std::ifstream evqread("../data/rtk_vision/vision_attitude_0.txt");
std::ifstream evpread("../data/rtk_vision/vision_position_0.txt");
std::ofstream euler_estimator("../results/euler_estimator.txt");
std::ofstream position_estimator("../results/position_estimator.txt");

bool bReadGPS, bmagread, bReadBaro,bReadevq,bReadevp;//是否读取gps,mag,baro,extel vision attitue and posion


namespace ekf2
{
Ekf2 *instance = nullptr;
}

Ekf2::Ekf2():
	_ekf(),
	_params(_ekf.getParamHandle())
{

}

Ekf2::~Ekf2()
{

}


void Ekf2::print_status()
{
	printf("local position OK %s", (_ekf.local_position_is_valid()) ? "[YES]" : "[NO]");
    printf("global position OK %s\n", (_ekf.global_position_is_valid()) ? "[YES]" : "[NO]");
}

void Ekf2::task_main()
{
	// initialise parameter cache// TODO
	//updateParams();
//	std::ifstream imuread("data/imu_data.txt");

	float gyro_integral_dt = 0;
	float accelerometer_integral_dt = 0;
	float last_IMUtime = 0;
    uint64_t now = 0;
    float mag_time_us_read=0, magx, magy, magz;

    //baro data
    uint64_t baro_time_us_read=0;float baro_alt_meter, rho;
    //gps data
    uint64_t gps_time_us=0,pos_itow_ms;
    int32_t lat,lon,alt;
    int fix_type,nsats;float eph,epv,sacc,vel_m_s,vel_ned[3],cog_rad,tempgps;
    bool vel_valid;
    //vision_data
    uint64_t evq_time_us=0,evp_time_us=0;
    matrix::Vector3f posNED;
    matrix::Quatf quat;
    float posErr,angErr;
    float ev_eph,ev_epv,ev_evh,ev_evv;
    int xy_valid,z_valid,v_xy_valid,v_z_valid;


    //while (!_task_should_exit && !imuread.eof() && !gpsread.eof() && !magread.eof() && !airread.eof()) {
    while (!_task_should_exit && !imuread.eof() &&  !imuread.eof() && !airread.eof()&& !evqread.eof()&& !evpread.eof()) {

		bool isa = true;
		bool mag_updated = false;
		bool baro_updated = false;
		bool gps_updated = false;
        bool evq_updated=false;
        bool evp_updated=false;

		bool vehicle_status_updated = false;

		// long gyro_integral_dt = 0.01;
		// // in replay mode we are getting the actual timestamp from the sensor topic      
        imuread >> now;	//us
//		float temp;
//		imuread >> temp;

        ECL_INFO("time now: %ld\n", now);
        //gyro_integral_dt = now - last_IMUtime;	//us
        //gyro_integral_dt /= 1.e6f;	//s
        //accelerometer_integral_dt = now - last_IMUtime;
        //accelerometer_integral_dt /=1.e6f;

		// // push imu data into estimator
		float gyro_integral[3],gyro_rad[3];
        imuread >> gyro_rad[0];	imuread >> gyro_rad[1];	imuread >> gyro_rad[2];imuread>>gyro_integral_dt;
        gyro_integral_dt /= 1.e6f;	//s
        ECL_DEBUG("[gyro]:%f,%f,%f,%f s ", gyro_rad[0], gyro_rad[1], gyro_rad[2],gyro_integral_dt);

		gyro_integral[0] = gyro_rad[0] * gyro_integral_dt;
		gyro_integral[1] = gyro_rad[1] * gyro_integral_dt;
		gyro_integral[2] = gyro_rad[2] * gyro_integral_dt;
        float accelerometer_timestamp_relative;imuread>>accelerometer_timestamp_relative;
		float accel_integral[3],accelerometer_m_s2[3];
        imuread >> accelerometer_m_s2[0];	imuread >> accelerometer_m_s2[1];	imuread >> accelerometer_m_s2[2];imuread>>accelerometer_integral_dt;
        accelerometer_integral_dt/=1.e6;  //s
        ECL_DEBUG("[acceler] accelerometer_m_s2:%f,%f,%f,%f s\n", accelerometer_m_s2[0], accelerometer_m_s2[1], accelerometer_m_s2[2],accelerometer_integral_dt);
		accel_integral[0] = accelerometer_m_s2[0] * accelerometer_integral_dt;
		accel_integral[1] = accelerometer_m_s2[1] * accelerometer_integral_dt;
		accel_integral[2] = accelerometer_m_s2[2] * accelerometer_integral_dt;
		_ekf.setIMUData(now, gyro_integral_dt * 1.e6f, accelerometer_integral_dt * 1.e6f,
				gyro_integral, accel_integral);		
		last_IMUtime = now;

        if(bmagread)
		{

            magread >> mag_time_us_read;	//us
            magread >> magx;
            magread >> magy;
            magread >> magz;
			//magx /= 100.0f;magy /= 100.0f;magz /= 100.0f;
            bmagread = false;

		}
        //if(mag_time_ms_read * 1.e3f < now)
        if(mag_time_us_read < now)
		{
			mag_updated = true;
            bmagread = true;
		}
		if(mag_updated)
		{
            //_timestamp_mag_us = mag_time_ms_read * 1.e3f;
            //_timestamp_mag_us = mag_time_ms_read;
            _timestamp_mag_us = mag_time_us_read;
			// If the time last used by the EKF is less than specified, then accumulate the
			// data and push the average when the 50msec is reached.
			_mag_time_sum_ms += _timestamp_mag_us / 1000.0f;
			_mag_sample_count++;
			_mag_data_sum[0] += magx;
			_mag_data_sum[1] += magy;
			_mag_data_sum[2] += magz;
			uint32_t mag_time_ms = _mag_time_sum_ms / _mag_sample_count;
			
			if (mag_time_ms - _mag_time_ms_last_used > _params->sensor_interval_min_ms) {
				float mag_sample_count_inv = 1.0f / (float)_mag_sample_count;
				float mag_data_avg_ga[3] = {_mag_data_sum[0] *mag_sample_count_inv, _mag_data_sum[1] *mag_sample_count_inv, _mag_data_sum[2] *mag_sample_count_inv};
				_ekf.setMagData(1000 * (uint64_t)mag_time_ms, mag_data_avg_ga);
                printf("[mag]: %d %d %d %f %f %f\n",now,mag_time_us_read,mag_time_ms,
                        _mag_data_sum[0],_mag_data_sum[1],_mag_data_sum[2]);
				_mag_time_ms_last_used = mag_time_ms;
				_mag_time_sum_ms = 0;
				_mag_sample_count = 0;
				_mag_data_sum[0] = 0.0f;
				_mag_data_sum[1] = 0.0f;
				_mag_data_sum[2] = 0.0f;	
			}		
		}


		if(bReadBaro)
		{
            airread >> baro_time_us_read;	//us

            airread>>baro_alt_meter;
            float temp; airread>>temp;airread>>temp;//baro_temp_celcius,baro_pressure_pa

            airread >>rho  ;
			// if(baroHeight_origin == 0)
			// 	baroHeight_origin = baroHeight;
			// baroHeight -= baroHeight_origin;
            bReadBaro= false;

		}
        if(baro_time_us_read <now)
		{
			baro_updated = true;
			bReadBaro = true;
		}
		if(baro_updated)
		{
                _timestamp_balt_us = baro_time_us_read;

				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the 50msec is reached.
				_balt_time_sum_ms += _timestamp_balt_us / 1000;
				_balt_sample_count++;
                _balt_data_sum += baro_alt_meter;
				uint32_t balt_time_ms = _balt_time_sum_ms / _balt_sample_count;

				if (balt_time_ms - _balt_time_ms_last_used > (uint32_t)_params->sensor_interval_min_ms) {
					float balt_data_avg = _balt_data_sum / (float)_balt_sample_count;
                ECL_DEBUG("[baro]: %f %f %d %f\n",now,baro_time_us_read,balt_time_ms,
                        balt_data_avg);
                _ekf.set_air_density(rho);


                //from v1.8.2 i dont why do this
                // calculate static pressure error = Pmeas - Ptruth
                // model position error sensitivity as a body fixed ellipse with different scale in the positive and negtive X direction
                float _aspd_max=20.0f,_K_pstatic_coef_xp=0.0f,_K_pstatic_coef_xn=0.0f,_K_pstatic_coef_y=0.0f,_K_pstatic_coef_z=0.0f;
                const float max_airspeed_sq = _aspd_max * _aspd_max;
                float K_pstatic_coef_x;

                const Vector3f vel_body_wind = get_vel_body_wind();

                if (vel_body_wind(0) >= 0.0f) {
                    K_pstatic_coef_x = _K_pstatic_coef_xp;

                } else {
                    K_pstatic_coef_x = _K_pstatic_coef_xn;
                }

                const float x_v2 = fminf(vel_body_wind(0) * vel_body_wind(0), max_airspeed_sq);
                const float y_v2 = fminf(vel_body_wind(1) * vel_body_wind(1), max_airspeed_sq);
                const float z_v2 = fminf(vel_body_wind(2) * vel_body_wind(2), max_airspeed_sq);

                const float pstatic_err = 0.5f * rho *
                              (K_pstatic_coef_x * x_v2) + (_K_pstatic_coef_y * y_v2) + (_K_pstatic_coef_z * z_v2);

                // correct baro measurement using pressure error estimate and assuming sea level gravity
                balt_data_avg += pstatic_err / (rho * CONSTANTS_ONE_G);

                _ekf.setBaroData(1000 * (uint64_t)balt_time_ms, balt_data_avg);
                _balt_time_ms_last_used = balt_time_ms;
                _balt_time_sum_ms = 0;
                _balt_sample_count = 0;
                _balt_data_sum = 0.0f;

            }
		}

		if(bReadGPS)
		{

            gpsread >> gps_time_us;	//us
            gpsread>>tempgps;//time_utc_usec
            gpsread>>lat;gpsread>>lon;gpsread>>alt;
            gpsread>>tempgps;//alt_ellipsoid
            gpsread>>pos_itow_ms;
            gpsread>>tempgps;/*fix_quality*/gpsread>>tempgps;gpsread>>sacc;gpsread>>tempgps;/*c_var_rad*/
            gpsread>>eph;gpsread>>epv;gpsread>>tempgps;/*hdop*/gpsread>>tempgps;/*vdop*/gpsread>>tempgps;/*noise_per_ms*/
            gpsread>>tempgps;/*jamming_indicator*/gpsread>>vel_m_s;/*vel*/gpsread>>vel_ned[0];gpsread>>vel_ned[1];gpsread>>vel_ned[2];
            gpsread>>cog_rad;gpsread>>tempgps;/*timerealtive*/gpsread>>fix_type;gpsread>>vel_valid;gpsread>>nsats;
            bReadGPS = false;

		}
        if(gps_time_us  < now)
		{
			gps_updated = true;
			bReadGPS = true;
		}
		if(gps_updated)
		{
			struct gps_message gps_msg = {};
            gps_msg.time_usec = gps_time_us;
            gps_msg.lat = lat;
            gps_msg.lon = lon;
            gps_msg.alt = alt;
            ECL_DEBUG("time now: %ld\n", now);

            gps_msg.fix_type = fix_type;
            gps_msg.eph = eph;
            gps_msg.epv = epv;
            gps_msg.sacc = sacc;
            gps_msg.vel_m_s = vel_m_s;
            gps_msg.vel_ned[0] = vel_ned[0];
            gps_msg.vel_ned[1] = vel_ned[1];
            gps_msg.vel_ned[2] = vel_ned[2];
            gps_msg.vel_ned_valid = vel_valid;
            gps_msg.nsats = nsats;
            ECL_DEBUG("[gps]: time %ld, %d, %d, %d, %d\n",gps_msg.time_usec,gps_msg.lat,gps_msg.lon,gps_msg.alt,gps_msg.fix_type);
			//TODO add gdop to gps topic
			gps_msg.gdop = 0.0f;

			_ekf.setGpsData(gps_msg.time_usec, &gps_msg);
		}


        if(bReadevq)
        {
            evqread>>evq_time_us;
            float evtmq;
            evqread>>evtmq;evqread>>evtmq;evqread>>evtmq;//roll pitch yaw speed
            evqread>>quat(0);evqread>>quat(1);evqread>>quat(2);evqread>>quat(3);
            evqread>>evtmq;evqread>>evtmq;evqread>>evtmq;evqread>>evtmq;evqread>>evtmq;//5个

            bReadevq=false;

        }
        if(bReadevp)
        {
            evpread>>evp_time_us;
            float evtmp;
            evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;
            evpread>>posNED(0);evpread>>posNED(1);evpread>>posNED(2);
            evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;
            evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;
            evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;
            evpread>>evtmp;evpread>>evtmp;//17
            evpread>>ev_eph;evpread>>ev_epv;evpread>>ev_evh;evpread>>ev_evv;
            evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;//4
            evpread>>xy_valid;evpread>>z_valid;evpread>>v_xy_valid;evpread>>v_z_valid;
            evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;evpread>>evtmp;
            evpread>>evtmp;evpread>>evtmp;//7

            bReadevp=false;
        }
        if(evp_time_us  < now)
        {
            evp_updated = true;
            bReadevp = true;
        }
        if(evq_time_us  < now)
        {
            evq_updated = true;
            bReadevq = true;
        }
        if (evq_updated || evp_updated) {
            ext_vision_message ev_data;
            ev_data.posNED=posNED;
            matrix::Quatf q(quat);
            ev_data.quat = q;

            // position measurement error from parameters. TODO : use covariances from topic
            ev_data.posErr = fmaxf(_params->ev_pos_noise, fmaxf(ev_eph, ev_epv));
            ev_data.angErr = _params->ev_ang_noise;
            // only set data if all positions and velocities are valid
            if (xy_valid && z_valid && v_xy_valid &&v_z_valid) {
                // use timestamp from external computer, clocks are synchronized when using MAVROS
                _ekf.setExtVisionData(evp_updated ? evp_time_us : evq_time_us, &ev_data);
            }
            ECL_DEBUG("[EV]now:%ld,ev_time %ld,posNED %f,%f,%f\n",now,evp_updated ? evp_time_us : evq_time_us,posNED(0),posNED(1),posNED(2));
        }


		//run the EKF update and output
		if (_ekf.update()) {
		

			matrix::Quaternion<float> q;
			_ekf.copy_quaternion(q.data());

			float velocity[3];
			_ekf.get_velocity(velocity);
            ECL_INFO("velocity: %lf,%lf,%lf\n", velocity[0], velocity[1], velocity[2]);

			float gyro_rad[3];

			{
				// generate control state data
				float gyro_bias[3] = {};
				_ekf.get_gyro_bias(gyro_bias);
				gyro_rad[0] = gyro_rad[0] - gyro_bias[0];
				gyro_rad[1] = gyro_rad[1] - gyro_bias[1];
				gyro_rad[2] = gyro_rad[2] - gyro_bias[2];

				// Velocity in body frame
				Vector3f v_n(velocity);
				matrix::Dcm<float> R_to_body(q.inversed());
				Vector3f v_b = R_to_body * v_n;


				// Local Position NED
				float position[3];
				_ekf.get_position(position);
                ECL_INFO("position: %lf,%lf,%lf\n", position[0], position[1], position[2]);
				position_estimator<< now/1.e6f <<" "<<position[0] <<" "<<position[1] - 0.278398 <<" "
				<<-position[2] + 0.0849676 <<" "<<std::endl;
				// Attitude quaternion
				//q.copyTo(ctrl_state.q);

				//_ekf.get_quat_reset(&ctrl_state.delta_q_reset[0], &ctrl_state.quat_reset_counter);

				// Acceleration data
				matrix::Vector<float, 3> acceleration(accelerometer_m_s2);

				float accel_bias[3];
				_ekf.get_accel_bias(accel_bias);
				// ctrl_state.x_acc = acceleration(0) - accel_bias[0];
				// ctrl_state.y_acc = acceleration(1) - accel_bias[1];
				// ctrl_state.z_acc = acceleration(2) - accel_bias[2];

				// // compute lowpass filtered horizontal acceleration
				acceleration = R_to_body.transpose() * acceleration;
				// _acc_hor_filt = 0.95f * _acc_hor_filt + 0.05f * sqrtf(acceleration(0) * acceleration(0) +
				// 		acceleration(1) * acceleration(1));
				// ctrl_state.horz_acc_mag = _acc_hor_filt;

				// ctrl_state.airspeed_valid = false;

			}
			
			// generate vehicle local position data

			float pos[3] = {};
			// Position of body origin in local NED frame
			_ekf.get_position(pos);
			//printf("%f  %f  %f\n", pos[0],pos[1],pos[2]);

			// Velocity of body origin in local NED frame (m/s)

			// TODO: better status reporting
	

			// Position of local NED origin in GPS / WGS84 frame
			
			// true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon)
			//_ekf.get_ekf_origin(&lpos.ref_timestamp, &ekf_origin, &lpos.ref_alt);
		
			// The rotation of the tangent plane vs. geographical north
			matrix::Eulerf euler(q);
			//printf("euler: %f  %f  %f\n", euler.phi(),euler.theta(),euler.psi());
				euler_estimator<< now/1.e6f <<" "<<euler.phi() <<" "<<euler.theta() <<" "
				<<euler.psi() <<" "<<std::endl;			
			// TODO: uORB definition does not define what these variables are. We have assumed them to be horizontal and vertical 1-std dev accuracy in metres
			Vector3f pos_var, vel_var;
			_ekf.get_pos_var(pos_var);
			_ekf.get_vel_var(vel_var);
            ECL_INFO("pos_var: %lf,%lf,%lf\n", pos_var(0), pos_var(1), pos_var(2) );
            ECL_INFO("vel_var: %lf,%lf,%lf\n", vel_var(0), vel_var(1), vel_var(2) );

		} 

	}
    ECL_INFO("end\n");


}
const Vector3f Ekf2::get_vel_body_wind()
{
    // Used to correct baro data for positional errors

    matrix::Quatf q;
    _ekf.copy_quaternion(q.data());
    matrix::Dcmf R_to_body(q.inversed());

    // Calculate wind-compensated velocity in body frame
    // Velocity of body origin in local NED frame (m/s)
    float velocity[3];
    _ekf.get_velocity(velocity);

    float velNE_wind[2];
    _ekf.get_wind_velocity(velNE_wind);

    Vector3f v_wind_comp = {velocity[0] - velNE_wind[0], velocity[1] - velNE_wind[1], velocity[2]};

    return R_to_body * v_wind_comp;
}


int main(int argc, char *argv[])
{
    ECL_INFO("begin\n");
	bReadGPS = true;
	Ekf2* _ekf2 = new Ekf2();
    _ekf2->print_status();
    _ekf2->task_main();
//    double timestamp=0,accelerometer_timestamp_relative=0;
//    double gyro_integral, gyro_rad[3];
//    double accel_integral,accelerometer_m_s2[3];
//   while(!imuread.eof())
//   {
//   imuread >> timestamp;
//   imuread >> gyro_rad[0];imuread >> gyro_rad[1];imuread >> gyro_rad[2];imuread >>gyro_integral;
//   imuread >> accelerometer_timestamp_relative;
//   imuread >> accelerometer_m_s2[0];imuread >> accelerometer_m_s2[1];imuread >> accelerometer_m_s2[2];
//   imuread >> accel_integral;
//   printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",timestamp,gyro_rad[0],gyro_rad[1],gyro_rad[2],gyro_integral,accelerometer_timestamp_relative,accelerometer_m_s2[0],accelerometer_m_s2[1],accelerometer_m_s2[2],accel_integral);

//   }

	return 1;
}
