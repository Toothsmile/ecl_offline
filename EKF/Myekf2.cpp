#include "Myekf2.h"
#include <sstream>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include<iostream>
#define ECL_STANDALONE

std::string num2str(float x)
{
    std::stringstream flowstream;
    flowstream<<x;
    return flowstream.str();
}

std::string mat2Str(float x[],int length){
    std::string str="";
    str.append(num2str(x[0]));
    int f;
    f=(int)(sizeof(x)/sizeof(float));

    for(int i=1;i<length;i++)
    {
        str.append(",");
        str.append(num2str(x[i]));
    }
    return str;
}


class Ekf2;
std::ifstream imuread("../data/rtk_vision/sensor_combined_0.txt");
std::ifstream gpsread("../data/rtk_vision/gps_position_0.txt");
std::ifstream magread("../data/rtk_vision/magnetometer_0.txt");
std::ifstream airread("../data/rtk_vision/air_data_0.txt");
std::ifstream evqread("../data/rtk_vision/vision_attitude_0.txt");
std::ifstream evpread("../data/rtk_vision/vision_position_0.txt");
std::ofstream vehicle_attitude_out("../data/rtk_vision/vechile_attitude.txt");
std::ofstream vehicle_local_out("../data/rtk_vision/vechile_local_position.txt");
std::ofstream vehicle_global_out("../data/rtk_vision/vechile_global_position.txt");
std::ofstream sensor_bias_out("../data/rtk_vision/sensor_bias.txt");
std::ofstream estimate_status_out("../data/rtk_vision/estimate_status.txt");
std::ofstream ekf_innovations_out("../data/rtk_vision/ekf_innovations.txt");
std::ofstream euler_estimator("../results/euler_estimator.txt");
std::ofstream position_estimator("../results/position_estimator.txt");

bool bReadGPS=false, bmagread=false, bReadBaro=false,bReadevq=false,bReadevp=false;//是否读取gps,mag,baro,extel vision attitue and posion


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

    //vehicle_status_s vehicle_status = {};//增加一些飞机的状态



    float gyro_integral_dt = 0;float gyro_rad[3],accelerometer_m_s2[3];
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
        float gyro_integral[3];
        imuread >> gyro_rad[0];	imuread >> gyro_rad[1];	imuread >> gyro_rad[2];imuread>>gyro_integral_dt;
        gyro_integral_dt /= 1.e6f;	//s
        ECL_DEBUG("[gyro]:%f,%f,%f,%f s ", gyro_rad[0], gyro_rad[1], gyro_rad[2],gyro_integral_dt);

		gyro_integral[0] = gyro_rad[0] * gyro_integral_dt;
		gyro_integral[1] = gyro_rad[1] * gyro_integral_dt;
		gyro_integral[2] = gyro_rad[2] * gyro_integral_dt;
        float accelerometer_timestamp_relative;imuread>>accelerometer_timestamp_relative;
        float accel_integral[3];
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
                printf("[mag]: %d %d %f %f %f %f\n",now,mag_time_us_read,mag_time_ms,
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

            // integrate time to monitor time slippage
            if (_start_time_us == 0) {
                _start_time_us = now;
                _last_time_slip_us = 0;

            } else if (_start_time_us > 0) {
                _integrated_time_us += gyro_integral_dt;
                _last_time_slip_us = (now - _start_time_us) - _integrated_time_us;
            }


            //output vehicle_attitude_s data
			matrix::Quaternion<float> q;
			_ekf.copy_quaternion(q.data());
            // In-run bias estimates
            float gyro_bias[3];
            std::string space_str=",";
            _ekf.get_gyro_bias(gyro_bias);
            {
                // generate vehicle attitude quaternion data
                vehicle_attitude_s att;
                att.timestamp = now;

                q.copyTo(att.q);
                _ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);

                att.rollspeed = gyro_rad[0] - gyro_bias[0];
                att.pitchspeed = gyro_rad[1] - gyro_bias[1];
                att.yawspeed = gyro_rad[2] - gyro_bias[2];

                // publish vehicle attitude
                //std::stringstream attitude_strs;

                //std::string file;
                vehicle_attitude_out<<att.timestamp<<space_str<<att.rollspeed<<space_str
                            <<att.pitchspeed<<space_str<<att.yawspeed<<space_str
                            <<att.q[0]<<space_str<<att.q[1]<<space_str<<att.q[2]
                            <<att.q[3]<<space_str<<att.delta_q_reset[0]<<space_str
                            <<att.delta_q_reset[1]<<space_str<<att.delta_q_reset[2]
                            <<space_str<<att.delta_q_reset[3]<<space_str<<att.quat_reset_counter<<std::endl;
                //printf((attitude_strs.str()));

                //vehicle_attitude_out<<attitude_strs.str()<<std::endl;

            }

            // generate vehicle local position data
            vehicle_local_position_s lpos={};
            lpos.timestamp = now;

            // Position of body origin in local NED frame
            float position[3];
            _ekf.get_position(position);
            const float lpos_x_prev = lpos.x;
            const float lpos_y_prev = lpos.y;
            lpos.x = (_ekf.local_position_is_valid()) ? position[0] : 0.0f;
            lpos.y = (_ekf.local_position_is_valid()) ? position[1] : 0.0f;
            lpos.z = position[2];

            // Velocity of body origin in local NED frame (m/s)
            float velocity[3];
            _ekf.get_velocity(velocity);
            lpos.vx = velocity[0];
            lpos.vy = velocity[1];
            lpos.vz = velocity[2];

            // vertical position time derivative (m/s)
            _ekf.get_pos_d_deriv(&lpos.z_deriv);

            // Acceleration of body origin in local NED frame
            float vel_deriv[3];
            _ekf.get_vel_deriv_ned(vel_deriv);
            lpos.ax = vel_deriv[0];
            lpos.ay = vel_deriv[1];
            lpos.az = vel_deriv[2];

            // TODO: better status reporting
            lpos.xy_valid = _ekf.local_position_is_valid() && !_preflt_horiz_fail;
            lpos.z_valid = !_preflt_vert_fail;
            lpos.v_xy_valid = _ekf.local_position_is_valid() && !_preflt_horiz_fail;
            lpos.v_z_valid = !_preflt_vert_fail;

            // Position of local NED origin in GPS / WGS84 frame
            map_projection_reference_s ekf_origin;
            uint64_t origin_time;
            // true if position (x,y,z) has a valid WGS-84 global reference (ref_lat, ref_lon, alt)
            const bool ekf_origin_valid = _ekf.get_ekf_origin(&origin_time, &ekf_origin, &lpos.ref_alt);
            lpos.xy_global = ekf_origin_valid;
            lpos.z_global = ekf_origin_valid;

            if (ekf_origin_valid && (origin_time > lpos.ref_timestamp)) {
                lpos.ref_timestamp = origin_time;
                lpos.ref_lat = ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
                lpos.ref_lon = ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees
            }

            // The rotation of the tangent plane vs. geographical north
            matrix::Eulerf euler(q);
            lpos.yaw = euler.psi();

            lpos.dist_bottom_valid = _ekf.get_terrain_valid();

            float terrain_vpos;
            _ekf.get_terrain_vert_pos(&terrain_vpos);
            lpos.dist_bottom = terrain_vpos - lpos.z; // Distance to bottom surface (ground) in meters

            // constrain the distance to ground to _rng_gnd_clearance
            if (lpos.dist_bottom < _rng_gnd_clearance) {
                lpos.dist_bottom = _rng_gnd_clearance;
            }

            lpos.dist_bottom_rate = -lpos.vz; // Distance to bottom surface (ground) change rate

            _ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
            _ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

            // get state reset information of position and velocity
            _ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
            _ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
            _ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
            _ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

            // get control limit information
            _ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max);

            // convert NaN to INFINITY
            if (!PX4_ISFINITE(lpos.vxy_max)) {
                lpos.vxy_max = INFINITY;
            }

            if (!PX4_ISFINITE(lpos.vz_max)) {
                lpos.vz_max = INFINITY;
            }

            if (!PX4_ISFINITE(lpos.hagl_min)) {
                lpos.hagl_min = INFINITY;
            }

            if (!PX4_ISFINITE(lpos.hagl_max)) {
                lpos.hagl_max = INFINITY;
            }
            //发布vehicle_local_position数据
            std::stringstream lposStream;//i need a printf function need neednnn
            vehicle_local_out<<lpos.timestamp<<space_str<<lpos.ref_timestamp<<space_str<<lpos.ref_lat<<space_str
                     <<lpos.ref_lon<<space_str<<lpos.x<<space_str<<lpos.y<<space_str<<lpos.z<<space_str<<lpos.delta_xy[0]
                    <<space_str<<lpos.delta_xy[1]<<space_str<<lpos.delta_z<<space_str<<lpos.vx<<space_str<<lpos.vy<<space_str
                   <<lpos.z_deriv<<space_str<<lpos.delta_vxy[0]<<space_str<<lpos.delta_vxy[2]<<space_str<<lpos.delta_vz<<space_str
                  <<lpos.yaw<<space_str<<lpos.ref_alt<<space_str<<lpos.dist_bottom<<space_str<<lpos.dist_bottom_rate<<space_str<<lpos.eph<<space_str
                 <<lpos.epv<<space_str<<lpos.evh<<space_str<<lpos.evv<<space_str<<lpos.vxy_max<<space_str<<lpos.hagl_min<<space_str<<lpos.hagl_max
                <<lpos.xy_valid<<space_str<<lpos.z_valid<<space_str<<lpos.v_xy_valid<<space_str<<lpos.v_z_valid<<space_str<<lpos.xy_reset_counter<<space_str
               <<lpos.z_reset_counter<<space_str<<lpos.vxy_reset_counter<<space_str<<lpos.vz_reset_counter<<space_str<<lpos.xy_global<<space_str<<lpos.z_global
              <<lpos.z_global<<space_str<<lpos.dist_bottom_valid<<space_str<<std::endl;

            //vehicle_local_out<<lposStream.str()<<std::endl;
            ECL_INFO("now:%ld,velocity: %f,%f,%f\n", now,velocity[0], velocity[1], velocity[2]);
            ECL_INFO("position: %lf,%lf,%lf\n", position[0], position[1], position[2]);


            if (_ekf.global_position_is_valid() && !_preflt_fail) {
                // generate and publish global position data
                vehicle_global_position_s global_pos;

                global_pos.timestamp = now;

                if (fabsf(lpos_x_prev - lpos.x) > FLT_EPSILON || fabsf(lpos_y_prev - lpos.y) > FLT_EPSILON) {
                    map_projection_reproject(&ekf_origin, lpos.x, lpos.y, &global_pos.lat, &global_pos.lon);
                }

                global_pos.lat_lon_reset_counter = lpos.xy_reset_counter;

                global_pos.alt = -lpos.z + lpos.ref_alt; // Altitude AMSL in meters

                // global altitude has opposite sign of local down position
                global_pos.delta_alt = -lpos.delta_z;

                global_pos.vel_n = lpos.vx; // Ground north velocity, m/s
                global_pos.vel_e = lpos.vy; // Ground east velocity, m/s
                global_pos.vel_d = lpos.vz; // Ground downside velocity, m/s

                global_pos.yaw = lpos.yaw; // Yaw in radians -PI..+PI.

                _ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv);

                global_pos.dead_reckoning = _ekf.inertial_dead_reckoning();

                global_pos.terrain_alt_valid = lpos.dist_bottom_valid;

                if (global_pos.terrain_alt_valid) {
                    global_pos.terrain_alt = lpos.ref_alt - terrain_vpos; // Terrain altitude in m, WGS84

                } else {
                    global_pos.terrain_alt = 0.0f; // Terrain altitude in m, WGS84
                }

                global_pos.dead_reckoning = _ekf.inertial_dead_reckoning(); // True if this position is estimated through dead-reckoning

                //_vehicle_global_position_pub.update();
                //发布vehicle_global_position信息
                //std::stringstream vehicleGlobalPosStream;
                vehicle_local_out<<global_pos.timestamp<<space_str<<global_pos.lat<<space_str<<global_pos.lon<<space_str<<global_pos.alt
                                     <<space_str<<global_pos.delta_alt<<space_str<<global_pos.vel_n<<space_str<<global_pos.vel_e<<space_str
                                    <<global_pos.vel_d<<space_str<<global_pos.yaw<<space_str<<global_pos.eph<<space_str<<global_pos.epv<<space_str
                                   <<global_pos.terrain_alt<<space_str<<global_pos.lat_lon_reset_counter<<space_str<<global_pos.alt_reset_counter<<space_str
                                  <<global_pos.terrain_alt_valid<<space_str<<global_pos.dead_reckoning<<std::endl;
                //vehicle_local_out<<vehicleGlobalPosStream.str()<<std::endl;
                ECL_INFO("now:%ld time :%ld position: %lf,%lf,%lf\n", now,global_pos.timestamp,global_pos.lat, global_pos.lon, global_pos.alt);


            }

            {
                // publish all corrected sensor readings and bias estimates after mag calibration is updated above
                float accel_bias[3];
                _ekf.get_accel_bias(accel_bias);

                sensor_bias_s bias;

                bias.timestamp = now;

                bias.accel_x = accelerometer_m_s2[0] - accel_bias[0];
                bias.accel_y = accelerometer_m_s2[1] - accel_bias[1];
                bias.accel_z = accelerometer_m_s2[2] - accel_bias[2];

                bias.gyro_x_bias = gyro_bias[0];
                bias.gyro_y_bias = gyro_bias[1];
                bias.gyro_z_bias = gyro_bias[2];

                bias.accel_x_bias = accel_bias[0];
                bias.accel_y_bias = accel_bias[1];
                bias.accel_z_bias = accel_bias[2];

                bias.mag_x_bias = _last_valid_mag_cal[0];
                bias.mag_y_bias = _last_valid_mag_cal[1];
                bias.mag_z_bias = _last_valid_mag_cal[2];

//                if (_sensor_bias_pub == nullptr) {
//                    _sensor_bias_pub = orb_advertise(ORB_ID(sensor_bias), &bias);

//                } else {
//                    orb_publish(ORB_ID(sensor_bias), _sensor_bias_pub, &bias);
//                }
                //发布传感器改正数据
                //std::stringstream sensorBiasStream;
                sensor_bias_out<<bias.timestamp<<space_str<<bias.accel_x<<space_str<<bias.accel_y<<space_str<<bias.accel_z
                               <<bias.gyro_x_bias<<space_str<<bias.gyro_y_bias<<space_str<<bias.gyro_z_bias<<space_str<<bias.accel_x_bias
                              <<space_str<<bias.accel_y_bias<<space_str<<bias.accel_z_bias<<space_str<<bias.mag_x_bias<<space_str<<bias.mag_y_bias
                             <<space_str<<bias.mag_z_bias<<std:endl;
                //sensor_bias_out<<sensorBiasStream.str()<<std::endl;
                ECL_INFO("[sensor_bias]now:%ld,time:%ld,gyro:%f %f %f",now,bias.timestamp,bias.accel_x_bias,bias.accel_y,bias.accel_z);


            }

            // publish estimator status
            estimator_status_s status;
            status.timestamp = now;
            _ekf.get_state_delayed(status.states);
            _ekf.get_covariances(status.covariances);
            _ekf.get_gps_check_status(&status.gps_check_fail_flags);
            _ekf.get_control_mode(&status.control_mode_flags);
            _ekf.get_filter_fault_status(&status.filter_fault_flags);
            _ekf.get_innovation_test_status(&status.innovation_check_flags, &status.mag_test_ratio,
                            &status.vel_test_ratio, &status.pos_test_ratio,
                            &status.hgt_test_ratio, &status.tas_test_ratio,
                            &status.hagl_test_ratio, &status.beta_test_ratio);

            status.pos_horiz_accuracy =lpos.eph;
            status.pos_vert_accuracy = lpos.epv
                    ;
            _ekf.get_ekf_soln_status(&status.solution_status_flags);
            _ekf.get_imu_vibe_metrics(status.vibe);
            status.time_slip = _last_time_slip_us / 1e6f;
            status.nan_flags = 0.0f; // unused
            status.health_flags = 0.0f; // unused
            status.timeout_flags = 0.0f; // unused
            status.pre_flt_fail = _preflt_fail;

//            if (_estimator_status_pub == nullptr) {
//                _estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &status);

//            } else {
//                orb_publish(ORB_ID(estimator_status), _estimator_status_pub, &status);
//            }
            //发布EKF滤波信息（主要是滤波约束条件）
            std::stringstream ekfStatusStream;
            std::string states_str=mat2Str(status.states,sizeof(status.states)/sizeof(float));//状态向量
            std::string cov_str=mat2Str(status.covariances,sizeof(status.covariances)/sizeof(float));//状态向量协方差
            estimate_status_out<<status.timestamp<<space_str<<states_str<<space_str<<status.n_states<<status.vibe[0]
                          <<space_str<<status.vibe[1]<<space_str<<status.vibe[2]<<space_str<<cov_str<<space_str<<status.control_mode_flags<<space_str
                         <<status.pos_horiz_accuracy<<space_str<<status.pos_vert_accuracy<<space_str<<status.mag_test_ratio<<space_str<<status.vel_test_ratio
                        <<space_str<<status.pos_test_ratio<<space_str<<status.hgt_test_ratio<<space_str<<status.tas_test_ratio<<space_str<<status.hagl_test_ratio
                       <<space_str<<status.beta_test_ratio<<space_str<<status.time_slip<<space_str<<status.gps_check_fail_flags<<space_str<<status.filter_fault_flags
                      <<space_str<<status.innovation_check_flags<<space_str<<status.solution_status_flags<<space_str<<status.nan_flags<<space_str<<status.health_flags
                     <<space_str<<status.timeout_flags<<space_str<<status.pre_flt_fail<<std::endl;
            std::cout<<"timestamp"<<status.timestamp<<std::endl;
            std::cout<<states_str<<std::endl;
            //ECL_WARN("status.timestamp%d,mag_test_ratio%f，vel_test_ratio%f",status.timestamp,status.mag_test_ratio,status.vel_test_ratio);
            //ECL_WARN("control_mode_flags%d",status.control_mode_flags);
            //estimate_status_out<<ekfStatusStream.str()<<std::endl;



           {
                // publish estimator innovation data
                ekf2_innovations_s innovations;
                innovations.timestamp = now;
                _ekf.get_vel_pos_innov(&innovations.vel_pos_innov[0]);
                _ekf.get_aux_vel_innov(&innovations.aux_vel_innov[0]);
                _ekf.get_mag_innov(&innovations.mag_innov[0]);
                _ekf.get_heading_innov(&innovations.heading_innov);
                _ekf.get_airspeed_innov(&innovations.airspeed_innov);
                _ekf.get_beta_innov(&innovations.beta_innov);
                _ekf.get_flow_innov(&innovations.flow_innov[0]);
                _ekf.get_hagl_innov(&innovations.hagl_innov);
                _ekf.get_drag_innov(&innovations.drag_innov[0]);

                _ekf.get_vel_pos_innov_var(&innovations.vel_pos_innov_var[0]);
                _ekf.get_mag_innov_var(&innovations.mag_innov_var[0]);
                _ekf.get_heading_innov_var(&innovations.heading_innov_var);
                _ekf.get_airspeed_innov_var(&innovations.airspeed_innov_var);
                _ekf.get_beta_innov_var(&innovations.beta_innov_var);
                _ekf.get_flow_innov_var(&innovations.flow_innov_var[0]);
                _ekf.get_hagl_innov_var(&innovations.hagl_innov_var);
                _ekf.get_drag_innov_var(&innovations.drag_innov_var[0]);

                _ekf.get_output_tracking_error(&innovations.output_tracking_error[0]);

                std::stringstream ekf_innovations_stream;
                std::string vel_pos_ino_str=mat2Str(innovations.vel_pos_innov,6);
                std::string mag_ino_str=mat2Str(innovations.mag_innov,3);
                std::string mag_ino_var_str=mat2Str(innovations.mag_innov_var,3);
                std::string vel_pos_var_ino_str=mat2Str(innovations.vel_pos_innov_var,6);
                std::string output_tra_ero_str=mat2Str(innovations.output_tracking_error,3);
                ekf_innovations_stream<<innovations.timestamp<<space_str<<vel_pos_ino_str<<space_str<<mag_ino_str
                                     <<space_str<<innovations.heading_innov<<space_str<<innovations.airspeed_innov
                                    <<space_str<<innovations.beta_innov<<space_str<<innovations.flow_innov[0]<<space_str
                                   <<innovations.flow_innov[1]<<space_str<<innovations.hagl_innov<<space_str<<vel_pos_var_ino_str<<space_str
                                  <<mag_ino_var_str<<space_str<<innovations.heading_innov_var<<space_str<<innovations.airspeed_innov_var
                                 <<space_str<<innovations.beta_innov_var<<space_str<<innovations.flow_innov_var[0]<<space_str<<innovations.flow_innov_var[1]
                                <<space_str<<innovations.hagl_innov_var<<space_str<<innovations.output_tracking_error<<space_str<<output_tra_ero_str<<space_str
                               <<innovations.drag_innov[0]<<space_str<<innovations.drag_innov[1]<<space_str<<innovations.drag_innov_var[0]<<space_str<<innovations.drag_innov_var[1]
                              <<space_str<<innovations.aux_vel_innov[0]<<space_str<<innovations.aux_vel_innov[1];

                ekf_innovations_out<<ekf_innovations_stream.str()<<std::endl;


                // calculate noise filtered velocity innovations which are used for pre-flight checking
                //判断的部分在组合导航分析中意义不大，故不移植，会影响position的有效标识






             //TODO: uORB definition does not define what these variables are. We have assumed them to be horizontal and vertical 1-std dev accuracy in metres
            matrix::Vector3f pos_var, vel_var;
            _ekf.get_pos_var(pos_var);
            _ekf.get_vel_var(vel_var);
            //ECL_INFO("pos_var: %lf,%lf,%lf\n", pos_var(0), pos_var(1), pos_var(2) );
            //ECL_INFO("vel_var: %lf,%lf,%lf\n", vel_var(0), vel_var(1), vel_var(2) );

            }

        }
    ECL_INFO("end\n");


    }
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

    return 0;
}
