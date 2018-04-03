/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*                                                                          */
/*  Version of 2016-03-21                                   Lukas Kautzsch  */
/*==========================================================================*/

/* ---------------------------- Revision Note ------------------------------*/
/* Nov. 2012																*/
/*  CACC Application Enhancement											*/
/*  Modified by Joyoung Lee, University of Virginia.						*/
/*
/* Jul. 2013
/*  CACC Lane changing Model Enhancement
/*  Modified by Joyoung Lee, University of Virginia
/*
/* May. 2014
/*  Algorithm Minor revision
/*  Modified by Joyoung Lee, New Jersey Institute of Technology
/*
/* Nov. 2014
/*  Revised for VISSIM 6 environment, Joyoung Lee NJIT
/*  Note : VISSIM 7 (up to -04) did not work; it kept crashing when the first CACC vehicle reaches at the position of about 80%  of link length.
/*
/* Dec. 2014
/*  Revised for the CACC Simulation Manager program, Joyoung Lee NJIT
/*
/* Feb. 2015
/* Code Block descriptions (input/output flow) were added.
/* July 30. 2015 : Major revision by Peng Su to implement Bart van Arem's algorithm
/* August 2015 : Enhanced from the revision of July 30.

/* Dec. 11: Added HWLong to CACC cars, for mixed traffic simulations. If leading car is not CACC, use HWLong
/* Mar. 21: Get rid of the platoon-size re-organization code, instead, use a loose average platoon size parameter.
/* 		The old re-organization logic kept on creating problems. The new logic works pretty well.
/*==========================================================================*/
#include  <direct.h>  
#include  <stdio.h>
#include <io.h>

#include "DriverModel.h"
#include <stdio.h>
#include <iostream> 
#include <fstream>
#include <list>
#include <math.h>
#include <iostream>
#include <ctime>
#include <map>
#include <string> 
#include <algorithm> 



using namespace std;

/*==========================================================================*/

double  desired_acceleration = 0.0;
//double  desired_lane_angle = 0.0; // Radian
//long    active_lane_change = 0;
long    veh_active_lane_change = 0;
//long    rel_target_lane = 0;
long	veh_rel_target_lane = 0;
double  desired_velocity = 0.0;
double  desired_loc = 0.0;
double desired_spd = 0.0;
map<int,double> prev_veh_velocity;
map<int, double> prev_veh_odometer;
long    turning_indicator = 0;
long    vehicle_color = RGB(0, 0, 0);

double veh_x = 0.0;
double veh_y = 0.0;
double veh_z = 0.0;

long veh_type = 0;
long current_link = 0;
long current_lane = 0;
long lanes_current_link = 0; // Dec. 15. 2014
double timestep = 0.0;
long current_veh_no = 0;
long vissim_suggestion = 0; // 0 indicates no: not listen to VISSIM, 1 otherwise
long simple_lanechange = 0;
long adj_veh;
long adj_veh_class;
char* dt;
time_t now;
long AdjVehicles[5][5];
double AdjVehiclesWidth[5][5];
double AdjVehiclesSpeed[5][5];
double AdjVehiclesDist[5][5];
double AdjVehiclesAcc[5][5];
long AdjVehiclesLaneChange[5][5];
long AdjVehiclesCurrentLane[5][5];
long AdjVehiclesTargetLane[5][5];

//SZ:preceding vehicles array for CACC paper
// map<int,int> preceding_vehicles;
// map<int,double> preceding_vehiclesWidth;
// map<int, double> preceding_vehiclesSpeed;
// map<int, double> preceding_vehiclesDist;
// map<int, double> preceding_vehiclesAcc;
// map<int, long> preceding_vehiclesLaneChange;
// map<int, long> preceding_vehiclesCurrentLane;
// map<int, long> preceding_vehiclesTargetLane;
////
ofstream fout;//("out_newdll.txt",std::ios_base::app);
ofstream fout_ncacc;
ofstream fout_flw_vehicles;
ofstream fout_lqr;
ofstream fout_mpc;
ofstream fout_velocity_bds;
std::streambuf* cout_sbuf;
ofstream fout_AMPL_debug;
ifstream fin;
string str;
char ch;


class MyOutputHandler : public ampl::OutputHandler {
public:
	void output(ampl::output::Kind kind, const char* output) {
		if (kind == ampl::output::SOLVE){
			//fout.open("C:\\Users\\sdzhao\\Documents\\2017_fall\\out_102_mpc.csv", ofstream::out);
			//std::printf("SZ Solver: %s\n", output);
			fout_mpc << output << endl;
		}
		else
		{
			//fout.open("C:\\Users\\sdzhao\\Documents\\2017_fall\\out_102_mpc.csv", ofstream::out);
			//std::printf("SZ SHELL: %s\n", output);
			fout_mpc << output << endl;
		}
	}
};

map<long, int> VehTargetLane;
double veh_len = 0.0;

double Spd_vehicle = 0.0;
double Acc_vehicle = 0.0;
double Leng_vehicle = 0.0;
double MaxAcc_vehicle = 0.0;
double lateral_pos = 0.0;
double lateral_pos_ind = 0.0;
double veh_odometer = 0.0;
bool WrtFlag = false;
double lane_angle = 0.021;
double Veh_Weight_kg = 0.0;

int MaxPlatoonSize = 5;
double HWShort = 0.6;
double HWLong = 2.0;  //headway when the vehicle in front is not Connected. Ego vehicle is actually in ACC mode.
int leaderID = 0;
int followerID = 0;
double front_dist = 0.0;
double rear_dist = 0.0;
double const head_speed = 26.3889;  // in meters per second

map<long, int[7]> platoonState;

//SZ:data-driven MPC
map<int, double>CAV_init_speed;
map<int, double>CAV_init_spacing;
map<int, double>CAV_init_acc;
map<int, double> veh_len_vec;
map<int, double> veh_odometer_vec;
map<int, double> veh_coord_x_vec;
map<int, double> veh_coord_y_vec;
map<int, double> front_veh_speed_vec;
map<int, double> front_veh_acc_vec;

vector<int> unique_vehicle_no;
int adj_index1, adj_index2 = -1;

int leading_CAV_ID = -1;
int rolling_horizon = 50; //subseconds
int alpha_vehicle_no = 5;
list<int> observed_accelerations; //acc*10

vector<double> MPC_control_acceleration;
map<int, double> MPC_optimal_accelerations;

// LQR
map<int, int> veh_no_to_index;
bool b_jam_density_method = false;
double Predictive_headway = 1.6; 
SolverID solver = Car_Following_Control;// Predictive_deter_Tracking_MPC;// Predictive_DRO_Reference_Stochastic_SDP;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// Predictive_constant_tracking_MPC;// Predictive_deter_instananeous_Regulating_MPC;// ; // ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// Predictive_constant_regulating_MPC;// ;// Predictive_deter_Tracking_MPC;// Predictive_constant_tracking_MPC;// ;// ;// ;// ;// Predictive_constant_tracking_MPC;// ;// ;// ;// Predictive_deter_Tracking_MPC;// Predictive_deter_instananeous_Tracking_MPC;// ;// Predictive_deter_instananeous_Tracking_MPC;// ;// ;// ;// ;// Predictive_DRO_Reference_Stochastic_SDP;// ;// ;// Predictive_constant_tracking_MPC;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// ;// Predictive_deter_instananeous_Regulating_MPC;// ;// Predictive_deter_Tracking_MPC;// Predictive_deter_instananeous_Regulating_MPC;// Predictive_DRO_Reference_Stochastic_SDP;// Predictive_DRO_Tracking_MPC;// ;// ;// ;// Predictive_constant_tracking_MPC;// Predictive_deter_instananeous_Tracking_MPC;// Predictive_constant_regulating_MPC;// Predictive_deter_Tracking_MPC;// Predictive_constant_tracking_MPC; //Predictive_deter_Regulating_MPC;// ;// Predictive_deter_Regulating_MPC;// ;// ;// Predictive_deter_instananeous_Regulating_MPC;// ;// ;// ;// Predictive_deter_Regulating_MPC;// observer_based_feedback_tracking; // LQR_state_feedback;// Predictive_deter_Regulating_MPC;// ;// ;//Car_Following_Control;// MPC_QP;// Data_driven_MPC;// Stochastic_MPC_QP;// MPC_QP;//observer_based_feedback_tracking;//LQR_state_feedback_reference_tracking; //LQR_state_feedback; //
Engine *ep = engOpen("");

bool b_lqr_init = false;
LQR_Data LQR_data;
//LQR lqr_global;
LQR_Initial_State lqr_initial_state;
//MPC
MPC_Initial_State mpc_initial_state;
bool b_mpc_init = false;
bool b_mpc_headway_ready =false;
map<int, bool> vno_mpc_headway_ready_flag;
MPC_Data mpc_data;

VehicleState lead_CAV_state;
map<int, VehicleState> front_veh_state_map;
SpeedPredictionModel speed_predict_model = NEWELLS_SIMPLIFIED_MODEL;

//front space-time region
int front_region_observation_length = 250;
deque<map<int, VehicleState>> lead_CV_front_vehicles_state_t_vno_indices;
vector<vector<FundamentalDiagram>> sensor_t_FD_vector;
//


// car-following calibration
CarFollowingParam car_following_param;
map<int, bool> veh_no_car_following_update_stauts;
ofstream fout_car_following;
double last_des_spd = 0.0;
ofstream fout_car_following_debug;
map<int, vector<NGSIMData>> NGSIM_data_vector;

map<int, NGSIMData> first_veh_trajectory_data;
vector<map<int, NGSIMData>> front_region_veh_time_trajectory_data;
//

double get_current_cpu_time_in_seconds()
{
	LARGE_INTEGER current_cpu_time;
	LARGE_INTEGER query_performance_frequency;
	double current_cpu_time_in_seconds;

	QueryPerformanceFrequency(&query_performance_frequency);
	QueryPerformanceCounter(&current_cpu_time);

	current_cpu_time_in_seconds = (double)(current_cpu_time.QuadPart / ((double)query_performance_frequency.QuadPart));

	return current_cpu_time_in_seconds;
};

void push_acceleration_observation(double acc){
	if (observed_accelerations.size() >= 50){
		observed_accelerations.pop_front();
	}
	observed_accelerations.push_back((int)(100.0*acc));
};

double find_min_acceleration(){
	return (double)((*std::min_element(observed_accelerations.begin(), observed_accelerations.end()))/100.0);
}

double find_predicted_acceleration(){
	map<int, int> cmap;
	using pair_type = decltype(cmap)::value_type;
	if (observed_accelerations.empty()) return find_min_acceleration();
	//int max_count=-1, p_ac=999;
	for (auto ac : observed_accelerations){		
		cmap[ac]++;
	}

	auto pr = std::max_element
		(
		std::begin(cmap), std::end(cmap),
		[](const pair_type & p1, const pair_type & p2) {
		return p1.second < p2.second;
		}
	);
	return double(double(pr->first) / 100.0);
	//observed_accelerations.
};
/*  &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& */




/*==========================================================================*/

BOOL APIENTRY DllMain(HANDLE  hModule,
	DWORD   ul_reason_for_call,
	LPVOID  lpReserved)
{
	switch (ul_reason_for_call) {
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelSetValue(long   type,
	long   index1,
	long   index2,
	long   long_value,
	double double_value,
	char   *string_value)
{
	/* Sets the value of a data object of type <type>, selected by <index1> */
	/* and possibly <index2>, to <long_value>, <double_value> or            */
	/* <*string_value> (object and value selection depending on <type>).    */
	/* Return value is 1 on success, otherwise 0.                           */

	switch (type) {
	case DRIVER_DATA_PATH:
	case DRIVER_DATA_TIMESTEP:
		return 1;
	case DRIVER_DATA_TIME:
		timestep = double_value;
		return 1;
	case DRIVER_DATA_VEH_ID:
		current_veh_no = long_value;
		return 1;
	case DRIVER_DATA_VEH_LANE:
		current_lane = long_value;
		return 1;
	case DRIVER_DATA_VEH_ODOMETER:
		veh_odometer = double_value;
		return 1;
	case DRIVER_DATA_VEH_LANE_ANGLE:
		return 1;
	case DRIVER_DATA_VEH_LATERAL_POSITION:
		lateral_pos = double_value;
		return 1;
	case DRIVER_DATA_VEH_VELOCITY:
		Spd_vehicle = double_value;
		return 1;
	case DRIVER_DATA_VEH_ACCELERATION:
		Acc_vehicle = double_value;
		return 1;
	case DRIVER_DATA_VEH_LENGTH:
		veh_len = double_value;
		return 1;
	case DRIVER_DATA_VEH_WIDTH:
	case DRIVER_DATA_VEH_WEIGHT:
		Veh_Weight_kg = double_value;
		return 1;
	case DRIVER_DATA_VEH_MAX_ACCELERATION:
		MaxAcc_vehicle = double_value;
		return 1;
	case DRIVER_DATA_VEH_TURNING_INDICATOR:
		turning_indicator = long_value;
		return 1;
	case DRIVER_DATA_VEH_CATEGORY:
	case DRIVER_DATA_VEH_PREFERRED_REL_LANE:
	case DRIVER_DATA_VEH_USE_PREFERRED_LANE:
		return 1;
	case DRIVER_DATA_VEH_DESIRED_VELOCITY:
		desired_velocity = double_value;
		return 1;
	case DRIVER_DATA_VEH_X_COORDINATE:
		veh_x = double_value;
		return 1;
	case DRIVER_DATA_VEH_Y_COORDINATE:
		veh_y = double_value;
		return 1;
	case DRIVER_DATA_VEH_TYPE:
		veh_type = long_value;
		return 1;
	case DRIVER_DATA_VEH_COLOR:
		vehicle_color = long_value;
		return 1;
	case DRIVER_DATA_VEH_CURRENT_LINK:
		current_link = long_value;
		return 0; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
		/* Must return 1 if these messages are to be sent from VISSIM!         */
	case DRIVER_DATA_VEH_NEXT_LINKS:
	case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE:
		veh_active_lane_change = long_value;
		return 1;
	case DRIVER_DATA_VEH_REL_TARGET_LANE:
		veh_rel_target_lane = long_value;
		return 1;
	case DRIVER_DATA_NVEH_ID:
		AdjVehicles[index1 + 2][index2 + 2] = long_value;
		//preceding_vehicles[current_veh_no] = long_value;
		adj_index1 = index1;
		adj_index2 = index2;
		return 1;
	case DRIVER_DATA_NVEH_LANE_ANGLE:
	case DRIVER_DATA_NVEH_LATERAL_POSITION:
		return 1;
	case DRIVER_DATA_NVEH_DISTANCE:
		AdjVehiclesDist[index1 + 2][index2 + 2] = double_value;
		//preceding_vehiclesDist[current_veh_no] = double_value;
		return 1;
	case DRIVER_DATA_NVEH_REL_VELOCITY:
		AdjVehiclesSpeed[index1 + 2][index2 + 2] = Spd_vehicle - double_value;
		//preceding_vehiclesSpeed[current_veh_no] = Spd_vehicle - double_value;
		return 1;
	case DRIVER_DATA_NVEH_ACCELERATION:
		AdjVehiclesAcc[index1 + 2][index2 + 2] = double_value;
		//preceding_vehiclesAcc[current_veh_no] = double_value;
		return 1;
	case DRIVER_DATA_NVEH_LENGTH:
		AdjVehiclesWidth[index1 + 2][index2 + 2] = double_value; // revised for VISSIM 7.
		//preceding_vehiclesWidth[current_veh_no] = double_value;
		return 1;
	case DRIVER_DATA_NVEH_WIDTH:
		return 1;
	case DRIVER_DATA_NVEH_WEIGHT:
	case DRIVER_DATA_NVEH_TURNING_INDICATOR:
	case DRIVER_DATA_NVEH_CATEGORY:
		return 1;
	case DRIVER_DATA_NVEH_LANE_CHANGE:
		AdjVehiclesLaneChange[index1 + 2][index2 + 2] = long_value;
		return 1;
	case DRIVER_DATA_NO_OF_LANES:
		lanes_current_link = long_value;
		return 1;
	case DRIVER_DATA_LANE_WIDTH:
	case DRIVER_DATA_LANE_END_DISTANCE:
	case DRIVER_DATA_RADIUS:
	case DRIVER_DATA_MIN_RADIUS:
	case DRIVER_DATA_DIST_TO_MIN_RADIUS:
	case DRIVER_DATA_SLOPE:
	case DRIVER_DATA_SLOPE_AHEAD:
	case DRIVER_DATA_SIGNAL_DISTANCE:
	case DRIVER_DATA_SIGNAL_STATE:
	case DRIVER_DATA_SIGNAL_STATE_START:
	case DRIVER_DATA_SPEED_LIMIT_DISTANCE:
	case DRIVER_DATA_SPEED_LIMIT_VALUE:
		return 1;
	case DRIVER_DATA_DESIRED_ACCELERATION:
		desired_acceleration = double_value;
		return 1;
	case DRIVER_DATA_DESIRED_LANE_ANGLE:
		//desired_lane_angle = double_value;
		return 1;
// 	case DRIVER_DATA_ACTIVE_LANE_CHANGE:
// 		active_lane_change = long_value;
// 		return 1;
// 	case DRIVER_DATA_REL_TARGET_LANE:
// 		rel_target_lane = long_value;
// 		return 1;
	default:
		return 0;
	}
}

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue(long   type,
	long   index1,
	long   index2,
	long   *long_value,
	double *double_value,
	char   **string_value)
{
	/* Gets the value of a data object of type <type>, selected by <index1> */
	/* and possibly <index2>, and writes that value to <*double_value>,     */
	/* <*float_value> or <**string_value> (object and value selection       */
	/* depending on <type>).                                                */
	/* Return value is 1 on success, otherwise 0.                           */

	switch (type) {
	case DRIVER_DATA_STATUS:
		*long_value = 0;
		return 1;
	case DRIVER_DATA_VEH_TURNING_INDICATOR:
		*long_value = turning_indicator;
		return 1;
	case DRIVER_DATA_VEH_DESIRED_VELOCITY:
		//*double_value = desired_velocity;
		return 1;
	case DRIVER_DATA_VEH_COLOR:
		*long_value = vehicle_color;
		return 1;
	case DRIVER_DATA_WANTS_SUGGESTION:  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		*long_value = 0;
		return 1;
	case DRIVER_DATA_DESIRED_ACCELERATION:
		*double_value = desired_acceleration;
		return 1;
// 	case DRIVER_DATA_DESIRED_LANE_ANGLE:
// 		*double_value = desired_lane_angle;
// 		return 1;
// 	case DRIVER_DATA_ACTIVE_LANE_CHANGE:
// 		*long_value = active_lane_change;
// 		return 1;
// 	case DRIVER_DATA_REL_TARGET_LANE:
// 		*long_value = rel_target_lane;
// 		return 1;
	case DRIVER_DATA_SIMPLE_LANECHANGE: //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		*long_value = 0;
		return 1;
	default:
		return 0;
	}
}

void get_all_file_names_under_one_folder(string working_dir, vector<string> &driving_cycle_csv_files, int &driving_cycle_csv_file_index, string suffix)
{

	intptr_t hfile;//long hfile;
	//struct _finddata_t data[MAX_Number_CSV_File];
	_finddata_t fileinfo;
	string driving_cycle_dir_str = working_dir;// +"\\driving_cycle";
	const char * driving_cycle_dir = driving_cycle_dir_str.c_str();

	string fname(driving_cycle_dir);
	fname = fname + "\\*." + suffix;// "\\*.csv";
	if ((hfile = _findfirst(fname.c_str(), &fileinfo)) == -1)
	{
		perror("_findfirst:");
		//exit(1);
	}
	else
	{
		do
		{
			driving_cycle_csv_files.push_back(string(fileinfo.name));
			driving_cycle_csv_file_index += 1;
			//if (driving_cycle_csv_file_index%50==0) cout << driving_cycle_csv_file_index << endl;
		} while (_findnext(hfile, &fileinfo) == 0);
		_findclose(hfile);
	}

}

string debug_dir = "\\AMPL_debug\\";
vector<string> AMPL_debug_fn_vec;
int AMPL_debug_fn_ctr = 0;
string last_debug_fn;
size_t debug_pos;
int debug_fn_index;
string out_debug_index;

string CACC_result_dir = "\\CACC_result\\";
vector<string> CACC_fn_vec;
int CACC_fn_ctr = 0;
string last_CACC_fn;
size_t CACC_pos;
int CACC_fn_index;
string out_CACC_index;
ampl::AMPL ampl_obj;

string S2V_Message_Data_to_string(S2V_Message_Data &s2v_message_data){
	string ret_message = "";
	string delimiter = ",";

	ret_message += to_string(s2v_message_data.S2V_platoon_size) + delimiter;
	ret_message += to_string(s2v_message_data.S2V_leading_CAV_ID) + delimiter;
	ret_message += to_string(s2v_message_data.S2V_vissim_timestep) + delimiter;
	ret_message += to_string(s2v_message_data.S2V_lead_CAV_speed) + delimiter;
	ret_message += to_string(s2v_message_data.S2V_front_veh_speed) + delimiter;
	ret_message += to_string(s2v_message_data.S2V_front_spacing) + delimiter;

	for (int i = 0; i < s2v_message_data.S2V_platoon_size; i++){
		ret_message += to_string(s2v_message_data.S2V_vehicle_init_speed[i]) + delimiter;
	}
	for (int i = 0; i < s2v_message_data.S2V_platoon_size; i++){
		ret_message += to_string(s2v_message_data.S2V_vehicle_init_spacing[i]) + delimiter;
	}
	for (int i = 0; i < s2v_message_data.S2V_platoon_size; i++){
		ret_message += to_string(s2v_message_data.S2V_vehicle_init_acc[i]) + delimiter;
	}
	for (int i = 0; i < s2v_message_data.S2V_platoon_size; i++){
		ret_message += to_string(s2v_message_data.S2V_vehicle_length[i]) + delimiter;
	}
	for (int i = 0; i < s2v_message_data.S2V_platoon_size; i++){
		ret_message += to_string(s2v_message_data.S2V_vehicle_odometer[i]) + delimiter;
	}
	for (int i = 0; i < s2v_message_data.S2V_platoon_size; i++)
	{
		ret_message += to_string(s2v_message_data.S2V_vehicle_x[i]) + delimiter;
	}	
	for (int i = 0; i < s2v_message_data.S2V_platoon_size; i++){
		ret_message += to_string(s2v_message_data.S2V_vehicle_y[i]) + delimiter;
	}
	for (int i = 0; i < s2v_message_data.S2V_platoon_size; i++){
		ret_message += to_string(s2v_message_data.S2V_front_CAV_spd[i]) + delimiter;
	}
	for (int i = 0; i < s2v_message_data.S2V_platoon_size; i++){
		ret_message += to_string(s2v_message_data.S2V_front_CAV_acc[i]) + delimiter;
	}
	
	for (int i = 0; i < s2v_message_data.S2V_platoon_size; i++){
		ret_message += to_string(s2v_message_data.S2V_unique_vehicle_no_vec[i]);
		if (i != s2v_message_data.S2V_platoon_size-1)
		{
			ret_message += delimiter;
		}
	}

	return ret_message;
};

void S2V_Message_Data_to_Real_Time_Sensing_Data(vector<RealTimeInVehicleSensingData> &real_time_sensing_data_vector, S2V_Message_Data &s2v_message_data, map<int, V2S_Message_Data> & V2S_message_vector)
{
	real_time_sensing_data_vector.clear();
	for (int i = 0; i < s2v_message_data.S2V_platoon_size; i++)
	{
		RealTimeInVehicleSensingData realtimeinvehiclesensingdata;

		realtimeinvehiclesensingdata.vehicle_id = s2v_message_data.S2V_unique_vehicle_no_vec[i];// -1;
		realtimeinvehiclesensingdata.simulation_time_step_second = s2v_message_data.S2V_vissim_timestep;//unit subsecond; 0.0;
		realtimeinvehiclesensingdata.current_time_second = realtimeinvehiclesensingdata.simulation_time_step_second;//should we use VISSIM time or clock time? 0.0;
		realtimeinvehiclesensingdata.vehicle_lane = 0;//currently not using
		realtimeinvehiclesensingdata.vehicle_odometer_meter = s2v_message_data.S2V_vehicle_odometer[i];// 0.0;
		realtimeinvehiclesensingdata.vehicle_lane_angle_rad = 0.0; //currently not using
		realtimeinvehiclesensingdata.vehicle_lateral_position_meter = 0.0;//currently not using
		realtimeinvehiclesensingdata.vehicle_velocity_meter_second = s2v_message_data.S2V_vehicle_init_speed[i];// 0.0;//
		realtimeinvehiclesensingdata.vehicle_accelaration_meter_second2 = s2v_message_data.S2V_vehicle_init_acc[i];// 0.0;
		realtimeinvehiclesensingdata.vehicle_x_coordinate = s2v_message_data.S2V_vehicle_x[i];// 0.0;
		realtimeinvehiclesensingdata.vehicle_y_coordinate = s2v_message_data.S2V_vehicle_y[i];// 0.0;
		realtimeinvehiclesensingdata.vehicle_z_coordinate = 0.0;//currently not using, wait for google API
		realtimeinvehiclesensingdata.front_vehicle_distance_gap_meter = s2v_message_data.S2V_vehicle_init_spacing[i];// 0.0;
		realtimeinvehiclesensingdata.front_vehicle_velocity_meter = s2v_message_data.S2V_front_CAV_spd[i];// 0.0;
		realtimeinvehiclesensingdata.front_vehicle_relative_velocity_meter_second = realtimeinvehiclesensingdata.front_vehicle_velocity_meter - realtimeinvehiclesensingdata.vehicle_velocity_meter_second;// 0.0;
		realtimeinvehiclesensingdata.front_vehicle_acceleration_meter_second2 = s2v_message_data.S2V_front_CAV_acc[i];// 0.0;

		realtimeinvehiclesensingdata.cloud_message_id = V2S_message_vector[realtimeinvehiclesensingdata.vehicle_id].Cloud_Message_ID;//0;

		real_time_sensing_data_vector.push_back(realtimeinvehiclesensingdata);
	}
	//return realtimeinvehiclesensingdata;
};
void write_vehicle_records_to_NGSIM(map<int, vector<NGSIMData>> & NGSIM_data, int veh_no){
	//fout_car_following;
	//fout_car_following << "size " << NGSIM_data.size() << endl;
	//for (auto &iter : NGSIM_data){
	vector<NGSIMData> &vec = NGSIM_data[veh_no];
	int total_frame = vec.size();
		for (int i = 0; i < total_frame; i++){
			fout_car_following
				<< vec[i].Vehicle_ID << ","
				<< vec[i].Frame_ID << ","
				<< total_frame << ","
				<< vec[i].Global_Time << ","
				<< fixed << setprecision(3) << vec[i].Local_X << ","
				<< fixed << setprecision(3) << vec[i].Local_Y << ","
				<< fixed << setprecision(3) << vec[i].Global_X << ","
				<< fixed << setprecision(3) << vec[i].Global_Y << ","
				<< fixed << setprecision(3) << vec[i].Vehicle_Length << ","
				<< fixed << setprecision(3) << vec[i].Vehicle_Width << ","
				<< vec[i].Vehicle_Class << ","
				<< fixed << setprecision(3) << vec[i].Vehicle_Velocity << ","
				<< fixed << setprecision(3) << vec[i].Vehicle_Acceleration << ","
				<< vec[i].Lane_Identification << ","
				<< vec[i].Preceding_Vehicle << ","
				<< vec[i].Following_Vehicle << ","
				<< fixed << setprecision(3) << vec[i].Spacing << ","
				<< fixed << setprecision(3) << vec[i].Headway << endl;
		}
//	}
	//fout_car_following.close();
};
void push_vehicle_records_to_NGSIM(map<int, vector<NGSIMData>> & NGSIM_data_map){
	
	NGSIMData NGSIM_data;
	NGSIM_data.Vehicle_ID = current_veh_no; //1 
	NGSIM_data.Frame_ID = (int)(timestep*10.0);					//2
	NGSIM_data.Total_Frames = NGSIM_data.Frame_ID; //3

	NGSIM_data.Global_Time = -1; //4

	NGSIM_data.Local_X = veh_x * 3.28084; //5

	NGSIM_data.Local_Y = veh_y * 3.28084; //6

	NGSIM_data.Global_X = veh_x * 3.28084; //7 NAD83

	NGSIM_data.Global_Y = veh_y * 3.28084; //8 NAD83

	NGSIM_data.Vehicle_Length = veh_len * 3.28084; //9

	NGSIM_data.Vehicle_Width = 4.1*3.28084; //10

	NGSIM_data.Vehicle_Class = 100; //11

	NGSIM_data.Vehicle_Velocity = Spd_vehicle*3.28084; //12

	NGSIM_data.Vehicle_Acceleration = Acc_vehicle*3.28084; //13

	NGSIM_data.Lane_Identification = current_lane; //14

	NGSIM_data.Preceding_Vehicle = leaderID; //15

	NGSIM_data.Following_Vehicle = followerID; //16

	NGSIM_data.Spacing = AdjVehiclesDist[2][3] * 3.28084; //17

	NGSIM_data.Headway = NGSIM_data.Spacing / NGSIM_data.Vehicle_Velocity; //18

	NGSIM_data_map[current_veh_no].push_back(NGSIM_data);
};

void read_NGSIM_summary(string scenario_fn){
	ifstream f_scenario(scenario_fn, fstream::in);
	string line;
	if (f_scenario.is_open())
	{
		//fout_car_following << "open NGSIM summary: " << scenario_fn << endl; 
		int iline = 0;
		vector<string> tokens;
		while (f_scenario.good())
		{
			getline(f_scenario, line);
			iline++;
			if (iline >= 4 && line != "") // skip the first line
			{
				string_split(tokens, line, ",");
				if (iline == 4) car_following_param.max_acc = stof(tokens[1]);
				else if (iline == 5) car_following_param.min_acc = stof(tokens[1]);
				else if (iline == 6) car_following_param.first_veh_NGSIM_ID = stoi(tokens[1]);
				else if (iline == 7) car_following_param.T = stof(tokens[1]);
				else if (iline == 8) car_following_param.desired_min_spacing = stof(tokens[1]);
			}
		}
	}
	else
	{
		//fout_car_following << "Cannot open " << scenario_fn << endl;
	}
	f_scenario.close();
}

void read_first_vehicle_trajectory(string input_fn, map<int, NGSIMData> &trajectory_data){
	double entry_time_deduct = 0.0;
	double prev_global_Y = 0.0;
	ifstream f(input_fn, fstream::in);
	string line;
	if (f.is_open())
	{
		//fout_car_following << "open first_vehicle_trajectory: " << input_fn << endl;
		//fout_mpc << "open first_vehicle_trajectory: " << input_fn << endl;
		int iline = 0;
		vector<string> tokens;
		while (f.good())
		{
			getline(f, line);
			iline++;
			if (iline >= 1 && line != "") // skip the first line
			{
				string_split(tokens, line, ",");
				NGSIMData ngsim_data;
				ngsim_data.Vehicle_ID = stoi(tokens[0]);
				ngsim_data.Frame_ID = stoi(tokens[1]);
				ngsim_data.Frame_ID = ngsim_data.Frame_ID - entry_time_deduct;
				if (iline == 1){
					entry_time_deduct = ngsim_data.Frame_ID - 3;
					ngsim_data.Frame_ID = 3;
				}
				
				ngsim_data.Vehicle_Velocity = stof(tokens[2]);		//mps
				//ngsim_data.Vehicle_Acceleration = stof(tokens[3]);	//mps2
				ngsim_data.Global_Y = stof(tokens[4]);				//meters
				if (iline == 1){
					ngsim_data.Vehicle_Acceleration = 0;
				}
				else
				{
					trajectory_data[ngsim_data.Frame_ID - 1].Vehicle_Acceleration = (2 * (ngsim_data.Global_Y - prev_global_Y - 0.1*trajectory_data[ngsim_data.Frame_ID - 1].Vehicle_Velocity)) / 0.01;
					//ngsim_data.Vehicle_Acceleration = (2 * (ngsim_data.Global_Y - prev_global_Y)) / 0.01; //mps
				}
				
				trajectory_data[ngsim_data.Frame_ID]=ngsim_data;
				fout_car_following_debug << "ngsim_data.Frame_ID: " << ngsim_data.Frame_ID << ",acc:" << trajectory_data[ngsim_data.Frame_ID - 1].Vehicle_Acceleration << ",spd," << trajectory_data[ngsim_data.Frame_ID - 1].Vehicle_Velocity << ",loc," << ngsim_data.Global_Y << ",prev_loc," << prev_global_Y << endl;
				
				prev_global_Y = ngsim_data.Global_Y;
			}
		}
	}
	else
	{
		fout_car_following_debug << "Cannot open " << input_fn << endl;
		//fout_mpc << "Cannot open " << input_fn << endl;
	}
	f.close();
};



void read_front_space_time_region_vehicle_trajectory(string input_fn, string out_raw_platoon_traj_fn, vector<map<int, NGSIMData>> &veh_time_trajectory_data){
	map<int, map<int, NGSIMData>> vno_time_trajectory_data;
	double entry_time_deduct = 0.0;

	ifstream f(input_fn, fstream::in);
	string line;
	if (f.is_open())
	{
		//fout_car_following << "open first_vehicle_trajectory: " << input_fn << endl;
		//fout_mpc << "open first_vehicle_trajectory: " << input_fn << endl;
		int iline = 0;
		vector<string> tokens;
		while (f.good())
		{
			getline(f, line);
			iline++;
			if (iline >= 2 && line != "") // skip the first line
			{
				string_split(tokens, line, ",");
				NGSIMData ngsim_data;
				ngsim_data.Vehicle_ID = stoi(tokens[0]);
				ngsim_data.Frame_ID = stoi(tokens[1]);
				ngsim_data.Frame_ID = ngsim_data.Frame_ID - entry_time_deduct;
				if (iline == 2){
					entry_time_deduct = ngsim_data.Frame_ID - 3.0;
					ngsim_data.Frame_ID = 3.0;
				}
				
				ngsim_data.Vehicle_Velocity = stof(tokens[2]);		//mps
				//ngsim_data.Vehicle_Acceleration = stof(tokens[3]);	//mps2
				ngsim_data.Global_Y = stof(tokens[3]);				//meters

				vno_time_trajectory_data[ngsim_data.Vehicle_ID][ngsim_data.Frame_ID] = ngsim_data;
			}
		}
	}
	else
	{
		//fout_car_following << "Cannot open " << input_fn << endl;
		//fout_mpc << "Cannot open " << input_fn << endl;
	}
	f.close();
	veh_time_trajectory_data.clear();
	

	//output raw platoon trajectories
	ofstream of(out_raw_platoon_traj_fn, fstream::out);
	int veh_no = 1;
	int prev_veh_no = -1;
	double veh_acc_ = 0.0;
	double prev_spd = 0.0;
	if (of.is_open())
	{
		for (auto &iter : vno_time_trajectory_data)
		{
			veh_time_trajectory_data.push_back(iter.second);
			for (auto & frame_: iter.second)
			{
				if (prev_veh_no != veh_no)				
					veh_acc_ = 0.0;
				else
				{
					veh_acc_ = (frame_.second.Vehicle_Velocity - prev_spd) / 0.1;
				}
				//while (of.good())
				//{
					of << veh_no << ","
						<< (double)frame_.second.Frame_ID / 10 << ","
						<< frame_.second.Global_Y << ","
						<< veh_acc_ << ","
						<< frame_.second.Vehicle_Velocity << ","
						<< -1 << ","
						<< -1 << ","
						<< -1 << ","
						<< -1 << ","
						<< -1 << ","
						<< -1 << ","
						<< -1 << ","
						<< -1 << ","
						<< -1 << ","
						<< -1 << endl;
				//}
					prev_veh_no = veh_no;
					prev_spd = frame_.second.Vehicle_Velocity;
			}
			fout_mpc << "____veh_time_trajectory_data----vn," << iter.first << ",num_obs," << iter.second.size() << endl;
			veh_no++;
		}
		
	}
	of.close(); 


	sort(veh_time_trajectory_data.begin(), veh_time_trajectory_data.end(), CompareByEntryTime);
};

int CountVehicles(int sensor_length, int ilink, int SelectedStartTime, int SelectedEndTime, float SelectedStartLocalY, float SelectedEndLocalY, float &time_mean_spd, int &num_spd)
{
	double sum_spd = 0.0;

	int m_SelectedVehicleCount = 0;

	if (SelectedStartTime > SelectedEndTime)  //swap timestamps and locations
	{
		int temp = SelectedStartTime;
		SelectedStartTime = SelectedEndTime;
		SelectedEndTime = temp;

		float temp_y = SelectedStartLocalY;
		SelectedStartLocalY = SelectedEndLocalY;
		SelectedEndLocalY = temp_y;
	}
	bool intersection_flag;
	for (int t = SelectedStartTime; t <= SelectedEndTime; t++)  // first loop
	{	//
		float lower_y = lead_CV_front_vehicles_state_t_vno_indices[t][leading_CAV_ID].odometer + ilink*sensor_length;
		float upper_y = lower_y + sensor_length;
		//
		map<int, VehicleState>& vno_state_map = lead_CV_front_vehicles_state_t_vno_indices[t];
		//fout_mpc << "t," << t << ",num_vehs(vno_state_map)," << vno_state_map.size() << endl;


		for (auto &vitr : vno_state_map)
		{
			 intersection_flag = false;  // default value: false
			float current_local_y;
			float previous_local_y = -1;

			int vno = vitr.first;
		
			current_local_y = SelectedStartLocalY + (float)(t - SelectedStartTime) / max(1, SelectedEndTime - SelectedStartTime)*(SelectedEndLocalY - SelectedStartLocalY);

			if (previous_local_y < 0)  // initialization
			{

				if (SelectedStartTime == SelectedEndTime)  // at the same time, special condition applied here
				{
					previous_local_y = SelectedEndLocalY;
				}
				else
				{
					previous_local_y = SelectedStartLocalY;
				}
			}

			float i_x = 0;
			float i_y = 0;
			if (lead_CV_front_vehicles_state_t_vno_indices[t - 1].find(vno) != lead_CV_front_vehicles_state_t_vno_indices[t - 1].end()) //if (vitr.second.  .StartTime < t && m_VehicleDataList[v].EndTime >t)
			{
				int intersection_test = g_get_line_intersection(t - 1, upper_y, t, lower_y,
					t - 1, lead_CV_front_vehicles_state_t_vno_indices[t - 1][vno].odometer, t, vitr.second.odometer, &i_x, &i_y);
				//fout_mpc << "1.vno," << vno << ",ilink," << ilink << ",upper_y," << upper_y << ",lower_y," << lower_y << ",t_1_POS," << lead_CV_front_vehicles_state_t_vno_indices[t - 1][vno].odometer << ",t_POS," << vitr.second.odometer << endl;
				if (intersection_test > 0)
				{
					intersection_flag = true;
					sum_spd += vitr.second.spd;
					//fout_mpc << "2.vno," << vno << ",ilink," << ilink << ",t," << t << ",num_spd," << num_spd << ",vitr.second.spd," << vitr.second.spd << endl;
					num_spd++;
					break;
				}  //
			}
		}
		
		if (intersection_flag)
		{
			m_SelectedVehicleCount++;
		}

	}  // for each vehicle
	time_mean_spd += sum_spd;// sum_spd / (double)m_SelectedVehicleCount;
	return m_SelectedVehicleCount;
}

void construct_density_profile_from_space_time_region(vector<double>& speed_vec, vector<double> &density_vec){
	int num_sensors =5 ;// 10;
	double sensor_length = 300.0 / num_sensors;
	int cell_time_steps = 50;

	map<int, double> density_multi_10_to_spd;
	//sensor_t_FD_vector.clear(); 
	//sensor_t_FD_vector.resize(num_sensors);
	double sensor_offset = 3.0; //meters
	double lower_y = 0.0;
	double upper_y = 0.0;
	double DetectorSpacing_in_miles = sensor_length / 1609.34;
	#pragma omp parallel for
	for (int ilink = 0; ilink < num_sensors; ilink++)  //300 meters DSRC range == 10 sensors (30 meters each)
	{
		//sensor_t_FD_vector[ilink].resize(front_region_observation_length / cell_time_steps);
		for (int t = 0; t < front_region_observation_length / cell_time_steps; t++)  // 50 steps a cell
		{
			int subsec_t = t * cell_time_steps;
			//fout_mpc << "subsec_t," << subsec_t << ",leading_CAV_ID," << leading_CAV_ID << ",lead_CV_front_vehicles_state_t_vno_indices[subsec_t].size," << lead_CV_front_vehicles_state_t_vno_indices[subsec_t].size() << endl;
			//set lower_y as lead CV real-time position
			lower_y = lead_CV_front_vehicles_state_t_vno_indices.front()[leading_CAV_ID].odometer + sensor_offset; // lead_CV_front_vehicles_state_t_vno_indices[subsec_t][leading_CAV_ID].odometer + sensor_offset;
			//fout_mpc << "lower_y," << lower_y << endl;
			upper_y = lower_y + sensor_length;
			int end_subsec_t = (t + 1) * cell_time_steps;
			if (front_region_observation_length / cell_time_steps == t + 1) end_subsec_t = end_subsec_t - 1;
			double avg_point_density = 0.0;
			float time_mean_spd = 0.0;
			int num_spd = 0;
			for (int tt = subsec_t+1; tt < end_subsec_t; tt++)
			{
				avg_point_density += CountVehicles(sensor_length, ilink, tt, tt + 1, lower_y, upper_y, time_mean_spd, num_spd);
				//fout_mpc << "tt," << tt << ",avg_point_density," << avg_point_density << endl;

			}
			int num_count = avg_point_density;
			if (num_spd <= 0) 
				continue;
			time_mean_spd = time_mean_spd / num_spd;

			avg_point_density = avg_point_density / (end_subsec_t - subsec_t);

			double instantaneous_avg_rect_density = avg_point_density*1.0 / DetectorSpacing_in_miles;

// 			sensor_t_FD_vector[ilink][t].density = instantaneous_avg_rect_density;
// 			sensor_t_FD_vector[ilink][t].speed = time_mean_spd;

			int density_multi_10 = (int)(instantaneous_avg_rect_density*10);
			if (density_multi_10_to_spd.find(density_multi_10) == density_multi_10_to_spd.end()){
				density_multi_10_to_spd[density_multi_10] = time_mean_spd * 2.23694;
			}
			else
			{
				if (time_mean_spd * 2.23694 < density_multi_10_to_spd[density_multi_10])
					density_multi_10_to_spd[density_multi_10] = time_mean_spd * 2.23694;
			}

			//speed_vec.push_back(time_mean_spd * 2.23694);  // mps to mph
			//density_vec.push_back(instantaneous_avg_rect_density);
			
			//fout_mpc << "ilink," << ilink << ",t," << t << ",num_spd," << num_spd << ",spd," << time_mean_spd * 2.23694 << ",density," << instantaneous_avg_rect_density << ",total_time_num_count," << num_count << endl;

		}
	}

	for (auto &iter : density_multi_10_to_spd){
		speed_vec.push_back(iter.second);  // mps to mph
		density_vec.push_back((double)iter.first / 10);
		//fout_mpc << ",density," << (double)iter.first / 10 << "spd," << iter.second << endl;
	}
};

void least_square_calculate(CarFollowingParam &param, vector<double> &spacing_vec, vector<double> &spd_vec){
	vector<double>& x = spacing_vec;
	vector<double>& y = spd_vec;
	if (x.size() < 10 || y.size() < 10)
	{
		param.T = 2.2;
		param.desired_min_spacing = 12.0;
	}
	else
	{
		const auto n = x.size();
		const auto s_x = std::accumulate(x.begin(), x.end(), 0.0);
		const auto s_y = std::accumulate(y.begin(), y.end(), 0.0);
		const auto s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
		const auto s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
		const auto a = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);//slope;

		double b = (s_y - a * s_x) / x.size();//intercept

		param.T = min(3.2, max(0.4, 1 / a));
		param.desired_min_spacing = min(14.0, max(6.0, abs(b*param.T)));
	}
};

double calculate_jam_density(vector<double> density_vec, vector<double> spd_vec) {
	vector<double>& x = density_vec;
	vector<double>& y = spd_vec;
	if (x.size() < 1 || y.size() < 1)
		return -1.0;
	const auto n = x.size();
	const auto s_x = std::accumulate(x.begin(), x.end(), 0.0);
	const auto s_y = std::accumulate(y.begin(), y.end(), 0.0);
	const auto s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
	const auto s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
	const auto a = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);//slope;
	
	double b = (s_y - a * s_x) / density_vec.size();//intercept
	fout_mpc << "s_x," << s_x << ",s_y," << s_y << ",a," << a << ",b," << b << endl;
	return -(b / a);

};
void least_square_calibrate_newell_parameters(map<int,CarFollowingParam> &heter_newell_parameters) // using 250 data points
{
	map<int, vector<double>> vehicleno_time_spacing_vec, vehicleno_time_spd_vec;
// 	vehicle_time_spacing_vec.resize(lead_CV_front_vehicles_state_t_vno_indices.front().size());
// 	vehicle_time_spd_vec.resize(lead_CV_front_vehicles_state_t_vno_indices.front().size());
	int num_data_points = lead_CV_front_vehicles_state_t_vno_indices.size();
	for (int t = 0; t < num_data_points; t++)
	{
		map<int, VehicleState> & vno_front_vehicle_state_map = lead_CV_front_vehicles_state_t_vno_indices[t];
		for (auto &veh_iter : vno_front_vehicle_state_map)
		{
			int veh_no = veh_iter.first;
			if (veh_iter.second.spacing > 0.0 && veh_iter.second.spd>0.0){
				vehicleno_time_spacing_vec[veh_no].push_back(veh_iter.second.spacing);
				vehicleno_time_spd_vec[veh_no].push_back(veh_iter.second.spd);
			}
		}
	}

	for (auto &veh_iter : vehicleno_time_spacing_vec){
		int veh_no = veh_iter.first;
		vector<double> &spacing_vec = vehicleno_time_spacing_vec[veh_no];
		vector<double> &spd_vec = vehicleno_time_spd_vec[veh_no];
		CarFollowingParam cur_param;
		least_square_calculate(cur_param, spacing_vec, spd_vec);
		heter_newell_parameters[veh_no] = cur_param;
	}
};

void predict_vehicle_trajectory(CarFollowingParam &param, vector<vector<VehicleState>> &t_vno_predicted_vehicle_trajectory){
	float num_dt = param.T / param.dt;

	map<int, VehicleState> front_vehs_map = lead_CV_front_vehicles_state_t_vno_indices.back();
//	fout_mpc << "front_vehs_map.size(), " << front_vehs_map.size() << endl;

// 	fout_mpc << "MPC_veh_no_init_acc size," << mpc_initial_state.MPC_veh_no_init_acc.size() << endl;
// 	fout_mpc << "MPC_veh_no_init_loc size," << mpc_initial_state.MPC_veh_no_init_loc.size() << endl;
// 	fout_mpc << "MPC_veh_no_init_spd size," << mpc_initial_state.MPC_veh_no_init_spd.size() << endl;
	////-----------------------------add CAV updated data to front_vehs_map-------------------------------------
	//-----------------add the follower CAVs in the platoon------------------------------------------ the t_vno_predicted_vehicle_trajectory [1,2,...,leadCAV,n2,n3,n4,n5] 
	for (auto &iter : mpc_initial_state.MPC_veh_no_init_spd){
		int veh_no = iter.first;
		if (veh_no != leading_CAV_ID)
		{
			//fout_mpc << "spd,veh_no, " << veh_no << "->" << mpc_initial_state.MPC_veh_no_init_spd[veh_no] << endl;
			VehicleState vehiclestate;
			vehiclestate.acc = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
			vehiclestate.spd = mpc_initial_state.MPC_veh_no_init_spd[veh_no];
			vehiclestate.odometer = mpc_initial_state.MPC_veh_no_init_loc[veh_no];

			//front_vehs_map[veh_no] = vehiclestate;
			front_vehs_map.insert(make_pair(veh_no, vehiclestate));
			//fout_mpc << "front_vehs_map[veh_no].spd, " << front_vehs_map[veh_no].spd << endl;
		}
	}
	//fout_mpc << "front_vehs_map.size(), " << front_vehs_map.size() << endl;

	VehicleState &first_front_vehicle = front_vehs_map.begin()->second;
	//predict the rolling horizon.
	double prev_spd = first_front_vehicle.spd;
	double prev_pos = first_front_vehicle.odometer;
	double cur_veh_pos = 0.0;
	double spacing = 0.0;
//	vector<vector<VehicleState>> t_vno_predicted_vehicle_trajectory;
	t_vno_predicted_vehicle_trajectory.resize(rolling_horizon+1);

	//init
	int cur_v_index = 0;
	

	t_vno_predicted_vehicle_trajectory[0].resize(front_vehs_map.size());
	for (auto & v_iter : front_vehs_map)
	{
		t_vno_predicted_vehicle_trajectory[0][cur_v_index].odometer = v_iter.second.odometer;
		t_vno_predicted_vehicle_trajectory[0][cur_v_index].spd = v_iter.second.spd;
		t_vno_predicted_vehicle_trajectory[0][cur_v_index].acc = v_iter.second.acc;
		cur_v_index++;
	}
	for (cur_v_index = 0; cur_v_index < t_vno_predicted_vehicle_trajectory[0].size(); cur_v_index++)
		//fout_mpc << "t|k," << 0 << ",cur_v_index," << cur_v_index << ",acc," << t_vno_predicted_vehicle_trajectory[0][cur_v_index].acc << ",spd," << t_vno_predicted_vehicle_trajectory[0][cur_v_index].spd << ",loc," << t_vno_predicted_vehicle_trajectory[0][cur_v_index].odometer << endl;

	for (int i = 1; i <= rolling_horizon; i++){
		int cur_v_index = 0;
		t_vno_predicted_vehicle_trajectory[i].resize(front_vehs_map.size());
		for (auto & v_iter : front_vehs_map)
		{
			int c_veh_n = v_iter.first;
			if (cur_v_index == 0)
			{//let first front vehicle driving in constant speed
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd = v_iter.second.spd;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].acc = 0.0;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].odometer = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].odometer + v_iter.second.spd* 0.1;// param.T; //pos at t+1
			}
			else if (cur_v_index < (front_vehs_map.size()+1-MaxPlatoonSize))
			{
				float des_delta_spd = 0.0;
				float des_acc=0.0;
				if (veh_no_car_following_update_stauts.find(c_veh_n) == veh_no_car_following_update_stauts.end())
					veh_no_car_following_update_stauts[c_veh_n] = false;
				else
				{
					if (veh_no_car_following_update_stauts[c_veh_n] == true)
					{
						des_delta_spd = car_following_param.control_des_delta_v_map[c_veh_n];
						

						if (i > (car_following_param.control_start_t_map[c_veh_n] + car_following_param.T))
							veh_no_car_following_update_stauts[c_veh_n] = false;
					}
					else
					{
						car_following_param.control_start_t_map[c_veh_n] = i;
						spacing = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index - 1].odometer - t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].odometer;
						double subj_veh_spd = (1 / param.T)*(spacing - param.desired_min_spacing);
 						if (subj_veh_spd < 0.0){
 							subj_veh_spd = 0.0;
 						}
						//car_following_param.control_des_spd_map[c_veh_n] = subj_veh_spd;
						car_following_param.control_des_delta_v_map[c_veh_n] = (subj_veh_spd - t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].spd) / num_dt;
						//fout_mpc << "t|k," << i << ",cur_v_index," << cur_v_index << ",param.T," << param.T << ",spacing," << spacing << ",param.desired_min_spacing," << param.desired_min_spacing << ",num_dt," << num_dt << endl;
						des_delta_spd = car_following_param.control_des_delta_v_map[c_veh_n];
						//
						veh_no_car_following_update_stauts[current_veh_no] = true;
					}
				}
				//
				des_acc = (des_delta_spd) / car_following_param.dt;
				des_acc = min(max(car_following_param.min_acc, des_acc), car_following_param.max_acc);
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].spd + des_delta_spd;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].odometer = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].odometer + t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd*0.1;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].acc = des_acc;
			}
			else
			{
				double cur_acc = t_vno_predicted_vehicle_trajectory[i-1][cur_v_index].acc;
				double cur_spd = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].spd;
				double front_spd = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index - 1].spd;
				double front_dist = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index - 1].odometer - t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].odometer - Leng_vehicle;
				double des_acc = CACC_Car_Following(cur_v_index, cur_acc, cur_spd, Leng_vehicle,
					cur_v_index - 1, 0, front_spd, AdjVehiclesWidth[2][3], front_dist,
					1.2);

				
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].spd + des_acc*0.1;
				if (t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd < 0){
					t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd = 0;
					des_acc = (t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd - t_vno_predicted_vehicle_trajectory[i-1][cur_v_index].spd)/0.1;
				}
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].acc = des_acc;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].odometer = t_vno_predicted_vehicle_trajectory[i-1][cur_v_index].odometer + t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].spd * 0.1 + 0.5*des_acc*0.01;
			}
			//fout_mpc << "t|k," << i << ",cur_v_index," << cur_v_index << ",acc," << t_vno_predicted_vehicle_trajectory[i][cur_v_index].acc << ",spd," << t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd << ",loc," << t_vno_predicted_vehicle_trajectory[i][cur_v_index].odometer << endl;
			//double spacing = 
			cur_v_index++;
		}
	}
};

void predict_vehicle_trajectory_reference_stochastic(map<int, CarFollowingParam> &param_map,
	map<int, vector<vector<VehicleState>>> &scenarioidx_tidx_vno_predicted_vehicle_trajectory)//scenarioidx_tidx_vno_predicted_vehicle_trajectory
{
	int predicted_idx=-1;

	int tidx = 0;
	for (int tidx = 0; tidx < lead_CV_front_vehicles_state_t_vno_indices.size(); tidx += 10)
	{
		predicted_idx++;
		map<int, VehicleState> front_vehs_map = lead_CV_front_vehicles_state_t_vno_indices[tidx];
		VehicleState &first_front_vehicle = front_vehs_map.begin()->second;
		//predict the rolling horizon.
		double first_front_vehicle_scenario_spd = first_front_vehicle.spd;
		double first_front_vehicle_pos = first_front_vehicle.odometer;
		double spacing = 0.0;
		//mpc_data.observed_acc_error;

		////-----------------------------add CAV updated data to front_vehs_map-------------------------------------
		//-----------------add the follower CAVs in the platoon------------------------------------------ the t_vno_predicted_vehicle_trajectory [1,2,...,leadCAV,n2,n3,n4,n5] 
		for (auto &iter : mpc_initial_state.MPC_veh_no_init_spd){
			int veh_no = iter.first;
			if (veh_no != leading_CAV_ID)
			{
				//fout_mpc << "spd,veh_no, " << veh_no << "->" << mpc_initial_state.MPC_veh_no_init_spd[veh_no] << endl;
				VehicleState vehiclestate;
				vehiclestate.acc = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
				vehiclestate.spd = mpc_initial_state.MPC_veh_no_init_spd[veh_no];
				vehiclestate.odometer = mpc_initial_state.MPC_veh_no_init_loc[veh_no];

				//front_vehs_map[veh_no] = vehiclestate;
				front_vehs_map.insert(make_pair(veh_no, vehiclestate));
				//fout_mpc << "front_vehs_map[veh_no].spd, " << front_vehs_map[veh_no].spd << endl;
			}
		}

		scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx].resize(rolling_horizon + 1);
		//initialcondition from real sensing data for front vehicles and platoon vehicles
		int cur_v_index = 0;
		scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][0].resize(front_vehs_map.size());
		for (auto & v_iter : front_vehs_map)
		{
			//fout_mpc << "for (auto & v_iter : front_vehs_map)" << endl;
			scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][0][cur_v_index].odometer = v_iter.second.odometer;
			scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][0][cur_v_index].spd = v_iter.second.spd;
			scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][0][cur_v_index].acc = v_iter.second.acc;
			cur_v_index++;
		}
		//for all front vehicles and platoon vehicles
		for (cur_v_index = 0; cur_v_index < scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][0].size(); cur_v_index++)
		{
		}
		//fout_mpc << "for all the rolling horizon" << endl;
		for (int i = 1; i <= rolling_horizon; i++)
		{//for all the rolling horizon start from t=1.
			int cur_v_index = 0;
			scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i].resize(front_vehs_map.size());
			//fout_mpc << "scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i].resize(front_vehs_map.size());" << endl;
			for (auto & v_iter : front_vehs_map)
			{
				int c_veh_n = v_iter.first;
				CarFollowingParam param = param_map[c_veh_n];
				float num_dt = param.T / param.dt;
				if (cur_v_index == 0)
				{//let first front vehicle driving in constant speed from current scenario				
					//fout_mpc << "cur_v_index == 0" << endl;
					scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].spd = first_front_vehicle_scenario_spd;
					scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].acc = 0.0;
					scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].odometer = scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].odometer + first_front_vehicle_scenario_spd* 0.1;// param.T; //pos at t+1
				}
				else if (cur_v_index < (front_vehs_map.size() + 1 - MaxPlatoonSize))
				{
					//fout_mpc << "else if cur_v_index: " << cur_v_index << ",i:" << i  << endl;
					float des_delta_spd = 0.0;
					float des_acc = 0.0;
					if (veh_no_car_following_update_stauts.find(c_veh_n) == veh_no_car_following_update_stauts.end())
						veh_no_car_following_update_stauts[c_veh_n] = false;
					else
					{
						if (veh_no_car_following_update_stauts[c_veh_n] == true)
						{
							des_delta_spd = car_following_param.control_des_delta_v_map[c_veh_n];


							if (i > (car_following_param.control_start_t_map[c_veh_n] + car_following_param.T))
								veh_no_car_following_update_stauts[c_veh_n] = false;
						}
						else
						{
							car_following_param.control_start_t_map[c_veh_n] = i;
							spacing = scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index - 1].odometer - scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].odometer;
							double subj_veh_spd = (1 / param.T)*(spacing - param.desired_min_spacing);
							if (subj_veh_spd < 0.0){
								subj_veh_spd = 0.0;
							}

							car_following_param.control_des_delta_v_map[c_veh_n] = (subj_veh_spd - scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].spd) / num_dt;
							des_delta_spd = car_following_param.control_des_delta_v_map[c_veh_n];
							//
							veh_no_car_following_update_stauts[current_veh_no] = true;
						}
					}
					//
					des_acc = (des_delta_spd) / car_following_param.dt;
					des_acc = min(max(car_following_param.min_acc, des_acc), car_following_param.max_acc);
					scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].spd = scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].spd + des_delta_spd;
					scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].odometer = scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].odometer + scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].spd*0.1;
					scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].acc = des_acc;
				}
				else
				{
					//fout_mpc << "else cur_v_index: " << cur_v_index << ",i:" << i << endl;
					double cur_acc = scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].acc;
					double cur_spd = scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].spd;
					double front_spd = scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index - 1].spd;
					double front_dist = scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index - 1].odometer - scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].odometer - Leng_vehicle;

					double des_acc = CACC_Car_Following(cur_v_index, cur_acc, cur_spd, Leng_vehicle,
						cur_v_index - 1, 0, front_spd, AdjVehiclesWidth[2][3], front_dist,
						1.2);
					//fout_mpc << "des_acc: " << des_acc <<  endl;

					scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].spd = scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].spd + des_acc*0.1;
					//fout_mpc << "spd = else cur_v_index: " << cur_v_index << ",i:" << i << endl;
					if (scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].spd < 0){
						scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].spd = 0;
						des_acc = (scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].spd - scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].spd) / 0.1;
					}
					//fout_mpc << "acc = else cur_v_index: " << cur_v_index << ",i:" << i << endl;
					scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].acc = des_acc;
					//fout_mpc << "odometer = else cur_v_index: " << cur_v_index << ",i:" << i << endl;
					scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i][cur_v_index].odometer = scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].odometer + scenarioidx_tidx_vno_predicted_vehicle_trajectory[predicted_idx][i - 1][cur_v_index].spd * 0.1 + 0.5*des_acc*0.01;
				}

				cur_v_index++;

			}
		}
	}
	
};

void predict_vehicle_trajectory(map<int,CarFollowingParam> &param_map, vector<vector<VehicleState>> &t_vno_predicted_vehicle_trajectory)
{
	

	map<int, VehicleState> front_vehs_map = lead_CV_front_vehicles_state_t_vno_indices.back();
	////-----------------------------add CAV updated data to front_vehs_map-------------------------------------
	//-----------------add the follower CAVs in the platoon------------------------------------------ the t_vno_predicted_vehicle_trajectory [1,2,...,leadCAV,n2,n3,n4,n5] 
	for (auto &iter : mpc_initial_state.MPC_veh_no_init_spd){
		int veh_no = iter.first;
		//if (veh_no != leading_CAV_ID)
		{
			//fout_mpc << "spd,veh_no, " << veh_no << "->" << mpc_initial_state.MPC_veh_no_init_spd[veh_no] << endl;
			VehicleState vehiclestate;
			vehiclestate.acc = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
			vehiclestate.spd = mpc_initial_state.MPC_veh_no_init_spd[veh_no];
			vehiclestate.odometer = mpc_initial_state.MPC_veh_no_init_loc[veh_no];

			//front_vehs_map[veh_no] = vehiclestate;
			front_vehs_map.insert(make_pair(veh_no, vehiclestate));
			fout_mpc << "vehno," << veh_no << ",front_vehs_map[veh_no].spd, " << front_vehs_map[veh_no].odometer << endl;
		}
	}
	//fout_mpc << "front_vehs_map.size(), " << front_vehs_map.size() << endl;

	VehicleState &first_front_vehicle = front_vehs_map.begin()->second;
	//predict the rolling horizon.
	double prev_spd = first_front_vehicle.spd;
	double prev_pos = first_front_vehicle.odometer;
	double cur_veh_pos = 0.0;
	double spacing = 0.0;
	//	vector<vector<VehicleState>> t_vno_predicted_vehicle_trajectory;
	t_vno_predicted_vehicle_trajectory.resize(rolling_horizon + 1);

	//init
	int cur_v_index = 0;


	t_vno_predicted_vehicle_trajectory[0].resize(front_vehs_map.size());
	for (auto & v_iter : front_vehs_map)
	{
		t_vno_predicted_vehicle_trajectory[0][cur_v_index].odometer = v_iter.second.odometer;
		t_vno_predicted_vehicle_trajectory[0][cur_v_index].spd = v_iter.second.spd;
		t_vno_predicted_vehicle_trajectory[0][cur_v_index].acc = v_iter.second.acc;
		cur_v_index++;
	}
	for (cur_v_index = 0; cur_v_index < t_vno_predicted_vehicle_trajectory[0].size(); cur_v_index++)
		fout_mpc << "t|k," << 0 << ",cur_v_index," << cur_v_index << ",acc," << t_vno_predicted_vehicle_trajectory[0][cur_v_index].acc << ",spd," << t_vno_predicted_vehicle_trajectory[0][cur_v_index].spd << ",loc," << t_vno_predicted_vehicle_trajectory[0][cur_v_index].odometer << endl;


	for (int i = 1; i <= rolling_horizon; i++){
		int cur_v_index = 0;
		t_vno_predicted_vehicle_trajectory[i].resize(front_vehs_map.size());
		for (auto & v_iter : front_vehs_map)
		{
			int c_veh_n = v_iter.first;
			CarFollowingParam param = param_map[c_veh_n];
			float num_dt = param.T / param.dt;
			if (cur_v_index == 0)
			{//let first front vehicle driving in constant speed
				//fout_mpc << "cur_v_index == 0" << endl;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd = v_iter.second.spd;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].acc = 0.0;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].odometer = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].odometer + v_iter.second.spd* 0.1;// param.T; //pos at t+1
			}
			else if (cur_v_index < (front_vehs_map.size() + 1 - MaxPlatoonSize))
			{
				//fout_mpc << "else if cur_v_index: " << cur_v_index << ",i:" << i  << endl;
				float des_delta_spd = 0.0;
				float des_acc = 0.0;
				if (veh_no_car_following_update_stauts.find(c_veh_n) == veh_no_car_following_update_stauts.end())
					veh_no_car_following_update_stauts[c_veh_n] = false;
				else
				{
					if (veh_no_car_following_update_stauts[c_veh_n] == true)
					{
						des_delta_spd = car_following_param.control_des_delta_v_map[c_veh_n];


						if (i > (car_following_param.control_start_t_map[c_veh_n] + car_following_param.T))
							veh_no_car_following_update_stauts[c_veh_n] = false;
					}
					else
					{
						car_following_param.control_start_t_map[c_veh_n] = i;
						spacing = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index - 1].odometer - t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].odometer;
						double subj_veh_spd = (1 / param.T)*(spacing - param.desired_min_spacing);
						if (subj_veh_spd < 0.0){
							subj_veh_spd = 0.0;
						}
				
						car_following_param.control_des_delta_v_map[c_veh_n] = (subj_veh_spd - t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].spd) / num_dt;
						des_delta_spd = car_following_param.control_des_delta_v_map[c_veh_n];
						//
						veh_no_car_following_update_stauts[current_veh_no] = true;
					}
				}
				//
				des_acc = (des_delta_spd) / car_following_param.dt;
				des_acc = min(max(car_following_param.min_acc, des_acc), car_following_param.max_acc);
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].spd + des_delta_spd;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].odometer = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].odometer + t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd*0.1;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].acc = des_acc;
			}
			else
			{
				//fout_mpc << "else cur_v_index: " << cur_v_index << ",i:" << i << endl;
				double cur_acc = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].acc;
				double cur_spd = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].spd;
				double front_spd = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index - 1].spd;
				double front_dist = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index - 1].odometer - t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].odometer - Leng_vehicle;
				
				double des_acc = CACC_Car_Following(cur_v_index, cur_acc, cur_spd, Leng_vehicle,
					cur_v_index - 1, 0, front_spd, AdjVehiclesWidth[2][3], front_dist,
					1.2);
				//fout_mpc << "des_acc: " << des_acc <<  endl;

				t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].spd + des_acc*0.1;
				//fout_mpc << "spd = else cur_v_index: " << cur_v_index << ",i:" << i << endl;
				if (t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd < 0){
					t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd = 0;
					des_acc = (t_vno_predicted_vehicle_trajectory[i][cur_v_index].spd - t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].spd) / 0.1;
				}
				//fout_mpc << "acc = else cur_v_index: " << cur_v_index << ",i:" << i << endl;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].acc = des_acc;
				//fout_mpc << "odometer = else cur_v_index: " << cur_v_index << ",i:" << i << endl;
				t_vno_predicted_vehicle_trajectory[i][cur_v_index].odometer = t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].odometer + t_vno_predicted_vehicle_trajectory[i - 1][cur_v_index].spd * 0.1 + 0.5*des_acc*0.01;
			}
 
			cur_v_index++;
		}
	}
};

void constant_mpc_QP_control_data_update(
	ampl::AMPL &ampl, SolverID solver, 
	MPC_Initial_State & mpc_initial_state, 
	vector<int> &unique_vehicle_no, 
	MPC_Data &mpc_data, 
	double nveh_loc,
	double nveh_spd)
{
	//	mpc_QP_control_data_update();
	mpc_initial_state.init_acc.clear();
	mpc_initial_state.init_spd.clear();
	mpc_initial_state.init_loc.clear();
	double ref_v[250], ref_l[250];  //rolling horizon 50
	double l_rows[5], l_cols[50];
	int vi = 0, li = 0;
	double prev_loc = nveh_loc, min_loc_dif = DBL_MAX;
	int veh_idx = 0;
	for (int i = 0; i < unique_vehicle_no.size(); i++)//(auto veh_no : unique_vehicle_no)
	{
		int veh_no = unique_vehicle_no[i];
		//fout_mpc << "veh_no," << veh_no << endl;
		double init_acc = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
		double init_spd = mpc_initial_state.MPC_veh_no_init_spd[veh_no];
		if (init_spd + init_acc*0.1 < 0)
			init_acc = max(mpc_data.u_lower, -init_spd / 0.1 +0.1);
		double init_loc = mpc_initial_state.MPC_veh_no_init_loc[veh_no];
		double front_loc = mpc_initial_state.MPC_veh_no_init_front_loc[veh_no];
		double front_spd = mpc_initial_state.MPC_veh_no_init_front_spd[veh_no];
		double gap = 0.0;
		if (veh_no == leading_CAV_ID)
			gap = front_loc - init_loc;
		else
			gap = mpc_initial_state.MPC_veh_no_init_loc[veh_no - 1] - init_loc;
		if (init_acc > (gap + 0.1*front_spd - init_spd*0.1 - mpc_data.head*init_spd - mpc_data.v_length[veh_idx]) / (0.1*mpc_data.head))
			init_acc = max(mpc_data.u_lower, (gap + 0.1*front_spd - init_spd*0.1 - mpc_data.head*init_spd - mpc_data.v_length[veh_idx]) / (0.1*mpc_data.head) - 0.1);
		double cal_hwdy = (prev_loc - init_loc - mpc_data.v_length[veh_idx++]) / init_spd;
		if (cal_hwdy < mpc_data.head){
			// 			if (cal_hwdy > 0.1)
			// 				mpc_data.head = cal_hwdy;
			// 			else
			fout_mpc << "cal_hwdy is too small: " << cal_hwdy << endl;
		}
		prev_loc = init_loc;
		fout_mpc << "t:" << timestep << ",vno:" << veh_no << ",acc," << init_acc << ",init_spd," << init_spd << ",init_loc," << init_loc << ",front_loc," << front_loc << ",front_spd," << front_spd << ",gap:" << gap << endl;
		mpc_initial_state.init_acc.push_back(init_acc);
		mpc_initial_state.init_spd.push_back(init_spd);
		mpc_initial_state.init_loc.push_back(init_loc);

		double ref_spd = init_spd;
		if (solver == Predictive_deter_instananeous_Tracking_MPC){
			predict_speeds_in_front_region(speed_predict_model, mpc_data);
			float cur_predict_spd = mpc_data.front_region_data.front().speed;
			ref_spd = cur_predict_spd;
		}
		double ref_loc = init_loc;
		for (int t = 0; t < mpc_data.Ph; t++)
		{		
			if (solver == Predictive_deter_instananeous_Tracking_MPC){
				ref_spd = max(nveh_spd, min(ref_spd, mpc_data.v_upper));
				int key_ = (int)(ref_loc / mpc_data.cell_dist);
				if (key_ < 10)
					ref_spd = mpc_data.front_region_data[key_].speed;
				else		
					ref_spd = mpc_data.front_region_data.back().speed;
			}
			ref_loc += ref_spd*0.1;
			ref_v[vi++] = ref_spd;
			ref_l[li++] = ref_loc;
		}
	}
	for (int s = 0; s < mpc_data.control_size; s++)
		l_rows[s] = s + 1;
	for (int i = 0; i < mpc_data.Ph; i++)
		l_cols[i] = i + 1;
	double cpt_t = get_current_cpu_time_in_seconds();
	ampl::Parameter control_size_ = ampl.getParameter("control_size");
	control_size_.set(mpc_data.control_size);
	ampl::Parameter dt_ = ampl.getParameter("dt");
	dt_.set(mpc_data.dt);
	ampl::Parameter T_ = ampl.getParameter("T");  //rolling horizon
	T_.set(mpc_data.Ph);
	ampl::Parameter u_lower_ = ampl.getParameter("acc_min");
	u_lower_.set(mpc_data.u_lower);
	ampl::Parameter u_upper_ = ampl.getParameter("acc_max");
	u_upper_.set(mpc_data.u_upper);
	ampl::Parameter v_upper_ = ampl.getParameter("v_upper");
	v_upper_.set(mpc_data.v_upper);
	ampl::Parameter v_lower_ = ampl.getParameter("v_lower");
	v_lower_.set(mpc_data.v_upper - 8.9); // - 20 mph
	ampl::Parameter head_ = ampl.getParameter("hdwy");
	head_.set(mpc_data.head);
	fout_mpc << "t:" << timestep << ",ampl mpc_data.head: " << mpc_data.head << endl;
	ampl::Parameter reac_ = ampl.getParameter("reac");
	reac_.set(mpc_data.reac);
	fout_mpc << "ampl set arrays..." << endl;

	//=================last front vehicle =======================
	vector<double> nveh_loc_array, nveh_spd_array;
	double cur_front_loc = nveh_loc;
	for (int i = 0; i < mpc_data.Ph; i++){
		
		nveh_loc_array.push_back(cur_front_loc);
		nveh_spd_array.push_back(nveh_spd);			//;(nveh_spd);
		//nveh_loc += mpc_data.dt * nveh_spd;
		cur_front_loc += nveh_spd*0.1;
	}
	ampl::Parameter nveh_loc_ = ampl.getParameter("nveh_loc");
	nveh_loc_.setValues(nveh_loc_array.data(), nveh_loc_array.size());
	ampl::Parameter nveh_spd_ = ampl.getParameter("nveh_spd");
	nveh_spd_.setValues(nveh_spd_array.data(), nveh_spd_array.size());

	ampl::Parameter length_ = ampl.getParameter("length");
	length_.setValues(mpc_data.v_length.data(), mpc_data.v_length.size());

	ampl::Parameter v0_ = ampl.getParameter("vehicle_speed_init");
	//fout_mpc << "mpc_initial_state.init_spd size " << mpc_initial_state.init_spd.size() << ", i 1 " << mpc_initial_state.init_spd.front() << endl;
	v0_.setValues(mpc_initial_state.init_spd.data(), mpc_initial_state.init_spd.size());

	ampl::Parameter l0_ = ampl.getParameter("vehicle_loc_init");
	l0_.setValues(mpc_initial_state.init_loc.data(), mpc_initial_state.init_loc.size());

	ampl::Parameter a0_ = ampl.getParameter("vehicle_acc_init");
	a0_.setValues(mpc_initial_state.init_acc.data(), mpc_initial_state.init_acc.size());

	if (solver == Predictive_constant_tracking_MPC || solver== Predictive_deter_instananeous_Tracking_MPC){
		ampl::Parameter ref_v_ = ampl.getParameter("ref_v");
		ref_v_.setValues(mpc_data.control_size, l_rows, mpc_data.Ph, l_cols, ref_v, false);
		ampl::Parameter ref_l_ = ampl.getParameter("ref_l");
		ref_l_.setValues(mpc_data.control_size, l_rows, mpc_data.Ph, l_cols, ref_l, false);
	}
	fout_mpc << "done init ampl ref" << endl;

};

void predictive_mpc_QP_control_data_update(vector<vector<VehicleState>> &t_vno_predicted_vehicle_trajectory, 
	ampl::AMPL &ampl, SolverID solver, MPC_Initial_State & mpc_initial_state, vector<int> &unique_vehicle_no, MPC_Data &mpc_data, double nveh_loc)
{
	fout_mpc << "mpc_initial_state.MPC_veh_no_init_acc," << mpc_initial_state.MPC_veh_no_init_acc.size() << ",spd," << mpc_initial_state.MPC_veh_no_init_spd.size() << ",init_loc," << mpc_initial_state.MPC_veh_no_init_loc.size() << endl;
	fout_mpc << "unique_vehicle_no.size() " << unique_vehicle_no.size() << endl;
	mpc_initial_state.init_acc.clear();
	mpc_initial_state.init_spd.clear();
	mpc_initial_state.init_loc.clear();
	double ref_v[250], ref_l[250];  //rolling horizon 50
	double l_rows[5], l_cols[50];
	int vi = 0, li = 0;
	double prev_loc = nveh_loc, min_loc_dif = DBL_MAX;
	int veh_idx = 0;
	for (int i = 0; i < unique_vehicle_no.size(); i++)//(auto veh_no : unique_vehicle_no)
	{
		int veh_no = unique_vehicle_no[i];
		fout_mpc << "veh_no," << veh_no << endl;
		double init_acc = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
		double init_acc_real = mpc_initial_state.MPC_veh_no_init_acc[veh_no];		
		double init_spd = mpc_initial_state.MPC_veh_no_init_spd[veh_no];
		if (init_spd + init_acc*0.1 < 0)
			init_acc = max(mpc_data.u_lower, -init_spd / 0.1 + 0.1);
		double init_loc = mpc_initial_state.MPC_veh_no_init_loc[veh_no];
		double front_loc = mpc_initial_state.MPC_veh_no_init_front_loc[veh_no];
		double front_spd = mpc_initial_state.MPC_veh_no_init_front_spd[veh_no];


		/*double gap = 0.0;
		if (veh_no == leading_CAV_ID)
			gap = front_loc - init_loc;
		else
			gap = mpc_initial_state.MPC_veh_no_init_loc[veh_no - 1] - init_loc;
		if (init_acc > (gap + 0.1*front_spd - init_spd*0.1 - mpc_data.head*init_spd - mpc_data.v_length[veh_idx]) / (0.1*mpc_data.head))
			init_acc = max(mpc_data.u_lower, (gap + 0.1*front_spd - init_spd*0.1 - mpc_data.head*init_spd - mpc_data.v_length[veh_idx]) / (0.1*mpc_data.head) - 0.1);
		double cal_hwdy = (prev_loc - init_loc - mpc_data.v_length[veh_idx++]) / init_spd;*/



// 		if (cal_hwdy < mpc_data.head){
// 			if (cal_hwdy > 0.1)
// 				mpc_data.head = cal_hwdy;
// 			//else
// 			//	fout_mpc << "cal_hwdy is too small: " << cal_hwdy << endl;
// 	
		prev_loc = init_loc;
		fout_mpc << "t:" << timestep << ",vno:" << veh_no << ",cal_acc," << init_acc << endl;
		fout_mpc << "t:" << timestep << ",vno:" << veh_no << ",REAL_acc," << init_acc_real << ",init_spd," << init_spd << ",init_loc," << init_loc << ",front_loc," << front_loc << ",front_spd," << front_spd <<  endl;
		mpc_initial_state.init_acc.push_back(init_acc_real);
		mpc_initial_state.init_spd.push_back(init_spd);
		mpc_initial_state.init_loc.push_back(init_loc);
		for (int t = 0; t < mpc_data.Ph; t++)
		{
			//sdzhao: change predict time index from t+1 to t
			int predict_num_vehs = t_vno_predicted_vehicle_trajectory[t].size();
			//fout_mpc << "t_vno_predicted_vehicle_trajectory,size," << t_vno_predicted_vehicle_trajectory.size() << endl;
			//fout_mpc << "t_vno_predicted_vehicle_trajectory," << t+1<<".size" << t_vno_predicted_vehicle_trajectory[t + 1].size() << endl;
			//fout_mpc << "predict_num_vehs - MaxPlatoonSize  + i:" << predict_num_vehs - MaxPlatoonSize  + i << endl;
			ref_v[vi++] = t_vno_predicted_vehicle_trajectory[t][predict_num_vehs - MaxPlatoonSize + i].spd;			// the last front vehicle in front region // init_spd;
			ref_l[li++] = t_vno_predicted_vehicle_trajectory[t][predict_num_vehs - MaxPlatoonSize  + i].odometer;	// init_loc;
			//init_loc += init_spd*mpc_data.dt;
			
		}
	}
	for (int s = 0; s < mpc_data.control_size; s++)
		l_rows[s] = s + 1;
	for (int i = 0; i < mpc_data.Ph; i++)
		l_cols[i] = i + 1;
	fout_mpc << "ampl set values..." << endl << "mpc_data.v_length " << mpc_data.v_length.size() << endl;
	
	//param - value
	double cpt_t = get_current_cpu_time_in_seconds();
	ampl::Parameter control_size_ = ampl.getParameter("control_size");
	control_size_.set(mpc_data.control_size);
	ampl::Parameter dt_ = ampl.getParameter("dt");
	dt_.set(mpc_data.dt);
	ampl::Parameter T_ = ampl.getParameter("T");  //rolling horizon
	T_.set(mpc_data.Ph);
	ampl::Parameter u_lower_ = ampl.getParameter("acc_min");
	u_lower_.set(mpc_data.u_lower);
	ampl::Parameter u_upper_ = ampl.getParameter("acc_max");
	u_upper_.set(mpc_data.u_upper);
	ampl::Parameter v_upper_ = ampl.getParameter("v_upper");
	v_upper_.set(mpc_data.v_upper);
	ampl::Parameter v_lower_ = ampl.getParameter("v_lower");
	v_lower_.set(mpc_data.v_upper - 8.9); // - 20 mph
	ampl::Parameter head_ = ampl.getParameter("hdwy");
	head_.set(mpc_data.head);
	ampl::Parameter reac_ = ampl.getParameter("reac");
	reac_.set(mpc_data.reac);
	fout_mpc << "ampl set arrays..." << endl;
	//param - array

	//=================last front vehicle =======================
	vector<double> nveh_loc_array, nveh_spd_array;
	//nveh_spd = nveh_spd > 10.0 ? 10.0 : nveh_spd;
	//fout_mpc << "nveh_loc: " << nveh_loc << endl;
	for (int i = 0; i < mpc_data.Ph; i++){
		int predict_num_vehs = t_vno_predicted_vehicle_trajectory[i].size();
		nveh_loc_array.push_back(t_vno_predicted_vehicle_trajectory[i][predict_num_vehs - MaxPlatoonSize -1].odometer);
		nveh_spd_array.push_back(t_vno_predicted_vehicle_trajectory[i][predict_num_vehs - MaxPlatoonSize - 1].spd);			//;(nveh_spd);
		//nveh_loc += mpc_data.dt * nveh_spd;
		if (i <= 3){
			fout_mpc << "ampl t:" << timestep<< ",vno," << current_veh_no<< ",k,=" << i<<",nveh_loc," << nveh_loc_array.back() << endl;
		}
	}
	ampl::Parameter nveh_loc_ = ampl.getParameter("nveh_loc");
	nveh_loc_.setValues(nveh_loc_array.data(), nveh_loc_array.size());
	ampl::Parameter nveh_spd_ = ampl.getParameter("nveh_spd");
	nveh_spd_.setValues(nveh_spd_array.data(), nveh_spd_array.size());

	ampl::Parameter length_ = ampl.getParameter("length");
	length_.setValues(mpc_data.v_length.data(), mpc_data.v_length.size());

	ampl::Parameter v0_ = ampl.getParameter("vehicle_speed_init");
	//fout_mpc << "mpc_initial_state.init_spd size " << mpc_initial_state.init_spd.size() << ", i 1 " << mpc_initial_state.init_spd.front() << endl;
	v0_.setValues(mpc_initial_state.init_spd.data(), mpc_initial_state.init_spd.size());

	ampl::Parameter l0_ = ampl.getParameter("vehicle_loc_init");
	l0_.setValues(mpc_initial_state.init_loc.data(), mpc_initial_state.init_loc.size());

	ampl::Parameter a0_ = ampl.getParameter("vehicle_acc_init");
	a0_.setValues(mpc_initial_state.init_acc.data(), mpc_initial_state.init_acc.size());

	//fout_mpc << "Predictive MPC_tracking set values to ampl..." << endl;
	//predict_speeds_in_front_region(speed_predict_model, mpc_data);

	//reference calculation...
// 	vector<double> ref_spd(mpc_data.Ph, 0.0);
// 	vector<double> ref_loc(mpc_data.Ph, 0.0);
// 	float cur_predict_spd = mpc_data.front_region_data.front().speed;
// 	float cur_front_dist = 0.0;
// 	for (int t_ = 0; t_ < mpc_data.Ph; t_++){
// 		cur_predict_spd = max_(0.0f, min_(cur_predict_spd, mpc_data.v_upper));
// 		cur_front_dist += t_*cur_predict_spd;
// 		ref_spd[t_] = cur_predict_spd;
// 		ref_loc[t_] = cur_front_dist + veh_odometer;
// 		int key_ = (int)(cur_front_dist / mpc_data.cell_dist);
// 		//fout_mpc << "front_region_data key: " << key_ << endl;
// 		if (key_ < 10)
// 			cur_predict_spd = mpc_data.front_region_data[key_].speed;
// 		else
// 		{
// 			cur_predict_spd = mpc_data.front_region_data.back().speed;
// 		}
// 	}
// 	ampl::Parameter ref_v_ = ampl.getParameter("ref_v");
// 	ref_v_.setValues(ref_spd.data(), ref_spd.size());
// 	ampl::Parameter ref_l_ = ampl.getParameter("ref_l");
// 	ref_l_.setValues(ref_loc.data(), ref_loc.size()
	//ref
	//fout_mpc << "init ampl ref..." << endl;
	ampl::Parameter ref_v_ = ampl.getParameter("ref_v");
	ref_v_.setValues(mpc_data.control_size, l_rows, mpc_data.Ph, l_cols, ref_v, false);
	ampl::Parameter ref_l_ = ampl.getParameter("ref_l");
	ref_l_.setValues(mpc_data.control_size, l_rows, mpc_data.Ph, l_cols, ref_l, false);
	fout_mpc << "done init ampl ref" << endl;

	//mpc_data.total_cpu_time += (get_current_cpu_time_in_seconds() - cpt_t);
};

DRIVERMODEL_API  int  DriverModelExecuteCommand(long number)
{
	/* Executes the command <number> if that is available in the driver */
	/* module. Return value is 1 on success, otherwise 0.               */

	switch (number) {
	case DRIVER_COMMAND_INIT:
		if (solver >= Predictive_deter_Regulating_MPC && solver <= Predictive_DRO_Tracking_MPC)
		{
			char cCurrentPath[FILENAME_MAX];
			if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
				cout << "error current dir" << endl;
			string first_veh_tra_fn(cCurrentPath);
			if (solver == Predictive_deter_Regulating_MPC)
				if (b_jam_density_method){
					fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc_Homogeneous Newell’s Prediction (Macroscopic) Regulating MPC.csv", ofstream::out);
					fout_ncacc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_Homogeneous Newell’s Prediction (Macroscopic) Regulating MPC.csv", ofstream::out);
				}
				else{
					fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc_Heterogeneous Prediction (Microscopic) Regulating MPC.csv", ofstream::out);
					fout_ncacc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_Heterogeneous Prediction (Microscopic) Regulating MPC.csv", ofstream::out);
				}
			else if (solver == Predictive_deter_Tracking_MPC)
				if (b_jam_density_method){
					fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc_Homogeneous Newell’s Prediction (Macroscopic) Tracking MPC.csv", ofstream::out);
					fout_ncacc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_Homogeneous Newell’s Prediction (Macroscopic) Tracking MPC.csv", ofstream::out);
				}
				else{
					fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc_Heterogeneous Prediction (Microscopic) Tracking MPC.csv", ofstream::out);
					fout_ncacc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_Heterogeneous Prediction (Microscopic) Tracking MPC.csv", ofstream::out);
				}
			else if (solver == Predictive_deter_instananeous_Regulating_MPC){
				fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc_Instantaneous Speed Prediction Regulating MPC.csv", ofstream::out);
				fout_ncacc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_Instantaneous Speed Prediction Regulating MPC.csv", ofstream::out);
			}
			else if (solver == Predictive_constant_regulating_MPC)
			{
				fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc_Constant Speed Prediction Regulating MPC.csv", ofstream::out);
				fout_ncacc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_Constant Speed Prediction Regulating MPC.csv", ofstream::out);
			}
			else if (solver == Predictive_constant_tracking_MPC)
			{
				fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc_Constant Speed Prediction Tracking MPC.csv", ofstream::out);
				fout_ncacc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_Constant Speed Prediction Tracking MPC.csv", ofstream::out);
			}
			else if (solver == Predictive_deter_instananeous_Tracking_MPC){
				fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc_Instantaneous Speed Prediction Tracking MPC.csv", ofstream::out);
				fout_ncacc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_Instantaneous Speed Prediction Tracking MPC.csv", ofstream::out);
			}
			else if (solver == Predictive_DRO_Regulating_MPC || solver == Predictive_DRO_Tracking_MPC || solver == Predictive_DRO_Reference_Stochastic_SDP)
			{
				if (solver == Predictive_DRO_Regulating_MPC){
					fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc_Predictive_DRO_Regulating_MPC.csv", ofstream::out);
					fout_ncacc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_Predictive_DRO_Regulating_MPC.csv", ofstream::out);
				}
				else if (solver == Predictive_DRO_Tracking_MPC)
				{
					fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc_Predictive_DRO_Tracking_MPC.csv", ofstream::out);
					fout_ncacc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_Predictive_DRO_Tracking_MPC.csv", ofstream::out);
				}
				else
				{
					fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc_Predictive_DRO_Reference_Stochastic_SDP.csv", ofstream::out);
					fout_ncacc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_Predictive_DRO_Reference_Stochastic_SDP.csv", ofstream::out);
				}
				

				Engine *ep = engOpen("");
				if (!(ep))
				{
					fout_mpc << "Can't start Matlab engine!" << endl;
					exit(1);
				}
				else
					fout_mpc << "Matlab engine Init!" << endl;
				
// 				if (mclInitializeApplication(NULL, 0))
// 				{
// 					if (Data_driven_DRO_MPCInitialize())
// 					{
// 						fout_mpc << "initialize succesfully" << endl;					
// 					}		
// 				}
// 				else
// 				{
// 					fout_mpc << "initialize fail" << endl;
// 				}
					
			}

			fout_flw_vehicles.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_ncacc_MPC_following_vehicles.csv", ofstream::out);

			fout_mpc << first_veh_tra_fn << endl ;
			first_veh_tra_fn +=  "/NGSIM_shockwave_vehicle_trajectory.csv";//lane 2 0515 // "/NGSIM_1_25_vehicle_trajectory.csv";//
			string out_raw_platoon_traj_fn(cCurrentPath);
			out_raw_platoon_traj_fn += "/out_raw_NGSIM_shockwave_vehicle_trajectory.csv";
			fout_mpc << "NGSIM_shockwave_vehicle_trajectory: " << first_veh_tra_fn << endl;
			read_front_space_time_region_vehicle_trajectory(first_veh_tra_fn, out_raw_platoon_traj_fn, front_region_veh_time_trajectory_data);//read_first_vehicle_trajectory(first_veh_tra_fn, first_veh_trajectory_data);

			fout_mpc << "front_region_veh_time_trajectory_data.size," << front_region_veh_time_trajectory_data.size() << endl;
			leading_CAV_ID = 14; // seney 16;//25; // 
			lead_CAV_state.odometer = DBL_MAX;

			

			try
			{			
				if (solver == Predictive_deter_Regulating_MPC || solver == Predictive_constant_regulating_MPC || solver == Predictive_deter_instananeous_Regulating_MPC)
					ampl_obj.read("X:/Project/ARPA-e/PTV/CACC-VISSIM-v2.0/CACC-VISSIM/Type101DriverModel/DriverModel/Predictive_QP_control_regulating.mod");
				else if (solver == Predictive_deter_Tracking_MPC || solver == Predictive_constant_tracking_MPC || solver == Predictive_deter_instananeous_Tracking_MPC)
					ampl_obj.read("X:/Project/ARPA-e/PTV/CACC-VISSIM-v2.0/CACC-VISSIM/Type101DriverModel/DriverModel/Predictive_QP_control.mod");
				//else if (solver == Predictive_deter_instananeous_Tracking_MPC)
				//	ampl_obj.read("X:/Project/ARPA-e/PTV/CACC-VISSIM-v2.0/CACC-VISSIM/Type101DriverModel/DriverModel/QP_control.mod");
				//else if (solver == Predictive_deter_instananeous_Regulating_MPC)
				//	ampl_obj.read("X:/Project/ARPA-e/PTV/CACC-VISSIM-v2.0/CACC-VISSIM/Type101DriverModel/DriverModel/QP_control_regulating.mod");
			}
			catch (ampl::AMPLException &e)
			{
				fout_mpc << "Read mod Error: " << e.getMessage() << endl;
				
			}
			mpc_QP_control_data_init(mpc_data);

			
		}
		else if (solver == Car_Following_Control )
		{
			char cCurrentPath[FILENAME_MAX];
			if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
				cout << "error current dir" << endl;
			string scenario_fn(cCurrentPath);
			scenario_fn += "\\MOBILE_SENSOR_DATA_ANALYZER_SoftwareRelease"; //"\NEXTA_DTALite_SoftwareRelease";
			string first_veh_tra_fn = scenario_fn;
			scenario_fn += "/NGSIM_summary.csv";
			first_veh_tra_fn += "/NGSIM_first_vehicle_trajectory.csv";
			car_following_param.T = 1.9;
			car_following_param.desired_min_spacing = 8.5;
			//
			//char   buffer[MAX_PATH];
			string out_traj_dir(cCurrentPath); // "\\out_calibration_trajectory.txt"; // C:\\NEXTA_DTALite_SoftwareRelease
			out_traj_dir += "\\MOBILE_SENSOR_DATA_ANALYZER_SoftwareRelease\\out_calibration_trajectory.txt";	//out_traj_dir = getcwd(buffer, MAX_PATH) + out_traj_dir;  
			//out_traj_dir = "C:\\Users\\sdzhao\\Google Drive\\SITS_Lab\\Students\\Shuaidong\\Traffic Flow\\Assignments\\MOBILE_SENSOR_DATA_ANALYZER_SoftwareRelease\\MOBILE_SENSOR_DATA_ANALYZER_SoftwareRelease\\out_calibration_trajectory.txt";// 
			string out_debug_dir(cCurrentPath);
			out_debug_dir += "\\MOBILE_SENSOR_DATA_ANALYZER_SoftwareRelease\\out_calibration_trajectory_debug.txt";
			fout_car_following.open(out_traj_dir, ofstream::out);
			fout_car_following_debug.open(out_debug_dir, ofstream::out);
			//fout_car_following << cCurrentPath << endl;
			//	<< "scenario_fn: " << scenario_fn << endl;
			//
			read_NGSIM_summary(scenario_fn);			
			car_following_param.dt = 0.1;

			read_first_vehicle_trajectory(first_veh_tra_fn,first_veh_trajectory_data);
			fout_car_following_debug << "read_first_vehicle_trajectory: size " << first_veh_trajectory_data.size() << endl;
		}
		else
		{
			InitArrays();
			now = time(0);
			//dt = ctime(&now);
			char   buffer[MAX_PATH];
			//fout_AMPL_debug << " getcwd(buffer, MAX_PATH): " << getcwd(buffer, MAX_PATH) << endl;
			debug_dir = _getcwd(buffer, MAX_PATH) + debug_dir;
			CACC_result_dir = "X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\" + CACC_result_dir;
			//CACC_result_dir = getcwd(buffer, MAX_PATH) + CACC_result_dir;

			get_all_file_names_under_one_folder(debug_dir, AMPL_debug_fn_vec, AMPL_debug_fn_ctr, "txt");
			if (AMPL_debug_fn_ctr < 1)
				fout_AMPL_debug.open(debug_dir + "out_102_AMPL_debug_01.txt", ofstream::out);
			else{
				sort(AMPL_debug_fn_vec.begin(), AMPL_debug_fn_vec.end());
				last_debug_fn = AMPL_debug_fn_vec.back();
				debug_pos = last_debug_fn.find(".");
				debug_fn_index = stoi(last_debug_fn.substr(19, 2));
				out_debug_index = to_string(debug_fn_index + 1);
				if (out_debug_index.length() < 2)
					out_debug_index = "0" + out_debug_index;
				last_debug_fn.replace(19, 2, out_debug_index);
				fout_AMPL_debug.open(debug_dir + last_debug_fn, ofstream::out);
				//fout_AMPL_debug.open("X:\\Project\\ARPA-e\\PTV\\CACC-VISSIM-v2.0\\CACC-VISSIM\\out_102_AMPL_debug.txt", ofstream::out); 
			}

			get_all_file_names_under_one_folder(CACC_result_dir, CACC_fn_vec, CACC_fn_ctr, "csv");
			if (AMPL_debug_fn_ctr < 1){
				fout_ncacc.open(CACC_result_dir + "out_102_newdll_ncacc_01.csv", ofstream::out);
				if (solver < MPC_QP_regulating)
					fout_lqr.open(CACC_result_dir + "out_102_lqr_01.csv", ofstream::out);
				else if (solver >= MPC_QP_regulating && solver < Predictive_deter_Regulating_MPC){
					//fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\out_102_mpc_01.csv", ofstream::out);
					fout_mpc.open(CACC_result_dir + "out_102_mpc_01.csv", ofstream::out);
					fout_velocity_bds.open(CACC_result_dir + "out_102_mpc_velocity_bounds_01.csv", ofstream::out);
				}
			}
			else{
				sort(CACC_fn_vec.begin(), CACC_fn_vec.end());
				last_CACC_fn = CACC_fn_vec.back();
				CACC_pos = last_CACC_fn.find(".");
				CACC_fn_index = stoi(last_CACC_fn.substr(21, 2));
				out_CACC_index = to_string(CACC_fn_index + 1);
				if (out_CACC_index.length() < 2)
					out_CACC_index = "0" + out_CACC_index;
				last_CACC_fn.replace(21, 2, out_CACC_index);
				fout_ncacc.open(CACC_result_dir + last_CACC_fn, ofstream::out);
				if (solver < MPC_QP_regulating)
					fout_lqr.open(CACC_result_dir + "_lqr_" + last_CACC_fn, ofstream::out);
				else if (solver >= MPC_QP_regulating){
					fout_mpc.open(CACC_result_dir + "_mpc_" + last_CACC_fn, ofstream::out);
					fout_velocity_bds.open(CACC_result_dir + "_mpc_velocity_bounds_" + last_CACC_fn, ofstream::out);
					//fout_ncacc.open("X:\\Project\\ARPA-e\\PTV\\CACC-VISSIM-v2.0\\CACC-VISSIM\\out_102_newdll_ncacc.csv", ofstream::out);//txt", std::ios_base::out);
				}
			}
			//

			//
			fout_ncacc << "current_veh_no,timestep,veh_odmeter, Acc_vehicle,Spd_vehicle,lateral_pos,current_link,current_lane,preceding_vehicles[current_veh_no],preceding_vehiclesDist[current_veh_no],preceding_vehiclesSpeed[current_veh_no],preceding_vehiclesAcc[current_veh_no],preceding_vehiclesWidth[current_veh_no],adj_index1,adj_index2" << endl;
			// the commented-out section below is for debugging purposes.
			//fout_ncacc.open("Debugging_101.csv", std::ios_base::out);
			//fout_ncacc << "EgoID" << "," << "Time" << "," << "LeadingID" << "," << "LeadingSpd" << "," << "D01" << ","<<"Lateral_Pos"<<endl;
			//fout_ncacc << "desired_lane_angle" << "," << "active_lane_change" << "," << "rel_target_lane" << ","<<"veh_rel_target_lane"<<","<< "veh_active_lane_change" << ","<<"lateral_pos"<<endl; //for lane changing test purpose
			//fout_ncacc << "EgoID" << "," << "Time" << "," << "Location" << "," << "Lane" << "," << "Head" << "," << "leaderID" << "," << "followerID" << "," << "front_Connect" << "," << "rear_Connect" << "," << "seq" << "," << "front_dist" << "," << "rear_dist" << endl;
			//fout_ncacc << "EgoID" << "," << "Time" << "," << "HeadID" << "," << "LeadingID" << "," << "LeadingSpd" << "," << "Front_Dist" << ","
			//	<< "EgoID" << "," << "EgoSpd" << "," << "EgoDesiredSpd" << "," << "LocationInPlatoon" << ","
			//	<< "FollowingID" << "," << "FollowingSpd" << "," << "Back_Dist" << "," << endl;
			//fout<<" --------------------- "<<dt;
			//fout.flush();
			bool b_lqr_load_first_24_vehicles = true;
			if (b_lqr_load_first_24_vehicles == true && (solver == LQR_state_feedback || solver == observer_based_feedback_tracking || solver == Stochastic_MPC_QP)){
				char cCurrentPath[FILENAME_MAX];
				if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
					cout << "error current dir" << endl;
				string first_veh_tra_fn(cCurrentPath);
				fout_mpc.open("X:\\Users\\sdzhao\\Class_Assistant\\Traffic Flow\\Car_Following_Model_Calibration\\COM_add_NGSIM_vehicles\\COM_Add_Vehicles\\COM_Add_Vehicles\\VISSIM\\fout_mpc.csv", ofstream::out);
				fout_mpc << first_veh_tra_fn << endl;
				first_veh_tra_fn += "/NGSIM_1_25_vehicle_trajectory.csv";
				fout_mpc << "first_veh_tra_fn: " << first_veh_tra_fn << endl;
				//read_front_space_time_region_vehicle_trajectory(first_veh_tra_fn, front_region_veh_time_trajectory_data);//read_first_vehicle_trajectory(first_veh_tra_fn, first_veh_trajectory_data);

				fout_mpc << "front_region_veh_time_trajectory_data.size," << front_region_veh_time_trajectory_data.size() << endl;
				leading_CAV_ID = 25;
				lead_CAV_state.odometer = DBL_MAX;
			}

			// AMPL init

			try
			{
				if (solver == Data_driven_MPC)
					ampl_obj.read("X:/Project/ARPA-e/PTV/CACC-VISSIM-v2.0/CACC-VISSIM/Type101DriverModel/DriverModel/MPC_control.mod");//"X:/Project/ARPA-e/PTV/CACC-VISSIM-v2.0/CACC-VISSIM/Type101DriverModel/DriverModel/x64/Debug/MPC_control.mod"
				else if (solver == MPC_tracking)
					ampl_obj.read("X:/Project/ARPA-e/PTV/CACC-VISSIM-v2.0/CACC-VISSIM/Type101DriverModel/DriverModel/QP_control.mod");
				else if (solver == MPC_QP_regulating)
					ampl_obj.read("X:/Project/ARPA-e/PTV/CACC-VISSIM-v2.0/CACC-VISSIM/Type101DriverModel/DriverModel/QP_control_regulating.mod");
				else if (solver == Stochastic_MPC_QP)
					ampl_obj.read("X:/Project/ARPA-e/PTV/CACC-VISSIM-v2.0/CACC-VISSIM/Type101DriverModel/DriverModel/QP_stochastic_control.mod");
			}
			catch (ampl::AMPLException &e)
			{
				fout_AMPL_debug << "Read mod Error: " << e.getMessage() << endl;
				return false;
			}
			catch (...)
			{
				fout_AMPL_debug << "Cannot read Error, try read C drive" << endl;
			}

			fin.open("caccconf101.dat", std::ios_base::in);
			if (fin) {
				getline(fin, str);
				MaxPlatoonSize = atoi(str.c_str()); // vehicles
				getline(fin, str);
				HWShort = atof(str.c_str()); // m/s/s
				getline(fin, str);
				HWLong = atof(str.c_str()); // m/s/s
			}
			else
			{
				MaxPlatoonSize = 5;
				HWShort = 0.6;
				HWLong = 2.0;
			}
			fin.close();

			lead_CAV_state.odometer = DBL_MAX;
		}
		return 1;
	case DRIVER_COMMAND_CREATE_DRIVER:
		VehTargetLane[current_veh_no] = 0;
		platoonState[current_veh_no][0] = 0;
		return 1;
	case DRIVER_COMMAND_KILL_DRIVER:
		VehTargetLane.erase(current_veh_no);
		platoonState.erase(current_veh_no);
		write_vehicle_records_to_NGSIM(NGSIM_data_vector, current_veh_no);
		return 1;
	case DRIVER_COMMAND_MOVE_DRIVER:

		leaderID = AdjVehicles[2][3];
		followerID = AdjVehicles[2][1];
		front_dist = AdjVehiclesDist[2][3];
		rear_dist = -AdjVehiclesDist[2][1];

		if (solver == Car_Following_Control)
		{
			float subject_veh_spd;
			float subject_veh_acc;
			float subject_veh_pos;
			
			if (current_veh_no == 1) // first vehicle
			{
				int trajectory_data_index = (int)(timestep *10);
				if (first_veh_trajectory_data.find(trajectory_data_index) == first_veh_trajectory_data.end()){
					desired_acceleration = 0;
					fout_car_following_debug << "Cannot find data for Control first vehicle data index:" << trajectory_data_index << " at t: " << timestep << endl;
				}
				else
				{
					desired_velocity = first_veh_trajectory_data[trajectory_data_index].Vehicle_Velocity;
					desired_acceleration = first_veh_trajectory_data[trajectory_data_index].Vehicle_Acceleration;
					fout_car_following_debug << "Control first vehicle at t: " << timestep << ", desired_velocity: " << desired_velocity << ", desired_acceleration: " << desired_acceleration << endl;
					last_des_spd = first_veh_trajectory_data[trajectory_data_index].Vehicle_Velocity;
					last_des_spd = 4.8731424;
				}

				//for traffic flow class term project of students
				int shockwavetime = 124;
				if (timestep > shockwavetime && Spd_vehicle > 0)
				{
					desired_acceleration = (0 - Spd_vehicle) / 0.1;
				}
				if (timestep > shockwavetime+10){
					
					desired_acceleration = (last_des_spd - Spd_vehicle) / 0.1;
				}

				 shockwavetime = 244;
				if (timestep > shockwavetime && Spd_vehicle > 0)
				{
					desired_acceleration = (0 - Spd_vehicle) / 0.1;
				}
				if (timestep > shockwavetime + 10){

					desired_acceleration = (last_des_spd - Spd_vehicle) / 0.1;
				}
			}
			else
			{

				//update_position_speed(speed_predict_model, car_following_param, subject_veh_spd, subject_veh_acc, subject_veh_pos);
				if (veh_no_car_following_update_stauts.find(current_veh_no) == veh_no_car_following_update_stauts.end())
					veh_no_car_following_update_stauts[current_veh_no] = false;
				else
				{
					if (veh_no_car_following_update_stauts[current_veh_no] == true)
					{
						if (current_veh_no > 1){
							float des_delta_spd = car_following_param.control_des_delta_v_map[current_veh_no];
							desired_acceleration = (des_delta_spd) / car_following_param.dt;
							desired_acceleration = min(max(car_following_param.min_acc, desired_acceleration), car_following_param.max_acc);
							//fout_car_following << "2.veh_no:" << current_veh_no<<",des_delta_spd:" << des_delta_spd << ", desired_acceleration : " << desired_acceleration << endl;
						}
						else
						{
							desired_acceleration = (desired_velocity - Spd_vehicle) / car_following_param.dt;
						}
						if (timestep > (car_following_param.control_start_t_map[current_veh_no] + car_following_param.T))
							veh_no_car_following_update_stauts[current_veh_no] = false;
					}
					else
					{
						if (current_veh_no > 1){  // only control the following vehicle
							car_following_param.control_start_t_map[current_veh_no] = timestep;
							update_position_speed(speed_predict_model, car_following_param, subject_veh_spd, subject_veh_acc, subject_veh_pos);
							car_following_param.control_des_spd_map[current_veh_no] = subject_veh_spd;
							float num_dt = car_following_param.T / car_following_param.dt;
							car_following_param.control_des_delta_v_map[current_veh_no] = (subject_veh_spd - Spd_vehicle) / num_dt;// abs(Spd_vehicle - subject_veh_spd) / num_dt;
							//
							float des_delta_spd = car_following_param.control_des_delta_v_map[current_veh_no];
							desired_acceleration = (des_delta_spd) / car_following_param.dt;
							desired_acceleration = min(max(car_following_param.min_acc, desired_acceleration), car_following_param.max_acc);
							//fout_car_following << "1.veh_no:"<< current_veh_no<<",des_delta_spd:" << des_delta_spd << ",desired_acceleration:" << desired_acceleration << endl;
						}
						else
						{
							desired_acceleration = (desired_velocity - Spd_vehicle) / car_following_param.dt;
						}
						veh_no_car_following_update_stauts[current_veh_no] = true;
					}
					fout_car_following_debug << "Newell control: , vehno:" << current_veh_no << ",des_acc:" << desired_acceleration << endl;
				}
				avoid_collision_acc(2.1);
			}
			push_vehicle_records_to_NGSIM(NGSIM_data_vector);
			fout_car_following_debug << "map size: " << NGSIM_data_vector.size() << endl;
		}
		else
		{
			veh_odometer += front_region_veh_time_trajectory_data[current_veh_no - 1].begin()->second.Global_Y;

			if (solver <= Predictive_DRO_Tracking_MPC &&solver >= Predictive_deter_Regulating_MPC)
			{
				//
				//
				if (current_veh_no < leading_CAV_ID){
					int trajectory_data_index = (int)(timestep * 10);
					if (front_region_veh_time_trajectory_data[current_veh_no - 1].find(trajectory_data_index) == front_region_veh_time_trajectory_data[current_veh_no - 1].end() ||
						front_region_veh_time_trajectory_data[current_veh_no - 1].find(trajectory_data_index+1) == front_region_veh_time_trajectory_data[current_veh_no - 1].end()){
						desired_acceleration = 0;
						desired_velocity = 29;
						//if (Spd_vehicle < 2.0){
							desired_acceleration = (desired_velocity - Spd_vehicle) / 0.1;
						//}
						//fout_mpc << "not find," << "vno," << current_veh_no << ",t," << trajectory_data_index << ",spd," << Spd_vehicle << ",desired_velocity," << desired_velocity << ",desired_acceleration," << desired_acceleration <<",real_acc," << Acc_vehicle << endl;
					}
					else
					{
						double desired_traveled_dist = front_region_veh_time_trajectory_data[current_veh_no - 1][trajectory_data_index+1].Global_Y - front_region_veh_time_trajectory_data[current_veh_no - 1][trajectory_data_index].Global_Y;
						desired_velocity = desired_traveled_dist/0.1;
						desired_acceleration = (desired_velocity - Spd_vehicle) / 0.1;
						//desired_acceleration = front_region_veh_time_trajectory_data[current_veh_no - 1][trajectory_data_index].Vehicle_Acceleration;
						//fout_mpc << "find," << "vno," << current_veh_no << ",t," << trajectory_data_index << ",spd," << Spd_vehicle << ",desired_velocity," << desired_velocity << ",desired_acceleration," << desired_acceleration << ",real_acc," << Acc_vehicle << endl;
					}
				}
// 				if (current_veh_no == 1) // first vehicle
// 				{
// 					int trajectory_data_index = (int)(timestep * 10);
// 					if (first_veh_trajectory_data.find(trajectory_data_index) == first_veh_trajectory_data.end()){
// 						desired_acceleration = 0;
// 					}
// 					else
// 					{
// 						desired_velocity = first_veh_trajectory_data[trajectory_data_index].Vehicle_Velocity;
// 						desired_acceleration = first_veh_trajectory_data[trajectory_data_index].Vehicle_Acceleration;
// 						//fout_mpc << "Control first vehicle at t: " << timestep << ", desired_velocity: " << desired_velocity << ", desired_acceleration: " << desired_acceleration << endl;
// 					}
// 			

				if (current_veh_no == leading_CAV_ID)
					lead_CAV_state.odometer = veh_odometer;
				//
				update_front_space_time_region_observation(current_veh_no);
				//fout_mpc << "lead_CV_front_vehicles_state_t_vno_indices.size(), " << lead_CV_front_vehicles_state_t_vno_indices.size() << endl; 

				//if (current_veh_no - leading_CAV_ID >= MaxPlatoonSize)
		

				if (veh_type == 102)  //CACC
				{
					if (find(unique_vehicle_no.begin(), unique_vehicle_no.end(), (int)current_veh_no) == unique_vehicle_no.end())
					{
						vno_mpc_headway_ready_flag[current_veh_no] = false;

						unique_vehicle_no.push_back((int)current_veh_no);
						veh_no_to_index[(int)current_veh_no] = unique_vehicle_no.size() - 1;
						fout_mpc << "current_veh_no," << current_veh_no << ",veh_no_to_index.size," << veh_no_to_index.size() << endl;
					}
					mpc_initial_state.MPC_veh_no_init_acc[current_veh_no] = Acc_vehicle;
					mpc_initial_state.MPC_veh_no_init_loc[current_veh_no] = veh_odometer;
					mpc_initial_state.MPC_veh_no_init_spd[current_veh_no] = Spd_vehicle;
					mpc_initial_state.MPC_veh_no_init_front_loc[current_veh_no] = veh_odometer+AdjVehiclesDist[2][3];
					mpc_initial_state.MPC_veh_no_init_front_spd[current_veh_no] = AdjVehiclesSpeed[2][3];
				}
				//

				if (current_veh_no <= (leading_CAV_ID + MaxPlatoonSize)){
					//for platoon vehicle, the desired_acceleration is from last timestep, which means it should be the current acc based on the definiteion of desired_acceleration.
					double des_spd = desired_acceleration*0.1 + prev_veh_velocity[current_veh_no];
					double des_loc = 0.5*desired_acceleration*0.1*0.1 + prev_veh_odometer[current_veh_no];
					fout_ncacc << current_veh_no << ","
						//			<< veh_type << ","
						<< timestep << ","
						<< veh_odometer << ","
						<< Acc_vehicle << ","
						<< Spd_vehicle << ","
						<< lateral_pos << ","
						<< current_link << ","
						<< current_lane << ","
						<< AdjVehicles[2][3] << ","
						<< AdjVehiclesDist[2][3] << ","
						<< AdjVehiclesSpeed[2][3] << ","
						<< AdjVehiclesAcc[2][3] << ","
						<< AdjVehiclesWidth[2][3] << ",";
					fout_ncacc << des_spd << ","
						<< des_loc << ","
						<< desired_acceleration << ","
						<< MaxAcc_vehicle << ",";
					//fout_ncacc << endl;

					prev_veh_velocity[current_veh_no] = Spd_vehicle;
					prev_veh_odometer[current_veh_no] = veh_odometer;
				}
				else if (current_veh_no <= (leading_CAV_ID + MaxPlatoonSize+6)) {
					double des_spd = desired_acceleration*0.1 + prev_veh_velocity[current_veh_no];
					double des_loc = 0.5*desired_acceleration*0.1*0.1 + prev_veh_odometer[current_veh_no];
					fout_flw_vehicles << current_veh_no << ","
						//			<< veh_type << ","
						<< timestep << ","
						<< veh_odometer << ","
						<< Acc_vehicle << ","
						<< Spd_vehicle << ","
						<< lateral_pos << ","
						<< current_link << ","
						<< current_lane << ","
						<< AdjVehicles[2][3] << ","
						<< AdjVehiclesDist[2][3] << ","
						<< AdjVehiclesSpeed[2][3] << ","
						<< AdjVehiclesAcc[2][3] << ","
						<< AdjVehiclesWidth[2][3] << ",";
					fout_flw_vehicles << des_spd << ","
						<< des_loc << ",";
					fout_flw_vehicles << endl;

					prev_veh_velocity[current_veh_no] = Spd_vehicle;
					prev_veh_odometer[current_veh_no] = veh_odometer;
				}

				if (solver == Predictive_deter_Regulating_MPC || solver == Predictive_deter_Tracking_MPC || solver == Predictive_DRO_Regulating_MPC || solver == Predictive_DRO_Tracking_MPC || solver == Predictive_DRO_Reference_Stochastic_SDP)
				{

					if (lead_CV_front_vehicles_state_t_vno_indices.size() >= front_region_observation_length &&
						lead_CV_front_vehicles_state_t_vno_indices.front().find(leading_CAV_ID) != lead_CV_front_vehicles_state_t_vno_indices.front().end() &&
						lead_CV_front_vehicles_state_t_vno_indices.back().size() == lead_CV_front_vehicles_state_t_vno_indices[front_region_observation_length - 2].size() &&
						unique_vehicle_no.size() == MaxPlatoonSize)
					{

// 						fout_mpc << "t," << timestep << ",vehno," << current_veh_no << ",Acc_vehicle," << Acc_vehicle << ",desired_acceleration," << desired_acceleration << endl;
// 						fout_mpc << "t," << timestep << ",vehno," << current_veh_no << ",Spd_vehicle," << Spd_vehicle << ",desired_velocity," << desired_velocity << endl;
// 						fout_mpc << "t," << timestep << ",vehno," << current_veh_no << ",veh_odometer," << veh_odometer << ",desired_loc," << desired_loc << endl;
						//measure disturbance
						if (solver == Predictive_DRO_Reference_Stochastic_SDP)
						{
							
						}
						else
						{
							if (mpc_data.observed_acc_error.find(current_veh_no) != mpc_data.observed_acc_error.end()){
								if (mpc_data.observed_acc_error[current_veh_no].size() == 25){
									mpc_data.observed_acc_error[current_veh_no].pop_front();
									mpc_data.observed_spd_error[current_veh_no].pop_front();
									mpc_data.observed_loc_error[current_veh_no].pop_front();
								}
								mpc_data.observed_acc_error[current_veh_no].push_back(Acc_vehicle - desired_acceleration);
								mpc_data.observed_spd_error[current_veh_no].push_back(Spd_vehicle - desired_spd);
								mpc_data.observed_loc_error[current_veh_no].push_back(veh_odometer - desired_loc);
							}
							else
							{
								mpc_data.observed_acc_error[current_veh_no].push_back(Acc_vehicle - desired_acceleration);
								mpc_data.observed_spd_error[current_veh_no].push_back(Spd_vehicle - desired_spd);
								mpc_data.observed_loc_error[current_veh_no].push_back(veh_odometer - desired_loc);
							}
						}
						
						
						//fout_mpc << "t " << timestep << ",current_veh_no," << current_veh_no << ",mpc_data.observed_spd_error size ," << mpc_data.observed_loc_error.size() << ",num obs per veh:" << mpc_data.observed_spd_error[current_veh_no].size() << endl;
						if (current_veh_no == leading_CAV_ID)
						{
							//fout_mpc << "start CACC at timstep: " << timestep << endl;

							//1. calculate jam density to calibrate Newell's model
							//2. predict the front vehicle trajectories and platoon vehicles's trajectories
							//3. run MPC for platoon

							//construct speed and density vectors
							//predict front and cav trajectory
							vector<vector<VehicleState>> t_vno_predicted_front_vehicle_trajectory;
							//for reference stochastic speed scenarios
							map<int, vector<vector<VehicleState>>> scenarioidx_tidx_vno_predicted_vehicle_trajectory;

							mpc_data.total_cpu_time = get_current_cpu_time_in_seconds();
							
							if (b_jam_density_method){
								vector<double> speed_vec;
								vector<double> density_vec;

								double calibrate_cpu_t = get_current_cpu_time_in_seconds();
								construct_density_profile_from_space_time_region(speed_vec, density_vec);
								//fout_mpc << "speed_vec," << speed_vec.size() << ",density_vec," << density_vec.size() << endl;
								double jam_density = calculate_jam_density(density_vec, speed_vec);
								fout_mpc << "CPUtime predict_mpc_QP at timestep: " << timestep << " : " << get_current_cpu_time_in_seconds() - calibrate_cpu_t << endl;
								fout_mpc << "timestep," << timestep << ",jam_density, " << jam_density << endl;
								if (jam_density < 60.0 || jam_density > 200.0 || DBL_MAX)
								{
									jam_density = 120.0;
								}
								double wave_spd = 17.0; //mph. Xuesong Zhou
								car_following_param.T = max(0.4, min(3600 / (wave_spd*jam_density), 3.2));
								car_following_param.dt = 0.1;
								car_following_param.desired_min_spacing = min(14.0, max(6.0, 1609.34 / jam_density));
								// 							fout_ncacc << car_following_param.T << ","
								// 								<< car_following_param.desired_min_spacing << ",";
								// 							fout_ncacc << endl;
								//fout_mpc << ",desired_min_spacing," << car_following_param.desired_min_spacing << ",jam_density," << jam_density << endl;
								
								predict_vehicle_trajectory(car_following_param, t_vno_predicted_front_vehicle_trajectory);
							}
							else  //least square method - calibrate parameters for different drivers heterogeneously
							{
								map<int, CarFollowingParam> heter_newell_parameters;
								double calibrate_cpu_t = get_current_cpu_time_in_seconds();
								least_square_calibrate_newell_parameters(heter_newell_parameters);
								fout_mpc << "CPUtime predict_mpc_QP at timestep: " << timestep << " : " << get_current_cpu_time_in_seconds() - calibrate_cpu_t << endl;
								// 							fout_ncacc << heter_newell_parameters[current_veh_no].T << ","
								// 								<< heter_newell_parameters[current_veh_no].desired_min_spacing << ",";
								// 							fout_ncacc << endl;
								
								if (solver == Predictive_DRO_Reference_Stochastic_SDP){
									fout_mpc << "Predict Predictive_DRO_Reference_Stochastic_SDP" << endl;
									predict_vehicle_trajectory_reference_stochastic(heter_newell_parameters,
										scenarioidx_tidx_vno_predicted_vehicle_trajectory);
									fout_mpc << "Done Predict Predictive_DRO_Reference_Stochastic_SDP" << endl;
								}
								else
								{
									predict_vehicle_trajectory(heter_newell_parameters, t_vno_predicted_front_vehicle_trajectory);
									fout_mpc << "done ... predict_vehicle_trajectory..." << endl; 
								}
								

							}
										
							//run mpc
							

							mpc_data.total_cpu_time = get_current_cpu_time_in_seconds();
							if (solver == Predictive_DRO_Regulating_MPC || solver == Predictive_DRO_Tracking_MPC)
							{
								//fout_mpc << "cur veh no: " << current_veh_no  << ",obs num vehs: " << mpc_data.observed_acc_error.size() <<  endl;
								if (mpc_data.observed_acc_error[current_veh_no].size() == 25){
									fout_mpc << "observed_acc_error size 25" << endl;

									mpc_data.v_upper = 40.0;
									mpc_data.u_upper = 10007.0;
									mpc_data.u_lower = -10007.28;
									mpc_data.head = Predictive_headway;
									
									double c_t = get_current_cpu_time_in_seconds();
									solve_DRO_MPC_Matlab(t_vno_predicted_front_vehicle_trajectory,
										solver, mpc_data, mpc_initial_state,
										unique_vehicle_no, veh_odometer + AdjVehiclesDist[2][3]);

									fout_mpc << "solve_DRO_MPC_Matlab done...CPU t:" << get_current_cpu_time_in_seconds()-c_t << endl;
									
									desired_acceleration = mpc_data.vec_u[veh_no_to_index[leading_CAV_ID]];
								}
							}
							else if (solver== Predictive_DRO_Reference_Stochastic_SDP)
							{
								//scenarioidx_tidx_vno_predicted_vehicle_trajectory
								mpc_data.v_upper = 40.0;
								mpc_data.u_upper = 10007.0;
								mpc_data.u_lower = -10007.28;
								mpc_data.head = Predictive_headway;
								fout_mpc << "solving solve_DRO_MPC_Matlab" << endl;
								double solv_cput = solve_DRO_MPC_Matlab(scenarioidx_tidx_vno_predicted_vehicle_trajectory,
									solver, mpc_data, mpc_initial_state,
									unique_vehicle_no, veh_odometer + AdjVehiclesDist[2][3]);
								fout_mpc << "Stochastic_SDP matlab done...CPU t:" << solv_cput << endl;
							}
							else
							{
								mpc_data.total_cpu_time = get_current_cpu_time_in_seconds();
								mpc_data.v_upper = 40.0;
								mpc_data.u_upper = 10007.0;
								mpc_data.u_lower = -10007.28;
								mpc_data.head = Predictive_headway;
								mpc_data.Ph = 50;
								predictive_mpc_QP_control_data_update(t_vno_predicted_front_vehicle_trajectory,
									ampl_obj, solver, mpc_initial_state, unique_vehicle_no, mpc_data, veh_odometer + AdjVehiclesDist[2][3]);
								fout_mpc << "CPUtime data_update at timestep: " << timestep << " : " << get_current_cpu_time_in_seconds() - mpc_data.total_cpu_time << endl;

		
								solve_MPC_QP(ampl_obj, mpc_data);

								fout_mpc << "done solve_MPC_QP, iter: " << mpc_data.solve_ampl_times << endl;
								fout_mpc << "CPUtime solve_MPC_QP at timestep: " << timestep << " : " << get_current_cpu_time_in_seconds() - mpc_data.total_cpu_time << endl;
								
								desired_acceleration = mpc_data.vec_u[veh_no_to_index[leading_CAV_ID]];
							}
							
							fout_mpc << "leading_CAV_ID," << current_veh_no << ",AdjVehiclesDist," << AdjVehiclesDist[2][3] << ",AdjVehicles," << AdjVehicles[2][3] << ",AdjVehiclesSpeed," << AdjVehiclesSpeed[2][3] << leading_CAV_ID << ",desired_velocity," << desired_velocity << ",Spd_vehicle," << Spd_vehicle << ",desired_acceleration," << desired_acceleration << ",max_acc," << MaxAcc_vehicle << ",real_acc," << Acc_vehicle << endl;
						}
						else if (veh_type == 102)
						{
							int veh_index = veh_no_to_index[current_veh_no];
							if (veh_index >= mpc_data.vec_u.size())
							{
								fout_mpc << "veh_index >= LQR_data.vec_u.size" << endl;
								HWShort = Predictive_headway + 1.0 ;
								HWLong = Predictive_headway + 1.0;
								ControlVehicle();
								fout << "veh_index >= mpc_data.vec_u.size(), ";
							}
							else
							{
								desired_acceleration = mpc_data.vec_u[veh_index];
								fout << "vec_u[veh_index], ";
							}
							fout_mpc << "current_veh_no," << current_veh_no << ",AdjVehiclesDist," << AdjVehiclesDist[2][3] << ",AdjVehicles," << AdjVehicles[2][3] << ",AdjVehiclesSpeed," << AdjVehiclesSpeed[2][3] << leading_CAV_ID << ",desired_velocity," << desired_velocity << ",Spd_vehicle," << Spd_vehicle << ",desired_acceleration," << desired_acceleration << ",max_acc," << MaxAcc_vehicle <<",real_acc,"<< Acc_vehicle<< endl;

						}
					}
					else if (veh_type == 102){
						//Since all CACC are connected. Set headway for Predictive control
						HWShort = Predictive_headway + 1.0;
						HWLong = Predictive_headway + 1.0;
						ControlVehicle();
						fout_mpc << "t," << timestep << ",precontrol:current_veh_no," << current_veh_no << ",lead_CV_front_vehicles_state_t_vno_indices.size(), " << lead_CV_front_vehicles_state_t_vno_indices.size() << ",desired_velocity," << desired_velocity << ",Spd_vehicle," << Spd_vehicle << ",desired_acceleration," << desired_acceleration << ",max_acc," << MaxAcc_vehicle << ",real_acc," << Acc_vehicle << endl;

					}
					double dn = max(7.0, Spd_vehicle * Predictive_headway);
					//
					if (current_veh_no != leading_CAV_ID && veh_type == 102 && (AdjVehiclesDist[2][3] < (dn - 2.5) || AdjVehiclesDist[2][3] >(dn + 5.0)))
					{
						//Since all CACC are connected. Set headway for Predictive control
						HWShort = Predictive_headway;
						HWLong = Predictive_headway;
						//ControlVehicle();
						fout_mpc << "Violate constraints, take over..." << "t," << timestep << ",vehno," << current_veh_no << ",desired_acceleration," << desired_acceleration << endl;
					}
					if ((veh_type == 100 && current_veh_no > 1 && AdjVehicles[2][3] != -1))// || current_veh_no == leading_CAV_ID)
					{	
						avoid_collision_acc(Predictive_headway);
						//fout_ncacc << 1.9 << "," << 10.0 << endl;					
						//ControlVehicle();
						fout_mpc << "avoid_collision_acc control: " << current_veh_no << ",loc," << veh_odometer << ",desired_acc," << desired_acceleration << ",real_acc:" << Acc_vehicle << ",AdjVehiclesDist," << AdjVehiclesDist[2][3] << ",AdjVehicles," << AdjVehicles[2][3] << endl;
					}
					//fout_mpc << "vno:" << current_veh_no << ",AdjVehiclesDist[2][3]:" << AdjVehiclesDist[2][3] << ",AdjVehicles[2][3]:" << AdjVehicles[2][3] << ",real_acc," << Acc_vehicle << ",desired_acc," << desired_acceleration << endl;
// 					if (AdjVehicles[2][3] != -1 && veh_type == 100 && (AdjVehiclesDist[2][3] < (dn - 2.5) || AdjVehiclesDist[2][3] >(dn + 5.0)))
// 					{
// 						//Since all CACC are connected. Set headway for Predictive control
// 						HWShort = Predictive_headway;
// 						HWLong = Predictive_headway;
// 						ControlVehicle();
// 					}
					//fout_mpc << "vno:" << current_veh_no << ",AdjVehiclesDist[2][3]:" << AdjVehiclesDist[2][3] << ",AdjVehicles[2][3]:" << AdjVehicles[2][3] << ",real_acc," << Acc_vehicle << ",desired_acc," << desired_acceleration << endl;

					desired_spd = Spd_vehicle + desired_acceleration*0.1;
					desired_loc = veh_odometer+ Spd_vehicle*0.1 + 0.5*desired_acceleration*0.01;			
				}
//				else if (solver == Predictive_deter_instananeous_Tracking_MPC || solver == Predictive_deter_instananeous_Regulating_MPC || solver == Predictive_constant_tracking_MPC || solver == Predictive_constant_regulating_MPC)
//				{
//					if (!b_mpc_init && unique_vehicle_no.size() == MaxPlatoonSize &&  timestep > 18.0)
//					{
//						//initialization		
//						fout_mpc << "start init... at t:" << timestep << endl;
//						double cpt_t = get_current_cpu_time_in_seconds();
//						mpc_QP_control_data_init(mpc_data);
//						fout_mpc << "CPU data_init time ...," << get_current_cpu_time_in_seconds() - cpt_t << endl;
//						mpc_data.total_cpu_time += (get_current_cpu_time_in_seconds() - cpt_t);
//						fout_mpc << "done mpc_qp init..." << endl;
//						b_mpc_init = true;
//					}
//					else if (b_mpc_init == true && b_mpc_headway_ready == true) // done initialization
//					{
//						//measuring vehicle acc, spd
//
//						if (mpc_data.observed_acc_error[current_veh_no].size() == 25){
//							mpc_data.observed_acc_error[current_veh_no].pop_front();
//							mpc_data.observed_spd_error[current_veh_no].pop_front();
//							mpc_data.observed_loc_error[current_veh_no].pop_front();
//						}
//						mpc_data.observed_acc_error[current_veh_no].push_back(Acc_vehicle - desired_acceleration);
//						mpc_data.observed_spd_error[current_veh_no].push_back(Spd_vehicle - desired_velocity);
//						mpc_data.observed_loc_error[current_veh_no].push_back(veh_odometer - desired_loc);
//
//						if (leading_CAV_ID == current_veh_no)
//						{
//							//solve mpc							
//						
//
//							fout_mpc << "update state and control..., v_upper: " << mpc_data.v_upper << endl;
//
//							lead_CAV_state.odometer = veh_odometer;
//							double nveh_loc = veh_odometer + AdjVehiclesDist[2][3];
//							fout_mpc << "t: " << timestep << "Lead front veh dist: " << AdjVehiclesDist[2][3] << ", nveh_loc: " << nveh_loc << endl;
//							double nveh_spd = AdjVehiclesSpeed[2][3];
//
//							double cpu_t = get_current_cpu_time_in_seconds();
//							mpc_data.v_upper = 40.0;
//							mpc_data.u_upper = 10007.0;
//							mpc_data.u_lower = -10007.28;
//							if (solver == Predictive_deter_instananeous_Regulating_MPC || solver == Predictive_deter_instananeous_Tracking_MPC){
//								mpc_data.head = Predictive_headway;
//								constant_mpc_QP_control_data_update(ampl_obj, solver, mpc_initial_state, unique_vehicle_no, mpc_data, nveh_loc, nveh_spd);
//								//mpc_QP_control_data_update(ampl_obj, solver, mpc_initial_state, unique_vehicle_no, mpc_data, nveh_loc, AdjVehiclesSpeed[2][3]);
//							}
//							else if (solver == Predictive_constant_tracking_MPC || solver == Predictive_constant_regulating_MPC)
//							{				
//								mpc_data.head = Predictive_headway;
//								constant_mpc_QP_control_data_update(ampl_obj, solver, mpc_initial_state, unique_vehicle_no, mpc_data, nveh_loc, nveh_spd);
//							}
//							fout_mpc << "CPU data_update...," << get_current_cpu_time_in_seconds() - cpu_t << endl;
//							cpu_t = get_current_cpu_time_in_seconds();
//							mpc_data.v_upper = 40.0;
//							mpc_data.u_upper = 10007.0;
//							mpc_data.u_lower = -10007.28;
//							mpc_data.head = Predictive_headway;
//							solve_MPC_QP(ampl_obj, mpc_data);
//							fout_mpc << "solve mpc CPU time at t: " << timestep << "-->" << get_current_cpu_time_in_seconds() - cpu_t << endl;
//							fout_mpc << "done solve_MPC_QP, iter: " << mpc_data.solve_ampl_times << endl;
//							//fout_mpc << "total_cpu_time at timestep: " << timestep << " : " << mpc_data.total_cpu_time << endl;
//
//
//							desired_acceleration = mpc_data.vec_u[veh_no_to_index[leading_CAV_ID]];
//							fout_mpc << "leading_CAV_ID," << ",AdjVehiclesDist," << AdjVehiclesDist[2][3] << ",AdjVehicles," << AdjVehicles[2][3] << ",AdjVehiclesSpeed," << AdjVehiclesSpeed[2][3] << leading_CAV_ID << ",desired_velocity," << desired_velocity << ",Spd_vehicle," << Spd_vehicle << ",desired_acceleration," << desired_acceleration << endl;
//
//						}
//						else if (veh_type == 102)
//						{
//							int veh_index = veh_no_to_index[current_veh_no];
//							if (veh_index >= mpc_data.vec_u.size())
//								fout_mpc << "veh_index >= LQR_data.vec_u.size" << endl;
//							else
//							{
//								desired_acceleration = mpc_data.vec_u[veh_index];
//							}
//							//fout_mpc << "current_veh_no," << current_veh_no << ",desired_velocity," << desired_velocity << ",Spd_vehicle," << Spd_vehicle << ",desired_acceleration," << desired_acceleration << endl;
//						}
//					}
//					else if (veh_type == 102){
//						
//						//fout_ncacc << 0.1 << "," << 0.6 << endl;
//						if (AdjVehiclesDist[2][3] >= Predictive_headway*Spd_vehicle){
//							vno_mpc_headway_ready_flag[current_veh_no] = true;			
//							fout_mpc << "vno_mpc_headway_ready_flag t:" << timestep << ",vno:" << current_veh_no << ",true" << endl;
//						}
//						b_mpc_headway_ready = true;
//						for (auto &iter : vno_mpc_headway_ready_flag){
//							if (iter.second == false){
//								fout_mpc << "ControlVehicle() t:" << timestep << ",vno:" << current_veh_no << ",false" << endl;
//								b_mpc_headway_ready = false;
//							}
//						}
//						//Since all CACC are connected. Set headway for Predictive control
//						HWShort = Predictive_headway;// +0.8;
//						HWLong = Predictive_headway;// +0.8;
//						ControlVehicle();
//						fout_mpc << "ControlVehicle() t:" << timestep << ",b_mpc_headway_ready," << b_mpc_headway_ready << ",vno:" << current_veh_no << ",front_vno:" << AdjVehicles[2][3] << ",front_dist: " << AdjVehiclesDist[2][3] << endl;
//
//					}
//					
//					double dn = max(10.0, Spd_vehicle * Predictive_headway);
//					if (veh_type == 102 && (AdjVehiclesDist[2][3] < 4.0))
//					{
//						fout_mpc << "unsafe gap t:" << timestep << ",vno:" << current_veh_no << ",front_vno:" << AdjVehicles[2][3] << ",front_dist: " << AdjVehiclesDist[2][3] << endl;
//					}
//
//// 					if (veh_type == 102 && (AdjVehiclesDist[2][3] < (dn - 2.5) || AdjVehiclesDist[2][3] >(dn + 5.0))){
//// 										//Since all CACC are connected. Set headway for Predictive control
//// 										fout_mpc << "maintain gap t:" << timestep << ",vno:" << current_veh_no << ",front_vno:" << AdjVehicles[2][3] << ",front_dist: " << AdjVehiclesDist[2][3] << endl;
//// 										HWShort = Predictive_headway;
//// 										HWLong = Predictive_headway;
//// 										//ControlVehicle();
//// 								
//					if (veh_type == 100 && current_veh_no > 1 && AdjVehicles[2][3] != -1){
//						//avoid_collision_acc();
//						//fout_ncacc << 1.9 << "," << 10.0 << endl;
//						//ControlVehicle();
//						fout_mpc << "avoid_collision_acc control: " << current_veh_no <<  "desired_acc" << desired_acceleration << ",real_acc:"<<Acc_vehicle << endl;
//					}
//// 					if (veh_type == 100 && (AdjVehiclesDist[2][3] < (dn - 2.5) || AdjVehiclesDist[2][3] >(dn + 5.0)))
//// 					{
//// 						//Since all CACC are connected. Set headway for Predictive control
//// 						HWShort = Predictive_headway + 0.8;
//// 						HWLong = Predictive_headway + 0.8;
//// 						ControlVehicle();
//// 					}
//// 					if (veh_type == 102)
//// 						avoid_collision_acc();
//
//					desired_velocity = Spd_vehicle + desired_acceleration*0.1;
//					desired_loc = veh_odometer + Spd_vehicle*0.1 + 0.5*desired_acceleration*0.01;
//				}
				

				//desired_velocity = Spd_vehicle + Acc_vehicle*0.1 + desired_acceleration*0.1;
				fout_ncacc << desired_velocity << ",";
				fout_ncacc << desired_acceleration << endl;
			}

			fout_mpc << "vehno:" << current_veh_no << ",t," << timestep << ",desired_acceleration," << desired_acceleration << endl;
//			else
// 			{
// 				if (current_veh_no <= (leading_CAV_ID + MaxPlatoonSize))
// 				{
// 					fout_ncacc << current_veh_no << ","
// 						//			<< veh_type << ","
// 						<< timestep << ","
// 						<< veh_odometer << ","
// 						<< Acc_vehicle << ","
// 						<< Spd_vehicle << ","
// 						<< lateral_pos << ","
// 						<< current_link << ","
// 						<< current_lane << ","
// 						<< AdjVehicles[2][3] << ","
// 						<< AdjVehiclesDist[2][3] << ","
// 						<< AdjVehiclesSpeed[2][3] << ","
// 						<< AdjVehiclesAcc[2][3] << ","
// 						<< AdjVehiclesWidth[2][3] << ",";
// 					fout_ncacc << endl;
// 				}
// 				//================================COM control front vehicles with veh type 100================================
// 				if (current_veh_no < leading_CAV_ID){
// 					int trajectory_data_index = (int)(timestep * 10);
// 					if (front_region_veh_time_trajectory_data[current_veh_no - 1].find(trajectory_data_index) == front_region_veh_time_trajectory_data[current_veh_no - 1].end()){
// 						desired_acceleration = 0;
// 						if (Spd_vehicle < 2.0){
// 							desired_acceleration = (desired_velocity - Spd_vehicle) / 0.1;
// 						}
// 						//fout_mpc << "not find," << "vno," << current_veh_no << ",t," << trajectory_data_index << ",spd," << Spd_vehicle << ",desired_velocity," << desired_velocity << ",desired_acceleration," << desired_acceleration << endl;
// 					}
// 					else
// 					{
// 						desired_velocity = front_region_veh_time_trajectory_data[current_veh_no - 1][trajectory_data_index].Vehicle_Velocity;
// 						desired_acceleration = (desired_velocity - Spd_vehicle) / 0.1;
// 						//desired_acceleration = front_region_veh_time_trajectory_data[current_veh_no - 1][trajectory_data_index].Vehicle_Acceleration;
// 						//fout_mpc << "find," << "vno," << current_veh_no << ",t," << trajectory_data_index <<",spd," << Spd_vehicle << ",desired_velocity," << ",desired_velocity," << desired_velocity << ",desired_acceleration," << desired_acceleration << endl;
// 					}
// 				}
// 				/*  &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& */
// 				if (veh_type == 100)
// 				{
// 					update_front_DSRC_region(current_veh_no);
// 					
// 				}
// 				else if (veh_type == 102)  //CACC model
// 				{
// 					if (!b_lqr_init && find(unique_vehicle_no.begin(), unique_vehicle_no.end(), (int)current_veh_no) == unique_vehicle_no.end()){
// 						unique_vehicle_no.push_back((int)current_veh_no);
// 						veh_no_to_index[(int)current_veh_no] = unique_vehicle_no.size() - 1;
// 						fout_lqr << "current_veh_no," << current_veh_no << ",veh_no_to_index.size," << veh_no_to_index.size() << endl;
// 					}
// // 
// // 					if (leading_CAV_ID < 0){
// // 						leading_CAV_ID = current_veh_no;
// // 					}
// 
// 					
// 
// 					VehTargetLane[current_veh_no] = 1;
// 
// 					current_lane = lanes_current_link - current_lane + 1;
// 					lateral_pos_ind = GetLateralPos(lateral_pos);
// 
// 					CAV_init_speed[current_veh_no] = Spd_vehicle;
// 					CAV_init_spacing[current_veh_no] = front_dist;
// 					CAV_init_acc[current_veh_no] = Acc_vehicle;
// 					veh_len_vec[current_veh_no] = veh_len;
// 					veh_odometer_vec[current_veh_no] = veh_odometer;
// 					veh_coord_x_vec[current_veh_no] = veh_x;
// 					veh_coord_y_vec[current_veh_no] = veh_y;
// 					front_veh_speed_vec[current_veh_no] = AdjVehiclesSpeed[2][3];
// 					front_veh_acc_vec[current_veh_no] = AdjVehiclesAcc[2][3];
// 
// 					if (leading_CAV_ID == current_veh_no){
// 						push_acceleration_observation(AdjVehiclesAcc[2][3]);
// 					}
// 
// 					/*  &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& */
// 
// 					if (solver < MPC_QP_regulating)
// 					{
// 						fout_lqr << "sim step," << timestep << ",leading_CAV_ID," << leading_CAV_ID << endl;
// 						lqr_initial_state.lqr_veh_no_init_spd[current_veh_no] = Spd_vehicle;
// 						lqr_initial_state.lqr_veh_no_init_loc[current_veh_no] = veh_odometer;
// 						lqr_initial_state.lqr_veh_no_init_acc[current_veh_no] = Acc_vehicle;
// 						if (!b_lqr_init && unique_vehicle_no.size() == MaxPlatoonSize &&  timestep > 18.0)
// 						{
// 							//initialization		
// 							fout_lqr << "start init..." << endl;
// 							lqr_state_feedback_control_data_init(lqr_initial_state, unique_vehicle_no);
// 							lqr_state_feedback_control_init(LQR_data, lqr_initial_state);
// 
// 							LQR lqr(LQR_data.state_size,
// 								LQR_data.control_size,
// 								LQR_data.dt,
// 								LQR_data.Q,
// 								LQR_data.R,
// 								LQR_data.N,
// 								LQR_data.A,
// 								LQR_data.B,
// 								LQR_data.T, 1);
// 							lqr_global = lqr;
// 							LQR_data.current_t = 0;
// 
// 							fout_lqr << "start lqr tracking init..." << endl;
// 							double lqr_cur_t = get_current_cpu_time_in_seconds();
// 							if (solver == LQR_state_feedback_reference_tracking)
// 							{
// 								lqr_state_feedback_tracking_control_init(LQR_data);
// 							}
// 							else if (solver == observer_based_feedback_tracking)
// 							{
// 								lqr_state_feedback_tracking_control_init(LQR_data);
// 								observer_based_feedback_tracking_control_init(lqr_global, LQR_data);
// 								fout_lqr << "done lqr observer based tracking init..." << endl;
// 							}
// 							LQR_data.total_cpu_time += get_current_cpu_time_in_seconds() - lqr_cur_t;
// 
// 							fout_lqr << "done lqr tracking init..." << endl;
// 							b_lqr_init = true;
// 							fout_lqr << "lqr_init_spd" << endl;
// 							for (auto i : lqr_initial_state.init_spd)
// 								fout_lqr << i << ",";
// 							fout_lqr << endl;
// 							fout_lqr << "lqr_init_loc" << endl;
// 							for (auto i : lqr_initial_state.init_loc)
// 								fout_lqr << i << ",";
// 							fout_lqr << endl;
// 							fout_lqr << "lqr_init_acc" << endl;
// 							for (auto i : lqr_initial_state.init_acc)
// 								fout_lqr << i << ",";
// 							fout_lqr << endl;
// 
// 							// 				char* filename = "C:\\Users\\lab-admin\\Documents\\2018_spring\\PTV_lqr_statefeedback_tracking_gain.csv";
// 							// 				Write_Matrices_To_File(filename, lqr_global.Ks);
// 							// 				fout_lqr << "done write gain matrix..." << endl;
// 						}
// 						else if (b_lqr_init == true) // done initialization
// 						{
// 							fout_lqr << "----current_veh_no: " << current_veh_no << endl;
// 							if (leading_CAV_ID == current_veh_no)
// 							{
// 								//write front uncontrolled vehicle record
// 								fout_lqr << "writing fout_ncacc" << endl;
// // 								fout_ncacc << AdjVehicles[2][3] << ","  //current_veh_no << ","
// // 									<< timestep << ","
// // 									<< veh_odometer + AdjVehiclesDist[2][3] << ","
// // 									<< AdjVehiclesAcc[2][3] << ","		// Acc_vehicle << ","
// // 									<< AdjVehiclesSpeed[2][3] << ","		// Spd_vehicle << ","
// // 									<< -1 << ","
// // 									<< -1 << ","
// // 									<< -1 << ","
// // 									<< -1 << "," << ","					// AdjVehicles[2][3] << ","
// // 									<< -1 << ","						//AdjVehiclesDist[2][3]
// // 									<< -1 << ","						//AdjVehiclesSpeed[2][3] << ","
// // 									<< -1 << ","						//AdjVehiclesAcc[2][3] << ","
// // 									<< -1 << endl;						//AdjVehiclesWidth[2][3] << endl;
// 
// 								//update state and control
// 								fout_lqr << "update state and control..." << endl;
// 								LQR_data.v_upper = AdjVehiclesSpeed[2][3]; //14.5
// 								LQR_data.current_t++;
// 
// 
// 								double lqr_cur_t = get_current_cpu_time_in_seconds();
// 								if (solver == LQR_state_feedback)
// 								{
// 									lqr_state_feedback_control(lqr_global, LQR_data);
// 								}
// 								else
// 								{
// 									//ref
// 									double r1[24];
// 									r1[0] = veh_odometer + AdjVehiclesDist[2][3] - veh_len - LQR_data.head*AdjVehiclesSpeed[2][3];
// 									fout_lqr << "front_loc " << veh_odometer + AdjVehiclesDist[2][3] << "r_loc " << r1[0] << endl;
// 									r1[1] = AdjVehiclesSpeed[2][3];
// 									for (int r_ix = 1; r_ix < LQR_data.control_size; r_ix++){
// 										double loc = r1[r_ix * 2 - 2] - LQR_data.head*AdjVehiclesSpeed[2][3] - veh_len;
// 										fout_lqr << "front_loc " << r1[r_ix * 2 - 2] << "r_loc " << loc << endl;
// 										r1[r_ix * 2] = loc < 0 ? 0 : loc;
// 										r1[r_ix * 2 + 1] = AdjVehiclesSpeed[2][3];
// 									}
// 									if (solver == LQR_state_feedback_reference_tracking){
// 										//control
// 										fout_lqr << "update LQR_state_feedback_reference_tracking..." << endl;
// 										//update state from VISSIM
// 
// 										lqr_state_update_from_VISSIM(lqr_initial_state, unique_vehicle_no, LQR_data);
// 										//
// 										lqr_state_feedback_tracking_control(lqr_global, LQR_data, r1);
// 									}
// 									else if (solver == observer_based_feedback_tracking)
// 									{
// 										fout_lqr << "update observer_based_feedback_tracking..." << endl;
// 										lqr_state_update_from_VISSIM(lqr_initial_state, unique_vehicle_no, LQR_data);
// 										observer_based_feedback_tracking_control(lqr_global, LQR_data, r1);
// 										fout_lqr << "done update observer_based_feedback_tracking..." << endl;
// 									}
// 								}
// 								LQR_data.total_cpu_time += get_current_cpu_time_in_seconds() - lqr_cur_t;
// 								fout_lqr << "total_cpu_time at timestep: " << timestep << " : " << LQR_data.total_cpu_time << endl;
// 
// 								desired_acceleration = LQR_data.vec_u[veh_no_to_index[leading_CAV_ID]];
// 								// 					if (Spd_vehicle > desired_velocity)
// 								// 						desired_acceleration = (desired_velocity - Spd_vehicle) / 0.1;
// 								//safety condition
// 								double safety_acc = (2 * AdjVehiclesDist[2][3] - veh_len - LQR_data.reac  * Spd_vehicle - LQR_data.head*Spd_vehicle - LQR_data.dt*Spd_vehicle) / (LQR_data.dt*LQR_data.dt);
// 								if (safety_acc < desired_acceleration)
// 								{
// 									fout_lqr << "current_veh_no " << current_veh_no << ", t: " << timestep << ",safety_acc " << safety_acc << ",desired_acceleration " << desired_acceleration << ",front spacing " << AdjVehiclesDist[2][3] << endl;
// 									//desired_acceleration = safety_acc < -4.0 ? -4.0 : safety_acc;
// 								}
// 								fout_lqr << "leading_CAV_ID," << ",AdjVehiclesDist," << AdjVehiclesDist[2][3] << ",AdjVehicles," << AdjVehicles[2][3] << ",AdjVehiclesSpeed," << AdjVehiclesSpeed[2][3] << leading_CAV_ID << ",desired_velocity," << desired_velocity << ",Spd_vehicle," << Spd_vehicle << ",desired_acceleration," << desired_acceleration << endl;
// 
// 							}
// 							else
// 							{
// 								int veh_index = veh_no_to_index[current_veh_no];
// 								if (veh_index >= LQR_data.vec_u.size())
// 									fout_lqr << "veh_index >= LQR_data.vec_u.size" << endl;
// 								else{
// 									desired_acceleration = LQR_data.vec_u[veh_index];
// 									//desired_velocity = LQR_data.vec_x[veh_index * 2 + 1];
// 									// 						if (Spd_vehicle > desired_velocity)
// 									// 							desired_acceleration = (desired_velocity - Spd_vehicle) / 0.1;
// 									double safety_acc = (2 * AdjVehiclesDist[2][3] - veh_len - LQR_data.reac * Spd_vehicle - LQR_data.head*Spd_vehicle - LQR_data.dt*Spd_vehicle) / (LQR_data.dt*LQR_data.dt);
// 									fout_lqr << "current_veh_no " << current_veh_no << ", t: " << timestep << "safety_acc: " << safety_acc << endl;
// 									if (safety_acc < desired_acceleration)
// 									{
// 										fout_lqr << "current_veh_no " << current_veh_no << ", t: " << timestep << ",safety_acc " << safety_acc << ",desired_acceleration " << desired_acceleration << ",front spacing " << AdjVehiclesDist[2][3] << endl;
// 										//	desired_acceleration = safety_acc < -4.0 ? -4.0 : safety_acc;
// 									}
// 									fout_lqr << "current_veh_no," << current_veh_no << ",desired_velocity," << desired_velocity << ",Spd_vehicle," << Spd_vehicle << ",desired_acceleration," << desired_acceleration << endl;
// 								}
// 							}
// 						}
// 
// 					}
// 					else if (solver == MPC_QP_regulating || solver == Stochastic_MPC_QP || solver == MPC_tracking)
// 					{
// 
// 						//fout_mpc << "sim step," << timestep << ",leading_CAV_ID," << leading_CAV_ID << endl;
// 						mpc_initial_state.MPC_veh_no_init_acc[current_veh_no] = Acc_vehicle;
// 						mpc_initial_state.MPC_veh_no_init_loc[current_veh_no] = veh_odometer;
// 						mpc_initial_state.MPC_veh_no_init_spd[current_veh_no] = Spd_vehicle;
// 						if (!b_mpc_init && unique_vehicle_no.size() == MaxPlatoonSize &&  timestep > 18.0)
// 						{
// 							//initialization		
// 							fout_mpc << "start init... at t:" << timestep << endl;
// 							double cpt_t = get_current_cpu_time_in_seconds();
// 							mpc_QP_control_data_init(mpc_data);
// 							mpc_data.total_cpu_time += (get_current_cpu_time_in_seconds() - cpt_t);
// 							fout_mpc << "done mpc_qp init..." << endl;
// 							b_mpc_init = true;
// 						}
// 						else if (b_mpc_init == true) // done initialization
// 						{
// 							//measuring vehicle acc, spd
// 							mpc_data.observed_acc_error[current_veh_no].push_back(Acc_vehicle- desired_acceleration);
// 							mpc_data.observed_spd_error[current_veh_no].push_back(Spd_vehicle- desired_velocity);
// 							mpc_data.observed_loc_error[current_veh_no].push_back(veh_odometer - desired_loc);
// 
// 							if (leading_CAV_ID == current_veh_no)
// 							{
// 								//solve mpc
// 								mpc_data.v_upper = 35; // AdjVehiclesSpeed[2][3]; //14.5
// 								if (solver == MPC_tracking)
// 									mpc_data.v_upper = 40;
// 								fout_mpc << "update state and control..., v_upper: " << mpc_data.v_upper << endl;
// 
// 								lead_CAV_state.odometer = veh_odometer;
// 								double nveh_loc = veh_odometer + AdjVehiclesDist[2][3];
// 								fout_mpc << "front veh dist: " << AdjVehiclesDist[2][3] << ", nveh_loc: " << nveh_loc << endl;
// 
// 								mpc_QP_control_data_update(ampl_obj, solver, mpc_initial_state, unique_vehicle_no, mpc_data, nveh_loc, AdjVehiclesSpeed[2][3]);
// 
// 								double cpu_t = get_current_cpu_time_in_seconds();
// 								solve_MPC_QP(ampl_obj, mpc_data);
// 								fout_mpc << "solve mpc CPU time at t: " << timestep << "-->" << get_current_cpu_time_in_seconds() - cpu_t << endl;
// 								fout_mpc << "done solve_MPC_QP, iter: " << mpc_data.solve_ampl_times << endl;
// 								fout_mpc << "total_cpu_time at timestep: " << timestep << " : " << mpc_data.total_cpu_time << endl;
// 
// 								fout_ncacc << AdjVehicles[2][3] << ","  //current_veh_no << ","
// 									<< timestep << ","
// 									<< veh_odometer + AdjVehiclesDist[2][3] << ","
// 									<< AdjVehiclesAcc[2][3] << ","		// Acc_vehicle << ","
// 									<< AdjVehiclesSpeed[2][3] << ","		// Spd_vehicle << ","
// 									<< -1 << ","
// 									<< -1 << ","
// 									<< -1 << ","
// 									<< -1 << "," << ","					// AdjVehicles[2][3] << ","
// 									<< -1 << ","						//AdjVehiclesDist[2][3]
// 									<< -1 << ","						//AdjVehiclesSpeed[2][3] << ","
// 									<< -1 << ","						//AdjVehiclesAcc[2][3] << ","
// 									<< -1 << endl;						//AdjVehiclesWidth[2][3] << endl;
// 
// 								//update state and control
// 
// 								//control
// 								// 					fout_mpc << "mpc_data.vec_u size " << mpc_data.vec_u.size() << endl;
// 								// 					for (int di_ = 0; di_ < mpc_data.vec_u.size(); di_++){
// 								// 						fout_mpc << "i " << di_ << ",vec_u " << mpc_data.vec_u[di_] << ",vec_x " << mpc_data.vec_x[di_] << endl;
// 								// 					}
// 
// 								desired_acceleration = mpc_data.vec_u[veh_no_to_index[leading_CAV_ID]];
// 								fout_mpc << "leading_CAV_ID," << ",AdjVehiclesDist," << AdjVehiclesDist[2][3] << ",AdjVehicles," << AdjVehicles[2][3] << ",AdjVehiclesSpeed," << AdjVehiclesSpeed[2][3] << leading_CAV_ID << ",desired_velocity," << desired_velocity << ",Spd_vehicle," << Spd_vehicle << ",desired_acceleration," << desired_acceleration << endl;
// 
// 							}
// 							else
// 							{
// 								int veh_index = veh_no_to_index[current_veh_no];
// 								if (veh_index >= mpc_data.vec_u.size())
// 									fout_mpc << "veh_index >= LQR_data.vec_u.size" << endl;
// 								else{
// 									desired_acceleration = mpc_data.vec_u[veh_index];
// 								}
// 								//fout_mpc << "current_veh_no," << current_veh_no << ",desired_velocity," << desired_velocity << ",Spd_vehicle," << Spd_vehicle << ",desired_acceleration," << desired_acceleration << endl;
// 							}
// 							desired_velocity = Spd_vehicle + desired_acceleration*0.1;
// 							desired_loc = veh_odometer + Spd_vehicle*0.1+ desired_acceleration*0.01;
// 						}
// 
// 					}
// 					else if (solver == Data_driven_MPC)
// 					{
// 						if (timestep > 50.0) //start to control after we get some observations
// 						{
// 							if (MPC_optimal_accelerations.find(current_veh_no) != MPC_optimal_accelerations.end()){
// 								desired_acceleration = MPC_optimal_accelerations[current_veh_no];
// 								MPC_optimal_accelerations.erase(MPC_optimal_accelerations.find(current_veh_no));
// 							}
// 							else
// 							{
// 								//check if the the vehicle is leading CAV
// 								if (leading_CAV_ID == current_veh_no)
// 								{
// 									double front_veh_speed = AdjVehiclesSpeed[2][3];
// 									double front_spacing = AdjVehiclesDist[2][3];
// 									// 					if (front_spacing <= 0.0)
// 									// 						front_spacing = 200;
// 
// 									vector<double>vehicle_init_speed;
// 									vector<double>vehicle_init_spacing;
// 									vector<double>vehicle_init_acc;
// 									vector<double>vehicle_length;
// 									vector<double> vehicle_odometer;
// 									vector<double> vehicle_x;
// 									vector<double> vehicle_y;
// 									vector<double> front_vehicle_spd;
// 									vector<double> front_vehicle_acc;
// 									//fout_AMPL_debug << "unique_vehicle_no: ";
// 									for (auto v_no : unique_vehicle_no){
// 										//fout_AMPL_debug << v_no << "->";
// 										vehicle_init_speed.push_back(CAV_init_speed[v_no]);
// 										vehicle_init_spacing.push_back(CAV_init_spacing[v_no]);
// 										vehicle_init_acc.push_back(CAV_init_acc[v_no]); //vehicle_init_acc.push_back(CAV_init_acc[current_veh_no]);
// 										vehicle_length.push_back(veh_len_vec[v_no]);
// 										vehicle_odometer.push_back(veh_odometer_vec[v_no]);
// 										vehicle_x.push_back(veh_coord_x_vec[v_no]);
// 										vehicle_y.push_back(veh_coord_y_vec[v_no]);
// 										front_vehicle_spd.push_back(front_veh_speed_vec[v_no]);
// 										front_vehicle_acc.push_back(front_veh_acc_vec[v_no]);
// 									}
// 									// 					if (MPC_optimal_accelerations.find(current_veh_no) != MPC_optimal_accelerations.end())
// 									// 					{
// 									// 					}
// 									fout_AMPL_debug << "timestep: " << timestep << endl;
// 
// 									//--------------communicate with CAVs through rabbitmq------------------
// 									bool b_rabbitmq = false;
// 									if (b_rabbitmq){
// 										S2V_Message_Data s2v_message_data;
// 										s2v_message_data.S2V_platoon_size = MaxPlatoonSize;
// 										s2v_message_data.S2V_leading_CAV_ID = leading_CAV_ID;
// 										s2v_message_data.S2V_vissim_timestep = timestep;
// 										s2v_message_data.S2V_lead_CAV_speed = Spd_vehicle;
// 										s2v_message_data.S2V_front_veh_speed = front_veh_speed;
// 										s2v_message_data.S2V_front_spacing = front_spacing;
// 										s2v_message_data.S2V_vehicle_init_speed = vehicle_init_speed;
// 										s2v_message_data.S2V_vehicle_init_spacing = vehicle_init_spacing;
// 										s2v_message_data.S2V_vehicle_init_acc = vehicle_init_acc;
// 										s2v_message_data.S2V_vehicle_length = vehicle_length;
// 										s2v_message_data.S2V_vehicle_odometer = vehicle_odometer;
// 										s2v_message_data.S2V_vehicle_x = vehicle_x;
// 										s2v_message_data.S2V_vehicle_y = vehicle_y;
// 										s2v_message_data.S2V_front_CAV_spd = front_vehicle_spd;
// 										s2v_message_data.S2V_front_CAV_acc = front_vehicle_acc;
// 										s2v_message_data.S2V_unique_vehicle_no_vec = unique_vehicle_no;
// 
// 										//convert to a string message
// 										string S2V_message = S2V_Message_Data_to_string(s2v_message_data);
// 
// 
// 										//received message
// 										string received_V2S_message;
// 
// 										desired_acceleration = stod(received_V2S_message);
// 									}
// 									else
// 									{
// 										//----------------------------------------------------------------------
// 										bool solved_f = ControlLeadingCAV(ampl_obj, Spd_vehicle, front_veh_speed, front_spacing, vehicle_init_speed, vehicle_init_spacing, vehicle_init_acc, vehicle_length);
// 										if (solved_f) //check if solved succesfully using AMPL
// 										{
// 											desired_acceleration = MPC_optimal_accelerations[current_veh_no];
// 											MPC_optimal_accelerations.erase(MPC_optimal_accelerations.find(current_veh_no));
// 										}
// 										else
// 											ControlVehicle();
// 									}
// 								}
// 								else
// 								{//if its following CAV
// 									ControlVehicle();
// 								}
// 							}
// 						}
// 					}
// 					else
// 					{
// 						ControlVehicle();
// 					}
// 				}
// 
// 				if (veh_type == 100 && current_veh_no > 1 && AdjVehicles[2][3] != -1){
// 					avoid_collision_acc();
// 					//fout_ncacc << 1.9 << "," << 10.0 << endl;
// 					//ControlVehicle();
// 					//fout_mpc << "avoid_collision_acc control: " << current_veh_no << endl;
// 				}
// 			}
		}
		return 1;
	default:
		return 0;
	}
	fout_ncacc.close();
	
	if (solver == Car_Following_Control && NGSIM_data_vector.size() > 0){
		//write_vehicle_records_to_NGSIM(NGSIM_data_vector, 1);
		fout_car_following.close();
	}
	else{
		fout_AMPL_debug.close();
		if (solver < MPC_QP_regulating){
			fout_lqr.close();
		}
		if (solver >= MPC_QP_regulating){
			fout_velocity_bds.close();
			fout_mpc.close();
		}
	}

}

void lqr_state_update_from_VISSIM(LQR_Initial_State&lqr_initial_state, vector<int> &unique_vehicle_no, LQR_Data & LQR_data){
	int u_ix = 0, x_ix = 0;
	for (auto veh_no : unique_vehicle_no){
		LQR_data.du0[u_ix++] = lqr_initial_state.lqr_veh_no_init_acc[veh_no];	
		LQR_data.dx0[x_ix++] = lqr_initial_state.lqr_veh_no_init_loc[veh_no];
		LQR_data.dx0[x_ix++] = lqr_initial_state.lqr_veh_no_init_spd[veh_no];
		fout_lqr << "VISSIM update states: acc: " << lqr_initial_state.lqr_veh_no_init_acc[veh_no] << ",loc:" << lqr_initial_state.lqr_veh_no_init_loc[veh_no] << ",spd:" << lqr_initial_state.lqr_veh_no_init_spd[veh_no] << endl;

	}
};

void mpc_QP_control_data_init( MPC_Data &mpc_data){
	mpc_data.total_cpu_time = 0.0;
	mpc_data.control_size = MaxPlatoonSize;
	mpc_data.Ph = 50;
	mpc_data.dt = 0.1;
	mpc_data.head = 1.2;
	mpc_data.u_lower = -4.87;// -3.0*(2.2 - (1720.929 / 15000.0));
	mpc_data.u_upper = 4.5;
	mpc_data.v_lower = 0.0;
	mpc_data.v_upper = 29.0;

	for (int i = 0; i < mpc_data.control_size; i++)
		mpc_data.v_length.push_back(4.572);
	//
// 	for (auto veh_no : unique_vehicle_no)
// 	{
// 		double init_acc = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
// 		double init_spd = mpc_initial_state.MPC_veh_no_init_spd[veh_no];
// 		double init_loc = mpc_initial_state.MPC_veh_no_init_loc[veh_no];
// 		mpc_initial_state.init_acc.push_back(init_acc);
// 		mpc_initial_state.init_spd.push_back(init_spd);
// 		mpc_initial_state.init_loc.push_back(init_loc);
// 	}
}

void mpc_QP_control_data_update(ampl::AMPL &ampl, SolverID solver, MPC_Initial_State & mpc_initial_state, vector<int> &unique_vehicle_no, MPC_Data &mpc_data, double nveh_loc, double nveh_spd)
{
	mpc_initial_state.init_acc.clear();
	mpc_initial_state.init_spd.clear();
	mpc_initial_state.init_loc.clear();
	//ampl.reset();
	//fout_mpc << "ampl reset" << endl;
	double ref_v[250], ref_l[250];  //rolling horizon 50
	double l_rows[5], l_cols[50];
	int vi = 0, li = 0;
	fout_mpc << "init size: " << mpc_initial_state.init_acc.size() << endl;
	//mpc_data.head = 0.6;
	fout_mpc << "mpc_data.head " << mpc_data.head << endl;
	double prev_loc = nveh_loc, min_loc_dif = DBL_MAX;
	int veh_idx = 0;
	for (auto veh_no : unique_vehicle_no)
	{
		double init_acc = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
		double init_acc_real = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
		double init_spd = mpc_initial_state.MPC_veh_no_init_spd[veh_no];
		if (init_spd + init_acc*0.1 < 0)
			init_acc = max(mpc_data.u_lower, -init_spd / 0.1 + 0.1);
		double init_loc = mpc_initial_state.MPC_veh_no_init_loc[veh_no];
		double front_loc = mpc_initial_state.MPC_veh_no_init_front_loc[veh_no];
		double front_spd = mpc_initial_state.MPC_veh_no_init_front_spd[veh_no];
		double gap = 0.0;
		if (veh_no == leading_CAV_ID)
			gap = front_loc - init_loc;
		else
			gap = mpc_initial_state.MPC_veh_no_init_loc[veh_no - 1] - init_loc;
		if (init_acc > (gap + 0.1*front_spd - init_spd*0.1 - mpc_data.head*init_spd - mpc_data.v_length[veh_idx]) / (0.1*mpc_data.head))
			init_acc = max(mpc_data.u_lower, (gap + 0.1*front_spd - init_spd*0.1 - mpc_data.head*init_spd - mpc_data.v_length[veh_idx]) / (0.1*mpc_data.head) - 0.1);
		double cal_hwdy = (prev_loc - init_loc - mpc_data.v_length[veh_idx++]) / init_spd;
		if (cal_hwdy < mpc_data.head){
// 			if (cal_hwdy > 0.1)
// 				mpc_data.head = cal_hwdy;
// 			else
				fout_mpc << "cal_hwdy is too small: " << cal_hwdy << endl;
		}
		prev_loc = init_loc;
		fout_mpc << "t:" << timestep << ",vno:" << veh_no << ",acc," << init_acc << ",init_spd," << init_spd << ",init_loc," << init_loc << ",front_loc," << front_loc << ",front_spd," << front_spd << ",gap:" << gap << endl;
		mpc_initial_state.init_acc.push_back(init_acc_real);
		mpc_initial_state.init_spd.push_back(init_spd);
		mpc_initial_state.init_loc.push_back(init_loc);
		for (int t = 0; t < mpc_data.Ph; t++)
		{
			ref_v[vi++] = init_spd;
			ref_l[li++] = init_loc;
			init_loc += init_spd*mpc_data.dt;
		}

	}

	for (int s = 0; s < mpc_data.control_size; s++)
		l_rows[s] = s + 1;
	for (int i = 0; i < mpc_data.Ph; i++)
		l_cols[i] = i + 1;

	fout_mpc << "ampl set values..." << endl;
	//param - value
	double cpt_t = get_current_cpu_time_in_seconds();
	ampl::Parameter control_size_ = ampl.getParameter("control_size");
	control_size_.set(mpc_data.control_size);
	ampl::Parameter dt_ = ampl.getParameter("dt");
	dt_.set(mpc_data.dt);
	ampl::Parameter T_ = ampl.getParameter("T");  //rolling horizon
	T_.set(mpc_data.Ph);
	ampl::Parameter u_lower_ = ampl.getParameter("acc_min");
	u_lower_.set(mpc_data.u_lower);
	ampl::Parameter u_upper_ = ampl.getParameter("acc_max");
	u_upper_.set(mpc_data.u_upper);
	ampl::Parameter v_upper_ = ampl.getParameter("v_upper");
	v_upper_.set(mpc_data.v_upper);
	ampl::Parameter v_lower_ = ampl.getParameter("v_lower");
	v_lower_.set(mpc_data.v_upper - 8.9); // - 20 mph
	ampl::Parameter head_ = ampl.getParameter("hdwy");
	head_.set(mpc_data.head);
	ampl::Parameter reac_ = ampl.getParameter("reac");
	reac_.set(mpc_data.reac);

	//ref

	fout_mpc << "ampl set arrays..." << endl;
	//param - array
	//nveh_loc
	vector<double> nveh_loc_array, nveh_spd_array;
	//nveh_spd = nveh_spd > 10.0 ? 10.0 : nveh_spd;
	fout_mpc << "nveh_loc: " << nveh_loc << ",nveh_spd," << nveh_spd << endl;
	for (int i = 0; i < mpc_data.Ph; i++){
		nveh_loc_array.push_back(nveh_loc);
		nveh_spd_array.push_back(nveh_spd);
		nveh_loc += mpc_data.dt * nveh_spd;
	}	
	ampl::Parameter nveh_loc_ = ampl.getParameter("nveh_loc");
	nveh_loc_.setValues(nveh_loc_array.data(), nveh_loc_array.size());
	ampl::Parameter nveh_spd_ = ampl.getParameter("nveh_spd");
	nveh_spd_.setValues(nveh_spd_array.data(), nveh_spd_array.size());

	ampl::Parameter length_ = ampl.getParameter("length");
	length_.setValues(mpc_data.v_length.data(), mpc_data.v_length.size());

	ampl::Parameter v0_ = ampl.getParameter("vehicle_speed_init");
	//fout_mpc << "mpc_initial_state.init_spd size " << mpc_initial_state.init_spd.size() << ", i 1 " << mpc_initial_state.init_spd.front() << endl;
	v0_.setValues(mpc_initial_state.init_spd.data(), mpc_initial_state.init_spd.size());

	ampl::Parameter l0_ = ampl.getParameter("vehicle_loc_init");
	l0_.setValues(mpc_initial_state.init_loc.data(), mpc_initial_state.init_loc.size());

	ampl::Parameter a0_ = ampl.getParameter("vehicle_acc_init");
	a0_.setValues(mpc_initial_state.init_acc.data(), mpc_initial_state.init_acc.size());

	if (solver == Stochastic_MPC_QP){
		vector<double> _w_, _uw_;
		std::default_random_engine generator;
		std::normal_distribution<double> distribution(0.0, 1.0);
		std::uniform_real_distribution<double> u_distribution(0.0, 1.0);

		double min_w = 100.0, max_w = -100.0;
		for (int i = 0; i < mpc_data.Ph; i++){
			double val = u_distribution(generator);// distribution(generator);
			val = (2 * val - 1)*0.2;
			_uw_.push_back(val);
		}
		ampl::Parameter lw_ = ampl.getParameter("lw_");
		lw_.setValues(_uw_.data(), _uw_.size());
		_uw_.clear();
		for (int i = 0; i < mpc_data.Ph; i++){
			double val = u_distribution(generator);// distribution(generator);
			val = (2 * val - 1) * 2;
			_uw_.push_back(val);
		}
		ampl::Parameter vw_ = ampl.getParameter("vw_");
		vw_.setValues(_uw_.data(), _uw_.size());
	}

	if (solver == MPC_tracking || solver == Predictive_deter_instananeous_Tracking_MPC)
	{
		fout_mpc << "MPC_tracking set values to ampl..." << endl;
		predict_speeds_in_front_region(speed_predict_model, mpc_data);
		//mpcdata.front_region_data[ix].speed
		//reference calculation...
		vector<double> ref_spd(mpc_data.Ph,0.0);
		vector<double> ref_loc(mpc_data.Ph, 0.0);
		float cur_predict_spd = mpc_data.front_region_data.front().speed;
		float cur_front_dist = 0.0;
		for (int t_ = 0; t_ < mpc_data.Ph; t_++){
			cur_predict_spd = max(nveh_spd, min(cur_predict_spd, mpc_data.v_upper));
			cur_front_dist += t_*cur_predict_spd;
			ref_spd[t_] = cur_predict_spd;
			ref_loc[t_] = cur_front_dist + veh_odometer;
			int key_ = (int)(cur_front_dist / mpc_data.cell_dist);
			//fout_mpc << "front_region_data key: " << key_ << endl;
			if (key_ < 10)
				cur_predict_spd = mpc_data.front_region_data[key_].speed;
			else
			{
				cur_predict_spd = mpc_data.front_region_data.back().speed;
			}
		}

		ampl::Parameter ref_v_ = ampl.getParameter("ref_v");
		ref_v_.setValues(ref_spd.data(), ref_spd.size());
		ampl::Parameter ref_l_ = ampl.getParameter("ref_l");
		ref_l_.setValues(ref_loc.data(), ref_loc.size());
	}
	mpc_data.total_cpu_time +=( get_current_cpu_time_in_seconds() - cpt_t);
}
void lqr_state_feedback_control_init(LQR_Data & LQR, LQR_Initial_State&lqr_initial_state){
	vector<double> &init_spd = lqr_initial_state.init_spd;
	vector<double> &init_loc = lqr_initial_state.init_loc;
	vector<double> &init_acc = lqr_initial_state.init_acc;
	LQR.control_size = 5;
	LQR.state_size = 2 * LQR.control_size;
	LQR.T = 18000;
	LQR.dt = 0.1;
	double dt = LQR.dt;
	LQR.head = 1.4;
	double head = LQR.head;
	LQR.reac = 0.66;
	LQR.u_lower = -3.0*(2.2 - (1720.929 / 15000.0));//longitudinal collision, Xiaoyun Lu //-8.0;
	LQR.u_upper = 3.0;
	LQR.v_lower = 0.0;
	LQR.v_upper = 29.0;
	LQR.v_length.clear();
	for (int i = 0; i < LQR.control_size; i++)
		LQR.v_length.push_back(4.572);
	double positionCost_[100]{
		1, 0, -1, -head, 0, 0, 0, 0, 0, 0,
			0, 1, 0, -1, 0, 0, 0, 0, 0, 0,
			-1, 0, 2, head, -1, -head, 0, 0, 0, 0,
			-head, -1, head, 2 + head*head, 0, -1, 0, 0, 0, 0,
			0, 0, -1, 0, 2, head, 0, 0, 0, 0,
			0, 0, -head, -1, head, 2 + head*head, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 2, head, -1, head,
			0, 0, 0, 0, 0, 0, head, 2 + head*head, 0, -1,
			0, 0, 0, 0, 0, 0, -1, 0, 1, head,
			0, 0, 0, 0, 0, 0, head, -1, head, 1 + head*head, };

	double controlCost[25] = {
		1, 0, 0, 0, 0,
		0, 1, 0, 0, 0,
		0, 0, 1, 0, 0,
		0, 0, 0, 1, 0,
		0, 0, 0, 0, 1,
	};
	for (int i = 0; i < LQR.control_size*LQR.control_size; i++){
		LQR.R[i] = controlCost[i];
	}

	for (int i = 0; i < LQR.state_size*LQR.state_size; i++){
		LQR.Q[i] = positionCost_[i];
		LQR.N[i] = 0;
	}
	vector<vector<double>> A_{
		{ 1, dt, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 1, dt, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 1, dt, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 1, dt, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 1, dt },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 } };

	vector<vector<double>> B_{
		{ (dt*dt) / 2, 0, 0, 0, 0 },
		{ dt, 0, 0, 0, 0 },
		{ 0, (dt*dt) / 2, 0, 0, 0 },
		{ 0, dt, 0, 0, 0 },
		{ 0, 0, (dt*dt) / 2, 0, 0 },
		{ 0, 0, dt, 0, 0 },
		{ 0, 0, 0, (dt*dt) / 2, 0 },
		{ 0, 0, 0, dt, 0 },
		{ 0, 0, 0, 0, (dt*dt) / 2 },
		{ 0, 0, 0, 0, dt } };
	int ctr_A = 0, ctr_B = 0;
	for (int i = 0; i < LQR.state_size; i++)
		for (int j = 0; j < LQR.state_size; j++)
			LQR.A[ctr_A++] = A_[j][i];
	
	for (int k = 0; k < LQR.control_size; k++)
		for (int j = 0; j < 10; j++)
			LQR.B[ctr_B++] = B_[j][k];

// 	LQR.Fs.clear();
// 	LQR.Gs.clear();
// 	for (int i = 0; i < LQR.T; i++){
// 		LQR.Fs.push_back(LQR.A);
// 		LQR.Gs.push_back(LQR.B);
// 	}

	for (int i = 0; i < LQR.state_size; i++){
		if (i % 2 == 0)		//loc
			LQR.dx0[i] = init_loc[i/2];
		else{				//spd
			LQR.dx0[i] = init_spd[i/2];
			LQR.du0[i / 2] = init_acc[i / 2];
		}			
	}



};

// 
// void lqr_state_feedback_tracking_control_init(LQR_Data & LQR_data){
// 	Matrix<double> _C_(LQR_data.state_size, 1.0);
// 	memcpy(LQR_data._C_array, _C_.GetData(), sizeof(double) *_C_.Getm() * _C_.Getn());
// 	Matrix<double> _D_(LQR_data.state_size, LQR_data.control_size, 0.0);
// 	Matrix<double> _CD_(LQR_data.state_size, 1.0);
// 	_CD_.AppendColumns(_D_);
// 	Matrix<double> _eye_(LQR_data.state_size, 1.0);
// 	Matrix<double> _Am_(LQR_data.state_size, LQR_data.state_size, LQR_data.A);
// 	Matrix<double> _W_ = _Am_ - _eye_;
// 	Matrix<double> _Bm_(LQR_data.state_size, LQR_data.control_size, LQR_data.B);
// 	_W_.AppendColumns(_Bm_);
// 	_W_.AppendRows(_CD_);
// 	Matrix<double> _G_rhs_(LQR_data.state_size, LQR_data.state_size, 0.0);
// 	_G_rhs_.AppendRows(_eye_);
// 
// 	//fout_lqr << "1_G_,m," << LQR_data._G_.Getm() << ",n," << LQR_data._G_.Getn() << endl;
// 
// 	Matrix<double> _G_ = PseudoInverse(_W_, 1e-12, NULL) *_G_rhs_;
// 	memcpy(LQR_data._G_array, _G_.GetData(), sizeof(double) *_G_.Getm() * _G_.Getn());
// 	LQR_data.G_m = _G_.Getm();
// 	LQR_data.G_n = _G_.Getn();
// 	fout_lqr << "2_G_,m," << _G_.Getm() << ",n," << _G_.Getn() << endl;
// 	//vector<vector<double>> reference;
// 
// // 	ifstream fin_("X:\\Project\\ARPA-e\\PTV\\CACC-VISSIM-v2.0\\CACC-VISSIM\\AnnArbor_freeway_single_lane\\LQR_Reference.csv", std::ios_base::in);
// // 	while (fin_) {
// // 		string s;
// // 		if (!getline(fin_, s)) break;
// // 		istringstream ss(s);
// // 		vector <string> record;
// // 		vector<double> ref_t;
// // 		int idx = 0;
// // 		while (ss)
// // 		{
// // 			string s;
// // 			if (!getline(ss, s, ',')) break;
// // 			//record.push_back(s);
// // 			idx++;
// // 			if (idx > 1) ref_t.push_back(stod(s));
// // 		}
// // 		double veh_spd = ref_t.back();
// // 		for (int i = 1; i < LQR_data.control_size; i++){			
// // 			double veh_loc = ref_t[ref_t.size()-2];
// // 			ref_t.push_back(veh_loc - LQR_data.head*veh_spd + veh_len);
// // 			ref_t.push_back(veh_spd);
// // 		}
// // 
// // 		LQR_data.reference.push_back(ref_t);
// // 	}
// // 
// // 	fin_.close();
// };

// void observer_based_feedback_tracking_control_init(LQR &lqr, LQR_Data & LQR_data){
// 
// 	int v_ctr = 0, w_ctr = 0;
// 	for (int i = 0; i < 10; i++) {
// 		for (int j = 0; j < 10; j++){
// 			if (i != j){
// 				LQR_data.V_cost[v_ctr++] = 0;
// 				LQR_data.W_cost[w_ctr++] = 0;
// 			}
// 			else{
// 				LQR_data.V_cost[v_ctr++] = 1e-7;
// 				if (j%2 == 0)
// 					LQR_data.W_cost[w_ctr++] = 0.01*1e-4;
// 				else
// 					LQR_data.W_cost[w_ctr++] = 0.02*1e-4;
// 			}
// 		}
// 	}
// // 		1e-7, 0, 0, 0,
// // 		0, 1e-7, 0, 0,
// // 		0, 0, 1e-7, 0,
// // 		0, 0, 0, 1e-7, };
// 
// 	fout_lqr << "build kalman filter..., T: " << LQR_data.T << ",v_ctr " << v_ctr << ",w_ctr " << w_ctr << endl;
// 
// 
// 	lqr.Build_Kalman_Filter_Gain(fout_lqr, LQR_data.state_size, LQR_data.control_size, LQR_data.dt,
// 		LQR_data.W_cost, LQR_data.V_cost, LQR_data.N,
// 		LQR_data.A, LQR_data._C_array, LQR_data.T, 1);
// 	fout_lqr << "done build kalman filter..." << endl;
// 	vector<double> vec_x;
// 
// 	for (int i = 0; i < 100; i++){
// 		LQR_data.W_cost[i] = sqrt(LQR_data.W_cost[i]);
// 	}
// 	for (int i = 0; i < 100; i++){
// 		LQR_data.V_cost[i] = sqrt(LQR_data.V_cost[i]);
// 	}
// 
// // 	Matrix<double> _V_sq_(LQR_data.state_size, LQR_data.state_size, V_cost);
// // 	Matrix<double> _x_(LQR_data.state_size, 1, LQR_data.dx0);
// 
// 	for (int i = 0; i < LQR_data.state_size; i++)
// 		LQR_data.dx_hat_plus1[i] = LQR_data.dx0[i];
// 
// 
// };
// 
// void lqr_state_feedback_control(LQR &lqr, LQR_Data & LQR_data){
// 
// 	lqr.ComputeControl(LQR_data.current_t, LQR_data.dx0, LQR_data.du0);
// 	//fout_lqr << "done compute control..." << endl;
// 	//control
// 	LQR_data.vec_u.clear();
// 	for (int nu = 0; nu < LQR_data.control_size; nu++){
// 		LQR_data.du0[nu] = LQR_data.du0[nu] < LQR_data.u_upper ? LQR_data.du0[nu] : LQR_data.u_upper;// std::max(-4.87, std::min(4, du_[nu]));
// 		LQR_data.du0[nu] = LQR_data.du0[nu] < LQR_data.u_lower ? LQR_data.u_lower : LQR_data.du0[nu];
// 		LQR_data.vec_u.push_back(LQR_data.du0[nu]);
// 	}
// 	LQR_data.du_array.push_back(LQR_data.vec_u);
// 	//fout_lqr << "done update control..." << endl;
// 	//state
// 	double dx_plus1[16];
// 	lqr.UpdateState(LQR_data.current_t, dx_plus1, LQR_data.A, LQR_data.B, LQR_data.dx0, LQR_data.du0);
// 	LQR_data.vec_x.clear();
// 	for (int nx = 0; nx < LQR_data.state_size; nx++)
// 	{
// 		//dx_array[i][nx] = dx_[nx];
// 		if (nx % 2 != 0){
// 			dx_plus1[nx] = dx_plus1[nx] < LQR_data.v_upper ? dx_plus1[nx] : LQR_data.v_upper;// std::max(-4.87, std::min(4, du_[nu]));
// 			dx_plus1[nx] = dx_plus1[nx] < LQR_data.v_lower ? LQR_data.v_lower : dx_plus1[nx];
// 		}
// 		LQR_data.vec_x.push_back(LQR_data.dx0[nx]);
// 		LQR_data.dx0[nx] = dx_plus1[nx];
// 	}
// 	LQR_data.dx_array.push_back(LQR_data.vec_x);
// 
// 	//fout_lqr << "done update state..." << endl;
// };
//

double solve_DRO_MPC_Matlab(vector<vector<VehicleState>> &t_vno_predicted_vehicle_trajectory, 
	SolverID solver, MPC_Data &mpc_data, MPC_Initial_State & mpc_initial_state,
	vector<int> &unique_vehicle_no, double nveh_loc)
{

	double cpu_t_start = 0,  cpu_t_end = 0;
	mpc_data.Ph = 3;

	mpc_data.vec_u.clear();
	mpc_data.vec_x.clear();
	//try
	{
		//double a[2] = { 20.0, 40.0 };
		//mwArray input1(1, 2, mxDOUBLE_CLASS);   //1, num_rows, 2, num_cols, 
		//input1.SetData(a, 2);
		//double b[2] = { 30.0, 50.0 };
		//mwArray input2(1, 2, mxDOUBLE_CLASS);
		//input2.SetData(b, 2);
		//mwArray output;
		//test(1, output, input1, input2);
		//double * i = new double;
		//output.GetData(i, 2);
		//cout << *i << endl;
		//cout << *(i + 1) << endl;
		vector<double> x_init;
		vector<vector<double>> observation;
		observation.resize(mpc_data.control_size * 2);

		mpc_initial_state.init_acc.clear();
		mpc_initial_state.init_spd.clear();
		mpc_initial_state.init_loc.clear();
		//double ref_v[250], ref_l[250];  //rolling horizon 50
		vector<vector<double>> ref_v_vec, ref_l_vec;
		ref_v_vec.resize(unique_vehicle_no.size());
		ref_l_vec.resize(unique_vehicle_no.size());

		double l_rows[5], l_cols[50];
		int vi = 0, li = 0;
		double prev_loc = nveh_loc, min_loc_dif = DBL_MAX;
		int veh_idx = 0;
		int obs_idx = 0;
		for (int i = 0; i < unique_vehicle_no.size(); i++)//(auto veh_no : unique_vehicle_no)
		{
			int veh_no = unique_vehicle_no[i];
			//fout_mpc << "veh_no," << veh_no << endl;
			double init_acc = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
			double init_acc_real = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
			double init_spd = mpc_initial_state.MPC_veh_no_init_spd[veh_no];
			if (init_spd + init_acc*0.1 < 0)
				init_acc = max(mpc_data.u_lower, -init_spd / 0.1 + 0.1);
			double init_loc = mpc_initial_state.MPC_veh_no_init_loc[veh_no];
			double front_loc = mpc_initial_state.MPC_veh_no_init_front_loc[veh_no];
			double front_spd = mpc_initial_state.MPC_veh_no_init_front_spd[veh_no];
			double gap = 0.0;
			if (veh_no == leading_CAV_ID)
				gap = front_loc - init_loc;
			else
				gap = mpc_initial_state.MPC_veh_no_init_loc[veh_no - 1] - init_loc;
			if (init_acc > (gap + 0.1*front_spd - init_spd*0.1 - mpc_data.head*init_spd - mpc_data.v_length[veh_idx]) / (0.1*mpc_data.head))
				init_acc = max(mpc_data.u_lower, (gap + 0.1*front_spd - init_spd*0.1 - mpc_data.head*init_spd - mpc_data.v_length[veh_idx]) / (0.1*mpc_data.head) - 0.1);
			double cal_hwdy = (prev_loc - init_loc - mpc_data.v_length[veh_idx++]) / init_spd;
			if (cal_hwdy < mpc_data.head){
				//if (cal_hwdy > 0.1)
				//	mpc_data.head = cal_hwdy;
				//else
				//	fout_mpc << "cal_hwdy is too small: " << cal_hwdy << endl;
			}
			prev_loc = init_loc;
			//fout_mpc << "acc," << init_acc << ",init_spd," << init_spd << ",init_loc," << init_loc << endl;
			mpc_initial_state.init_acc.push_back(init_acc_real);
			mpc_initial_state.init_spd.push_back(init_spd);
			mpc_initial_state.init_loc.push_back(init_loc);
			x_init.push_back(init_loc);
			x_init.push_back(init_spd);
			for (int o = 0; o < mpc_data.observed_loc_error[veh_no].size(); o++){
				observation[(veh_idx - 1) * 2].push_back(mpc_data.observed_loc_error[veh_no][o]);
				observation[(veh_idx - 1) * 2 + 1].push_back(mpc_data.observed_spd_error[veh_no][o]);
			}
			//fout_mpc << "obs_idx:" << (veh_idx - 1) * 2<< ",obs size: " << observation[(veh_idx - 1) * 2].size() << endl;
			//fout_mpc << "obs_idx:" << (veh_idx - 1) * 2 + 1 << ",obs size: " << observation[(veh_idx - 1) * 2 + 1].size() << endl;
			for (int t = 0; t < mpc_data.Ph; t++)
			{
				int predict_num_vehs = t_vno_predicted_vehicle_trajectory[t + 1].size();
				//fout_mpc << "t_vno_predicted_vehicle_trajectory,size," << t_vno_predicted_vehicle_trajectory.size() << endl;
				//fout_mpc << "t_vno_predicted_vehicle_trajectory," << t+1<<".size" << t_vno_predicted_vehicle_trajectory[t + 1].size() << endl;
				//fout_mpc << "predict_num_vehs - MaxPlatoonSize  + i:" << predict_num_vehs - MaxPlatoonSize  + i << endl;
				ref_v_vec[veh_idx - 1].push_back(t_vno_predicted_vehicle_trajectory[t + 1][predict_num_vehs - MaxPlatoonSize + i].spd);
				ref_l_vec[veh_idx - 1].push_back(t_vno_predicted_vehicle_trajectory[t + 1][predict_num_vehs - MaxPlatoonSize + i].odometer);
// 				ref_v[vi++] = t_vno_predicted_vehicle_trajectory[t + 1][predict_num_vehs - MaxPlatoonSize + i].spd;			// the last front vehicle in front region // init_spd;
// 				ref_l[li++] = t_vno_predicted_vehicle_trajectory[t + 1][predict_num_vehs - MaxPlatoonSize + i].odometer;	// init_loc;
				//init_loc += init_spd*mpc_data.dt;

			}
		}
		fout_mpc << "Start DRO MPC Matlab Data Init" << endl;

		vector<double> nveh_loc_array, nveh_spd_array;
		//nveh_spd = nveh_spd > 10.0 ? 10.0 : nveh_spd;
		//fout_mpc << "nveh_loc: " << nveh_loc << endl;
		for (int i = 0; i < mpc_data.Ph; i++){

			int predict_num_vehs = t_vno_predicted_vehicle_trajectory[i + 1].size();
			cout << "predict_num_vehs," << predict_num_vehs << endl;
			nveh_loc_array.push_back(t_vno_predicted_vehicle_trajectory[i + 1][predict_num_vehs - MaxPlatoonSize - 1].odometer);
			nveh_spd_array.push_back(t_vno_predicted_vehicle_trajectory[i + 1][predict_num_vehs - MaxPlatoonSize - 1].spd);			//;(nveh_spd);
			//nveh_loc += mpc_data.dt * nveh_spd;
		}
		//=====================Data_driven_DRO_MPC(x_init, u_init, nveh_loc, dt, head, vlength, Obs)
		//-------------------INIT-------------------------
		mxArray *x_init_ = mxCreateDoubleMatrix(1, mpc_data.control_size * 2, mxREAL);
		memcpy(mxGetPr(x_init_), x_init.data(), (mpc_data.control_size * 2)*sizeof(double));

// 		mwArray x_init_(1, mpc_data.control_size * 2, mxDOUBLE_CLASS);
// 		x_init_.SetData(x_init.data(), mpc_data.control_size * 2);
		fout_mpc << "x_init_..." << endl;

		mxArray *u_init_ = mxCreateDoubleMatrix(1, mpc_data.control_size, mxREAL);
		memcpy(mxGetPr(u_init_), mpc_initial_state.init_acc.data(), (mpc_data.control_size)*sizeof(double));

// 		mwArray u_init_(1, mpc_data.control_size, mxDOUBLE_CLASS);
// 		u_init_.SetData(mpc_initial_state.init_acc.data(), mpc_data.control_size);
 		fout_mpc << "u_init_..." << endl;

		//double nveh_loc_ary[1] = { nveh_loc };
		mxArray *nveh_loc_ = mxCreateDoubleMatrix(1, mpc_data.Ph, mxREAL);
		memcpy(mxGetPr(nveh_loc_), nveh_loc_array.data(), sizeof(double));

// 		mwArray nveh_loc_(1, 1, mxDOUBLE_CLASS);	
// 		nveh_loc_.SetData(nveh_loc_ary, 1);
		fout_mpc << "nveh_loc_..." << endl;

		double dt_ary[1] = { mpc_data.dt };
		mxArray *dt_ = mxCreateDoubleMatrix(1, 1, mxREAL);
		memcpy(mxGetPr(dt_), dt_ary, sizeof(double));

// 		mwArray dt_(1, 1, mxDOUBLE_CLASS);
// 		dt_.SetData(dt_ary, 1);
 		fout_mpc << "dt_..." << endl;

		double head_ary[1] = { mpc_data.head };
		mxArray *head_ = mxCreateDoubleMatrix(1, 1, mxREAL);
		memcpy(mxGetPr(head_), head_ary, sizeof(double));

// 		mwArray head_(1, 1, mxDOUBLE_CLASS);	
// 		head_.SetData(dt_ary, 1);
		fout_mpc << "head_..." << endl;

		double vlength_ary[1] = { 4.2 };
		mxArray *vlength_ = mxCreateDoubleMatrix(1, 1, mxREAL);
		memcpy(mxGetPr(vlength_), vlength_ary, sizeof(double));

// 		mwArray vlength_(1, 1, mxDOUBLE_CLASS);	
// 		vlength_.SetData(vlength_ary, 1);
 		fout_mpc << "vlength_..." << endl;

// 		//observation
		vector<double> obs_ary;
		for (int j = 0; j < 24; j++){		
			for (int i = 0; i < observation.size(); i++){
				//fout_mpc << "i," << i << ",j," << j << ",obs." << observation[i].size() << endl;
				obs_ary.push_back(observation[i][j]);
			}
		}
		mxArray *obs_ary_ = mxCreateDoubleMatrix(mpc_data.control_size * 2, 24, mxREAL);
		memcpy(mxGetPr(obs_ary_), obs_ary.data(), (mpc_data.control_size * 2 * 24)*sizeof(double));

		//reference spd and loc
		//ref_v_vec
		vector<double> ref_v_ary, ref_l_ary;
		for (int t = 0; t < mpc_data.Ph; t++)
		{		
			for (int i = 0; i < unique_vehicle_no.size(); i++){
				ref_v_ary.push_back(ref_v_vec[i][t]);
				ref_l_ary.push_back(ref_l_vec[i][t]);
			}
		}
		mxArray *ref_l_ary_ = mxCreateDoubleMatrix(mpc_data.control_size, mpc_data.Ph, mxREAL);
		memcpy(mxGetPr(ref_l_ary_), ref_l_ary.data(), (mpc_data.control_size *mpc_data.Ph)*sizeof(double));

		mxArray *ref_v_ary_ = mxCreateDoubleMatrix(mpc_data.control_size, mpc_data.Ph, mxREAL);
		memcpy(mxGetPr(ref_v_ary_), ref_v_ary.data(), (mpc_data.control_size *mpc_data.Ph)*sizeof(double));
// 		mwArray obs_ary_(mpc_data.control_size * 2, 25, mxDOUBLE_CLASS);
// 		obs_ary_.SetData(obs_ary.data(), mpc_data.control_size * 2 * 25);
		

		engPutVariable(ep, "x_init", x_init_);
		engPutVariable(ep, "u_init", u_init_);
		engPutVariable(ep, "nveh_loc", nveh_loc_);
		engPutVariable(ep, "dt", dt_);
		engPutVariable(ep, "head", head_);
		engPutVariable(ep, "vlength", vlength_);
		engPutVariable(ep, "Obs", obs_ary_);
		engPutVariable(ep, "ref_l_ary", ref_l_ary_);
		engPutVariable(ep, "ref_v_ary", ref_v_ary_);
		fout_mpc << "Set matlab variable..." << endl;

// 		mxArray *res_ = mxCreateDoubleMatrix(1, mpc_data.control_size, mxREAL);
// 		memcpy(mxGetPr(res_), x_init.data(), (mpc_data.control_size)*sizeof(double)
		fout_mpc << "Set path DRO MPC Matlab..." << endl;
		engEvalString(ep, "addpath(genpath('X:\\Paper\\CACC\\Code\\Solver\\YALMIP-master\\YALMIP-master'));");

		engEvalString(ep, "userpath('X:\\Paper\\CACC\\Code\\Solver\\DRO_MPC'); ");	

		fout_mpc << "Call DRO MPC Matlab solver..." << endl;
		cpu_t_start = get_current_cpu_time_in_seconds();
		if (solver == Predictive_DRO_Tracking_MPC){
			engEvalString(ep, "acc = Data_driven_DRO_MPC(x_init, u_init, nveh_loc, dt, head, vlength, Obs,ref_l_ary,ref_v_ary); ");
		}
		else if (solver == Predictive_DRO_Regulating_MPC){
			engEvalString(ep, "acc = Data_driven_DRO_MPC_Regulating(x_init, u_init, nveh_loc, dt, head, vlength, Obs,ref_l_ary,ref_v_ary); ");
		}
		
		cpu_t_end = get_current_cpu_time_in_seconds();
		mxArray *res_acc_ = engGetVariable(ep, "acc");
		fout_mpc << "res_acc_ pos..." << res_acc_ << endl;
		for (int i = 0; i < mpc_data.control_size; i++)
		{
			mpc_data.vec_u.push_back(*(mxGetPr(res_acc_) + i));
		}
		engEvalString(ep, "clear; ");

		//destroy
		mxDestroyArray(x_init_);
		mxDestroyArray(u_init_);
		mxDestroyArray(nveh_loc_);
		mxDestroyArray(dt_);
		mxDestroyArray(head_);
		mxDestroyArray(vlength_);
		mxDestroyArray(obs_ary_);
		mxDestroyArray(ref_l_ary_);
		mxDestroyArray(ref_v_ary_);

// 		mwArray acc;
// 		Data_driven_DRO_MPC(1, acc, x_init_, u_init_, nveh_loc_, dt_, head_, vlength_, obs_ary_);
// 		fout_mpc << "Data_driven_DRO_MPC solved..." << endl;
// 		
// 		vector<double>acc_res;
// 		for (int i = 0; i < mpc_data.control_size; i++){
// 			mpc_data.vec_u.push_back(acc(i,1));
// 		}	

	}
// 	catch (mwException ex)
// 	{
// 		fout_mpc << ex.what() << endl;
// 

	return cpu_t_end - cpu_t_start;
};

double solve_DRO_MPC_Matlab(
	map<int, vector<vector<VehicleState>>> &scenarioidx_tidx_vno_predicted_vehicle_trajectory,
	SolverID solver, MPC_Data &mpc_data, MPC_Initial_State & mpc_initial_state,
	vector<int> &unique_vehicle_no, double nveh_loc)
{

	double cpu_t_start = 0, cpu_t_end = 0;
	mpc_data.Ph = 3;

	mpc_data.vec_u.clear();
	mpc_data.vec_x.clear();
	//try
	{

		vector<double> x_init;
		vector<vector<double>> observation;
		observation.resize(mpc_data.control_size * 2);

		vector<vector<vector<double>>> td_observation;  // idx 1: l1,v1,l2,v2,...,
		td_observation.resize(mpc_data.control_size * 2);

		mpc_initial_state.init_acc.clear();
		mpc_initial_state.init_spd.clear();
		mpc_initial_state.init_loc.clear();
		//double ref_v[250], ref_l[250];  //rolling horizon 50
		vector<vector<double>> ref_v_vec, ref_l_vec;
		ref_v_vec.resize(unique_vehicle_no.size());
		ref_l_vec.resize(unique_vehicle_no.size());

		double l_rows[5], l_cols[50];
		int vi = 0, li = 0;
		double prev_loc = nveh_loc, min_loc_dif = DBL_MAX;
		int veh_idx = 0;
		int obs_idx = 0;
		for (int i = 0; i < unique_vehicle_no.size(); i++)//(auto veh_no : unique_vehicle_no)
		{
			int veh_no = unique_vehicle_no[i];
			//fout_mpc << "veh_no," << veh_no << endl;
			double init_acc = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
			double init_acc_real = mpc_initial_state.MPC_veh_no_init_acc[veh_no];
			double init_spd = mpc_initial_state.MPC_veh_no_init_spd[veh_no]; if (init_spd + init_acc*0.1 < 0)
				init_acc = max(mpc_data.u_lower, -init_spd / 0.1 + 0.1);
			double init_loc = mpc_initial_state.MPC_veh_no_init_loc[veh_no];
			double front_loc = mpc_initial_state.MPC_veh_no_init_front_loc[veh_no];
			double front_spd = mpc_initial_state.MPC_veh_no_init_front_spd[veh_no];
			double gap = 0.0;
			if (veh_no == leading_CAV_ID)
				gap = front_loc - init_loc;
			else
				gap = mpc_initial_state.MPC_veh_no_init_loc[veh_no - 1] - init_loc;
			if (init_acc > (gap + 0.1*front_spd - init_spd*0.1 - mpc_data.head*init_spd - mpc_data.v_length[veh_idx]) / (0.1*mpc_data.head))
				init_acc = max(mpc_data.u_lower, (gap + 0.1*front_spd - init_spd*0.1 - mpc_data.head*init_spd - mpc_data.v_length[veh_idx]) / (0.1*mpc_data.head) - 0.1);
			double cal_hwdy = (prev_loc - init_loc - mpc_data.v_length[veh_idx++]) / init_spd;
			if (cal_hwdy < mpc_data.head){
				//if (cal_hwdy > 0.1)
				//	mpc_data.head = cal_hwdy;
				//else
				//	fout_mpc << "cal_hwdy is too small: " << cal_hwdy << endl;
			}
			prev_loc = init_loc;
			//fout_mpc << "acc," << init_acc << ",init_spd," << init_spd << ",init_loc," << init_loc << endl;
			mpc_initial_state.init_acc.push_back(init_acc_real);
			mpc_initial_state.init_spd.push_back(init_spd);
			mpc_initial_state.init_loc.push_back(init_loc);
			x_init.push_back(init_loc);
			x_init.push_back(init_spd);

			// 			for (int o = 0; o < mpc_data.observed_loc_error[veh_no].size(); o++){
			// 				observation[(veh_idx - 1) * 2].push_back(mpc_data.observed_loc_error[veh_no][o]);
			// 				observation[(veh_idx - 1) * 2 + 1].push_back(mpc_data.observed_spd_error[veh_no][o]);
			// 			}
			//PUSH scenario observations 
			//fout_mpc << "PUSH scenario observations  " << endl;
			for (int scenario = 0; scenario < scenarioidx_tidx_vno_predicted_vehicle_trajectory.size(); scenario++){

				int predict_num_vehs = scenarioidx_tidx_vno_predicted_vehicle_trajectory[scenario][1].size();
				//fout_mpc << "scenario: " << scenario << ",predict_num_vehs" << endl;
				//get from t=1 in rolling horizon, because all vehicles are driving in constant spd.
				observation[(veh_idx - 1) * 2].push_back(scenarioidx_tidx_vno_predicted_vehicle_trajectory[scenario][1][predict_num_vehs - MaxPlatoonSize + i].odometer);
				observation[(veh_idx - 1) * 2 + 1].push_back(scenarioidx_tidx_vno_predicted_vehicle_trajectory[scenario][1][predict_num_vehs - MaxPlatoonSize + i].spd);

				td_observation[(veh_idx - 1) * 2].resize(mpc_data.Ph);
				td_observation[(veh_idx - 1) * 2 + 1].resize(mpc_data.Ph);

				for (int t = 0; t < mpc_data.Ph; t++)
				{
					td_observation[(veh_idx - 1) * 2][t].push_back(scenarioidx_tidx_vno_predicted_vehicle_trajectory[scenario][t + 1][predict_num_vehs - MaxPlatoonSize + i].odometer);
					td_observation[(veh_idx - 1) * 2 + 1][t].push_back(scenarioidx_tidx_vno_predicted_vehicle_trajectory[scenario][t + 1][predict_num_vehs - MaxPlatoonSize + i].spd);
				}
			}

			//fout_mpc << "obs_idx:" << (veh_idx - 1) * 2<< ",obs size: " << observation[(veh_idx - 1) * 2].size() << endl;
			//fout_mpc << "obs_idx:" << (veh_idx - 1) * 2 + 1 << ",obs size: " << observation[(veh_idx - 1) * 2 + 1].size() << endl;
			for (int scenario = 0; scenario < scenarioidx_tidx_vno_predicted_vehicle_trajectory.size(); scenario++)
				for (int t = 0; t < mpc_data.Ph; t++)
				{
					int predict_num_vehs = scenarioidx_tidx_vno_predicted_vehicle_trajectory[scenario][t + 1].size();
					//fout_mpc << "t_vno_predicted_vehicle_trajectory,size," << t_vno_predicted_vehicle_trajectory.size() << endl;
					//fout_mpc << "t_vno_predicted_vehicle_trajectory," << t+1<<".size" << t_vno_predicted_vehicle_trajectory[t + 1].size() << endl;
					//fout_mpc << "predict_num_vehs - MaxPlatoonSize  + i:" << predict_num_vehs - MaxPlatoonSize  + i << endl;
					ref_v_vec[veh_idx - 1].push_back(scenarioidx_tidx_vno_predicted_vehicle_trajectory[scenario][t + 1][predict_num_vehs - MaxPlatoonSize + i].spd);
					ref_l_vec[veh_idx - 1].push_back(scenarioidx_tidx_vno_predicted_vehicle_trajectory[scenario][t + 1][predict_num_vehs - MaxPlatoonSize + i].odometer);
					// 				ref_v[vi++] = t_vno_predicted_vehicle_trajectory[t + 1][predict_num_vehs - MaxPlatoonSize + i].spd;			// the last front vehicle in front region // init_spd;
					// 				ref_l[li++] = t_vno_predicted_vehicle_trajectory[t + 1][predict_num_vehs - MaxPlatoonSize + i].odometer;	// init_loc;
					//init_loc += init_spd*mpc_data.dt;

				}
			}
		
		fout_mpc << "Start DRO MPC Matlab Data Init" << endl;

		vector<double> nveh_loc_array, nveh_spd_array;
		//nveh_spd = nveh_spd > 10.0 ? 10.0 : nveh_spd;
		//fout_mpc << "nveh_loc: " << nveh_loc << endl;
		fout_mpc << "scenarioidx_tidx_vno_predicted_vehicle_trajectory size: " << scenarioidx_tidx_vno_predicted_vehicle_trajectory.size() << endl;
		//for (int scenario = 0; scenario < scenarioidx_tidx_vno_predicted_vehicle_trajectory.size(); scenario++)
		{		
			int scenario = scenarioidx_tidx_vno_predicted_vehicle_trajectory.size() - 1;
			for (int i = 0; i < mpc_data.Ph; i++){
				int predict_num_vehs = scenarioidx_tidx_vno_predicted_vehicle_trajectory[scenario][i + 1].size();
				nveh_loc_array.push_back(scenarioidx_tidx_vno_predicted_vehicle_trajectory[scenario][i + 1][predict_num_vehs - MaxPlatoonSize - 1].odometer);
				nveh_spd_array.push_back(scenarioidx_tidx_vno_predicted_vehicle_trajectory[scenario][i + 1][predict_num_vehs - MaxPlatoonSize - 1].spd);			//;(nveh_spd);
				//nveh_loc += mpc_data.dt * nveh_spd;
			}
		}
		//=====================Data_driven_DRO_MPC(x_init, u_init, nveh_loc, dt, head, vlength, Obs)
		//-------------------INIT-------------------------
		mxArray *x_init_ = mxCreateDoubleMatrix(1, mpc_data.control_size * 2, mxREAL);
		memcpy(mxGetPr(x_init_), x_init.data(), (mpc_data.control_size * 2)*sizeof(double));

		// 		mwArray x_init_(1, mpc_data.control_size * 2, mxDOUBLE_CLASS);
		// 		x_init_.SetData(x_init.data(), mpc_data.control_size * 2);
		fout_mpc << "x_init_..." << endl;

		mxArray *u_init_ = mxCreateDoubleMatrix(1, mpc_data.control_size, mxREAL);
		memcpy(mxGetPr(u_init_), mpc_initial_state.init_acc.data(), (mpc_data.control_size)*sizeof(double));

		// 		mwArray u_init_(1, mpc_data.control_size, mxDOUBLE_CLASS);
		// 		u_init_.SetData(mpc_initial_state.init_acc.data(), mpc_data.control_size);
		fout_mpc << "u_init_..." << endl;

		//double nveh_loc_ary[1] = { nveh_loc };
		mxArray *nveh_loc_ = mxCreateDoubleMatrix(1, mpc_data.Ph, mxREAL);
		memcpy(mxGetPr(nveh_loc_), nveh_loc_array.data(), sizeof(double));

		// 		mwArray nveh_loc_(1, 1, mxDOUBLE_CLASS);	
		// 		nveh_loc_.SetData(nveh_loc_ary, 1);
		fout_mpc << "nveh_loc_..." << endl;

		double dt_ary[1] = { mpc_data.dt };
		mxArray *dt_ = mxCreateDoubleMatrix(1, 1, mxREAL);
		memcpy(mxGetPr(dt_), dt_ary, sizeof(double));

		// 		mwArray dt_(1, 1, mxDOUBLE_CLASS);
		// 		dt_.SetData(dt_ary, 1);
		fout_mpc << "dt_..." << endl;

		double head_ary[1] = { mpc_data.head };
		mxArray *head_ = mxCreateDoubleMatrix(1, 1, mxREAL);
		memcpy(mxGetPr(head_), head_ary, sizeof(double));

		// 		mwArray head_(1, 1, mxDOUBLE_CLASS);	
		// 		head_.SetData(dt_ary, 1);
		fout_mpc << "head_..." << endl;

		double vlength_ary[1] = { 4.2 };
		mxArray *vlength_ = mxCreateDoubleMatrix(1, 1, mxREAL);
		memcpy(mxGetPr(vlength_), vlength_ary, sizeof(double));

		// 		mwArray vlength_(1, 1, mxDOUBLE_CLASS);	
		// 		vlength_.SetData(vlength_ary, 1);
		fout_mpc << "vlength_..." << endl;

		// 		//observation
		vector<double> obs_ary;
		for (int j = 0; j < observation.front().size(); j++){
			for (int i = 0; i < observation.size(); i++){
				//fout_mpc << "i," << i << ",j," << j << ",obs." << observation[i].size() << endl;
				obs_ary.push_back(observation[i][j]);
			}
		}
		mxArray *obs_ary_ = mxCreateDoubleMatrix(mpc_data.control_size * 2, 24, mxREAL);
		memcpy(mxGetPr(obs_ary_), obs_ary.data(), (mpc_data.control_size * 2 * 24)*sizeof(double));

		//td observations - td_observation
		vector<double> full_obs_ary;
		for (int d = 0; d < mpc_data.Ph; d++){
			for (int j = 0; j < td_observation.front()[d].size(); j++){
				for (int i = 0; i < td_observation.size(); i++){
					//fout_mpc << "i," << i << ",j," << j << ",obs." << observation[i].size() << endl;
					full_obs_ary.push_back(td_observation[i][d][j]);
				}
			}
		}
		size_t  ndim = 3;
		size_t  dims[3] = { mpc_data.control_size * 2, 24, mpc_data.Ph };

		mxArray *full_obs_ary_ = mxCreateNumericArray(ndim, dims, mxDOUBLE_CLASS, mxREAL);
		memcpy(mxGetPr(full_obs_ary_), full_obs_ary.data(), (mpc_data.control_size * 2 * 24 * mpc_data.Ph) * sizeof(double));

		//reference spd and loc
		//ref_v_vec
		vector<double> ref_v_ary, ref_l_ary;
		for (int t = 0; t < mpc_data.Ph; t++)
		{
			for (int i = 0; i < unique_vehicle_no.size(); i++){
				ref_v_ary.push_back(ref_v_vec[i][t]);
				ref_l_ary.push_back(ref_l_vec[i][t]);
			}
		}
		mxArray *ref_l_ary_ = mxCreateDoubleMatrix(mpc_data.control_size, mpc_data.Ph, mxREAL);
		memcpy(mxGetPr(ref_l_ary_), ref_l_ary.data(), (mpc_data.control_size *mpc_data.Ph)*sizeof(double));

		mxArray *ref_v_ary_ = mxCreateDoubleMatrix(mpc_data.control_size, mpc_data.Ph, mxREAL);
		memcpy(mxGetPr(ref_v_ary_), ref_v_ary.data(), (mpc_data.control_size *mpc_data.Ph)*sizeof(double));
		// 		mwArray obs_ary_(mpc_data.control_size * 2, 25, mxDOUBLE_CLASS);
		// 		obs_ary_.SetData(obs_ary.data(), mpc_data.control_size * 2 * 25);


		engPutVariable(ep, "x_init", x_init_);
		engPutVariable(ep, "u_init", u_init_);
		engPutVariable(ep, "nveh_loc", nveh_loc_);
		engPutVariable(ep, "dt", dt_);
		engPutVariable(ep, "head", head_);
		engPutVariable(ep, "vlength", vlength_);
		engPutVariable(ep, "Obs", full_obs_ary_); //  obs_ary_);
		engPutVariable(ep, "ref_l_ary", ref_l_ary_);
		engPutVariable(ep, "ref_v_ary", ref_v_ary_);
		fout_mpc << "Set matlab variable..." << endl;

		// 		mxArray *res_ = mxCreateDoubleMatrix(1, mpc_data.control_size, mxREAL);
		// 		memcpy(mxGetPr(res_), x_init.data(), (mpc_data.control_size)*sizeof(double)
		fout_mpc << "Set path DRO MPC Matlab..." << endl;
		engEvalString(ep, "addpath(genpath('X:\\Paper\\CACC\\Code\\Solver\\YALMIP-master\\YALMIP-master'));");

		engEvalString(ep, "userpath('X:\\Paper\\CACC\\Code\\Solver\\DRO_MPC'); ");

		fout_mpc << "Call DRO MPC Matlab solver..." << endl;
		cpu_t_start = get_current_cpu_time_in_seconds();
		//if (solver == Predictive_DRO_Tracking_MPC){
			engEvalString(ep, "acc = Data_driven_DRO_MPC_reference_stochastic_full_observations(x_init, u_init, nveh_loc, dt, head, vlength, Obs,ref_l_ary,ref_v_ary); ");
		//}
		//else if (solver == Predictive_DRO_Regulating_MPC){
		//	engEvalString(ep, "acc = Data_driven_DRO_MPC_Regulating(x_init, u_init, nveh_loc, dt, head, vlength, Obs,ref_l_ary,ref_v_ary); ");
		//}

		cpu_t_end = get_current_cpu_time_in_seconds();
		mxArray *res_acc_ = engGetVariable(ep, "acc");
		fout_mpc << "res_acc_ pos..." << res_acc_ << endl;
		for (int i = 0; i < mpc_data.control_size; i++)
		{
			mpc_data.vec_u.push_back(*(mxGetPr(res_acc_) + i));
		}
		engEvalString(ep, "clear;");

		//destroy
		mxDestroyArray(x_init_);
		mxDestroyArray(u_init_);
		mxDestroyArray(nveh_loc_);
		mxDestroyArray(dt_);
		mxDestroyArray(head_);
		mxDestroyArray(vlength_);
		mxDestroyArray(obs_ary_);
		mxDestroyArray(ref_l_ary_);
		mxDestroyArray(ref_v_ary_);

		// 		mwArray acc;
		// 		Data_driven_DRO_MPC(1, acc, x_init_, u_init_, nveh_loc_, dt_, head_, vlength_, obs_ary_);
		// 		fout_mpc << "Data_driven_DRO_MPC solved..." << endl;
		// 		
		// 		vector<double>acc_res;
		// 		for (int i = 0; i < mpc_data.control_size; i++){
		// 			mpc_data.vec_u.push_back(acc(i,1));
		// 		}	

	}
	// 	catch (mwException ex)
	// 	{
	// 		fout_mpc << ex.what() << endl;
	// 

	return cpu_t_end - cpu_t_start;
};

void solve_MPC_QP(ampl::AMPL &ampl, MPC_Data &mpc_data){
	try
	{
		fout_mpc << "solving ampl mpc..." << endl;

		MyOutputHandler outputHandler;
		ampl.setOutputHandler(&outputHandler);
		double cpt_t = get_current_cpu_time_in_seconds();
		ampl.solve();
		fout_mpc << "CPU solving function time: " << get_current_cpu_time_in_seconds() - cpt_t << endl;
		mpc_data.total_cpu_time += (get_current_cpu_time_in_seconds() - cpt_t);

		ampl::DataFrame res_U_ = ampl.getVariable("_u").getValues();
		//fout_mpc << "get res_U_ variables... " <<endl;
		ampl::DataFrame res_L_ = ampl.getVariable("_l").getValues();
		ampl::DataFrame res_V_ = ampl.getVariable("_v").getValues();
		//fout_mpc << res_U_.toString() << endl;
		//fout_mpc << res_L_.toString() << endl;
		//fout_mpc << res_V_.toString() << endl;
		int n_rows = res_U_.getNumRows();  //all values in 1D array, 1,2,..100,1,2,..,100 - {{1,2,..,100},{1,2,..,100}}
		mpc_data.vec_u.clear();
		mpc_data.vec_x.clear();
		mpc_data.du_array.clear();
		mpc_data.dx_array.clear();
		//for (int var_index = 0; var_index < n_rows / 2; var_index++)
		int var_index = 1 ; // get desired acc from second t.
		//vector<double> _du, _dx;
		for (int ui = 0; ui < mpc_data.control_size; ui++)
		{
			mpc_data.vec_u.push_back(res_U_.getRowByIndex(var_index + ui*mpc_data.Ph)[2].dbl());
			fout_mpc << "U," << mpc_data.vec_u.back() << endl;
			mpc_data.vec_x.push_back(res_L_.getRowByIndex(var_index + ui*mpc_data.Ph)[2].dbl());
			fout_mpc << "L," << mpc_data.vec_x.back() << endl;
			mpc_data.vec_x.push_back(res_V_.getRowByIndex(var_index + ui*mpc_data.Ph)[2].dbl());
			fout_mpc << "V," << mpc_data.vec_x.back() << endl;		
		}
		var_index = 0;
		for (int ui = 0; ui < n_rows/2; ui++){
			//sim_t, veh_no, horizon_step, v, v-acc*dt
			//v_up-dt*u_low, mpc_data.v_upper - 8.9
			float v_up = res_V_.getRowByIndex(var_index + ui)[2].dbl();
			float v_follower = res_V_.getRowByIndex(var_index + ui + mpc_data.Ph)[2].dbl();
			float u_follower = res_U_.getRowByIndex(var_index + ui + mpc_data.Ph)[2].dbl();
			float spacing = res_L_.getRowByIndex(var_index + ui)[2].dbl() - res_L_.getRowByIndex(var_index + ui + mpc_data.Ph)[2].dbl();
			float v_low = (spacing - (v_follower + u_follower*mpc_data.dt)*mpc_data.dt) / mpc_data.dt;
			fout_velocity_bds
				<< timestep << ","
				<< res_V_.getRowByIndex(var_index + ui)[0].dbl() << ","
				<< res_V_.getRowByIndex(var_index + ui)[1].dbl() << ","
				<< v_up << ","
				//<< v_up + (mpc_data.dt*mpc_data.u_lower) << ","
				<< v_up + (mpc_data.dt*(mpc_data.u_lower + u_follower< 0 ? u_follower : -u_follower)) << ","
				<< min(v_follower, v_up) - 8.9 << endl;

				//<< min(v_up - (mpc_data.dt*mpc_data.u_lower), min(v_up, mpc_data.v_upper) - 8.9) << endl;
		}
		mpc_data.du_array.push_back(mpc_data.vec_u);
		mpc_data.dx_array.push_back(mpc_data.vec_x);

		mpc_data.solve_ampl_times++;
		
	}
	catch (ampl::AMPLException &e)
	{
		fout_mpc << "Solving Error: " << e.getMessage() << endl;

	}
	catch (...) {
		fout_mpc << "Unknown Solving Error" << endl;
	}
};
// 
// void lqr_state_feedback_tracking_control(LQR &lqr, LQR_Data & LQR_data, double *r1){
// 	Matrix<double> _G_(LQR_data.G_m, LQR_data.G_n, LQR_data._G_array);
// 	Matrix<double> _ref_(LQR_data.state_size, 1, r1);
// 	
// 	Matrix<double> _xue_ = _G_*_ref_;
// 	Matrix<double> _xue_x_(_xue_);
// 	Matrix<double> _xue_u_(_xue_);
// 	_xue_u_.RemoveRows(0, LQR_data.state_size);
// 	_xue_x_.RemoveRows(LQR_data.state_size, LQR_data.state_size + LQR_data.control_size);
// 
// 	//control
// 	lqr.ComputeControl(LQR_data.current_t, LQR_data.dx0, LQR_data.du0, _xue_x_, _xue_u_);
// 	LQR_data.vec_u.clear();
// 	for (int nu = 0; nu < LQR_data.control_size; nu++){
// 		LQR_data.du0[nu] = LQR_data.du0[nu] < LQR_data.u_upper ? LQR_data.du0[nu] : LQR_data.u_upper;// std::max(-4.87, std::min(4, du_[nu]));
// 		LQR_data.du0[nu] = LQR_data.du0[nu] < LQR_data.u_lower ? LQR_data.u_lower : LQR_data.du0[nu];
// 		LQR_data.vec_u.push_back(LQR_data.du0[nu]);
// 	}
// 	LQR_data.du_array.push_back(LQR_data.vec_u);
// 
// 	//state
// 	//double dx_plus1[16];
// 	lqr.UpdateState(LQR_data.current_t, LQR_data.dx0, LQR_data.A, LQR_data.B, LQR_data.dx0, LQR_data.du0);
// 	LQR_data.vec_x.clear();
// 	for (int nx = 0; nx < LQR_data.state_size; nx++)
// 	{
// 		//dx_array[i][nx] = dx_[nx];
// 		if (nx % 2 != 0){
// 			LQR_data.dx0[nx] = LQR_data.dx0[nx] < LQR_data.v_upper ? LQR_data.dx0[nx] : LQR_data.v_upper;// std::max(-4.87, std::min(4, du_[nu]));
// 			LQR_data.dx0[nx] = LQR_data.dx0[nx] < LQR_data.v_lower ? LQR_data.v_lower : LQR_data.dx0[nx];
// 		}
// 		LQR_data.vec_x.push_back(LQR_data.dx0[nx]);
// 		//LQR_data.dx0[nx] = dx_plus1[nx];
// 	}
// 	LQR_data.dx_array.push_back(LQR_data.vec_x);
// }
// 
// void observer_based_feedback_tracking_control(LQR &lqr, LQR_Data & LQR_data, double *r1){
// 	Matrix<double> _W_sq_(LQR_data.state_size, LQR_data.state_size, LQR_data.W_cost);
// 
// 
// 
// 	int i = LQR_data.current_t;
// 	fout_lqr << "observer Computing Control..., i " <<i << endl;
// 	Matrix<double> _V_sq_(LQR_data.state_size, LQR_data.state_size, LQR_data.V_cost);
// 	Matrix<double> _x_(LQR_data.state_size, 1, LQR_data.dx0);
// 	Matrix<double> _eye_(LQR_data.state_size, 1.0);
// 	Matrix<double> _AM_(LQR_data.state_size, LQR_data.state_size, LQR_data.A);
// 	Matrix<double> _BM_(LQR_data.state_size, LQR_data.control_size, LQR_data.B);
// 
// 	Matrix<double> _C_(LQR_data.state_size, LQR_data.state_size, LQR_data._C_array);
// 	Matrix<double> A_sskf = (_eye_ - lqr.KFs[i] * _C_)*_AM_; //;A_sskf = (eye(nx) - L*H)*A;
// 	Matrix<double> B_sskf = (_eye_ - lqr.KFs[i] * _C_)*_BM_; //B_sskf = (eye(nx) - L*H)*B;
// 
// 	Matrix<double> _ref_(LQR_data.state_size, 1, r1);
// 	fout_lqr << "r1, m " << _ref_.Getm() << ",n "<< _ref_.Getn()<< endl;
// 	Matrix<double> _G_(LQR_data.G_m, LQR_data.G_n, LQR_data._G_array);
// 	fout_lqr << "_G_, m " << _G_.Getm() << ", n" << _G_.Getn() << endl;
// 	Matrix<double> _xue_ = _G_*_ref_;
// 	fout_lqr << "_xue_ " << endl;
// 	Matrix<double> _xue_x_(_xue_);
// 	Matrix<double> _xue_u_(_xue_);
// 	fout_lqr << "_xue_u_ " << endl;
// 	_xue_u_.RemoveRows(0, LQR_data.state_size);
// 	_xue_x_.RemoveRows(LQR_data.state_size, LQR_data.state_size + LQR_data.control_size);
// 
// 	//control
// 	fout_lqr << "observer Computing Control..." << endl;
// 	lqr.ComputeControl(i, LQR_data.dx_hat_plus1, LQR_data.du0, _xue_x_, _xue_u_);
// 	LQR_data.vec_u.clear();
// 	for (int nu = 0; nu < LQR_data.control_size; nu++){
// 		LQR_data.du0[nu] = LQR_data.du0[nu] < LQR_data.u_upper ? LQR_data.du0[nu] : LQR_data.u_upper;// std::max(-4.87, std::min(4, du_[nu]));
// 		LQR_data.du0[nu] = LQR_data.du0[nu] < LQR_data.u_lower ? LQR_data.u_lower : LQR_data.du0[nu];
// 		LQR_data.vec_u.push_back(LQR_data.du0[nu]);
// 	}
// 	LQR_data.du_array.push_back(LQR_data.vec_u);
// 
// 	fout_lqr << "observer updating state..." << endl;
// 	double dx_plus1[10];
// 	lqr.UpdateState(i, dx_plus1, LQR_data.A, LQR_data.B, LQR_data.dx0, LQR_data.du0, _W_sq_);
// 	LQR_data.vec_x.clear();
// 	for (int nx = 0; nx < LQR_data.state_size; nx++)
// 	{
// 		//dx_array[i][nx] = dx_[nx];
// 		if (nx % 2 != 0){
// 			dx_plus1[nx] = dx_plus1[nx] < LQR_data.v_upper ? dx_plus1[nx] : LQR_data.v_upper;// std::max(-4.87, std::min(4, du_[nu]));
// 			dx_plus1[nx] = dx_plus1[nx] < LQR_data.v_lower ? LQR_data.v_lower : dx_plus1[nx];
// 			LQR_data.dx_hat_plus1[nx] = LQR_data.dx_hat_plus1[nx] < LQR_data.v_upper ? LQR_data.dx_hat_plus1[nx] : LQR_data.v_upper;// std::max(-4.87, std::min(4, du_[nu]));
// 			LQR_data.dx_hat_plus1[nx] = LQR_data.dx_hat_plus1[nx] < LQR_data.v_lower ? LQR_data.v_lower : LQR_data.dx_hat_plus1[nx];
// 		}
// 		LQR_data.vec_x.push_back(LQR_data.dx0[nx]);
// 		LQR_data.dx0[nx] = dx_plus1[nx];
// 	}
// 	LQR_data.dx_array.push_back(LQR_data.vec_x);
// 	
// 	std::default_random_engine generator;
// 	std::normal_distribution<double> distribution(0.0, 1.0);
// 
// 	double random_w[100];
// 	for (int i = 0; i < LQR_data.state_size; i++){
// 		random_w[i] = distribution(generator) * 50;
// 	}
// 	Matrix<double> random_w_(LQR_data.state_size, 1, random_w);
// 
// 	Matrix<double> _y_ = _C_*_x_ + _V_sq_*random_w_;
// 	//Update x_hat
// 	Matrix<double> _x_hat_ = A_sskf*Matrix<double>(LQR_data.state_size, 1, LQR_data.dx_hat_plus1) + B_sskf*(Matrix<double>(LQR_data.control_size, 1, LQR_data.du0)) + lqr.KFs[i] * _y_;
// 	memcpy(LQR_data.dx_hat_plus1, _x_hat_.GetData(), sizeof(double) * LQR_data.state_size);//lqr.UpdateState(i, dx_hat_plus1, Fs, Gs, dx_, du_, _W_sq_);
// };

void predict_speeds_in_front_region(SpeedPredictionModel model, MPC_Data &mpcdata){
	mpcdata.cell_dist = 30; // 2 * (int)300 / mpcdata.Ph;
	mpcdata.front_region_data.resize(10); //mpcdata.Ph / 2
	for (int i = 0; i < mpcdata.front_region_data.size(); i++){
		mpcdata.front_region_data[i].speed = -1.0;
		mpcdata.front_region_data[i].density = -1.0;
		mpcdata.front_region_data[i].flow = -1.0;
	}

	vector<vector<VehicleState>> veh_state_vec(10);
	switch (model)
	{
	case INTELLIGENT_DRIVER_MODEL:
		//pos of leading CAV as start point
		//calculate desired gap d_star using IDM car following model

		//space mean speed


		//referenced location l_r(n) = l_(n-1) - d_star;  d_star = s_min + T_des*v_n + (v_n*v_diff)/(2*sqrt(u_max*abs(u_comfort)));
		//s_min: a minimum desired net distance. A car can't move if the distance from the car in the front is not at least s_min
		//T_des: desired time headway
		//v_diff: the difference between front vehicle and current vehicle
// 		double s_min = 3.0;
// 		double T_des = 1.2;
// 		double u_comfort = 2.0;


		break;
	case SPACE_MEAN_SPEED:
		//v_N = N / sum(1/v_i);
		for (auto iter : front_veh_state_map){
			int key_ = (int) ((iter.second.odometer - veh_odometer) / mpcdata.cell_dist);
			veh_state_vec[key_].push_back(iter.second);
		}
		int ix = 0;
		for (auto itr_v : veh_state_vec){
			vector<VehicleState>&veh_state_array = itr_v;
			if (veh_state_array.size() == 0)
			{
				mpcdata.front_region_data[ix].speed = desired_velocity;
				mpcdata.front_region_data[ix].flow = 0.0;
			}
			else
			{
				int num_veh_ = veh_state_array.size();
				double sum_veh_inv = 0.0;
				for (int j = 0; j < num_veh_; j++){
					sum_veh_inv += 1.0f / veh_state_array[j].spd;
				}
				mpcdata.front_region_data[ix].flow = num_veh_;
				mpcdata.front_region_data[ix].speed = num_veh_ / sum_veh_inv;
			}		
			ix++;
		}
		break;
	}
};

void update_position_speed(SpeedPredictionModel car_following_mode, CarFollowingParam &param, float &spd, float &acc, float &pos){
	float alphaT = 1.0; //not 1 if considering bottleneck
	float alphaV0 = 1.0; //not 1 if considering bottleneck
	float s = AdjVehiclesDist[2][3]; // spacing
	float dv = Spd_vehicle- AdjVehiclesSpeed[2][3]; // relative spd
	float v0Local = min(alphaV0*desired_velocity, param.spd_limit);
	if (car_following_mode == NEWELLS_SIMPLIFIED_MODEL)
	{
		//calc acc
// 		double dt_local = alphaT* param.dt;
// 		float vnew = min(max((s - param.safety_spacing) / dt_local, 0.0), v0Local);
// 		acc = (vnew - Spd_vehicle) / dt_local;
// 		// update pos and spd
// 		float advance = Spd_vehicle * dt_local + acc * dt_local * dt_local;
// 		spd += dt_local * acc;
// 		pos += advance;
		double tao = 2 * param.T;  //eq.6 https://ac.els-cdn.com/S0191261500000448/1-s2.0-S0191261500000448-main.pdf?_tid=93d75634-fd46-11e7-8091-00000aacb360&acdnat=1516386703_0175b8438d0e5b7e53b26efad55a89c5
		double desired_spd = (1 / tao)*(s - param.desired_min_spacing);  //eq.7
		
		//spd = desired_spd;
		//acc = (desired_spd - Spd_vehicle) / param.dt; //
		acc = (AdjVehiclesSpeed[2][3] - Spd_vehicle ) / tao; // eq. 8
		
		spd = Spd_vehicle + acc*param.T;
		pos += spd*param.T;
		//fout_car_following << "veh_no:" << current_veh_no << ",Tao:" << tao << ",spacing:" << s << ",desired_min_spacing: " << param.desired_min_spacing << ",desired_spd: " << desired_spd << ",Spd_vehicle:" << Spd_vehicle << ",des_acc: " << acc << endl;
	}
	else if (car_following_mode == INTELLIGENT_DRIVER_MODEL){
		
	}
};

int InitArrays()
{
	for (int i = 0; i <= 4; i++)
	{
		for (int j = 0; j <= 4; j++)
		{
			AdjVehicles[i][j] = 0;
			AdjVehiclesWidth[i][j] = 0;
			AdjVehiclesSpeed[i][j] = 0;
			AdjVehiclesDist[i][j] = 0;
			AdjVehiclesAcc[i][j] = 0;
			AdjVehiclesLaneChange[i][j] = 0;
			AdjVehiclesCurrentLane[i][j] = 0;
			AdjVehiclesTargetLane[i][j] = 0;
		}
	}
	return 0;
}

double GetLateralPos(double latpos)
{
	if (latpos > 0.2 || latpos < -0.2)
		return latpos;
	else
		return 0.0;
}

bool ControlLeadingCAV(
	ampl::AMPL &ampl_obj, 
	double lead_CAV_speed, 
	double front_veh_speed,
	double front_veh_spacing,
	vector<double> &vehicle_init_speed, 
	vector<double> &spacing_init_vec,
	vector<double> &vehicle_init_acc,
	vector<double> &vehicle_length)
{
	//MPC_control_acceleration;
	int platoon_size = unique_vehicle_no.size();// 4;
	double time_interval = 0.1;
	double min_acc = -3.0*(2.2 - (Veh_Weight_kg/15000.0));//longitudinal collision, Xiaoyun Lu //-8.0;
	double max_acc = 3.0;
	int control_horizon = 30;
	double 	beta_const_ = 0.1*platoon_size*platoon_size;// spacing, -0.6*(platoon_size + 1 - i);  //Lili Du: constrained optimization and distributed computation
	double alpha_const_ = 0.3*platoon_size*platoon_size; //speed, - 1.2(platoon_size + 1 - i);  
	double reaction_time=1.5;
	double sensor_detection_range=200;
	double desired_time_headway = 1.6;


	double predicted_front_speed_ = front_veh_speed + time_interval*find_min_acceleration();	//find_predicted_acceleration();//55;
	double predicted_front_spacing_ = front_veh_spacing + predicted_front_speed_*time_interval - lead_CAV_speed*time_interval;//30;
	//vector<double>vehicle_init_speed{ 24, 20, 15, 10 };
	//vector<double> spacing_init_vec{ 55, 45, 35, 30 };
	//


 	bool solved_flag	= run_MPC_ampl(
		ampl_obj,
		platoon_size,
		time_interval,
		min_acc,
		max_acc,
		control_horizon,
		alpha_const_,
		beta_const_,
		 reaction_time,
		 sensor_detection_range,
		desired_time_headway,
		predicted_front_speed_,
		predicted_front_spacing_,
		vehicle_init_speed,
		spacing_init_vec,
		vehicle_init_acc,
		vehicle_length,
		0,
		MPC_control_acceleration);

	if (solved_flag == true)
	{
		for (int vno = 0; vno < MPC_control_acceleration.size(); vno++)
			MPC_optimal_accelerations[unique_vehicle_no[vno]] = MPC_control_acceleration[vno];
		//desired_acceleration = MPC_control_acceleration.front();
	}
		
	return solved_flag;
};

void avoid_collision_acc(double hwdy)
{
	double dn = max(5.0, Spd_vehicle * hwdy);// 12.0;
	dn = 10.0;
	double tao = 1.9;

	double desired_s = .0;
	double spacing = AdjVehiclesDist[2][3];
	double rel_acc = AdjVehiclesAcc[2][3] - Acc_vehicle;
	double rel_spd = AdjVehiclesAcc[2][3] - Spd_vehicle;
	//if (AdjVehiclesSpeed[2][3] >= Spd_vehicle){
	double predict_advance1 = abs((Spd_vehicle - AdjVehiclesSpeed[2][3]) * (Spd_vehicle - AdjVehiclesSpeed[2][3]) / (2 * rel_acc));  //AdjVehiclesAcc[2][3]; //
	double predict_advance2 = (Spd_vehicle - AdjVehiclesSpeed[2][3]) * 0.1 - (0.5*rel_acc*(0.1*0.1));
	//fout_mpc << "predict_advance2," << predict_advance2 << ",s," << AdjVehiclesDist[2][3] << endl;
	double predict_acc;// = -(Spd_vehicle - AdjVehiclesSpeed[2][3]) * (Spd_vehicle - AdjVehiclesSpeed[2][3]) / (2 * spacing);
	predict_acc = (2 * (spacing - desired_s - (Spd_vehicle - AdjVehiclesSpeed[2][3])*0.1) / 0.01) + AdjVehiclesAcc[2][3];
// 	if (spacing < desired_s)//if (predict_advance2 > spacing)  //predict_advance1 >= AdjVehiclesDist[2][3] ||  //if (Acc_vehicle > predict_acc){
// 	{
// 		//fout_mpc << "t," << timestep << ",veh," << current_veh_no << ",p_veh," << AdjVehicles[2][3] << "___spacing < 2________" << ",predict_acc," << predict_acc << endl;
// 		//fout_mpc << "predict_advance1," << predict_advance1 << ",predict_advance2," << predict_advance2 << ",spacing," << AdjVehiclesDist[2][3] << ",Acc_vehicle," << Acc_vehicle << ",predict_acc," << predict_acc << endl;
// 		desired_acceleration = predict_acc < -4.0 ? -4.0 : predict_acc;
// 	}
// 	else if (spacing > 50.0 && AdjVehicles[2][3] != -1){

	if (spacing < (dn - 2.5) || spacing >(dn + 5.50)){
// 		double des_v = (spacing - dn) / tao;
// 		double delta_v = (des_v - Spd_vehicle) / ((tao / 0.1) / 2);
// 		desired_acceleration = delta_v / 0.1;
		HWShort = Predictive_headway+0.1;
		HWLong = Predictive_headway+0.1;

		//ControlVehicle();
// 		desired_acceleration = CACC_Car_Following(current_veh_no, Acc_vehicle, Spd_vehicle, Leng_vehicle,
// 			AdjVehicles[2][3], AdjVehiclesAcc[2][3], AdjVehiclesSpeed[2][3], AdjVehiclesWidth[2][3], AdjVehiclesDist[2][3],
// 			HWShort);
		float des_spd;
		float des_pos = veh_odometer;
		float des_acc;
		CarFollowingParam cf_param;
		cf_param.T = 0.2;
		update_position_speed(NEWELLS_SIMPLIFIED_MODEL, cf_param, des_spd, des_acc, des_pos);
		desired_acceleration = (double)des_acc;
		// 		fout_mpc << "avoid_newell," << ",t," << timestep << ",veh," << current_veh_no << ",p_veh," << AdjVehicles[2][3] 
// 			<< ",des_v," << des_v 
// 			<< ",spacing," << spacing
// 			<< ",desired_acceleration," << desired_acceleration << endl;
 		//desired_acceleration = predict_acc > 4.5 ? 4.5 : predict_acc;
		fout_mpc << "if avoid_newell_ vehno:" << current_veh_no << ",dn=" << dn << ",spacing," << spacing << ",desired_acceleration," << desired_acceleration << ",predict_acc," << predict_acc << endl;
	}
	else
	{
		desired_acceleration = 0.0;
		fout_mpc << "else avoid_newell_ vehno:" << current_veh_no << ", dn = " << dn << ", spacing, " << spacing << ",desired_acceleration," << desired_acceleration << ",predict_acc," << predict_acc  << endl;
	}
// 	}
//}

	//desired_acceleration = (1/0.2)*(AdjVehiclesSpeed[2][3] - Spd_vehicle);
	
//	fout_mpc << "aovid_t," << timestep << ",veh," << current_veh_no << ",p_veh," << AdjVehicles[2][3] << ",nspd," << AdjVehiclesSpeed[2][3] << ",spd," << Spd_vehicle << ",nacc," << AdjVehiclesAcc[2][3] << ",acc," << Acc_vehicle << ",spacing," << spacing << ",desired_acceleration," << desired_acceleration << endl;
};

int ControlVehicle()
{
	double temp_hw = 0;
	if (platoonState[current_veh_no][0] == 0)
	{
		if (rand() % 100 < 20)
			desired_velocity = head_speed;
		else
			desired_velocity = head_speed + 5;
		platoonState[current_veh_no][0] = 1;
	}
	//It needs to tell if the vehicle in front is Connected or not. Since the API does not provide vehicle type information, vehicle
	//length is used an indicator, i.e. CACC vehicles are less than 3.8m long, while others are longer than 3.8m. The vehicle types in the 
	//VISSIM network need to be set-up accordingly.
	if (AdjVehiclesWidth[2][3] < 3.8 || AdjVehiclesDist[2][3] < 200.0)
		//When front vehicle is connected, use HWShort, and front vehicle's acceleration is available (AdjVehiclesAcc[2][3])
		desired_acceleration = CACC_Car_Following(current_veh_no, Acc_vehicle, Spd_vehicle, Leng_vehicle,
		AdjVehicles[2][3], AdjVehiclesAcc[2][3], AdjVehiclesSpeed[2][3], AdjVehiclesWidth[2][3], AdjVehiclesDist[2][3],
		HWShort);
	else
		//When front vehicle is not connected and beyond a sensing range, use HWLong, and front vehicle's acceleration is not available (assuming ZERO)
		desired_acceleration = CACC_Car_Following(current_veh_no, Acc_vehicle, Spd_vehicle, Leng_vehicle,
		AdjVehicles[2][3], 0, AdjVehiclesSpeed[2][3], AdjVehiclesWidth[2][3], AdjVehiclesDist[2][3],
		HWLong);
	return 1;
}

bool run_MPC_ampl(
	ampl::AMPL &ampl,
	int platoon_size,
	double time_interval,
	double min_acc,
	double max_acc,
	int control_horizon,
	double alpha_const_,
	double	beta_const_,
	double reaction_time,
	double sensor_detection_range,
	double desired_time_headway_,
	double predicted_front_speed_,
	double predicted_front_spacing_,
	vector<double> &vehicle_init_speed,
	vector<double> &spacing_init_vec,
	vector<double> &vehicle_init_acc,
	vector<double> &vehicle_length,
	double approximated_likelihood_threshold,
	vector<double> &control_acceleration)
{
	//ampl::AMPL ampl;
// 	try
// 	{
// 		ampl.read("X:/Project/ARPA-e/PTV/CACC-VISSIM-v2.0/CACC-VISSIM/Type101DriverModel/DriverModel/x64/Debug/MPC_control.mod");
// 	}
//	catch (ampl::AMPLException &e)
// 	{
// 		fout_AMPL_debug << "Read mod Error: " << e.getMessage() << endl;
// 		return false;
// 	}
// 	catch (...)
// 	{
// 		fout_AMPL_debug << "Cannot read Error, try read C drive" << endl;
// 		ampl.read("C:/AMPL/MPC_control.mod");
// 	fout_AMPL_debug << "read mod sucessfully" << endl;

	fout_AMPL_debug << "platoon size: " << platoon_size << endl;

	ampl::Parameter num_vehicles = ampl.getParameter("num_vehicles");
	num_vehicles.set(platoon_size);

	ampl::Parameter delta_t = ampl.getParameter("delta_t");
	delta_t.set(time_interval);

	ampl::Parameter acc_min = ampl.getParameter("acc_min");
	acc_min.set(min_acc);

	ampl::Parameter acc_max = ampl.getParameter("acc_max");
	acc_max.set(max_acc);

	ampl::Parameter horizon = ampl.getParameter("horizon");
	horizon.set(control_horizon);

	ampl::Parameter alpha_const = ampl.getParameter("alpha_const");
	alpha_const.set(alpha_const_);

	ampl::Parameter beta_const = ampl.getParameter("beta_const");
	beta_const.set(beta_const_);

	ampl::Parameter human_reaction_time = ampl.getParameter("human_reaction_time");
	human_reaction_time.set(reaction_time);

	ampl::Parameter sensor_range = ampl.getParameter("sensor_range");
	sensor_range.set(sensor_detection_range);

	ampl::Parameter desired_time_headway = ampl.getParameter("desired_time_headway");
	desired_time_headway.set(desired_time_headway_);

	ampl::Parameter predicted_front_speed = ampl.getParameter("predicted_front_speed");
	predicted_front_speed.set(predicted_front_speed_);

	ampl::Parameter predicted_front_spacing = ampl.getParameter("predicted_front_spacing");
	predicted_front_spacing.set(predicted_front_spacing_);

	ampl::Parameter vehicle_speed_init = ampl.getParameter("vehicle_speed_init");
	vehicle_speed_init.setValues(vehicle_init_speed.data(), vehicle_init_speed.size());

	ampl::Parameter spacing_init = ampl.getParameter("spacing_init");
	spacing_init.setValues(spacing_init_vec.data(), spacing_init_vec.size());

	ampl::Parameter acc_init = ampl.getParameter("acc_init");
	acc_init.setValues(vehicle_init_acc.data(), vehicle_init_acc.size());
	
	ampl::Parameter vehicle_len = ampl.getParameter("vehicle_length");
	vehicle_len.setValues(vehicle_length.data(), vehicle_length.size());

	try
	{
		ampl.solve();
		ampl::DataFrame results = ampl.getVariable("acceleration").getValues();
		ampl::DataFrame spacing_res = ampl.getVariable("future_spacing").getValues();
		ampl::DataFrame spd_res = ampl.getVariable("future_vehicle_speed").getValues();
		fout_AMPL_debug << "predicted_front_speed_: " << predicted_front_speed_ << endl;
		fout_AMPL_debug << "predicted_front_spacing_: " << predicted_front_spacing_ << endl;
		fout_AMPL_debug << "acc: "	 << results.toString() << endl;
		fout_AMPL_debug << "spacing: " << spacing_res.toString() << endl;
		fout_AMPL_debug << "speed: " << spd_res.toString() << endl;
		// 		int n_col = results.getNumCols();
				int n_rows = results.getNumRows();
		control_acceleration.clear();
		for (int var_index = 1; var_index<n_rows; var_index += control_horizon){
			control_acceleration.push_back(results.getRowByIndex(var_index)[2].dbl());
		}
		//control_acceleration.push_back(results.getRowByIndex(1)[2].dbl());
		//control_acceleration.push_back(results.getRowByIndex(51)[2].dbl());
		//control_acceleration.push_back(results.getRowByIndex(101)[2].dbl());
		//control_acceleration.push_back(results.getRowByIndex(151)[2].dbl());
		fout_AMPL_debug << "AMPL solving succeed" << endl;
		return true;
		// 		for (int i = 0; i < n_rows; i++){  //num vehicles
		// 			double val0 = results.getRowByIndex(i)[0].dbl();
		// 			double val1 = results.getRowByIndex(i)[1].dbl();
		// 			double val2 = results.getRowByIndex(i)[2].dbl();
		// 		}
	}
	catch (ampl::AMPLException &e)
	{
		fout_AMPL_debug << "Solving Error: " << e.getMessage() << endl;
		return false;
	}
	catch (...) {
		fout_AMPL_debug << "Unknown Solving Error" << endl;
		return false;
	}
	
	return !control_acceleration.empty();
};

void update_front_DSRC_region(int cur_veh_no){
	front_veh_state_map.erase(cur_veh_no);
	if (cur_veh_no < 5 && veh_odometer > lead_CAV_state.odometer && veh_odometer - lead_CAV_state.odometer <= 300.0){
		front_veh_state_map[cur_veh_no].odometer = veh_odometer;
		front_veh_state_map[cur_veh_no].spd = Spd_vehicle;
		front_veh_state_map[cur_veh_no].acc = Acc_vehicle;
	}
};

map<int, int> current_subsec_step_ctr;

void update_front_space_time_region_observation(int current_veh_no){
	//lead_CV_front_vehicles_state
	//time t;
	int subsec_step = (int)(timestep*10); 
	//fout_mpc << "subsec_step," << subsec_step << ",current_veh_no," << current_veh_no << ",veh_odometer," << veh_odometer << ",lead_CAV_state.odometer," << lead_CAV_state.odometer << endl;
	if (current_veh_no <= leading_CAV_ID &&  veh_odometer >= lead_CAV_state.odometer && (veh_odometer - lead_CAV_state.odometer) < 300.0){
		
		if (lead_CV_front_vehicles_state_t_vno_indices.size() > front_region_observation_length)// && lead_CV_front_vehicles_state_t_vno_indices.back().size() == lead_CV_front_vehicles_state_t_vno_indices[front_region_observation_length-2].size()){
		{	
			//fout_mpc << "pop__back().size()," << lead_CV_front_vehicles_state_t_vno_indices.back().size() << ",back_1.size()," << lead_CV_front_vehicles_state_t_vno_indices[front_region_observation_length - 2].size() << endl;
			lead_CV_front_vehicles_state_t_vno_indices.pop_front();
		}
		if (current_subsec_step_ctr.find(subsec_step) == current_subsec_step_ctr.end())
		{
			map<int, VehicleState> t_front_vehicles;
			t_front_vehicles[current_veh_no].acc = Acc_vehicle;
			t_front_vehicles[current_veh_no].spd = Spd_vehicle;
			t_front_vehicles[current_veh_no].odometer = veh_odometer;
			t_front_vehicles[current_veh_no].time_step = timestep;
			t_front_vehicles[current_veh_no].spacing = AdjVehiclesDist[2][3] - Leng_vehicle;
			lead_CV_front_vehicles_state_t_vno_indices.push_back(t_front_vehicles);
		}
		else
		{
			lead_CV_front_vehicles_state_t_vno_indices.back()[current_veh_no].acc = Acc_vehicle;
			lead_CV_front_vehicles_state_t_vno_indices.back()[current_veh_no].spd = Spd_vehicle;
			lead_CV_front_vehicles_state_t_vno_indices.back()[current_veh_no].odometer = veh_odometer;
			lead_CV_front_vehicles_state_t_vno_indices.back()[current_veh_no].time_step = timestep;
			lead_CV_front_vehicles_state_t_vno_indices.back()[current_veh_no].spacing = AdjVehiclesDist[2][3] - Leng_vehicle;
		}

		//fout_mpc << "---subsec_step," << subsec_step << ",current_veh_no," << current_veh_no << ",current_subsec_step_ctr[subsec_step]," << current_subsec_step_ctr[subsec_step] << ",back size," << lead_CV_front_vehicles_state_t_vno_indices.back().size() << endl;
		current_subsec_step_ctr[subsec_step]++;
	}
};

double CACC_Car_Following(long lvid0, double a0, double v0, double leng0, long lvid1, double a1, double v1, double leng1, double d01,
	double t_system)
	//Implemented Bart van Arem's MIXIC model
{
	double a_ref = 0;
	double a_ref_v = 0;
	double a_ref_d = 0;
	double vint = desired_velocity;
	double ap = a1;
	double vp = v1;
	double r_ref = 0;
	double r_safe = 0;
	double r_system = 0;
	double r_min = 2; //in meters
	double r = d01 - (leng0 + leng1) / 2;
	double dp = -3; //deceleration capability of the leading vehicle
	double d = -3; //deceleration capability of the ego vehicle

	double k = 0.3;  //based on MIXIC studies
	double ka = 1.0; //
	double kv = 0.58;
	double kd = 0.1;
	double r_ref1 = 0.0;
	double r_ref2 = 0.0;

	a_ref_v = k*(vint - v0);

	r_safe = v0*v0 / 2 * (1 / dp - 1 / d);
	r_system = t_system*v0;

	r_ref1 = max(r_safe, r_system);
	r_ref2 = max(r_safe, r_min);

	r_ref = max(r_ref1, r_ref2);
	a_ref_d = ka*ap + kv*(vp - v0) + kd*(r - r_ref);

	a_ref = min(a_ref_v, a_ref_d);
	if (lvid1 == -1)
		return a_ref_v;
	else
		return a_ref;
}