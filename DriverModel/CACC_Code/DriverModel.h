/*==========================================================================*/
/*  DriverModel.h                                    DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*                                                                          */
/*  Version of 2010-06-17                                   Lukas Kautzsch  */
/*==========================================================================*/

#ifndef __DRIVERMODEL_H
#define __DRIVERMODEL_H

#ifndef _CONSOLE
#include <windows.h>
#endif
//#include "Data_driven_DRO_MPC.h"
#include "engine.h"
// #include "mclmcr.h"
// #include "mclmcrrt.h"
// #include "mclcppclass.h"
// #include "matrix.h"

#include <iomanip>

// #include "matrix.h"
// #include "matrixIO.h"
#include "ampl/ampl.h"
//#include "lqr.h"
#include <sstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <direct.h>
#include <deque>
#include <random>
#define GetCurrentDir _getcwd


using namespace std;
/* In the creation of DriverModel.DLL all files must be compiled */
/* with the preprocessor definition DRIVERMODEL_EXPORTS.         */
/* Programs that use DriverModel.DLL must not be compiled        */
/* with that preprocessor definition.                            */

#ifdef DRIVERMODEL_EXPORTS
#define DRIVERMODEL_API extern "C" __declspec(dllexport)
#else
#define DRIVERMODEL_API extern "C" __declspec(dllimport)
#endif

/*==========================================================================*/

/* general data: */
/* (index1, index2 irrelevant) */
#define  DRIVER_DATA_PATH                    101
           /* string: absolute path to the model's data files directory */
#define  DRIVER_DATA_TIMESTEP                102
           /* double: simulation time step length [s] */
#define  DRIVER_DATA_TIME                    103
           /* double: current simulation time [s] */
#define  DRIVER_DATA_PARAMETERFILE           104
           /* string: name (including absolute path) of a vehicle type's parameter file */
#define  DRIVER_DATA_STATUS                  105
           /* long:                                                           
           /* 0=OK,                                                             */
           /* 1=Info (there's some further information available),              */
           /* 2=Warning (there are warnings available),                         */
           /* 3=Error (an recoverable error occured but simulation can go on),  */
           /* 4=Heavy (an unrecoverable error occured and simulation must stop) */
           /* (used by DriverModelGetValue()!)                                  */
#define  DRIVER_DATA_STATUS_DETAILS          106
           /* string: XML format according to the schema file VDMStatus.xsd  */
           /* (used by DriverModelGetValue()!)                               */
           /* (is retrieved by VISSIM only if DRIVER_DATA_STATUS is nonzero) */

/* current vehicle driver unit data (VDU to be moved next): */
/* (index1, index2 irrelevant) */
#define  DRIVER_DATA_VEH_ID                  201
           /* long:   vehicle number */
#define  DRIVER_DATA_VEH_LANE                202
           /* long:   current lane number (rightmost = 1) */
#define  DRIVER_DATA_VEH_ODOMETER            203
           /* double: total elapsed distance in the network [m] */
#define  DRIVER_DATA_VEH_LANE_ANGLE          204
           /* double: angle relative to the middle of the lane [rad] */
           /*         (positive = turning left)                      */
#define  DRIVER_DATA_VEH_LATERAL_POSITION    205
           /* double: distance of the front end from the middle of the lane [m] */
           /*         (positive = left of the middle, negative = right)         */
#define  DRIVER_DATA_VEH_VELOCITY            206
           /* double: current speed [m/s] */
#define  DRIVER_DATA_VEH_ACCELERATION        207
           /* double: current acceleration [m/s²] */
#define  DRIVER_DATA_VEH_LENGTH              208
           /* double: vehicle length [m] */
#define  DRIVER_DATA_VEH_WIDTH               209
           /* double: vehicle width [m] */
#define  DRIVER_DATA_VEH_WEIGHT              210
           /* double: vehicle weight [kg] */
#define  DRIVER_DATA_VEH_MAX_ACCELERATION    211
           /* double: maximum possible acceleration [m/s²] */
           /*         (depending on current speed)         */
#define  DRIVER_DATA_VEH_TURNING_INDICATOR   212
           /* long:   left = 1, right = -1, none = 0, both = 2 */
           /*         (also used by DriverModelGetValue()!) */
#define  DRIVER_DATA_VEH_CATEGORY            213
           /* long:   car = 1, truck = 2, bus = 3, tram = 4, */
           /*         pedestrian = 5, bike = 6               */
#define  DRIVER_DATA_VEH_PREFERRED_REL_LANE  214
           /* long:   positive = left, 0 = current lane, negative = right */
#define  DRIVER_DATA_VEH_USE_PREFERRED_LANE  215
           /* long:   0 = only preferable (e.g. European highway) */
           /*         1 = necessary (e.g. before a connector)     */
#define  DRIVER_DATA_VEH_DESIRED_VELOCITY    216
           /* double: desired speed [m/s]              */
           /*         (also used by DriverModelGetValue()!) */
#define  DRIVER_DATA_VEH_X_COORDINATE        217
           /* double: world coordinate X (vehicle front end) */
#define  DRIVER_DATA_VEH_Y_COORDINATE        218
           /* double: world coordinate Y (vehicle front end) */
#define  DRIVER_DATA_VEH_TYPE                219
           /* long:   vehicle type number (user defined) */
#define DRIVER_DATA_VEH_COLOR                220
           /* long:   vehicle color (24 bit RGB value) */
           /*         (also used by DriverModelGetValue()!) */
#define DRIVER_DATA_VEH_CURRENT_LINK         221
           /* long:   current link number */
#define DRIVER_DATA_VEH_NEXT_LINKS           222
           /* long:   following link number(s) of the vehicle's route        */
           /* This message is sent from VISSIM only if DriverModelSetValue() */
           /* returned 1 for DRIVER_DATA_VEH_CURRENT_LINK.                   */
           /* It is sent once for each link in the route.                    */
#define  DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE  223
           /* long:   direction of an active lane change movement */
           /*         (+1 = to the left, 0 = none, -1 = to the right) */
#define  DRIVER_DATA_VEH_REL_TARGET_LANE     224
           /* long:   target lange */
           /*         (+1 = next one left, 0 = current lane, -1 = next one right) */
             
/* nearby vehicle driver unit data: */
/* (index1 = relative lane: +2 = second to the left, +1 = next one to the left,   */
/*                           0 = current lane,                                    */
/*                          -1 = next one to the right, -2 = second to the right) */
/* (index2 = relative position: positive = downstream (+1 next, +2 second next)   */
/*                              negative = upstream (-1 next, -2 second next))    */
#define  DRIVER_DATA_NVEH_ID                 301
           /* long:   vehicle number */
           /*         (negative = no vehicle at this lane/position) */
#define  DRIVER_DATA_NVEH_LANE_ANGLE         302
           /* double: angle relative to the middle of the lane [rad] */
           /*         (positive = turning left)                      */
#define  DRIVER_DATA_NVEH_LATERAL_POSITION   303
           /* double: distance of the front end from the middle of the lane [m] */
           /*         (positive = left of the middle, negative = right)         */
#define  DRIVER_DATA_NVEH_DISTANCE           304
           /* double: gross distance [m] (front end to front end) */
#define  DRIVER_DATA_NVEH_REL_VELOCITY       305
           /* double: speed difference [m/s] (veh. speed - nveh. speed) */
#define  DRIVER_DATA_NVEH_ACCELERATION       306
           /* double: current acceleration [m/s²] */
#define  DRIVER_DATA_NVEH_LENGTH             307
           /* double: vehicle length [m] */
#define  DRIVER_DATA_NVEH_WIDTH              308
           /* double: vehicle width [m] */
#define  DRIVER_DATA_NVEH_WEIGHT             309
           /* double: vehicle weight [kg] */
#define  DRIVER_DATA_NVEH_TURNING_INDICATOR  310
           /* long:   left = 1, right = -1, none = 0, both = 2 */
#define  DRIVER_DATA_NVEH_CATEGORY           311
           /* long:   car = 1, truck = 2, bus = 3, tram = 4, */
           /*         pedestrian = 5, bike = 6               */
#define  DRIVER_DATA_NVEH_LANE_CHANGE        312
           /* long:   direction of a current lane change              */
           /*         (+1 = to the left, 0 = none, -1 = to the right) */

/* link data: */
/* (index1, index2 irrelevant) */
#define  DRIVER_DATA_NO_OF_LANES             401

/* lane data: */
/* (index1 = lane number: rightmost = 1) */
/* (index2 irrelevant) */
#define  DRIVER_DATA_LANE_WIDTH              501
           /* double: lane width [m] */
#define  DRIVER_DATA_LANE_END_DISTANCE       502
           /* distance to end of lane [m] */
           /* (can be emergency stop position before connector) */
           /* (negative = no end of lane in visibility range) */

/* environment data: */
/* (index1, index2 irrelevant) */
#define  DRIVER_DATA_RADIUS                  601
           /* double: current curve radius [m] */
#define  DRIVER_DATA_MIN_RADIUS              602
           /* double: minimum curve radius [m] in visibility range */
#define  DRIVER_DATA_DIST_TO_MIN_RADIUS      603
           /* double: distance [m] to spot of minimum curve radius */
#define  DRIVER_DATA_SLOPE                   604
           /* double: current slope (negative = drop) */
           /* (e.g. -0.026 = -2.6%)                   */
#define  DRIVER_DATA_SLOPE_AHEAD             605
           /* double: slope at end of visibility range */
           /* (negative = drop)                        */

/* traffic sign data: */
/* (index1, index2 irrelevant) */
#define  DRIVER_DATA_SIGNAL_DISTANCE         701
           /* double: distance [m] to next signal head */
           /* (negative = no signal head visible)      */
#define  DRIVER_DATA_SIGNAL_STATE            702
           /* long:   red = 1, amber = 2, green = 3, red/amber = 4, */
           /*         amber flashing = 5, off = 6, green arrow = 7  */
#define  DRIVER_DATA_SIGNAL_STATE_START      703
           /* double: simulation time [s] when signal changed to current state */
#define  DRIVER_DATA_SPEED_LIMIT_DISTANCE    704
           /* double: distance [m] to "speed limit sign" */
           /*         (reduced speed area: real distance) */
           /*         (desired speed decision: 1.0 m when just passed) */
           /*         (negative: no sign visible) */
#define  DRIVER_DATA_SPEED_LIMIT_VALUE       705
           /* double: speed limit [km/h] */
           /*         (0 = end of reduced speed area) */

/* driving behaviour data: */
/* (index1, index2 irrelevant) */
/* (must be provided by the driver model after DRIVER_COMMAND_MOVE_DRIVER) */
/* (usually for GetValue(), but will be set by SetValue() as suggestion    */
/*  if GetValue (DRIVER_DATA_WANTS_SUGGESTION, ...) delivers 1)            */
#define  DRIVER_DATA_WANTS_SUGGESTION        801
           /* long:   flag: does driver model want suggestion? */
           /*         (1 = yes, 0 = no) */
#define  DRIVER_DATA_DESIRED_ACCELERATION    802
           /* double: desired acceleration [m/s²] in next time step */
#define  DRIVER_DATA_DESIRED_LANE_ANGLE      803
           /* double: desired angle relative to the middle of the lane [rad] */
           /*         (positive = turning left) */
#define  DRIVER_DATA_ACTIVE_LANE_CHANGE      804
           /* long:   direction of an active lane change movement */
           /*         (+1 = to the left, 0 = none, -1 = to the right) */
           /*         (must be != 0 while lane change is not completed) */
           /*         (will be used for NVEH_LANE_CHANGE) */
#define  DRIVER_DATA_REL_TARGET_LANE         805
           /* long:   target lange */
           /*         (+1 = next one left, 0 = current lane, -1 = next one right) */
#define  DRIVER_DATA_SIMPLE_LANECHANGE       806
           /* long:   flag: does driver model want VISSIM to control the lateral  */
           /*         movement during the lane change (i.e. start lane change     */
           /*         when ACTIVE_LANE_CHANGE != 0 but ignore DESIRED_LANE_ANGLE) */
           /*         and stop the lane change after the vehicle has reached the  */
           /*         middle of the new lane?                                     */
           /*         (1 = yes, 0 = no)                                           */

/*--------------------------------------------------------------------------*/

template <class T> const T& min_(const T& a, const T& b) {
	return !(b < a) ? a : b;     // or: return !comp(b,a)?a:b; for version (2)
}

template <class T> const T& max_(const T& a, const T& b) {
	return (a < b) ? b : a;     // or: return comp(a,b)?b:a; for version (2)
}

struct V2S_Message_Data
{
	double V2S_desired_acc;
	double Cloud_Message_ID;
	double Vehicle_ID;
};

struct S2V_Message_Data
{
	int S2V_platoon_size;
	int S2V_leading_CAV_ID;
	double S2V_vissim_timestep;
	double S2V_lead_CAV_speed;
	double S2V_front_veh_speed;
	double S2V_front_spacing;
	vector<double> S2V_vehicle_init_speed;
	vector<double> S2V_vehicle_init_spacing;
	vector<double> S2V_vehicle_init_acc;
	vector<double> S2V_vehicle_length;
	vector<double> S2V_vehicle_odometer;
	vector<double> S2V_vehicle_x;
	vector<double> S2V_vehicle_y;
	vector<double> S2V_front_CAV_spd;
	vector<double> S2V_front_CAV_acc;
	
	vector<int> S2V_unique_vehicle_no_vec;//int platoon_size = unique_vehicle_no.size();

};

struct RealTimeInVehicleSensingData{
	int vehicle_id = -1;
	double simulation_time_step_second = 0.0;
	double current_time_second = 0.0;
	int vehicle_lane = 0;
	double vehicle_odometer_meter = 0.0;
	double vehicle_lane_angle_rad = 0.0;
	double vehicle_lateral_position_meter = 0.0;
	double vehicle_velocity_meter_second = 0.0;
	double vehicle_accelaration_meter_second2 = 0.0;
	double vehicle_x_coordinate = 0.0;
	double vehicle_y_coordinate = 0.0;
	double vehicle_z_coordinate = 0.0;
	double front_vehicle_distance_gap_meter = 0.0;
	double front_vehicle_velocity_meter = 0.0;
	double front_vehicle_relative_velocity_meter_second = 0.0;
	double front_vehicle_acceleration_meter_second2 = 0.0;
	int cloud_message_id = 0;
};

enum SolverID
{
	LQR_state_feedback = 0,
	LQR_state_feedback_reference_tracking,
	observer_based_feedback_tracking,
	PMP_Hamiltonian,
	MPC_QP_regulating,
	MPC_tracking,
	Stochastic_MPC_QP,
	Data_driven_MPC,
	//
	Predictive_deter_Regulating_MPC = 50,
	Predictive_deter_Tracking_MPC,
	Predictive_deter_instananeous_Tracking_MPC,
	Predictive_deter_instananeous_Regulating_MPC,
	Predictive_constant_regulating_MPC,
	Predictive_constant_tracking_MPC,
	Predictive_DRO_Reference_Stochastic_SDP,
	Predictive_DRO_Regulating_MPC,
	Predictive_DRO_Tracking_MPC,
	//
	Car_Following_Control = 100,


};

struct FundamentalDiagram 
{
	double flow;
	double density;
	float speed;
};

struct MPC_Data
{
	int solve_ampl_times = 0;

	int Ph = 50;
	double dt = 0.1;
	int control_size = 5;
	double head = 1.6;
	float reac = 0.66;

	float u_lower = -4.87;
	float u_upper = 4.0;
	float v_lower = 0.0;
	float v_upper = 29.0;
	vector<double> v_length;

	vector<double> vec_u; // at current t
	vector<double> vec_x; // at current t
	vector<vector<double>> dx_array;
	vector<vector<double>> du_array;

	double total_cpu_time = 0.0;

	//predict front speeds over rolling horizons using car following model
	double cell_dist;
	vector<FundamentalDiagram> front_region_data;
	map<int, deque<float>> observed_acc_error;
	map<int, deque<float>> observed_spd_error;
	map<int, deque<float>> observed_loc_error;

};
struct LQR_Data{
	int current_t=0;
	int state_size = 4;
	int control_size = 2;
	int T = 18000;
	float dt = 0.1;
	float head = 1.2;
	float reac = 0.66;
	float u_lower = -4.87;
	float u_upper = 4.0;
	float v_lower = 0.0;
	float v_upper = 29.0;
	vector<double> v_length;

	double Q[100];
	double R[25];
	double N[100]; // terminal
	double A[100];
	double B[100];
	//2D matrices of A and B
	//std::vector<double*> Fs;
	//std::vector<double*> Gs;
	//initial
	double dx0[10];
	double du0[5];
	double dx_hat_plus1[10];

	vector<double> vec_u; // at current t
	vector<double> vec_x; // at current t
	vector<vector<double>> dx_array;
	vector<vector<double>> du_array; 

	vector<vector<double>> reference;
// 
// 	Matrix<double> _C_;
// 	Matrix<double> _D_;
// 	Matrix<double> _G_;
	int G_m;
	int G_n;
	double _G_array[300];
	double _C_array[300];

	double W_cost[128];
	double V_cost[128];

	double total_cpu_time = 0.0;
};

struct VehicleState 
{
	double time_step;
	double odometer;
	double spd;
	double acc;
	double spacing;
};

void update_front_DSRC_region(int current_veh_no);


void update_front_space_time_region_observation(int current_veh_no);

struct LQR_Initial_State{
	vector<double> init_spd;
	vector<double> init_loc;
	vector<double> init_acc;
	map<int, double> lqr_veh_no_init_spd;
	map<int, double> lqr_veh_no_init_loc;
	map<int, double> lqr_veh_no_init_acc;
};

struct MPC_Initial_State{
	vector<double> init_spd;
	vector<double> init_loc;
	vector<double> init_acc;
	map<int, double> MPC_veh_no_init_spd;
	map<int, double> MPC_veh_no_init_loc;
	map<int, double> MPC_veh_no_init_acc;
	map<int, double> MPC_veh_no_init_front_loc;
	map<int, double> MPC_veh_no_init_front_spd;
};



void lqr_state_feedback_control_data_init(LQR_Initial_State&lqr_initial_state, vector<int> &unique_vehicle_no){
	for (auto veh_no : unique_vehicle_no){
		lqr_initial_state.init_acc.push_back( lqr_initial_state.lqr_veh_no_init_acc[veh_no]);
		lqr_initial_state.init_spd.push_back(lqr_initial_state.lqr_veh_no_init_spd[veh_no]);
		lqr_initial_state.init_loc.push_back(lqr_initial_state.lqr_veh_no_init_loc[veh_no]);
	}
};

void lqr_state_update_from_VISSIM(LQR_Initial_State&lqr_initial_state, vector<int> &unique_vehicle_no, LQR_Data & LQR_data);

void mpc_QP_control_data_init(MPC_Data &mpc_data);
void mpc_QP_control_data_update(ampl::AMPL &ampl, SolverID solver, MPC_Initial_State & mpc_initial_state, vector<int> &unique_vehicle_no, MPC_Data &mpc_data, double nveh_loc, double nveh_spd);
void solve_MPC_QP(ampl::AMPL &ampl, MPC_Data &mpc_data);

double solve_DRO_MPC_Matlab(vector<vector<VehicleState>> &t_vno_predicted_vehicle_trajectory, SolverID solver, MPC_Data &mpc_data, MPC_Initial_State & mpc_initial_state, vector<int> &unique_vehicle_no, double nveh_loc);

double solve_DRO_MPC_Matlab(map<int,vector<vector<VehicleState>>> &t_vno_predicted_vehicle_trajectory, SolverID solver, MPC_Data &mpc_data, MPC_Initial_State & mpc_initial_state, vector<int> &unique_vehicle_no, double nveh_loc);

// void lqr_state_feedback_control_init(LQR_Data & LQR, LQR_Initial_State&lqr_initial_state);
// void lqr_state_feedback_tracking_control_init(LQR_Data & LQR);
// void observer_based_feedback_tracking_control_init(LQR &lqr, LQR_Data & LQR);
// void lqr_state_feedback_control(LQR &lqr, LQR_Data & LQR);
// void lqr_state_feedback_tracking_control(LQR &lqr, LQR_Data & LQR, double *ref);
// void observer_based_feedback_tracking_control(LQR &lqr, LQR_Data & LQR, double *ref);

// implement car following models to predict front vehicle speed in rolling horizon regions
enum SpeedPredictionModel
{
	NEWELLS_SIMPLIFIED_MODEL,
	INTELLIGENT_DRIVER_MODEL,
	CELL_TRANSMISSION_MODEL,
	SPACE_MEAN_SPEED,
};

void predict_speeds_in_front_region(SpeedPredictionModel model, MPC_Data &mpcdata);



struct CarFollowingParam 
{
	int first_veh_NGSIM_ID;

	float spd_limit = 49;
	//float safety_spacing; //should be different for different driver

	float min_acc = -4.5;
	float max_acc = 5.0;

	float dt = 0.1;
	float T = 1.9;
	float desired_min_spacing;

	map<int, float> control_start_t_map;
	map<int, float> control_des_spd_map;
	map<int, float> control_des_delta_v_map;
};

struct NGSIMData
{
	int Vehicle_ID; //1 
	int Frame_ID;//2
	int Total_Frames; //3
	int Global_Time; //4
	float Local_X; //5
	float Local_Y; //6
	float Global_X; //7
	float Global_Y; //8
	float Vehicle_Length; //9
	float Vehicle_Width; //10
	int	Vehicle_Class; //11
	float	Vehicle_Velocity; //12
	float	Vehicle_Acceleration; //13
	int 	Lane_Identification; //14
	int	Preceding_Vehicle; //15
	int	Following_Vehicle; //16
	float	Spacing; //17
	float	Headway; //18
};

bool CompareByEntryTime(const map<int, NGSIMData> &a, const map<int, NGSIMData> &b) {
	return a.begin()->second.Frame_ID <  b.begin()->second.Frame_ID;	// a.entry_time < b.entry_time;
};


void push_vehicle_records_to_NGSIM(map<int, vector<NGSIMData>> & NGSIM_data);
void write_vehicle_records_to_NGSIM(map<int, vector<NGSIMData>> & NGSIM_data, int veh_no);


void update_position_speed(SpeedPredictionModel car_following_mode, CarFollowingParam &param, float &spd, float &acc, float &pos);

template <class real>
int Write2DMatrixToDiskTextFile(const char * filename, int m, int n, vector<vector<real>> matrix)
{
	// open file
	FILE * fout;
	fout = fopen(filename, "w");

	if (!fout)
	{
		printf("Couldn't open output file %s.\n", filename);
		return 1;
	}

	fprintf(fout, "%d\n%d\n", m, n);

	// matrix is m x n
	int i, j;
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++){
			//double val = matrix[ELT(m, i, j)];
			fprintf(fout, "%.15G,", matrix[i][j]);
		}
		fprintf(fout, "\n");
	}

	fclose(fout);

	return 0;
}
// 
// template<class real>
// void Write_Matrices_To_File(const char * filename, vector<Matrix<real>> Ks)
// {
// 	FILE * fout;
// 	fout = fopen(filename, "w");
// 
// 	//char formatString[6];
// 	fprintf(fout, "%d\n", Ks.size());
// 	for (int t = 0; t < Ks.size(); t++){
// 		fprintf(fout, "%d\n", t);	
// 		for (int i = 0; i < Ks[t].Getm(); i++)
// 		{
// 			for (int j = 0; j < Ks[t].Getn(); j++)
// 				fprintf(fout, "%.15G,", (Ks[t])(i, j));
// 			fprintf(fout, "\n");
// 		}
// 	}
// 	fclose(fout);
// }


string S2V_Message_Data_to_string(S2V_Message_Data &s2v_message_data);

void S2V_Message_Data_to_Real_Time_Sensing_Data(vector<RealTimeInVehicleSensingData> &real_time_sensing_data_vector, S2V_Message_Data &s2v_message_data, map<int, V2S_Message_Data> & V2S_message_vector);

bool ControlLeadingCAV(
	ampl::AMPL &ampl_obj,
	double lead_CAV_speed, 
	double front_veh_speed,
	double front_veh_spacing,
	std::vector<double> &vehicle_init_speed,
	std::vector<double> &spacing_init_vec,
	std::vector<double> &vehicle_init_acc,
	std::vector<double> &vehicle_length);

int ControlVehicle();

void avoid_collision_acc(double hwdy);


double GetDesiredAccCadilac();

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
	std::vector<double> &vehicle_init_speed,
	std::vector<double> &spacing_init_vec,
	std::vector<double> &vehicle_init_acc,
	std::vector<double> &vehicle_length,
	double approximated_likelihood_threshold,
	std::vector<double> &control_acceleration);

double CACC_Car_Following(long lvid0, double a0, double v0, double leng0, long lvid1, double a1, double v1, double leng1, double d01,
	double t_system);
int InitArrays();

double GetLateralPos(double lateralpos); 

double GetAdjHeadway(double v, 
					 double lv, 
					 double hwy);


enum empties_t { empties_ok, no_empties };
template <typename Container>
Container& string_split(
	Container&                            result,
	const typename Container::value_type& s,
	const typename Container::value_type& delimiters,
	empties_t                      empties = empties_ok)
{
	result.clear();
	size_t current;
	size_t next = -1;
	do
	{
		if (empties == no_empties)
		{
			next = s.find_first_not_of(delimiters, next + 1);
			if (next == Container::value_type::npos) break;
			next -= 1;
		}
		current = next + 1;
		next = s.find_first_of(delimiters, current);
		result.push_back(s.substr(current, next - current));
	} while (next != Container::value_type::npos);
	return result;
};
bool g_get_line_intersection(float Ax, float Ay,
float Bx, float By,
float Cx, float Cy,
float Dx, float Dy,
float *X, float *Y) {

	double  distAB, theCos, theSin, newX, ABpos;

	//  Fail if either line segment is zero-length.
	//  if (Ax==Bx && Ay==By || Cx==Dx && Cy==Dy) return false;
	if (Ax == Bx && Ay == By) return false;  // comment: C and D can be the same point from a vehile with the same timestamp

	//  Fail if the segments share an end-point.
	if (Ax == Cx && Ay == Cy || Bx == Cx && By == Cy
		|| Ax == Dx && Ay == Dy || Bx == Dx && By == Dy) {
		return false;
	}

	//  (1) Translate the system so that point A is on the origin.
	Bx -= Ax; By -= Ay;
	Cx -= Ax; Cy -= Ay;
	Dx -= Ax; Dy -= Ay;

	//  Discover the length of segment A-B.
	distAB = sqrt(Bx*Bx + By*By);

	//  (2) Rotate the system so that point B is on the positive X axis.
	theCos = Bx / distAB;
	theSin = By / distAB;
	newX = Cx*theCos + Cy*theSin;
	Cy = Cy*theCos - Cx*theSin; Cx = newX;
	newX = Dx*theCos + Dy*theSin;
	Dy = Dy*theCos - Dx*theSin; Dx = newX;

	//  Fail if segment C-D doesn't cross line A-B.
	if (Cy < 0. && Dy < 0. || Cy >= 0. && Dy >= 0.) return false;

	//  (3) Discover the position of the intersection point along line A-B.
	ABpos = Dx + (Cx - Dx)*Dy / (Dy - Cy);

	//  Fail if segment C-D crosses line A-B outside of segment A-B.
	if (ABpos<0. || ABpos>distAB) return false;

	//  (4) Apply the discovered position to line A-B in the original coordinate system.
	*X = Ax + ABpos*theCos;
	*Y = Ay + ABpos*theSin;

	//  Success.
	return true;
};
int CountVehicles(int sensor_length, int ilink, int SelectedStartTime, int SelectedEndTime, float SelectedStartLocalY, float SelectedEndLocalY, float &time_mean_spd, int &num_spd);
void construct_density_profile_from_space_time_region(vector<double>& speed_vec, vector<double> &density_vec);
void predict_vehicle_trajectory(CarFollowingParam &car_following_param, vector<vector<VehicleState>> &t_vno_predicted_vehicle_trajectory);
void predict_vehicle_trajectory(map<int,CarFollowingParam> &car_following_param_vec, vector<vector<VehicleState>> &t_vno_predicted_vehicle_trajectory);
void predict_vehicle_trajectory_reference_stochastic(map<int, CarFollowingParam> &car_following_param_vec, map<int,vector<vector<VehicleState>>> &scenarioidx_tidx_vno_predicted_vehicle_trajectory);

double calculate_jam_density(vector<double> density_vec, vector<double> spd_vec);
void least_square_calibrate_newell_parameters(map<int,CarFollowingParam> &heter_newell_parameters); // using 250 data points
void least_square_calculate(CarFollowingParam &param, vector<double> &spacing_vec, vector<double> &spd_vec);
void predictive_mpc_QP_control_data_update(vector<vector<VehicleState>> &t_vno_predicted_vehicle_trajectory,
	ampl::AMPL &ampl, SolverID solver, MPC_Initial_State & mpc_initial_state, vector<int> &unique_vehicle_no, MPC_Data &mpc_data, double nveh_loc);
void constant_mpc_QP_control_data_update(
	ampl::AMPL &ampl, SolverID solver, MPC_Initial_State & mpc_initial_state, vector<int> &unique_vehicle_no, MPC_Data &mpc_data, double nveh_loc, double nveh_spd);

void predictive_solve_MPC_QP(ampl::AMPL &ampl, MPC_Data &mpc_data);


DRIVERMODEL_API  int  DriverModelSetValue (long   type, 
                                           long   index1,
                                           long   index2,
                                           long   long_value,
                                           double double_value,
                                           char   *string_value);

/* Sets the value of a data object of type <type>, selected by <index1> */
/* and possibly <index2>, to <long_value>, <double_value> or            */
/* <*string_value> (object and value selection depending on <type>).    */
/* Return value is 1 on success, otherwise 0.                           */

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue (long   type, 
                                           long   index1,
                                           long   index2,
                                           long   *long_value,
                                           double *double_value,
                                           char   **string_value);

/* Gets the value of a data object of type <type>, selected by <index1> */
/* and possibly <index2>, and writes that value to <*double_value>,     */
/* <*float_value> or <**string_value> (object and value selection       */
/* depending on <type>).                                                */
/* Return value is 1 on success, otherwise 0.                           */

/*==========================================================================*/

#define  DRIVER_COMMAND_INIT            0
           /* called from VISSIM once at the start of a simulation run */
           /* values set before: DRIVER_DATA_PATH     */
           /*                    DRIVER_DATA_TIMESTEP */
           /*                    DRIVER_DATA_TIME     */
           /* values got after:  DRIVER_DATA_WANTS_SUGGESTION     */
           /*                    DRIVER_DATA_AUTOMATIC_LANECHANGE */

#define  DRIVER_COMMAND_CREATE_DRIVER   1
           /* called from VISSIM once per vehicle entering the network */
           /* values set before: DRIVER_DATA_VEH_ID               */
           /*                    DRIVER_DATA_VEH_DESIRED_VELOCITY */

#define  DRIVER_COMMAND_KILL_DRIVER     2
           /* called from VISSIM once per vehicle leaving the network */
           /* value set before: DRIVER_DATA_VEH_ID */

#define  DRIVER_COMMAND_MOVE_DRIVER     3
           /* called from VISSIM once per time step during a simulation run */
           /* values set before: all values                      */
           /*                    (driving behaviour data only if */
           /*                     DRIVER_DATA_WANTS_SUGGESTION)  */

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelExecuteCommand (long number);

/* Executes the command <number> if that is available in the driver */
/* module. Return value is 1 on success, otherwise 0.               */

/*==========================================================================*/

#endif /* __DRIVERMODEL_H */

/*==========================================================================*/
/*  Ende of DriverModel.h                                                   */
/*==========================================================================*/
