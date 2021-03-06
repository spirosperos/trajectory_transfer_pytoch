#pragma once
#ifndef _ROBOT_q_DIM_
	#define _ROBOT_q_DIM_ 9
#endif
#ifndef _Traj_N_Knot_
	#define _Traj_N_Knot_ 500
#endif
typedef char CHAR ;
typedef uint64_t ULONGLONG;
typedef uint32_t ULONG;
typedef int64_t LONGLONG;
typedef uint8_t BYTE;

///<AutoGeneratedContent id="DataTypes">
#if !defined(_TC_TYPE_D04B54F7_6D27_45C3_A767_BCBFE3E5DA18_INCLUDED_)
#define _TC_TYPE_D04B54F7_6D27_45C3_A767_BCBFE3E5DA18_INCLUDED_
typedef struct _acin_control_Pose_t
{
	double position[3];
	double orientation[4];
} acin_control_Pose_t, *Pacin_control_Pose_t;
#endif // !defined(_TC_TYPE_D04B54F7_6D27_45C3_A767_BCBFE3E5DA18_INCLUDED_)

#if !defined(_TC_TYPE_DD73AE19_D66C_424E_AE8D_0527DE2768F6_INCLUDED_)
#define _TC_TYPE_DD73AE19_D66C_424E_AE8D_0527DE2768F6_INCLUDED_
#pragma pack(push,1)
typedef struct _acin_control_TaskSpaceTraj_t
{
	acin_control_Pose_t PoseArray[30];
	double t_k[30];
	bool OrientationAsQaut;
} acin_control_TaskSpaceTraj_t, *Pacin_control_TaskSpaceTraj_t;
#pragma pack(pop)
#endif // !defined(_TC_TYPE_DD73AE19_D66C_424E_AE8D_0527DE2768F6_INCLUDED_)



#if !defined(_TC_TYPE_18071995_0000_0000_0000_00010000001F_INCLUDED_)
#define _TC_TYPE_18071995_0000_0000_0000_00010000001F_INCLUDED_
typedef CHAR STRING31[32];
#endif // !defined(_TC_TYPE_18071995_0000_0000_0000_00010000001F_INCLUDED_)

#if !defined(_TC_TYPE_8CC5F804_88C2_44A4_87C7_BAC13220AFBE_INCLUDED_)
#define _TC_TYPE_8CC5F804_88C2_44A4_87C7_BAC13220AFBE_INCLUDED_
#pragma pack(push,1)
typedef struct _acin_control_Header_t
{
	ULONG seq;
	double stamp;
	STRING31 frame_id;
} acin_control_Header_t, *Pacin_control_Header_t;
#pragma pack(pop)
#endif // !defined(_TC_TYPE_8CC5F804_88C2_44A4_87C7_BAC13220AFBE_INCLUDED_)

#if !defined(_TC_TYPE_9E49FE52_CBCF_4B94_A68F_EF6A287D3ACF_INCLUDED_)
#define _TC_TYPE_9E49FE52_CBCF_4B94_A68F_EF6A287D3ACF_INCLUDED_
#pragma pack(push,1)
typedef struct _acin_control_ToolParam_t
{
	acin_control_Header_t header;
	double m;
	double spx;
	double spy;
	double spz;
} acin_control_ToolParam_t, *Pacin_control_ToolParam_t;
#pragma pack(pop)
#endif // !defined(_TC_TYPE_9E49FE52_CBCF_4B94_A68F_EF6A287D3ACF_INCLUDED_)

#if !defined(_TC_TYPE_0875D3A1_2BD4_4A0F_926D_38D48831C264_INCLUDED_)
#define _TC_TYPE_0875D3A1_2BD4_4A0F_926D_38D48831C264_INCLUDED_
#pragma pack(push,1)
typedef struct _acin_control_BoolMsg_t
{
	bool data;
} acin_control_BoolMsg_t, *Pacin_control_BoolMsg_t;
#pragma pack(pop)
#endif // !defined(_TC_TYPE_0875D3A1_2BD4_4A0F_926D_38D48831C264_INCLUDED_)

#if !defined(_TC_TYPE_6D76E062_7BF2_4C2B_A0FC_4236BA10A4F4_INCLUDED_)
#define _TC_TYPE_6D76E062_7BF2_4C2B_A0FC_4236BA10A4F4_INCLUDED_
typedef struct _acin_control_JointState_t
{
	double position[7];
	double velocity[7];
	double effort[7];
} acin_control_JointState_t, *Pacin_control_JointState_t;
#endif // !defined(_TC_TYPE_6D76E062_7BF2_4C2B_A0FC_4236BA10A4F4_INCLUDED_)

#if !defined(_TC_TYPE_591D78E2_9DC7_4099_AB21_9F813062F487_INCLUDED_)
#define _TC_TYPE_591D78E2_9DC7_4099_AB21_9F813062F487_INCLUDED_
typedef struct _acin_control_calcTimeMeasurement_t
{
	double t_TS_ctrl;
	double t_QS_ctrl;
	double t_TS_intpl;
	double t_QS_intpl;
	STRING31 unit;
} acin_control_calcTimeMeasurement_t, *Pacin_control_calcTimeMeasurement_t;
#endif // !defined(_TC_TYPE_591D78E2_9DC7_4099_AB21_9F813062F487_INCLUDED_)

#if !defined(_TC_TYPE_C548D0DB_E9E5_4009_A9B6_A02882E5E6D3_INCLUDED_)
#define _TC_TYPE_C548D0DB_E9E5_4009_A9B6_A02882E5E6D3_INCLUDED_
typedef struct _acin_control_Joints_t
{
	double Joints[7];
} acin_control_Joints_t, *Pacin_control_Joints_t;
#endif // !defined(_TC_TYPE_C548D0DB_E9E5_4009_A9B6_A02882E5E6D3_INCLUDED_)

#if !defined(_TC_TYPE_24581482_F86D_45DC_BCEB_9EA617C0D40E_INCLUDED_)
#define _TC_TYPE_24581482_F86D_45DC_BCEB_9EA617C0D40E_INCLUDED_
#pragma pack(push,1)
typedef struct _acin_control_JointSpaceTraj_t
{
	acin_control_Header_t header;
	acin_control_Joints_t JointsArray[30];
	double t_k[30];
} acin_control_JointSpaceTraj_t, *Pacin_control_JointSpaceTraj_t;
#pragma pack(pop)
#endif // !defined(_TC_TYPE_24581482_F86D_45DC_BCEB_9EA617C0D40E_INCLUDED_)

#if !defined(_TC_TYPE_C5F4B933_B9FA_4F8F_AAC9_5087D5B78D21_INCLUDED_)
#define _TC_TYPE_C5F4B933_B9FA_4F8F_AAC9_5087D5B78D21_INCLUDED_
#pragma pack(push,1)
typedef struct _acin_control_CartPoint_t
{
	double x;
	double y;
	double z;
} acin_control_CartPoint_t, *Pacin_control_CartPoint_t;
#pragma pack(pop)
#endif // !defined(_TC_TYPE_C5F4B933_B9FA_4F8F_AAC9_5087D5B78D21_INCLUDED_)

#if !defined(_TC_TYPE_631825C5_3F38_49B0_A39F_DF333519728C_INCLUDED_)
#define _TC_TYPE_631825C5_3F38_49B0_A39F_DF333519728C_INCLUDED_
#pragma pack(push,1)
typedef struct _acin_control_PoseState_t
{
	acin_control_CartPoint_t p_vel;
	acin_control_CartPoint_t omega;
	acin_control_CartPoint_t p_accl;
	acin_control_CartPoint_t omegaDot;
	acin_control_Pose_t pose;
	acin_control_Header_t header;
} acin_control_PoseState_t, *Pacin_control_PoseState_t;
#pragma pack(pop)
#endif // !defined(_TC_TYPE_631825C5_3F38_49B0_A39F_DF333519728C_INCLUDED_)

#if !defined(_TC_TYPE_18071995_0000_0000_0000_000100000100_INCLUDED_)
#define _TC_TYPE_18071995_0000_0000_0000_000100000100_INCLUDED_
typedef CHAR STRING256[257];
#endif // !defined(_TC_TYPE_18071995_0000_0000_0000_000100000100_INCLUDED_)


#if !defined(_TC_TYPE_BB0B666D_584A_48C3_8067_6318F9A4A2AC_INCLUDED_)
#define _TC_TYPE_BB0B666D_584A_48C3_8067_6318F9A4A2AC_INCLUDED_
#pragma pack(push,1)
typedef struct _acin_control_CoordAdj_t
{
	double adj_val;
	double T_traj;
	double acc_perc;
	bool TaskSpace;
	ULONGLONG coord_idx;
} acin_control_CoordAdj_t, *Pacin_control_CoordAdj_t;
#pragma pack(pop)
#endif // !defined(_TC_TYPE_BB0B666D_584A_48C3_8067_6318F9A4A2AC_INCLUDED_)

#if !defined(_TC_TYPE_4B9DD431_30BF_47BD_9DBD_479109B36C74_INCLUDED_)
#define _TC_TYPE_4B9DD431_30BF_47BD_9DBD_479109B36C74_INCLUDED_
#pragma pack(push,1)
typedef struct _acin_control_acinSollPos_t
{
	bool coord[7];
	double sollPos[7];
	double T_traj;
	double acc_perc;
	bool TaskSpace;
	ULONG planAlgoSel;
} acin_control_acinSollPos_t, *Pacin_control_acinSollPos_t;
#pragma pack(pop)
#endif // !defined(_TC_TYPE_4B9DD431_30BF_47BD_9DBD_479109B36C74_INCLUDED_)

#pragma pack(push,1)
typedef struct _ct_controller_twinCatads_Inputs
{
	acin_control_TaskSpaceTraj_t TaskSpaceTraj_d;
	acin_control_JointState_t JointState_actual;
	acin_control_BoolMsg_t GravityMode;
	acin_control_CoordAdj_t CoordAdjustment;
	acin_control_JointSpaceTraj_t JointSpaceTraj_d;
	acin_control_JointState_t JointsNS_d;
	acin_control_ToolParam_t ToolParam;
	acin_control_BoolMsg_t motion_EN_in;
	acin_control_Joints_t Tau_FB_m;
	acin_control_BoolMsg_t FrictionComp;
	acin_control_acinSollPos_t acinSollPos;
	acin_control_Joints_t motor_enable_in;
	acin_control_BoolMsg_t SingularPert;
} ct_controller_twinCatads_Inputs, *Pct_controller_twinCatads_Inputs;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct _ct_controller_twinCatads_Outputs
{
	acin_control_Joints_t motor_enable_out;
	acin_control_Joints_t Tau_ctrl;
	acin_control_PoseState_t PoseTraj_desired;
	acin_control_JointState_t JointSpace_error;
	acin_control_PoseState_t TaskSpace_error;
	acin_control_PoseState_t PoseTraj_actuel;
	bool motion_EN_out;
} ct_controller_twinCatads_Outputs, *Pct_controller_twinCatads_Outputs;
#pragma pack(pop)


#pragma pack(push,1)
typedef struct _ct_controller_twinCatads_InputChangedFlags
{
	bool JointState_a_flag;
	bool GravityMode_flag;
	bool TaskSpaceTraj_d_flag;
	bool CoordAdjustment_flag;
	bool JointSpaceTraj_d_flag;
	bool JointNS_d_flag;
	bool ToolParam_flag;
	bool acinSollPos_flag;
} ct_controller_twinCatads_InputChangedFlags, *Pct_controller_twinCatads_InputChangedFlags;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct _ct_controller_twinCatads_DebugOut
{
	ULONGLONG tms_diff;
	acin_control_Joints_t Tau_filter;
	acin_control_Joints_t Tau_p_filter;
	acin_control_Joints_t Tau_CT;
	acin_control_Joints_t Tau_friction;
	acin_control_Joints_t Tau_SP;
	acin_control_Joints_t q_p_filter;
} ct_controller_twinCatads_DebugOut, *Pct_controller_twinCatads_DebugOut;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct _ct_controller_twinCatrobot_interface_out
{
	bool motionDisable;
	bool motionEnable;
	acin_control_Joints_t Tau_ctrl;
	acin_control_Joints_t motor_enable_out;
} ct_controller_twinCatrobot_interface_out, *Pct_controller_twinCatrobot_interface_out;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct _ct_controller_twinCatrobot_interface_in
{
	acin_control_JointState_t JointState_actual;
	acin_control_Joints_t Tau_FB_m;
	double robotState;
	double motionEnabled;
} ct_controller_twinCatrobot_interface_in, *Pct_controller_twinCatrobot_interface_in;
#pragma pack(pop)
///<AutoGeneratedContent id="EventClasses">
///</AutoGeneratedContent>
