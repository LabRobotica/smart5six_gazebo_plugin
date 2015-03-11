/**
 * @file eORL.h
 * @author  Fabrizio Romanelli <fabrizio.romanelli@comau.com>
 * @version 1.0
 * @date 24.01.2013
 *
 * @section LICENSE
 *    This  material  is the exclusive property of Comau S.p.A.  and must be
 *    returned   to   Comau   S.p.A.,   Robotics  Division,  Software  Group
 *    immediately   upon   request.    This  material  and  the  information
 *    illustrated or contained herein may not be used, reproduced, stored in
 *    a retrieval system, or transmitted in whole or in part in  any  way  -
 *    electronic, mechanical, photocopying, recording, or otherwise, without
 *    the prior written consent of Comau S.p.A..
 *
 *                      All Rights Reserved
 *                      Copyright (C)  2014
 *                          Comau S.p.A.
 *
 * @section DESCRIPTION
 *
 * This module contains the Open realistic Robot Library defines and prototypes
 * to be used in the custom user program.
 *
 * @section HISTORY
 *
 *  31.05.2013  FR First release from C5G ORL\n
 *
 */

#ifndef __EORL_H
#define __EORL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Controllers labels */
#define ORL_CNTRL01           0
#define ORL_CNTRL02           1
#define ORL_CNTRL03           2
#define ORL_CNTRL04           3
#define ORL_CNTRL05           4
#define ORL_CNTRL06           5
#define ORL_CNTRL07           6
#define ORL_CNTRL08           7
#define ORL_CNTRL09           8
#define ORL_CNTRL10           9
#define ORL_CNTRL11          10
#define ORL_CNTRL12          11
#define ORL_CNTRL13          12
#define ORL_CNTRL14          13
#define ORL_CNTRL15          14
#define ORL_CNTRL16          15
#define ORL_CNTRL17          16
#define ORL_CNTRL18          17
#define ORL_CNTRL19          18
#define ORL_CNTRL20          19
#define ORL_CNTRL21          20
#define ORL_CNTRL22          21
#define ORL_CNTRL23          22
#define ORL_CNTRL24          23
#define ORL_CNTRL25          24
#define ORL_CNTRL26          25
#define ORL_CNTRL27          26
#define ORL_CNTRL28          27
#define ORL_CNTRL29          28
#define ORL_CNTRL30          29
#define ORL_CNTRL31          30
#define ORL_CNTRL32          31
#define ORL_MAX_CNTRLS       32

/* Arms labels */
#define ORL_ARM1            0
#define ORL_ARM2            1
#define ORL_ARM3            2
#define ORL_ARM4            3
#define ORL_MAX_ARMS        4

/* Axes labels */
#define ORL_AX1            0
#define ORL_AX2            1
#define ORL_AX3            2
#define ORL_AX4            3
#define ORL_AX5            4
#define ORL_AX6            5
#define ORL_AX7            6
#define ORL_AX8            7
#define ORL_AX9            8
#define ORL_AX10           9
#define ORL_MAX_AXIS      10

/* fly types */
#define ORL_FLY_NORMAL     0
/*VF 24/01/2014 Deprecated #define ORL_FLY_HIGH       1*/
#define ORL_FLY_CART       2
#define ORL_FLY_AUTO       3
#define ORL_FLY_PASS       4
#define ORL_FLY_TOL        5
#define ORL_FLY_FROM       6

/* fly parameters */
#define ORL_FLY_PER        7
#define ORL_FLY_DIST       8

/* interpolation type */
#define ORL_TRJNT          1
#define ORL_TRCARLIN       2
#define ORL_TRCARCIR       4

/* Dominant interpolation */
#define ORL_SPD_LIN        1
#define ORL_SPD_JNT        3
#define ORL_SPD_CST        4
#define ORL_SPD_ORNT       2
/* Dominant interpolation under ORL_SPD_ORNT */
#define ORL_SPD_ROT        1
#define ORL_SPD_SPN        2
#define ORL_SPD_AZI        3
#define ORL_SPD_ELV        4
#define ORL_SPD_RLL        5
#define ORL_SPD_FST        6
#define ORL_SPD_SND        7
#define ORL_SPD_THR        8
#define ORL_SPD_AUX1       9
#define ORL_SPD_AUX2      10
#define ORL_SPD_AUX3      11

/* Orientation interpolation modes */
#define ORL_ORT_RS_WORLD   2
#define ORL_ORT_EUL_WORLD  3
#define ORL_ORT_WRIST_JNT  4
#define ORL_ORT_TRAJ       6

/* Fly/no Fly mode */
#define ORL_NO_FLY         0
#define ORL_FLY            1

/* Wait/no wait mode */
#define ORL_WAIT           0
#define ORL_ADVANCE        1

/* term types */
#define ORL_TOL_FINE       0
#define ORL_TOL_COARSE     1
#define ORL_TOL_NOSETTLE   2
#define ORL_TOL_JNT_FINE   3
#define ORL_TOL_JNT_COARSE 4

/* verbosity level */
#define ORL_SILENT         0
#define ORL_VERBOSE        1

/* Booleans */
#define ORL_FALSE          0
#define ORL_TRUE           1
#define ORL_OFF            0
#define ORL_ON             1

/* Cycle times */
#define ORL_0_4_MILLIS        0
#define ORL_2_0_MILLIS        1
#define ORL_4_0_MILLIS        2
#define ORL_8_0_MILLIS        3
#define ORL_16_0_MILLIS       4

/*Open Modality*/
#define CRCOPEN_LISTEN                      0
#define CRCOPEN_POS_ABSOLUTE                4
#define CRCOPEN_POS_RELATIVE                5
#define CRCOPEN_POS_ADDITIVE                7
#define CRCOPEN_POS_ADDITIVE_SB            17
#define CRCOPEN_POS_ADDITIVE_SBE           170

/*Status Controller*/
#define CRCOPEN_STS_DRIVEOFF                  1    /* stato dell'Amodulo : situazione di DRIVE   OFF */
#define CRCOPEN_STS_DRIVINGON                 2    /* stato dell'Amodulo : situazione di DRIVING ON  */
#define CRCOPEN_STS_DRIVINGOFF                3    /* stato dell'Amodulo : situazione di DRIVING OFF */
#define CRCOPEN_STS_DRIVEON                   4    /* stato dell'Amodulo : situazione di DRIVE   ON  */


#define ORLOPEN_RES_OK      0
#define ORLOPEN_ERROR       -1
#define PWL_ACTIVE      1
#define PWL_NOT_ACTIVE  0
#define PWL_ERROR      -1
/*
 *  Typedefs
 */

typedef int (*ORLOPEN_callback) (int i);

typedef enum {
  ORL_POSITION_MOTORROUNDS    = 1,
  ORL_POSITION_LINK_RAD       = 2,
  ORL_POSITION_LINK_DEGREE    = 3,
  ORL_SPEED                   =10,
  ORL_SPEED_MOTORROUNDS       =11,
  ORL_SPEED_MOTOR_RAD_SEC     =12,
  ORL_SPEED_MOTOR_DEGREE_SEC  =13,
  ORL_SPEED_LINK_RAD_SEC      =14,
  ORL_SPEED_LINK_DEGREE_SEC   =15,
  ORL_ACC                     =20,
  ORL_ACC_MOTORROUNDS         =21,
  ORL_ACC_MOTOR_RAD_SEC2      =22,
  ORL_ACC_MOTOR_DEGREE_SEC2   =23,
  ORL_ACC_LINK_RAD_SEC2       =24,
  ORL_ACC_LINK_DEGREE_SEC2    =25,
  ORL_TAU_MOTOR_NM            =30,
  ORL_TAU_LINK_NM             =31,
  ORL_CURRENT_AMPERES         =40,
  ORL_CART_POSITION           =50,
  ORL_CART_SPEED              =51,
  ORL_TO_DEFINE               =60
} unit_enum;

typedef struct carpos_val
{
  unit_enum   unit_type;
  double  x, y, z,
          a, e, r;
  char    config_flags[80];
} ORL_cartesian_position;

typedef struct jnt_value
{
  unit_enum   unit_type;
  double  value[ORL_MAX_AXIS];
} ORL_joint_value;

typedef struct Jacobian_val
{
  double element[ORL_MAX_AXIS][ORL_MAX_AXIS];
} ORL_Jacobian;

typedef struct DynMod_val
{
  double tau_link[ORL_MAX_AXIS];
  double tau_motor[ORL_MAX_AXIS];
  double tau_frict[ORL_MAX_AXIS];
  double current[ORL_MAX_AXIS];
} ORL_Dynamic_Model;

typedef struct DynModFull_val
{
  double       ag_tau_inerziapropriaMOT[ORL_MAX_AXIS];
  double       ag_tau_inerziapropriaRNE[ORL_MAX_AXIS];
  double       ag_tau_inerziaOTHER[ORL_MAX_AXIS];  /*somma della riga*/
  double       ag_tau_coriolis[ORL_MAX_AXIS];
  double       ag_tau_gravitazionale[ORL_MAX_AXIS];
  double       ag_tau_bilanciamento[ORL_MAX_AXIS];
  double       ag_tau_attritomotorenonlineare[ORL_MAX_AXIS];
  double       ag_tau_attritomotorelineare[ORL_MAX_AXIS];
  double       ag_tau_attritolinknonlineare[ORL_MAX_AXIS];
  double       ag_tau_attritolinklineare[ORL_MAX_AXIS];
  double       bg_B [ORL_MAX_AXIS][ORL_MAX_AXIS];
  double       ag_Jm[ORL_MAX_AXIS];
  double       ag_max_motor_torque[ORL_MAX_AXIS]; /*unused*/
} ORL_Dynamic_Model_Full;

typedef enum {ORL_INT=1, ORL_BOOL=2, ORL_REAL=3, ORL_STRING=4} type_enum;

typedef struct System_Variable
{
  type_enum ctype;
  int       permission;
  int       iv;
  double    dv;
  char      sv[255];
  char      sysvar_name[80];
} ORL_System_Variable;


/*
 *  Prototypes for Motion functions
 */

int ORL_initialize_controller       (const char*,               /* [IN]      C5G Configuration filename */
                                    const char*,                /* [IN]      Directory */
                                    int,                        /* [IN]      Verbose ON/OFF */
                                    int                         /* [IN]      Controller Index */);

int ORL_terminate_controller        (int,                        /* [IN]      Verbose ON/OFF */ 
                                     int                         /* [IN]      Controller Index */);

int ORL_initialize_frames           (ORL_cartesian_position,     /* [IN]      $BASE */
                                     ORL_cartesian_position,     /* [IN]      $TOOL */
                                     ORL_cartesian_position,     /* [IN]      $UFRAME */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_select_point_accuracy       (int,                        /* [IN]      Accuracy type */ 
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_set_interpolation_time      (double,                     /* [IN]      Interpolation period [ms] */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_get_next_interpolation_step (ORL_joint_value *           /* [OUT]     Interpolated joint position */,
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_set_position                (ORL_cartesian_position *,   /* [IN]      Initial Cartesian position (leave NULL if using Joint position) */ 
                                     ORL_joint_value *,          /* [IN]      Initial Joint position (leave NULL if using Cartesian position) */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_inverse_kinematics          (ORL_cartesian_position *,   /* [IN]      Cartesian position to be converted into Joint position */
                                     ORL_joint_value *,          /* [OUT]     Joint position converted from Cartesian position */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_direct_kinematics           (ORL_cartesian_position *,   /* [OUT]     Cartesian position converted from Joint position */
                                     ORL_joint_value *,          /* [IN]      Joint position to be converted into Cartesian position */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_set_move_parameters         (int,                        /* [IN]      Fly flag ORL_FLY/ORL_NO_FLY */
                                     int,                        /* [IN]      Advance or Wait clause ORL_ADVANCE/ORL_WAIT */
                                     int,                        /* [IN]      Fly type */
                                     int,                        /* [IN]      Motion interpolation type ORL_TRJNT/ORL_TRCARLIN/ORL_TRCARCIR */
                                     ORL_cartesian_position *,   /* [IN]      Move End-point Cartesian position (leave NULL if using Joint position) */
                                     ORL_joint_value *,          /* [IN]      Move End-point Joint position (leave NULL if using Cartesian position) */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_axis_2_joints               (ORL_joint_value *,          /* [IN/OUT]  From robot motor axis [rounds] to joint values [degrees] */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_joints_2_axis               (ORL_joint_value *,          /* [IN/OUT]  From joint values [degrees] to robot motor axis [rounds] */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_joints_conversion           (ORL_joint_value *,          /* [IN/OUT]  */
                                     unit_enum,                  /* [IN]      */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_set_data                    (ORL_System_Variable,        /* [IN]      Structure for the system variable */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int                         /* [IN]      Controller Index */);

int ORL_get_data                    (ORL_System_Variable *,      /* [OUT]     Value of the system variable */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int                         /* [IN]      Controller Index */);

int ORL_dominant_interpolation      (int,                        /* [IN]      Set speed option parameter */
                                     int,                        /* [IN]      Set speed option orientation parameter */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_orientation_interpolation   (int,                        /* [IN]      Set orientation interpolation mode parameter */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_get_jacobian                (ORL_joint_value *,          /* [IN]      Joint position from which compute Jacobian geometric matrix */
                                     double pg_matB[4][4],       /* [IN]      Base */
                                     double pg_matT[4][4],       /* [IN]      Tool */
                                     int frame_flag,             /* [IN]      Frame Flag, If 1 Base and Tool provided by the user, instead the internal values of Base and Tool */
                                     ORL_Jacobian *,             /* [OUT]     Jacobian geometric matrix */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_direct_kine_speed           (ORL_joint_value * ,         /* [IN]      Joint position from which compute Jacobian geometric matrix */
                                     double pg_matB[4][4],       /* [IN]      Base */
                                     double pg_matT[4][4],       /* [IN]      Tool */
                                     int frame_flag,             /* [IN]      Frame Flag, If 1 Base and Tool provided by the user, instead the internal values of Base and Tool */
                                     ORL_joint_value * px_joint_speed,  /* [IN]      Joint Speed*/
                                     ORL_cartesian_position * px_cart_speed,   /* [OUT]     Cart Speed*/
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);
int ORL_inverse_kine_speed           (ORL_joint_value * ,        /* [IN]      Joint position from which compute Jacobian geometric matrix */
                                     double pg_matB[4][4],       /* [IN]      Base */
                                     double pg_matT[4][4],       /* [IN]      Tool */
                                     int frame_flag,             /* [IN]      Frame Flag, If 1 Base and Tool provided by the user, instead the internal values of Base and Tool */
                                     ORL_cartesian_position * px_cart_speed,   /* [IN]      Cart Speed*/
                                     ORL_joint_value * px_joint_speed,  /* [OUT]     Joint Speed*/
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);
int ORL_direct_dyn_torque           (ORL_joint_value * ,         /* [IN]      Joint position from which compute Jacobian geometric matrix */
                                     double pg_matB[4][4],       /* [IN]      Base */
                                     double pg_matT[4][4],       /* [IN]      Tool */
                                     int frame_flag,             /* [IN]      Frame Flag, If 1 Base and Tool provided by the user, instead the internal values of Base and Tool */
                                     ORL_joint_value * px_tau_input,    /* [IN]      */
                                     ORL_joint_value * px_tau_output,   /* [OUT]     */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);
int ORL_inverse_dyn_torque           (ORL_joint_value * ,        /* [IN]      Joint position from which compute Jacobian geometric matrix */
                                     double pg_matB[4][4],       /* [IN]      Base */
                                     double pg_matT[4][4],       /* [IN]      Tool */
                                     int frame_flag,             /* [IN]      Frame Flag, If 1 Base and Tool provided by the user, instead the internal values of Base and Tool */
                                     ORL_joint_value * px_tau_input,    /* [IN]      */
                                     ORL_joint_value * px_tau_output,   /* [OUT]     */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);
									 
int ORL_get_dynamic_model           (ORL_joint_value *,          /* [IN]      Joint position from which compute the Dynamic Model, if NULL it will be use zero(10) */
                                     ORL_joint_value *,          /* [IN]      Joint Speed from which compute the Dynamic Model, if NULL it will be use zero(10) */
                                     ORL_joint_value *,          /* [IN]      Joint Acceleration from which compute the Dynamic Model, if NULL it will be use zero(10) */
                                     ORL_Dynamic_Model *,        /* [OUT]     Dynamic Model */
                                     int,                        /* [IN]      INLT Flag, If 1 use the NextInterpolationStep output, instead the previous input parameters */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_get_dynamic_model_full      (ORL_joint_value *,          /* [IN]      Joint position from which compute the Dynamic Model, if NULL it will be use zero(10) */
                                     ORL_joint_value *,          /* [IN]      Joint Speed from which compute the Dynamic Model, if NULL it will be use zero(10) */
                                     ORL_joint_value *,          /* [IN]      Joint Acceleration from which compute the Dynamic Model, if NULL it will be use zero(10) */
                                     ORL_Dynamic_Model_Full *,   /* [OUT]     Dynamic Model */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_cancel_motion               (int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_stop_motion                 (int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_resume_motion               (int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);

int ORL_set_fly_parameter           (int,                        /* [IN]      Set the fly type parameter, should be ORL_FLY_PER/ORL_FLY_DIST */
                                     double,                     /* [IN]      Set the fly parameter value */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);



/*
 *  Prototypes for ORL CRC OPEN functions
 */

int ORLOPEN_initialize_controller  (const char*,                /* [IN]      C5G target IP */
                                    const char*,                /* [IN]      C5G controller ID */
                                    int,                        /* [IN]      Verbose ON/OFF */
                                    int                         /* [IN]      Controller Index */);

int ORLOPEN_StartCommunication            ( int,                        /* [IN]      Verbose ON/OFF */
                                            int                         /* [IN]      Controller Index */);

int ORLOPEN_StopCommunication             ( int,                        /* [IN]      Verbose ON/OFF */
                                            int                         /* [IN]      Controller Index */);

int ORLOPEN_GetPowerlinkState             ( int                        /* [IN]      Verbose ON/OFF */);



int ORLOPEN_set_period              (int,                        /* [IN]      Global cycle time [ms], must be 0.4, 2, 4, 8, 16 */
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int                         /* [IN]      Controller Index */);


int ORLOPEN_SetCallBackFunction          ( ORLOPEN_callback ,          /* [IN]      callback function */
                                            int,                        /* [IN]      Verbose ON/OFF */
                                            int                         /* [IN]      Controller Index */);



int ORLOPEN_sync_position           (ORL_joint_value *px_j_pos,
                                     int,                        /* [IN]      Verbose ON/OFF */
                                     int,                        /* [IN]      Controller Index */
                                     int                         /* [IN]      Arm Index */);


/*Get from received crcopen message*/
int  ORLOPEN_GetModeAx          (int,                        /* [IN]      Verbose ON/OFF */
                                 int,                        /* [IN]      Controller Index */
                                 int,                        /* [IN]      Arm Index */
                                 int                         /* [IN]      Ax Index */);
int  ORLOPEN_GetModeMasterAx    (int,                        /* [IN]      Verbose ON/OFF */
                                 int,                        /* [IN]      Controller Index */
                                 int                         /* [IN]      Arm Index */);
int  ORLOPEN_GetOpenMask        (int,                        /* [IN]      Verbose ON/OFF */
                                 int,                        /* [IN]      Controller Index */
                                 int                         /* [IN]      Arm Index */);

int ORLOPEN_GetStatusMasterAx   (int,                         /* [IN]      Verbose ON/OFF */
                                 int,                         /* [IN]      Controller Index */
                                 int                          /* [IN]      Arm Index */);

int  ORLOPEN_get_pos_target_mr          (ORL_joint_value *,                  /* [OUT]     Position Target from CRCOPEN Message [Axis Motor Round] */
                                         long *,                     /* [IN/OUT]  Mask Links */
                                         int,                        /* [IN]      Step Old */
                                         int,                        /* [IN]      Verbose ON/OFF */
                                         int,                        /* [IN]      Controller Index */
                                         int                         /* [IN]      Arm Index */);
int  ORLOPEN_get_speed_target_mrpm      (ORL_joint_value *,                  /* [OUT]     Speed Target from CRCOPEN Message [Axis Motor Round Per Minute] */
                                         long *,                     /* [IN/OUT]  Mask Links */
                                         int,                        /* [IN]      Step Old */
                                         int,                        /* [IN]      Verbose ON/OFF */
                                         int,                        /* [IN]      Controller Index */
                                         int                         /* [IN]      Arm Index */);
int  ORLOPEN_get_pos_measured_mr        (ORL_joint_value *,                  /* [OUT]     Measured Position from CRCOPEN Message [Axis Motor Round] */
                                         long *,                     /* [IN/OUT]  Mask Links */
                                         int,                        /* [IN]      Step Old */
                                         int,                        /* [IN]      Verbose ON/OFF */
                                         int,                        /* [IN]      Controller Index */
                                         int                         /* [IN]      Arm Index */);
int  ORLOPEN_get_speed_measured_mrpm    (ORL_joint_value *,                  /* [OUT]     Measured Speed from CRCOPEN Message [Axis Motor Round Per Minute] */
                                         long *,                     /* [IN/OUT]  Mask Links */
                                         int,                        /* [IN]      Step Old */
                                         int,                        /* [IN]      Verbose ON/OFF */
                                         int,                        /* [IN]      Controller Index */
                                         int                         /* [IN]      Arm Index */);
int  ORLOPEN_get_current_measured       (ORL_joint_value *,                  /* [OUT]     Mesured Position from CRCOPEN Message [Axis Motor Round] */
                                         long *,                     /* [IN/OUT]  Mask Links */
                                         int,                        /* [IN]      Step Old */
                                         int,                        /* [IN]      Verbose ON/OFF */
                                         int,                        /* [IN]      Controller Index */
                                         int                         /* [IN]      Arm Index */);
int  ORLOPEN_get_current_dyn            (ORL_joint_value *,                  /* [OUT]     Current from CRCOPEN Message */
                                         long *,                     /* [IN/OUT]  Mask Links */
                                         int,                        /* [IN]      Step Old */
                                         int,                        /* [IN]      Verbose ON/OFF */
                                         int,                        /* [IN]      Controller Index */
                                         int                         /* [IN]      Arm Index */);
int  ORLOPEN_get_inertia_dyn            (ORL_joint_value *,                  /* [OUT]     Inertia Values from CRCOPEN Message */
                                         long *,                     /* [IN/OUT]  Mask Links */
                                         int,                        /* [IN]      Step Old */
                                         int,                        /* [IN]      Verbose ON/OFF */
                                         int,                        /* [IN]      Controller Index */
                                         int                         /* [IN]      Arm Index */);
int  ORLOPEN_get_ExtData                (ORL_joint_value*,           /* [OUT]     EXT_DATA */
                                         long *,                     /* [IN/OUT]  Mask Links */
                                         int,                        /* [IN]      Step Old */
                                         int,                        /* [IN]      Verbose ON/OFF */
                                         int,                        /* [IN]      Controller Index */
                                         int                         /* [IN]      Arm Index */);

int  ORLOPEN_set_ExtData               (ORL_joint_value *,          /* [IN]      EXT_DATA */
                                        long *,                     /* [IN/OUT]  Mask Links */
                                        int,                        /* [IN]      Verbose ON/OFF */
                                        int,                        /* [IN]      Controller Index */
                                        int                         /* [IN]      Arm Index */);
										
/*Get from received crcopen message*/
int  ORLOPEN_get_pos_measured(	ORL_joint_value *,       /* [OUT]     Joint position to be converted into Joint position */
                                ORL_cartesian_position *,   /* [OUT]     Cartesian position to be converted into Joint position */
                                int,                        /* [IN]      Step Old */
                                int,                        /* [IN]      Verbose ON/OFF */
                                int,                        /* [IN]      Controller Index */
                                int                         /* [IN]      Arm Index */);

int  ORLOPEN_get_pos_target(    ORL_joint_value *,       /* [OUT]     Joint position to be converted into Joint position */
                                ORL_cartesian_position *,   /* [OUT]     Cartesian position to be converted into Joint position */
                                int,                        /* [IN]      Step Old */
                                int,                        /* [IN]      Verbose ON/OFF */
                                int,                        /* [IN]      Controller Index */
                                int                         /* [IN]      Arm Index */);



/*Set new values in the next crcopen message*/
int  ORLOPEN_ExitFromOpen    ( int,                        /* [IN]      Verbose ON/OFF */
                               int,                        /* [IN]      Controller Index */
                               int                         /* [IN]      Arm Index */);

int  ORLOPEN_DriveOffFromPC  ( int,                        /* [IN]      Verbose ON/OFF */
                               int,                        /* [IN]      Controller Index */
                               int                         /* [IN]      Arm Index */);

int  ORLOPEN_FollowingError  ( int,                        /* [IN]      Verbose ON/OFF */
                               int,                        /* [IN]      Controller Index */
                               int                         /* [IN]      Arm Index */);



int  ORLOPEN_set_absolute_pos_target_mr    (ORL_joint_value *,          /* [IN]      Position Target for CRCOPEN Message MODALITY 4 [Axis Motor Round]*/
                                            long *,                     /* [IN/OUT]  Mask Links */
                                            int,                        /* [IN]      Verbose ON/OFF */
                                            int,                        /* [IN]      Controller Index */
                                            int                         /* [IN]      Arm Index */);
int  ORLOPEN_set_absolute_pos_target_degree    (ORL_joint_value *,          /* [IN]      Position Target for CRCOPEN Message MODALITY 4 [Degree]*/
                                                int,                        /* [IN]      Verbose ON/OFF */
                                                int,                        /* [IN]      Controller Index */
                                                int                         /* [IN]      Arm Index */);

int  ORLOPEN_set_relative_pos_target_mr    (ORL_joint_value *,          /* [IN]      Delta Position Target for CRCOPEN Message MODALITY 5 [Axis Motor Round]*/
                                            long *,                     /* [IN/OUT]  Mask Links */
                                            int,                        /* [IN]      Verbose ON/OFF */
                                            int,                        /* [IN]      Controller Index */
                                            int                         /* [IN]      Arm Index */);

int  ORLOPEN_set_relative_pos_target_degree    (ORL_joint_value *,          /* [IN]      Delta Position Target for CRCOPEN Message MODALITY 5 [Degree]*/
                                                int,                        /* [IN]      Verbose ON/OFF */
                                                int,                        /* [IN]      Controller Index */
                                                int                         /* [IN]      Arm Index */);

int  ORLOPEN_set_relative_pos_target_cartesian     (ORL_cartesian_position * ,  /* [IN]      Delta Position Target for CRCOPEN Message MODALITY 5 */
                                                    int,                        /* [IN]      Verbose ON/OFF */
                                                    int,                        /* [IN]      Controller Index */
                                                    int                         /* [IN]      Arm Index */);

int  ORLOPEN_set_additive_pos_target_mr    (ORL_joint_value *,          /* [IN]      Delta Position Target for CRCOPEN Message MODALITY 7 [Axis Motor Round]*/
                                            long *,                     /* [IN/OUT]  Mask Links */
                                            int,                        /* [IN]      Verbose ON/OFF */
                                            int,                        /* [IN]      Controller Index */
                                            int                         /* [IN]      Arm Index */);

int  ORLOPEN_set_additive_pos_target_degree    (ORL_joint_value *,          /* [IN]      Delta Position Target for CRCOPEN Message MODALITY 7 [Degree]*/
                                                int,                        /* [IN]      Verbose ON/OFF */
                                                int,                        /* [IN]      Controller Index */
                                                int                         /* [IN]      Arm Index */);

int  ORLOPEN_set_additive_pos_target_cartesian     (ORL_cartesian_position * ,  /* [IN]      Delta Position Target for CRCOPEN Message MODALITY 7 */
                                                    int,                        /* [IN]      Verbose ON/OFF */
                                                    int,                        /* [IN]      Controller Index */
                                                    int                         /* [IN]      Arm Index */);
#ifdef __cplusplus
}
#endif

#endif
