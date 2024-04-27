// Copyright (c) 2021 Ziga Miklosic
// All Rights Reserved
// This software is under MIT licence (https://opensource.org/licenses/MIT)
////////////////////////////////////////////////////////////////////////////////
/**
*@file      pid.h
*@brief     PID controller
*@author    Ziga Miklosic
*@date      17.08.2021
*@version   V1.0.0
*/
////////////////////////////////////////////////////////////////////////////////
/**
*@addtogroup PID_API
* @{ <!-- BEGIN GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
#ifndef __PID_H
#define __PID_H

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

/**
 * 	Module version
 */
#define PID_VER_MAJOR		( 1 )
#define PID_VER_MINOR		( 0 )
#define PID_VER_DEVELOP		( 0 )

/**
 * Float 32-bit definition
 */
typedef float float32_t;


/**
 * 	PID status
 */
typedef enum
{
	ePID_OK				= 0,		/**<Normal operation */
	ePID_ERROR			= 0x01,		/**<General error */
	ePID_ERROR_INIT		= 0x02,		/**<Initialization error  */
	ePID_ERROR_CFG		= 0x04,		/**<PID controller settings error */
} pid_status_t;

/**
 * 	PID configuration
 */
typedef struct
{
	float32_t	kp;			/**<Proportional coeficient */
	float32_t	ki;			/**<Integral coeficient */
	float32_t	kd;			/**<Derivitive coeficient */
	float32_t	ts;			/**<Time sample - period of main handler */
	float32_t	min;		/**<Minimum value of output */
	float32_t	max;		/**<Maximum value of output */
	float32_t	lpf_d_fc;	/**<Cutoff freq of LPF for derivative part */
	float32_t 	windup_k;	/**<Anti-Windup coefficient */
} pid_cfg_t;

/**
 * 	PID Input data
 *
 * 	@note	Reference and actual value must have the same unit.
 */
typedef struct
{
	float32_t ref;		/**<Reference value */
	float32_t act;		/**<Actual value */
	float32_t ff;		/**<Feed-Forward value */
} pid_in_t;

/**
 * 	PID Output data
 */
typedef struct
{
	float32_t out;		/**<Output value */
	float32_t err;		/**<Error - difference between reference and actual variable */
	float32_t p_part;	/**<Proportional part of PID controller */
	float32_t i_part;	/**<Integral part of PID controller */
	float32_t d_part;	/**<Derivative part of PID controller */
} pid_out_t;

/**
 * 	Pointer to PID instance
 */
typedef struct pid_s * p_pid_t;

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_init		(p_pid_t * p_inst, const pid_cfg_t * const p_cfg);
pid_status_t pid_is_init	(p_pid_t pid_inst, bool * const p_is_init);
pid_status_t pid_hndl		(p_pid_t pid_inst, const pid_in_t * const p_in, pid_out_t * const p_out);
pid_status_t pid_set_cfg	(p_pid_t pid_inst, const pid_cfg_t * const p_cfg);
pid_status_t pid_get_cfg	(p_pid_t pid_inst, pid_cfg_t * const p_cfg);
pid_status_t pid_reset      (p_pid_t pid_inst);

#endif // __PID_H

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
