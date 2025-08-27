// Copyright (c) 2025 Ziga Miklosic
// All Rights Reserved
// This software is under MIT licence (https://opensource.org/licenses/MIT)
////////////////////////////////////////////////////////////////////////////////
/**
*@file      pid.h
*@brief     PID controller
*@author    Ziga Miklosic
*@mail      ziga.miklosic@gmail.com
*@date      25.08.2025
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
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Common goods
#include "common/utils/src/utils.h"

// Middleware
#include "middleware/filter/src/filter.h"

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
 * 	PID status
 */
enum
{
	ePID_OK				= 0,		/**<Normal operation */
	ePID_ERROR			= 0x01,		/**<General error */
	ePID_ERROR_INIT		= 0x02,		/**<Initialization error  */
	ePID_ERROR_CFG		= 0x04,		/**<PID controller settings error */
};
typedef uint8_t pid_status_t;

/**
 * 	PID configuration
 */
typedef struct
{
    float32_t ts;       /**<Time sample - period of main handler */
	float32_t kp;	    /**<Proportional coefficient */
	float32_t ki;	    /**<Integral coefficient */
	float32_t kd;	    /**<Derivitive coefficient */
	float32_t min;	    /**<Minimum value of output */
	float32_t max;	    /**<Maximum value of output */
	float32_t d_lpf_fc; /**<Cutoff freq of LPF for derivative part */
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
 *  PID controller
 */
typedef struct
{
    pid_cfg_t   cfg;        /**<Controller configurations */
    pid_in_t    in;         /**<Input data */
    pid_out_t   out;        /**<Output data */
    filter_rc_t d_lpf;      /**<D part low pass filter (LPF) */
    float32_t   d_lpf_mem;  /**<D part LPF memory */
    float32_t   act_prev;   /**<Previous sample of input actual (measured) signal */
    float32_t   ki_ts;      /**<Precalculated Ki*ts */
    float32_t   kd_ts;      /**<Precalculated Kd/ts */
    float32_t   aw;         /**<Anti-windup value */
    bool        is_init;    /**<Success initialisation flag */
} pid_t;
typedef pid_t * p_pid_t;

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_init           (p_pid_t * pid_inst, const pid_cfg_t * const p_cfg);
pid_status_t pid_init_static    (p_pid_t pid_inst, const pid_cfg_t * const p_cfg);
bool         pid_is_init	    (p_pid_t pid_inst);
float32_t    pid_hndl           (p_pid_t pid_inst, const pid_in_t * const p_in);
pid_status_t pid_set_cfg	    (p_pid_t pid_inst, const pid_cfg_t * const p_cfg);
pid_cfg_t *  pid_get_cfg	    (p_pid_t pid_inst);
pid_status_t pid_reset          (p_pid_t pid_inst);

pid_status_t pid_set_kp         (p_pid_t pid_inst, const float32_t kp);
float32_t    pid_get_kp         (p_pid_t pid_inst);
pid_status_t pid_set_ki         (p_pid_t pid_inst, const float32_t ki);
float32_t    pid_get_ki         (p_pid_t pid_inst);
pid_status_t pid_set_kd         (p_pid_t pid_inst, const float32_t kd);
float32_t    pid_get_kd         (p_pid_t pid_inst);
pid_status_t pid_set_min        (p_pid_t pid_inst, const float32_t min);
float32_t    pid_get_min        (p_pid_t pid_inst);
pid_status_t pid_set_max        (p_pid_t pid_inst, const float32_t max);
float32_t    pid_get_max        (p_pid_t pid_inst);
pid_status_t pid_set_d_lpf_fc   (p_pid_t pid_inst, const float32_t max);
float32_t    pid_get_d_lpf_fc   (p_pid_t pid_inst);

float32_t    pid_get_out        (p_pid_t pid_inst);
float32_t    pid_get_err        (p_pid_t pid_inst);
float32_t    pid_get_p_part     (p_pid_t pid_inst);
float32_t    pid_get_i_part     (p_pid_t pid_inst);
float32_t    pid_get_d_part     (p_pid_t pid_inst);

#endif // __PID_H

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
