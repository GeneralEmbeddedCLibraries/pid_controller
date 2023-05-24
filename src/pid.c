// Copyright (c) 2021 Ziga Miklosic
// All Rights Reserved
// This software is under MIT licence (https://opensource.org/licenses/MIT)
////////////////////////////////////////////////////////////////////////////////
/**
*@file      pid.c
*@brief     PID controller
*@author    Ziga Miklosic
*@date      17.08.2021
*@version   V1.0.0
*
*
*@section   Description
*   
*   PID controler module
*
*/
////////////////////////////////////////////////////////////////////////////////
/*!
* @addtogroup PID
* @{ <!-- BEGIN GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pid.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////


/**
 * 	PID controler
 */
typedef struct pid_s
{
	pid_cfg_t 	cfg;			/**<Controller configurations */
	pid_in_t	in;				/**<Input data */
	pid_out_t	out;			/**<Output data */
	float32_t	err_prev;		/**<Previous error */
	float32_t	i_prev;			/**<Previous value of integral part */
	float32_t	a;				/**<Current value of anti-windup part */
	float32_t	a_prev;			/**<Previous value of anti-windup part */
	float32_t	p_ff_d;			/**<Summed & limited P+FF+D */
	bool		is_init;		/**<Success initialization flag */

	// TODO: Add filter for D part
	// p_filter_rc_t	lpf_d;
} pid_t;


////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Function prototypes
////////////////////////////////////////////////////////////////////////////////
static inline float32_t pid_limiter		(const float32_t in, const float32_t max, const float32_t min);
static inline float32_t pid_calc_p_part	(const float32_t err, const float32_t kp);
static inline float32_t pid_calc_d_part	(const float32_t err, const float32_t err_prev, const pid_cfg_t * const p_cfg);
static inline float32_t pid_calc_p_ff_d	(const float32_t p, const float32_t ff, const float32_t d, const pid_cfg_t * const p_cfg);
static inline float32_t pid_calc_i_part	(const float32_t err, const float32_t i_prev, const float32_t a_prev, const pid_cfg_t * const p_cfg);
static inline float32_t	pid_calc_out	(const float32_t p_ff_d, const float32_t i, const pid_cfg_t * const p_cfg, float32_t * const p_a);
static bool				pid_check_cfg	(const pid_cfg_t * const p_cfg);

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/**
*   Limit to min/max
*
* @param[in] 	in	- Input value to limit
* @param[in] 	max	- Maximum allowed value
* @param[in] 	min	- Minimum allowed value
* @return 		out	- Output limited value
*/
////////////////////////////////////////////////////////////////////////////////
static inline float32_t pid_limiter(const float32_t in, const float32_t max, const float32_t min)
{
	float32_t out = 0.0f;

	if ( in > max )
	{
		out = max;
	}
	else if ( in < min )
	{
		out = min;
	}
	else
	{
		out = in;
	}

	return out;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Calculate P part - PROPORTIONAL
*
* @param[in] 	err - Error in reference and actual value
* @param[in] 	kp	- Proportional coefficient
* @return 		p	- P part of controller
*/
////////////////////////////////////////////////////////////////////////////////
static inline float32_t pid_calc_p_part(const float32_t err, const float32_t kp)
{
	float32_t p = 0.0f;

	// Calculation of P part
	p = ( kp * err );

	return p;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Calculate D part - DERIVATIVE
*
* @param[in] 	err 		- Error in reference and actual value
* @param[in] 	err_prev 	- Previous value of error
* @param[in] 	p_cfg		- Pointer to controller configuration
* @return 		d			- D part of controller
*/
////////////////////////////////////////////////////////////////////////////////
static inline float32_t pid_calc_d_part(const float32_t err, const float32_t err_prev, const pid_cfg_t * const p_cfg)
{
    float32_t d         = 0.0f;

    // unused params
    (void) err;
    (void) err_prev;
    (void) p_cfg;

    //TODO: Needs to be implemented!
	/*
	float32_t err_dlt	= 0.0f;

	if ( p_cfg->kd > 0.0f )
	{
		// Calculate error change
		err_dlt = ( err - err_prev );

		// Apply LPF
		// TODO: ...
		// d = filter_rc_hndl( ..., err_dlt );

		// Multiply with D coefficient
		d = ( p_cfg->kd * d );

		// Apply time sample
		d = ( d / p_cfg->ts );
	}*/

	return d;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Calculate P+FF+D
*
* @param[in] 	p		- P part of controller
* @param[in] 	ff		- FF (feed forward) part of controller
* @param[in] 	d		- D part of controller
* @param[in] 	p_cfg	- Pointer to controller configuration
* @return 		p_ff_d	- Value of P+FF+D
*/
////////////////////////////////////////////////////////////////////////////////
static inline float32_t pid_calc_p_ff_d(const float32_t p, const float32_t ff, const float32_t d, const pid_cfg_t * const p_cfg)
{
	float32_t p_ff_d = 0.0f;

	// Sum P, FF and D part
	p_ff_d = (( p + ff ) + d );

	// Limit P+FF+D
	p_ff_d = pid_limiter( p_ff_d, p_cfg->max, p_cfg->min );

	return p_ff_d;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Calculate I part - INTEGRAL
*
* @param[in] 	err 	- Error in reference and actual value
* @param[in] 	i_prev 	- Previous I part
* @param[in] 	a_prev	- Anti-windup previous part
* @param[in] 	p_cfg	- Pointer to controller configuration
* @return 		i		- I part of controller
*/
////////////////////////////////////////////////////////////////////////////////
static inline float32_t pid_calc_i_part(const float32_t err, const float32_t i_prev, const float32_t a_prev, const pid_cfg_t * const p_cfg)
{
	float32_t i = 0.0f;

	// Apply integral coefficient
	i = ( p_cfg->ki * err );

	// Apply time sample
	i = ( p_cfg->ts * i );

	// Accumulate previous integral and anti-windup part
	i = ( i_prev + a_prev + i );

	return i;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Calculate controller output
*
* @param[in] 	p_ff_d	- Value of P+FF+D
* @param[in] 	i		- I part of controller
* @param[in] 	p_cfg	- Pointer to controller configuration
* @param[out] 	p_a		- Pointer to anti-windup value
* @return 		out		- Output of controller
*/
////////////////////////////////////////////////////////////////////////////////
static inline float32_t pid_calc_out(const float32_t p_ff_d, const float32_t i, const pid_cfg_t * const p_cfg, float32_t * const p_a)
{
	float32_t out 			= 0.0f;
	float32_t out_no_lim 	= 0.0f;
	float32_t out_err		= 0.0f;

	// Sum P+FF+D and I parts
	out_no_lim = ( p_ff_d + i );

	// Limit
	out = pid_limiter( out_no_lim, p_cfg->max, p_cfg->min );

	// Calculate out above limit
	out_err = ( out - out_no_lim );

	// Calculate anti-windup value
	*p_a = ( p_cfg->windup_k * out_err );

	return out;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Check PID controller configurations
*
* @param[in] 	p_cfg	- Pointer to controller configuration
* @return 		valid	- Validation check result
*/
////////////////////////////////////////////////////////////////////////////////
static bool	pid_check_cfg(const pid_cfg_t * const p_cfg)
{
	bool valid = true;

	if ( p_cfg->ts <= 0.0f )
	{
		valid = false;
	}

	if ( p_cfg->max <= p_cfg->min )
	{
		valid = false;
	}

	// TODO: add more checks here...

	return valid;
}

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/**
*@addtogroup PID_API
* @{ <!-- BEGIN GROUP -->
*
* Following functions are part of API
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/**
*   Initialize PID controller
*
* @param[in] 	p_inst	- Pointer to PID instance
* @param[in] 	p_cfg	- Pointer to controller configuration
* @return 		status	- Status of initialization
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_init(p_pid_t * p_inst, const pid_cfg_t * const p_cfg)
{
	pid_status_t status = ePID_OK;

	if 	(	( NULL != p_inst )
		&& 	( NULL != p_cfg ))
	{
		*p_inst = malloc( sizeof( pid_t ));

		// Allocate succeed
		if ( NULL != *p_inst )
		{
			// Validate settings
			if ( true == pid_check_cfg( p_cfg ))
			{
				// Copy settings
				memcpy( &(*p_inst)->cfg, p_cfg, sizeof( pid_cfg_t ));

				// Set to zero
				(*p_inst)->in.act = 0.0f;
				(*p_inst)->in.ref = 0.0f;
				(*p_inst)->in.ff = 0.0f;

				(*p_inst)->out.out = 0.0f;
				(*p_inst)->out.err = 0.0f;
				(*p_inst)->out.p_part = 0.0f;
				(*p_inst)->out.i_part = 0.0f;
				(*p_inst)->out.d_part = 0.0f;

				(*p_inst)->err_prev = 0.0f;
				(*p_inst)->i_prev = 0.0f;
				(*p_inst)->a_prev = 0.0f;

				// Init succeed
				(*p_inst)->is_init = true;
			}
			else
			{
				// Init fail
				(*p_inst)->is_init = false;
				status = ePID_ERROR_CFG;
			}
		}
		else
		{
			status = ePID_ERROR_INIT;
		}
	}
	else
	{
		status = ePID_ERROR_INIT;
	}

	return status;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get controller initialization flag
*
* @param[in] 	p_is_init 	- Pointer to init flag
* @return 		status		- Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_is_init(p_pid_t pid_inst, bool * const p_is_init)
{
	pid_status_t status = ePID_OK;

	if 	(	( NULL != pid_inst )
		&& 	( NULL != p_is_init ))
	{
		*p_is_init = pid_inst->is_init;
	}
	else
	{
		status = ePID_ERROR;
	}

	return status;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Main PID control handler
*
* @brief	This function must be called at "Ts" period. That value is set in
* 			configuration at init.
*
* @param[in] 	pid_inst	- PID instance
* @param[in] 	p_in		- Pointer to input data
* @param[in] 	p_out		- Pointer to output data
* @return 		status		- Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_hndl(p_pid_t pid_inst, const pid_in_t * const p_in, pid_out_t * const p_out)
{
	pid_status_t status = ePID_OK;

	// Is initialized
	if ( true == pid_inst->is_init )
	{
		if 	(	( NULL != pid_inst )
			&&	( NULL != p_in )
			&&	( NULL != p_out ))
		{
			// Copy input
			memcpy( &pid_inst->in, p_in, sizeof( pid_in_t ));

			// Calculate error
			pid_inst->out.err = ( pid_inst->in.ref - pid_inst->in.act );

			// Calculate P part
			pid_inst->out.p_part = pid_calc_p_part( pid_inst->out.err, pid_inst->cfg.kp );

			// Calculate D part
			pid_inst->out.d_part = pid_calc_d_part( pid_inst->out.err, pid_inst->err_prev, &pid_inst->cfg );

			// Sum + limit P+FF+D
			pid_inst->p_ff_d = pid_calc_p_ff_d( pid_inst->out.p_part, pid_inst->in.ff, pid_inst->out.d_part, &pid_inst->cfg );

			// Calculate I part
			pid_inst->out.i_part = pid_calc_i_part( pid_inst->out.err, pid_inst->i_prev, pid_inst->a_prev, &pid_inst->cfg );

			// Calculate and limit output + anti-windup
			pid_inst->out.out = pid_calc_out( pid_inst->p_ff_d, pid_inst->out.i_part, &pid_inst->cfg, &pid_inst->a );

			// Copy output
			memcpy( p_out, &pid_inst->out, sizeof( pid_out_t ));

			// Store for next iteration
			pid_inst->err_prev 	= pid_inst->out.err;
			pid_inst->i_prev 	= pid_inst->out.i_part;
			pid_inst->a_prev	= pid_inst->a;
		}
		else
		{
			status = ePID_ERROR;
		}
	}
	else
	{
		status = ePID_ERROR_INIT;
	}

	return status;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Set PID controller configurations
*
* @param[in] 	pid_inst	- PID instance
* @param[in] 	p_cfg		- Pointer to controller configuration
* @return 		status		- Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_set_cfg(p_pid_t pid_inst, const pid_cfg_t * const p_cfg)
{
	pid_status_t status = ePID_OK;

	// Is initialized
	if ( true == pid_inst->is_init )
	{
		if 	(	( NULL != pid_inst )
			&&	( NULL != p_cfg ))
		{
			memcpy( &pid_inst->cfg, p_cfg, sizeof( pid_cfg_t ));
		}
		else
		{
			status = ePID_ERROR;
		}
	}
	else
	{
		status = ePID_ERROR_INIT;
	}

	return status;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get PID controller configurations
*
* @param[in] 	pid_inst	- PID instance
* @param[in] 	p_cfg		- Pointer to controller configuration
* @return 		status		- Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_get_cfg(p_pid_t pid_inst, pid_cfg_t * const p_cfg)
{
	pid_status_t status = ePID_OK;

	if 	(	( NULL != pid_inst )
		&&	( NULL != p_cfg ))
	{
		memcpy( p_cfg, &pid_inst->cfg, sizeof( pid_cfg_t ));
	}
	else
	{
		status = ePID_ERROR;
	}

	return status;
}

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
