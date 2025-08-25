// Copyright (c) 2025 Ziga Miklosic
// All Rights Reserved
// This software is under MIT licence (https://opensource.org/licenses/MIT)
////////////////////////////////////////////////////////////////////////////////
/**
*@file      pid.c
*@brief     PID controller
*@mail      ziga.miklosic@gmail.com
*@date      25.08.2025
*@version   V1.1.0
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

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Function prototypes
////////////////////////////////////////////////////////////////////////////////
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
*   Calculate proportional (P) part
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
*   Calculate derivative (D) part
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
	p_ff_d = LIMIT( p_ff_d, p_cfg->min, p_cfg->max );

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
	// Calculate integral part
	float32_t i = p_cfg->ki * err * p_cfg->ts;

	// Accumulate previous integral and anti-windup part
	i += i_prev + a_prev;

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
    // Sum P+FF+D and I parts
    const float32_t out_no_lim = p_ff_d + i;

	// Limit
	const float32_t out = LIMIT( out_no_lim, p_cfg->min, p_cfg->max );

	// Calculate out above limit
	const float32_t out_over_saturation = out - out_no_lim;

	// Calculate anti-windup value
	*p_a = 0.5f * out_over_saturation;

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
*   Initialise PID controller
*
* @param[in]    p_inst  - Pointer to PID instance
* @param[in]    p_cfg   - Pointer to controller configuration
* @return       status  - Status of initialisation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_init(p_pid_t * p_inst, const pid_cfg_t * const p_cfg)
{
    pid_status_t status = ePID_OK;

    if  (   ( NULL != p_inst )
        &&  ( NULL != p_cfg ))
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
                (void) pid_reset( *p_inst );

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
*   Initialise statically PID controller
*
* @param[in] 	p_inst	- Pointer to PID instance
* @param[in] 	p_cfg	- Pointer to controller configuration
* @return 		status	- Status of initialisation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_init_static(p_pid_t pid_inst, const pid_cfg_t * const p_cfg)
{
	pid_status_t status = ePID_OK;

	if 	(	( NULL != pid_inst )
		&& 	( NULL != p_cfg ))
	{
        // Validate settings
        if ( true == pid_check_cfg( p_cfg ))
        {
            // Copy settings
            memcpy( &pid_inst->cfg, p_cfg, sizeof( pid_cfg_t ));

            // Set to zero
            (void) pid_reset( pid_inst );

            // Init succeed
            pid_inst->is_init = true;
        }
        else
        {
            // Init fail
            pid_inst->is_init = false;
            status = ePID_ERROR_CFG;
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
* @return 		status		- Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_hndl(p_pid_t pid_inst, const pid_in_t * const p_in)
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

    // Store for next iteration
    pid_inst->err_prev 	= pid_inst->out.err;
    pid_inst->i_prev 	= pid_inst->out.i_part;
    pid_inst->a_prev	= pid_inst->a;

	return pid_inst->out.out;
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
*   Reset PID controller
*
* @param[in]    pid_inst    - PID instance
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_reset(p_pid_t pid_inst)
{
    pid_status_t status = ePID_OK;

    if ( NULL != pid_inst )
    {
        // Set to zero
        pid_inst->in.act = 0.0f;
        pid_inst->in.ref = 0.0f;
        pid_inst->in.ff = 0.0f;

        pid_inst->out.out = 0.0f;
        pid_inst->out.err = 0.0f;
        pid_inst->out.p_part = 0.0f;
        pid_inst->out.i_part = 0.0f;
        pid_inst->out.d_part = 0.0f;

        pid_inst->err_prev = 0.0f;
        pid_inst->i_prev = 0.0f;
        pid_inst->a_prev = 0.0f;
    }
    else
    {
        status = ePID_ERROR;
    }

    return status;
}



pid_status_t pid_set_kp(p_pid_t pid_inst, const float32_t kp)
{
    pid_status_t status = ePID_OK;

    if ( NULL != pid_inst )
    {
        pid_inst->cfg.kp = kp;
    }
    else
    {
        status = ePID_ERROR;
    }

    return status;
}


pid_status_t pid_set_ki(p_pid_t pid_inst, const float32_t ki)
{
    pid_status_t status = ePID_OK;

    if ( NULL != pid_inst )
    {
        pid_inst->cfg.ki = ki;
    }
    else
    {
        status = ePID_ERROR;
    }

    return status;
}

pid_status_t pid_set_kd(p_pid_t pid_inst, const float32_t kd)
{
    pid_status_t status = ePID_OK;

    if ( NULL != pid_inst )
    {
        pid_inst->cfg.kd = kd;
    }
    else
    {
        status = ePID_ERROR;
    }

    return status;
}

pid_status_t pid_set_kw(p_pid_t pid_inst, const float32_t kw)
{
    pid_status_t status = ePID_OK;

    if ( NULL != pid_inst )
    {
        pid_inst->cfg.windup_k = kw;
    }
    else
    {
        status = ePID_ERROR;
    }

    return status;
}



float32_t pid_get_out(p_pid_t pid_inst)
{
    float32_t out = 0.0f;

    if ( NULL != pid_inst )
    {
        out = pid_inst->out.out;
    }

    return out;
}

float32_t pid_get_err(p_pid_t pid_inst)
{
    float32_t err = 0.0f;

    if ( NULL != pid_inst )
    {
        err = pid_inst->out.err;
    }

    return err;
}

float32_t pid_get_p_part(p_pid_t pid_inst)
{
    float32_t p_part = 0.0f;

    if ( NULL != pid_inst )
    {
        p_part = pid_inst->out.p_part;
    }

    return p_part;
}

float32_t pid_get_i_part(p_pid_t pid_inst)
{
    float32_t i_part = 0.0f;

    if ( NULL != pid_inst )
    {
        i_part = pid_inst->out.i_part;
    }

    return i_part;
}

float32_t pid_get_d_part(p_pid_t pid_inst)
{
    float32_t d_part = 0.0f;

    if ( NULL != pid_inst )
    {
        d_part = pid_inst->out.d_part;
    }

    return d_part;
}




// End of Ofast optimisation
#pragma GCC reset_options

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
