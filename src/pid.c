// Copyright (c) 2025 Ziga Miklosic
// All Rights Reserved
// This software is under MIT licence (https://opensource.org/licenses/MIT)
////////////////////////////////////////////////////////////////////////////////
/**
*@file      pid.c
*@brief     PID controller
*@mail      ziga.miklosic@gmail.com
*@date      25.08.2025
*@version   V1.0.0
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
#include <math.h>

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
static inline float32_t pid_calc_d_part         (p_pid_t pid_inst);
static inline float32_t pid_calc_p_ff_d	        (p_pid_t pid_inst);
static inline float32_t pid_calc_i_part	        (p_pid_t pid_inst);
static inline float32_t	pid_calc_out	        (p_pid_t pid_inst);
static bool				pid_check_cfg	        (const pid_cfg_t * const p_cfg);
static bool             pid_rc_calculate_alpha  (const float32_t fc, const float32_t fs, float32_t * const p_alpha);

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/**
*   Calculate derivative (D) part
*
* @param[in] 	pid_inst - PID controller instance
* @return 		d        - D part of controller
*/
////////////////////////////////////////////////////////////////////////////////
static inline float32_t pid_calc_d_part(p_pid_t pid_inst)
{
    float32_t d = 0.0f;

	if ( pid_inst->cfg.kd > 0.0f )
	{
	    // Filter actual input
		pid_inst->act_filt += ( pid_inst->cfg.lpf_d.alpha * ( pid_inst->in.act - pid_inst->act_filt ));

		// Calculate difference
		const float32_t act_dlt = pid_inst->act_filt - pid_inst->act_filt_prev;
        pid_inst->act_filt_prev = pid_inst->act_filt;

		// Calculate D part
		d = act_dlt * pid_inst->kd_ts;
	}

	return d;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Calculate P+FF+D
*
* @param[in]    pid_inst - PID controller instance
* @return 		p_ff_d	 - Value of P+FF+D
*/
////////////////////////////////////////////////////////////////////////////////
static inline float32_t pid_calc_p_ff_d(p_pid_t pid_inst)
{
	// Sum P, FF and D part
	float32_t p_ff_d = pid_inst->out.p_part + pid_inst->in.ff  + pid_inst->out.d_part;

	// Limit P+FF+D
	p_ff_d = LIMIT( p_ff_d, pid_inst->cfg.min, pid_inst->cfg.max );

	return p_ff_d;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Calculate I part - INTEGRAL
*
* @param[in]    pid_inst - PID controller instance
* @return 		i		 - I part of controller
*/
////////////////////////////////////////////////////////////////////////////////
static inline float32_t pid_calc_i_part(p_pid_t pid_inst)
{
	// Calculate integral part
	float32_t i = pid_inst->ki_ts * pid_inst->out.err;

	// Accumulate previous integral and anti-windup part
	i += pid_inst->i_prev + pid_inst->a_prev;

	return i;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Calculate controller output
*
* @param[in]    pid_inst - PID controller instance
* @return 		out      - Output of controller
*/
////////////////////////////////////////////////////////////////////////////////
static inline float32_t pid_calc_out(p_pid_t pid_inst)
{
    // Sum P+FF+D and I parts
    const float32_t out_no_lim = pid_inst->p_ff_d + pid_inst->out.i_part;

	// Limit
	const float32_t out = LIMIT( out_no_lim, pid_inst->cfg.min, pid_inst->cfg.max );

	// Calculate out above limit
	const float32_t out_over_saturation = out - out_no_lim;

	// Calculate anti-windup value
	pid_inst->a = 0.5f * out_over_saturation;

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

	return valid;
}

////////////////////////////////////////////////////////////////////////////////
/**
*       Calculate RC LPF alpha
*
* @param[in]    fc      - Cutoff frequency
* @param[in]    fs      - Sample frequency
* @param[out]   alpha   - CR alpha
* @return       true if alpha is calculated
*/
////////////////////////////////////////////////////////////////////////////////
static bool pid_rc_calculate_alpha(const float32_t fc, const float32_t fs, float32_t * const p_alpha)
{
    // Check Nyquist/Shannon sampling theorem
    if  (   ( fc < ( fs / 2.0f ))
        &&  ( p_alpha != NULL ))
    {
        *p_alpha = (float32_t) ( 1.0f / ( 1.0f + ( fs / ( M_TWOPI * fc ))));

        return true;
    }
    else
    {
        return false;
    }
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

                // Precalculations
                (*p_inst)->ki_ts = p_cfg->ki * p_cfg->ts;
                (*p_inst)->kd_ts = p_cfg->kd / p_cfg->ts;

                if ( true == pid_rc_calculate_alpha( p_cfg->lpf_d.fc, (float32_t)( 1.0f / p_cfg->ts ), &((*p_inst)->cfg.lpf_d.alpha )))
                {
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

            // Precalculations
            pid_inst->ki_ts = p_cfg->ki * p_cfg->ts;
            pid_inst->kd_ts = p_cfg->kd / p_cfg->ts;

            if ( true == pid_rc_calculate_alpha( p_cfg->lpf_d.fc, (float32_t)( 1.0f / p_cfg->ts ), &pid_inst->cfg.lpf_d.alpha ))
            {
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
    // Check inputs
    if (( NULL != pid_inst ) || ( NULL != p_in ) || ( !pid_inst->is_init )) return 0.0f;

    // Get input
    pid_inst->in.act = p_in->act;
    pid_inst->in.ref = p_in->ref;
    pid_inst->in.ff  = p_in->ff;

    // Calculate error
    pid_inst->out.err = ( pid_inst->in.ref - pid_inst->in.act );

    // Calculate P part
    pid_inst->out.p_part = pid_inst->cfg.kp * pid_inst->out.err;

    // Calculate D part
    pid_inst->out.d_part = pid_calc_d_part( pid_inst );

    // Sum + limit P+FF+D
    pid_inst->p_ff_d = pid_calc_p_ff_d( pid_inst);

    // Calculate I part
    pid_inst->out.i_part = pid_calc_i_part( pid_inst );

    // Calculate and limit output + anti-windup
    pid_inst->out.out = pid_calc_out( pid_inst );

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
        memset( pid_inst, 0U, sizeof(pid_t));
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


////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
