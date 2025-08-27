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
static inline float32_t pid_calc_d_part (p_pid_t pid_inst);
static bool				pid_check_cfg	(const pid_cfg_t * const p_cfg);
static void             pid_precalculate(p_pid_t pid_inst);

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
        // Filter
        float32_t act_filt;
        if ( pid_inst->cfg.d_lpf_fc > 0.0f )
        {
            filter_rc_hndl( &pid_inst->d_lpf, pid_inst->in.act, &act_filt );
        }

        // Filter disabled
        else
        {
            act_filt = pid_inst->in.act;
        }

        // Calculate difference
        const float32_t act_dlt = act_filt - pid_inst->act_prev;
        pid_inst->act_prev = act_filt;

		// Calculate D part
		d = act_dlt * pid_inst->kd_ts;
	}
	else
	{
	    filter_rc_reset( &pid_inst->d_lpf, pid_inst->in.act );
	}

	return d;
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

	if (( p_cfg->kp < 0.0f ) || ( p_cfg->ki < 0.0f ) || ( p_cfg->kd < 0.0f ))
    {
        valid = false;
    }

	return valid;
}

////////////////////////////////////////////////////////////////////////////////
/**
*       Precalculate factors
*
* @param[in]    p_cfg - Pointer to controller configuration
* @return       void
*/
////////////////////////////////////////////////////////////////////////////////
static void pid_precalculate(p_pid_t pid_inst)
{
    // Precalculate Ki and Kd factors with time sample
    pid_inst->ki_ts = pid_inst->cfg.ki * pid_inst->cfg.ts;
    pid_inst->kd_ts = -pid_inst->cfg.kd / pid_inst->cfg.ts; // NOTE: Derivative part is being done on measured signal, therefore negative sign here!
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
pid_status_t pid_init(p_pid_t * pid_inst, const pid_cfg_t * const p_cfg)
{
    // Invalid arguments
    if (( NULL == pid_inst ) || ( NULL == p_cfg ))
    {
        return ePID_ERROR_INIT;
    }

    // Allocate memory for PID object
    *pid_inst = calloc( 1U, sizeof( pid_t ));

    // Check allocation status
    if ( NULL == *pid_inst )
    {
        return ePID_ERROR_INIT;
    }

    // Check for valid configuration
    if ( false == pid_check_cfg( p_cfg ))
    {
        free( *pid_inst );
        return ePID_ERROR_CFG;
    }

    // Copy settings
    memcpy( &(*pid_inst)->cfg, p_cfg, sizeof( pid_cfg_t ));

    // Precalculate all factors
    pid_precalculate( *pid_inst );

    // Setup memory for D part LPF
    (*pid_inst)->d_lpf.p_y = &(*pid_inst)->d_lpf_mem;

    // Initialized D part LPF
    if ( eFILTER_OK == filter_rc_init_static( &(*pid_inst)->d_lpf, p_cfg->d_lpf_fc, ( 1.0f / p_cfg->ts ), 1U, 0.0f ))
    {
        // Set to zero
        (void) pid_reset( *pid_inst );

        // Init succeed
        (*pid_inst)->is_init = true;

        return ePID_OK;
    }
    else
    {
        free( *pid_inst );
        return ePID_ERROR_CFG;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Initialise statically PID controller
*
* @param[in]    pid_inst - PID controller instance
* @param[in] 	p_cfg	 - Pointer to controller configuration
* @return 		status	 - Status of initialisation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_init_static(p_pid_t pid_inst, const pid_cfg_t * const p_cfg)
{
    // Invalid arguments
    if (( NULL == pid_inst ) || ( NULL == p_cfg ))
    {
        return ePID_ERROR_INIT;
    }

    // Check for valid configuration
    if ( false == pid_check_cfg( p_cfg ))
    {
        return ePID_ERROR_CFG;
    }

    // Copy settings
    memcpy( &pid_inst->cfg, p_cfg, sizeof( pid_cfg_t ));

    // Precalculate all factors
    pid_precalculate( pid_inst );

    // Setup memory for D part LPF
    pid_inst->d_lpf.p_y = &pid_inst->d_lpf_mem;

    // Initialized D part LPF
    if ( eFILTER_OK == filter_rc_init_static( &pid_inst->d_lpf, p_cfg->d_lpf_fc, ( 1.0f / p_cfg->ts ), 1U, 0.0f ))
    {
        // Set to zero
        (void) pid_reset( pid_inst );

        // Init succeed
        pid_inst->is_init = true;

        return ePID_OK;
    }
    else
    {
        return ePID_ERROR_CFG;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get controller initialization flag
*
* @param[in]    pid_inst - PID controller instance
* @return 		true if PID object initialized
*/
////////////////////////////////////////////////////////////////////////////////
bool pid_is_init(p_pid_t pid_inst)
{
	if ( NULL != pid_inst )
	{
	    return pid_inst->is_init;
	}
	else
	{
	    return false;
	}
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Main PID control handler
*
* @brief	This function must be called at "Ts" period. That value is set in
* 			configuration at init.
*
* @param[in]    pid_inst - PID controller instance
* @param[in] 	p_in     - Pointer to input data
* @return 		PID controller output
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_hndl(p_pid_t pid_inst, const pid_in_t * const p_in)
{
    // Check inputs
    if (( NULL == pid_inst ) || ( NULL == p_in ) || ( !pid_inst->is_init )) return 0.0f;

    // Get input
    pid_inst->in.act = p_in->act;
    pid_inst->in.ref = p_in->ref;
    pid_inst->in.ff  = p_in->ff;

    // Calculate error
    pid_inst->out.err = pid_inst->in.ref - pid_inst->in.act;

    // Calculate P part
    pid_inst->out.p_part = pid_inst->cfg.kp * pid_inst->out.err;

    // Calculate D part
    pid_inst->out.d_part = pid_calc_d_part( pid_inst );

    // Sum + limit P+FF+D
    const float32_t p_ff_d = pid_inst->out.p_part + pid_inst->in.ff + pid_inst->out.d_part;

    // Calculate I part + anti-windup part
    pid_inst->out.i_part += ( pid_inst->ki_ts * pid_inst->out.err ) - pid_inst->aw;

    // Sum P+FF+D and I parts
    const float32_t out_no_lim = p_ff_d + pid_inst->out.i_part;

    // Limit output
    pid_inst->out.out = LIMIT( out_no_lim, pid_inst->cfg.min, pid_inst->cfg.max );

    // Calculate anti-windup value
    pid_inst->aw = pid_inst->ki_ts * ( out_no_lim - pid_inst->out.out );

	return pid_inst->out.out;
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Set PID controller configurations
*
* @param[in]    pid_inst - PID controller instance
* @param[in] 	p_cfg    - Pointer to controller configuration
* @return 		status   - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_set_cfg(p_pid_t pid_inst, const pid_cfg_t * const p_cfg)
{
    if ((pid_inst == NULL) || (p_cfg == NULL))  return ePID_ERROR;
    if ( true != pid_inst->is_init )            return ePID_ERROR_INIT;

    // Get new configs
    memcpy( &pid_inst->cfg, p_cfg, sizeof( pid_cfg_t ));
    pid_precalculate( pid_inst );

    if ( eFILTER_OK != filter_rc_fc_set( &pid_inst->d_lpf, pid_inst->cfg.d_lpf_fc ))
    {
        return ePID_ERROR_CFG;
    }
    else
    {
        return ePID_OK;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get PID controller configurations
*
* @param[in]    pid_inst - PID controller instance
* @return 		p_cfg    - Pointer to controller configuration
*/
////////////////////////////////////////////////////////////////////////////////
const pid_cfg_t * pid_get_cfg(p_pid_t pid_inst)
{
	if ( NULL != pid_inst )
	{
	    return &pid_inst->cfg;
	}
	else
	{
	    return NULL;
	}
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
    if ( NULL != pid_inst )
    {
        // Inputs
        pid_inst->in.ref = 0.0f;
        pid_inst->in.act = 0.0f;
        pid_inst->in.ff  = 0.0f;

        // Outputs/parts
        pid_inst->out.out    = 0.0f;
        pid_inst->out.err    = 0.0f;
        pid_inst->out.p_part = 0.0f;
        pid_inst->out.i_part = 0.0f;
        pid_inst->out.d_part = 0.0f;
        pid_inst->aw         = 0.0f;

        // Initialise LPF state to current measurement to avoid startup D spike
        pid_inst->act_prev = pid_inst->in.act;
        filter_rc_reset( &pid_inst->d_lpf, pid_inst->in.act );

        return ePID_OK;
    }
    else
    {
        return ePID_ERROR;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Set proportional coefficient (Kp)
*
* @param[in]    pid_inst - PID instance
* @param[in]    kp       - Kp value
* @return       status   - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_set_kp(p_pid_t pid_inst, const float32_t kp)
{
    if (( NULL != pid_inst ) &&  ( kp >= 0.0f ))
    {
        pid_inst->cfg.kp = kp;
        return ePID_OK;
    }
    else
    {
        return ePID_ERROR_CFG;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get proportional coefficient (Kp)
*
* @param[in]    pid_inst - PID instance
* @return       Proportional coefficient
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_get_kp(p_pid_t pid_inst)
{
    if ( NULL != pid_inst )
    {
        return pid_inst->cfg.kp;
    }
    else
    {
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Set integral coefficient (Ki)
*
* @param[in]    pid_inst - PID instance
* @param[in]    ki       - Ki value
* @return       status   - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_set_ki(p_pid_t pid_inst, const float32_t ki)
{
    if (( NULL != pid_inst ) && ( ki >= 0.0f ))
    {
        pid_inst->cfg.ki = ki;
        pid_precalculate( pid_inst );
        return ePID_OK;
    }
    else
    {
        return ePID_ERROR_CFG;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get integral coefficient (Ki)
*
* @param[in]    pid_inst - PID instance
* @return       Integral coefficient
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_get_ki(p_pid_t pid_inst)
{
    if ( NULL != pid_inst )
    {
        return pid_inst->cfg.ki;
    }
    else
    {
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Set derivative coefficient (Kd)
*
* @param[in]    pid_inst - PID instance
* @param[in]    kd       - Kd value
* @return       status   - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_set_kd(p_pid_t pid_inst, const float32_t kd)
{
    if (( NULL != pid_inst ) && ( kd >= 0.0f ))
    {
        pid_inst->cfg.kd = kd;
        pid_precalculate( pid_inst );
        return ePID_OK;
    }
    else
    {
        return ePID_ERROR_CFG;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get derivative coefficient (Kd)
*
* @param[in]    pid_inst - PID instance
* @return       Derivative coefficient
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_get_kd(p_pid_t pid_inst)
{
    if ( NULL != pid_inst )
    {
        return pid_inst->cfg.kd;
    }
    else
    {
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Set minimal value of PID output
*
* @param[in]    pid_inst - PID instance
* @param[in]    min      - Minimal PID output value
* @return       status   - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_set_min(p_pid_t pid_inst, const float32_t min)
{
    if (( NULL != pid_inst ) && ( min <= pid_inst->cfg.max ))
    {
        pid_inst->cfg.min = min;
        return ePID_OK;
    }
    else
    {
        return ePID_ERROR_CFG;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get minimal PID output value
*
* @param[in]    pid_inst - PID instance
* @return       Minimal output value
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_get_min(p_pid_t pid_inst)
{
    if ( NULL != pid_inst )
    {
        return pid_inst->cfg.min;
    }
    else
    {
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Set maximum value of PID output
*
* @param[in]    pid_inst - PID instance
* @param[in]    max      - Maximum PID output value
* @return       status   - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_set_max(p_pid_t pid_inst, const float32_t max)
{
    if (( NULL != pid_inst ) && ( max >= pid_inst->cfg.min ))
    {
        pid_inst->cfg.max = max;
        return ePID_OK;
    }
    else
    {
        return ePID_ERROR_CFG;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get maximum PID output value
*
* @param[in]    pid_inst - PID instance
* @return       Maximum output value
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_get_max(p_pid_t pid_inst)
{
    if ( NULL != pid_inst )
    {
        return pid_inst->cfg.max;
    }
    else
    {
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Set D part LPF cutoff frequency
*
* @param[in]    pid_inst - PID instance
* @param[in]    fc       - LPF cutoff frequency in Hz
* @return       status   - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
pid_status_t pid_set_d_lpf_fc(p_pid_t pid_inst, const float32_t fc)
{
    if ( NULL != pid_inst )
    {
        if ( eFILTER_OK == filter_rc_fc_set( &pid_inst->d_lpf, fc ))
        {
            pid_inst->cfg.d_lpf_fc = fc;
            return ePID_OK;
        }
        else
        {
            return ePID_ERROR_CFG;
        }
    }
    else
    {
        return ePID_ERROR;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get D part LPF cutoff frequency
*
* @param[in]    pid_inst - PID instance
* @return       LPF cutoff frequency in Hz
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_get_d_lpf_fc(p_pid_t pid_inst)
{
    if ( NULL != pid_inst )
    {
        return pid_inst->cfg.d_lpf_fc;
    }
    else
    {
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get PID output value
*
* @param[in]    pid_inst - PID instance
* @return       PID output value
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_get_out(p_pid_t pid_inst)
{
    if ( NULL != pid_inst )
    {
        return pid_inst->out.out;
    }
    else
    {
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get error
*
* @param[in]    pid_inst - PID instance
* @return       Error
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_get_err(p_pid_t pid_inst)
{
    if ( NULL != pid_inst )
    {
        return pid_inst->out.err;
    }
    else
    {
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get P part
*
* @param[in]    pid_inst - PID instance
* @return       P part
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_get_p_part(p_pid_t pid_inst)
{
    if ( NULL != pid_inst )
    {
        return pid_inst->out.p_part;
    }
    else
    {
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get I part
*
* @param[in]    pid_inst - PID instance
* @return       I part
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_get_i_part(p_pid_t pid_inst)
{
    if ( NULL != pid_inst )
    {
        return pid_inst->out.i_part;
    }
    else
    {
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
*   Get D part
*
* @param[in]    pid_inst - PID instance
* @return       D part
*/
////////////////////////////////////////////////////////////////////////////////
float32_t pid_get_d_part(p_pid_t pid_inst)
{
    if ( NULL != pid_inst )
    {
        return pid_inst->out.d_part;
    }
    else
    {
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
