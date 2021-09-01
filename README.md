# PID Controller
C code implementation of PID controller for general embedded application. 

PID controller are written in such way that can be used as individual instances with no relations between the at all.

PID controller memory space is dynamically allocated and success of allocation is taken into consideration before using that instance. Deallocation on exsisting controller instance is not supported as it's not good practice to free memory in C world.

## Dependencies
---
PID controller is dependent from filter module. Filter module sources can be found under this [link](https://github.com/GeneralEmbeddedCLibraries/filter). 

Definition of flaot32_t must be provided by user. In current implementation it is defined in "*project_config.h*". Just add following statement to your code where it suits the best.

```C
// Define float
typedef float float32_t;
```

## Top overview
---

![](doc/pid_controller_V1_0_0_diagram.png)

 ## API
---


| API Functions | Description | Prototype |
| --- | ----------- | ----- |
| **pid_init** | Initialization of PID controller | pid_status_t pid_init(p_pid_t * p_inst, const pid_cfg_t * const p_cfg) |
| **pid_is_init** | Get initialization status | pid_status_t pid_is_init(p_pid_t pid_inst, bool * const p_is_init) |
| **pid_hndl** | Main handler of PID controller | pid_status_t pid_hndl(p_pid_t pid_inst, const pid_in_t * const p_in, pid_out_t * const p_out) |
| **pid_set_cfg** | Set controller settings | pid_status_t pid_set_cfg(p_pid_t pid_inst, const pid_cfg_t * const p_cfg) |
| **pid_get_cfg** | Get controller settings | pid_status_t pid_get_cfg(p_pid_t pid_inst, pid_cfg_t * const p_cfg) |

 NOTE: Detailed usage of filters be found in doxygen!



## Example of usage
---

```C
/**
 * 	Current regulator
 */
static p_pid_t 	g_current_ctrl = NULL;

/**
 * 	Current regulator settings
 *  
 * @note    Main handler is being process every 1ms (1kHz)! 
 */
static pid_cfg_t g_current_ctrl_cfg = 
{
    .kp 		= 1.0f,
	.ki 		= 10.0f,
	.kd 		= 0.0f,
	.ts 		= 1E-3f,
	.min 		= 0.0f,
	.max		= 1.0f,
	.lpf_d_fc	= 0.0f,
	.windup_k	= 0.0f,
};

/**
 *  Controller input/output data
 */
static pid_in_t  in;
static pid_out_t out;

@init
{
    // Init current controler
    if ( ePID_OK != pid_init( &g_current_ctrl, &g_current_ctrl_cfg ))
    {
        // Initialization failed
        // Further actions here...
    }
}

@1kHz period
{
    // Fill input data
    in.act = ...
    in.ref = ...
    in.ff = ...

    // Handle PID controller
	pid_hndl( g_current_ctrl, &in, &out );

    // Use output data here...
    // Output value: out.out
}

```

