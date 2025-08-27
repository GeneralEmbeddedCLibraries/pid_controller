# Changelog
All notable changes to this project/module will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project/module adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---
## V1.0.0 - 27.08.2025

### Added 
 - PID controller written to be used as instance
 - Dual init option: dynamically and statically
 - Fully configurable in runtime
 - Support for **derivative on measurement** (no setpoint kick).  
 - Optional **low-pass filter (LPF)** for derivative term with configurable cutoff frequency.  
 - **Feed-forward input** included in the control law.  
 - `pid_set_d_lpf_fc()` API to adjust derivative LPF cutoff at runtime.  
 - New API functions for min/max setters: `pid_set_min()`, `pid_set_max()`.   
 - Telemetry accessors for P/I/D parts: `pid_get_p_part()`, `pid_get_i_part()`, `pid_get_d_part()`.  

---