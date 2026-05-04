//! Shared embedded glue for `m5stack-core2` (LX6) and `m5stack-s3`
//! (LX7) FT8-RX bins. See `Cargo.toml` for module list.

#![no_std]

extern crate alloc;

pub mod dual_core;
pub mod esp_dsp_fft;
pub mod internal_pool;
pub mod stage1_inc;
pub mod wav_sim;
