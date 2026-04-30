//! Generic DSP primitives shared by every MFSK protocol.
//!
//! Nothing in this module knows about FT8, FT4 or any specific modulation —
//! it operates on raw sample buffers, sample rates, and target frequencies.
//! Protocol-aware DSP (sync correlators, LLR, etc.) lives outside `dsp`.

// rustfft consumers — gated behind `std` until the FFT trait
// abstraction lands. Embedded (`alloc`-only) builds still get the
// synthesis-side helpers (gfsk, resample) below.
#[cfg(feature = "std")]
pub mod downsample;
pub mod gfsk;
pub mod resample;
#[cfg(feature = "std")]
pub mod subtract;

#[cfg(feature = "std")]
pub use downsample::{DownsampleCfg, build_fft_cache, downsample, downsample_cached};
pub use gfsk::{GfskCfg, synth_f32, synth_i16};
pub use resample::{resample_f32_to_12k, resample_to_12k};
#[cfg(feature = "std")]
pub use subtract::{SubtractCfg, subtract_tones};
