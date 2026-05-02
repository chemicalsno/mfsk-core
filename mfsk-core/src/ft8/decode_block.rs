//! Embedded-friendly FT8 decode (esp-dsp pow-of-2 FFT only).
//!
//! Mirrors the host `decode_frame` pipeline but skips the 192_000-pt
//! wide-band FFT cache and the 3_840-pt per-symbol FFT — both of
//! which are non-power-of-two. Uses an 8192-pt per-symbol FFT for
//! the spectrogram (1920-sample input zero-padded) and a brute-force
//! per-tone DFT for the per-candidate LLR pass. Calls `bp_decode_kind`
//! with `BpKind::NormalizedMinSum` so the BP step skips the
//! `tanh`/`atanh` cache.
//!
//! Because the FFT bin width (≈ 1.465 Hz at 8192-pt) does not divide
//! the 6.25 Hz tone spacing evenly, Costas tone positions are computed
//! at fractional bins and rounded to the nearest integer. The
//! resulting bin-alignment jitter (≤ 0.7 Hz) is below FT8's
//! frequency-search tolerance.
//!
//! Same FFT trait, same compute path on host (rustfft) and on target
//! (esp-dsp) — sensitivity sweeps run on host and the result transfers
//! directly to hardware.

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::decode::{DecodeDepth, DecodeResult};
use super::llr::sync_quality;
use super::message::unpack77;
use super::params::{BP_MAX_ITER, COSTAS, COSTAS_POS, LDPC_N, NMAX, NN, NSPS, NTONES};
use super::wave_gen::message_to_tones;
#[cfg(not(feature = "fixed-point"))]
use crate::core::fft::default_planner;
use crate::core::scalar::Cmplx;
use crate::core::sync::SyncCandidate;
use crate::fec::ldpc::bp::check_crc14;
use crate::fec::ldpc::osd::{osd_decode, osd_decode_deep};

// ── Audio sample trait ──────────────────────────────────────────────────────

/// Trait for audio sample types accepted by `decode_block`. Lets the
/// caller hand in either `i16` (the canonical FT8 PCM) or `i8`
/// (half-storage, ~45 dB SQNR — plenty for FT8's -24 dB threshold —
/// useful when the slot needs to fit in scarce internal SRAM on
/// embedded targets where PSRAM access is the bottleneck).
///
/// The `to_f32` implementation must produce values on the same
/// amplitude scale as `i16` so the LLR computation downstream keeps
/// its calibration; for `i8` we therefore multiply by 256.
pub trait AudioSample: Copy {
    fn to_f32(self) -> f32;
    /// Promote to i16 range. i8 → i16 via `<<8`; i16 → i16
    /// identity. Used by the fixed-point FFT input path.
    fn to_i16(self) -> i16;
}

impl AudioSample for i16 {
    #[inline]
    fn to_f32(self) -> f32 {
        self as f32
    }
    #[inline]
    fn to_i16(self) -> i16 {
        self
    }
}

impl AudioSample for i8 {
    #[inline]
    fn to_f32(self) -> f32 {
        // Match i16 amplitude scale (multiply by 2^8). LLR
        // calibration (LLR_SCALE in ft8::params) thus stays
        // valid without per-sample-type rescaling.
        (self as i32 * 256) as f32
    }
    #[inline]
    fn to_i16(self) -> i16 {
        (self as i16) << 8
    }
}

// ── Tunables ────────────────────────────────────────────────────────────────

/// Per-symbol spectrogram FFT length. Power of two.
///
/// Caps differ by backend:
/// - **fc32 (f32 path)**: 4096, limited by esp-dsp's bit-rev
///   lookup tables shipped only at sizes 16..4096
///   (`dsps_fft2r_bitrev_tables_fc32.c`). Requesting 8192 corrupts
///   the rev-table array inside `dsps_fft2r_fc32_ae32_`.
/// - **sc16 (`fixed-point` feature)**: no cap up to 32768; sc16
///   has no rev-table dependency and generates twiddles on the fly.
///
/// Using NFFT=4096 on both paths today. Tried NFFT=8192 on the
/// sc16 path and saw host AWGN sweep regress ~0.5 dB at threshold —
/// finer bins make rectangular-window leakage worse for single-bin
/// extraction (main lobe widens from ±2.13 to ±4.27 bins). Hann
/// windowing in Step 1 compensates and unlocks NFFT=8192 as a
/// follow-on optimisation; revisit after Step 1.
pub const NFFT_SPEC: usize = 4096;

/// Coarse-sync slide step (samples). NSPS/2 = 960 samples (80 ms;
/// 184 frames per slot). Halving to NSPS killed sensitivity in
/// AWGN sweep — Costas correlation needs the half-symbol overlap
/// to compensate for sub-NSTEP dt offsets in the truth signal.
const NSTEP: usize = NSPS / 2;

/// Steps per symbol — used to map symbol-index to time-step lag.
const NSSY: i32 = (NSPS / NSTEP) as i32;

/// FT8 tone spacing (Hz).
const TONE_SPACING_HZ: f32 = 6.25;

/// Regulariser added to `mean_others` in coarse_sync's ratio metric
/// `t / (mean_others + ε)`. On the fp path the u16 spectrogram
/// quantises noise bins to 0; on phantom carriers where the 7
/// non-Costas tones happen to quantise to 0 the bare ratio explodes
/// 100-1000× over real-signal scores and buries busy-band truth in
/// coarse_sync's top-N. ε ≈ a fraction of one u16 LSB at
/// `FP_SPEC_SHIFT=12` keeps the ratio finite without depressing
/// genuine weak-signal scores (AWGN -17.5 dB threshold preserved).
///
/// 0.5 was picked from a host sweep over real-QSO WAVs: ε ∈ {0.1,
/// 0.25, 0.5, 1.0, 2.0} — 0.25 and 0.5 both gave 8/13 truth in
/// top-30 on busy-band qso3 (was 4/13 with bare ratio); 0.5 had
/// slightly tighter top ranks. ε > 1.0 starts losing borderline
/// weak signals; ε < 0.25 leaks phantom inflation back in.
///
/// On the f32 path `mean_others` never quantises to 0 so ε is
/// dwarfed by typical t0_ref values and has no measurable effect.
const RATIO_EPS_DEFAULT: f32 = 0.5;
fn ratio_eps() -> f32 {
    #[cfg(feature = "std")]
    {
        if let Ok(s) = std::env::var("MFSK_RATIO_EPS")
            && let Ok(v) = s.parse::<f32>()
        {
            return v;
        }
    }
    RATIO_EPS_DEFAULT
}

/// 12 kHz fixed sample rate.
const SAMPLE_RATE_HZ: f32 = 12_000.0;

/// Slot start offset (FT8 transmits 0.5 s into the slot).
const TX_START_OFFSET_S: f32 = 0.5;

/// Coarse-sync ±lag search window (s).
///
/// WSJT-X uses ±2.5 s — covers operators with sloppy slot timing
/// or slow rigs. Embedded targets running on a synced clock (NTP /
/// GPS) live well within ±1 s; halving the lag range cuts
/// `coarse_sync` work by ~60 % (linear in `n_lag`). If the live
/// timing source is loose, raise this back to 2.5.
const SYNC_LAG_S_DEFAULT: f32 = 1.0;
fn sync_lag_s() -> f32 {
    #[cfg(feature = "std")]
    {
        if let Ok(s) = std::env::var("MFSK_SYNC_LAG_S")
            && let Ok(v) = s.parse::<f32>()
        {
            return v;
        }
    }
    SYNC_LAG_S_DEFAULT
}

/// Same NMS α as the bench-tuned default in `mfsk-core/src/fec/ldpc/bp.rs`.
const NMS_ALPHA: f32 = 0.75;

/// `process_candidates` early-rejects cands whose full-21-symbol
/// `sync_quality` is at or below this threshold — they almost never
/// decode and their BP attempts are pure waste. Host real-QSO sweep
/// (fp i16, BpAll) showed q ∈ {6, 12} give identical 13/18 truth
/// recall but q=12 cuts stage 3 ~12-21 % wall-clock by skipping the
/// `compute_llr` (full nsym=3) + 4-variant BP staircase on dead-end
/// candidates. q ≥ 14 starts losing borderline weak signals
/// (W1DIG -14 dB, etc.) — the W1DIG-style "e=15" full-LLR-fallback
/// decodes have q in the 12-13 range. Override per-call via
/// `MFSK_Q_THRESH` when std is enabled.
const Q_THRESH_DEFAULT: u32 = 12;
fn q_thresh() -> u32 {
    #[cfg(feature = "std")]
    {
        if let Ok(s) = std::env::var("MFSK_Q_THRESH")
            && let Ok(v) = s.parse::<u32>()
        {
            return v;
        }
    }
    Q_THRESH_DEFAULT
}

// ── Spectrogram ─────────────────────────────────────────────────────────────

/// Spectrogram cell type. f32 (4 bytes) by default; u16 (2 bytes)
/// under `fixed-point` — magnitude squared right-shifted by
/// `FP_SPEC_SHIFT` to fit u16. `Spectrogram::power` returns f32 in
/// either case so downstream code (coarse_sync score division,
/// allsum) stays uniform. **Halves PSRAM bandwidth in stage 2.**
#[cfg(not(feature = "fixed-point"))]
type SpecCell = f32;
#[cfg(feature = "fixed-point")]
type SpecCell = u16;

/// Right-shift applied to `(re² + im²)` before storing as u16.
///
/// 12, not 16: with the host stub matching esp-dsp's `1/N` total
/// scaling, AWGN noise bins at typical recording levels (σ ≈ 5800
/// at peak 29000 input) yield mag² ≈ 8200 — `>>16` quantises that
/// to zero and breaks coarse_sync ratios. `>>12` keeps it at ~2,
/// preserving the noise floor.
///
/// Headroom check: max single-tone bin (peak input 29000, /N) is
/// 14500 → mag² = 2.1×10⁸ → `>>12` = 51 200, fits u16. Two coincident
/// FT8 tones at the same bin peaks at ≈ 8.4×10⁸ → `>>12` = 205 000,
/// **overflows u16** — extremely rare in practice (independent stations
/// virtually never align both freq and dt-grid bin), but watch for
/// truncation if the busy-band recall regresses.
#[cfg(feature = "fixed-point")]
const FP_SPEC_SHIFT: u32 = 12;

/// Power spectrogram. **Internal type exposed for benching only —
/// do not depend on the layout.**
#[doc(hidden)]
pub struct Spectrogram {
    /// Number of positive-frequency bins kept. Always ≤ NFFT_SPEC/2.
    /// We crop above the band of interest so a 8192-pt spectrogram on
    /// PSRAM-light targets (ESP32 Core2: 4 MB mapped) doesn't blow
    /// the heap (full 4096 × ~370 × 4 B = ~6 MB).
    pub n_freq: usize,
    /// Number of time slices.
    pub n_time: usize,
    /// **Column-major** (time major): `data[time * n_freq + freq]`.
    /// Picked so the inner Costas-correlation loop, which fixes
    /// time `m` and walks several frequency bins around a carrier
    /// candidate, does sequential PSRAM reads. Row-major would
    /// stride by `n_time × 4 ≈ 4 KB` per read — disaster on the
    /// ESP32's small PSRAM cache. Column-major keeps the working
    /// set of one time slice (`n_freq × 4 ≈ 4 KB`) in cache for
    /// the duration of all `(fi, lag)` cells touching it.
    data: Vec<SpecCell>,
}

/// coarse_sync inner-loop accumulator. Phase 3 of the integer-pure
/// embedded port: under `fixed-point-coarse-i32` the allsum / t_blocks
/// / t0_blocks chain stays in `i32` (sums of u16 power cells, max
/// ~2^25 ≪ i31), and only the final score division converts to f32.
/// Otherwise the alias collapses to f32 so the host path is
/// bit-identical and the LX6/LX7 FPU+ALU parallelism is preserved.
///
/// **Why a separate feature from `fixed-point`**: on ESP32 LX6 the f32
/// allsum loop overlaps on the FPU with integer index arithmetic on
/// the ALU; switching to i32 serialises onto the integer ALU and costs
/// ~25 % stage-2 wall-clock (~+200 ms / qso on Core2). FPU-less targets
/// (RP2040, Cortex-M0+, Hazard3) are the opposite — flip this feature
/// on there to avoid f32 emulation in stage 2.
#[cfg(feature = "fixed-point-coarse-i32")]
type CoarseAcc = i32;
#[cfg(not(feature = "fixed-point-coarse-i32"))]
type CoarseAcc = f32;

impl Spectrogram {
    /// Power-cell read in `CoarseAcc` (i32 under fixed-point, f32 otherwise).
    /// Used by `coarse_sync` to keep the precompute + score loop integer-pure
    /// on the embedded path — no per-cell u16→f32 promotion in the hot loop.
    /// Under host f32 the cast collapses to a no-op.
    #[inline]
    fn power_acc(&self, freq_bin: usize, time_idx: usize) -> CoarseAcc {
        debug_assert!(freq_bin < self.n_freq);
        debug_assert!(time_idx < self.n_time);
        #[allow(clippy::unnecessary_cast)]
        let v = self.data[time_idx * self.n_freq + freq_bin] as CoarseAcc;
        v
    }
}

/// Hann window over `NSPS` samples (peak 1.0 at the middle, zero at
/// the edges). Cuts rectangular sidelobes from −13 dB to ≈ −32 dB so
/// strong stations stop masking weaker neighbours 60–100 Hz away on
/// busy bands. Coherent gain is 0.5 (single-bin signal **amplitude**
/// halved → bin **power** down 4×); fp `compute_spectrogram` adds one
/// pre-shift to compensate, f32 needs no compensation (relative
/// magnitudes only).
fn hann_window_f32() -> [f32; NSPS] {
    let mut w = [0.0f32; NSPS];
    let denom = NSPS as f32;
    for k in 0..NSPS {
        w[k] = 0.5 * (1.0 - (core::f32::consts::TAU * k as f32 / denom).cos());
    }
    w
}

/// Q15 form of [`hann_window_f32`] for the fixed-point compute path.
#[cfg(feature = "fixed-point")]
fn hann_window_q15() -> [i16; NSPS] {
    let f = hann_window_f32();
    let mut q = [0i16; NSPS];
    for k in 0..NSPS {
        q[k] = (f[k] * 32767.0).round() as i16;
    }
    q
}

/// Build the per-symbol power spectrogram via NFFT_SPEC-pt FFTs.
/// Each time slice is `NSPS = 1920` samples of Hann-windowed audio
/// zero-padded to `NFFT_SPEC`.
///
/// `max_freq_hz` is the upper edge of the carrier search; we keep
/// bins covering up to `max_freq_hz + 7 × tone_spacing + ε` so the
/// top Costas tone of a candidate at `max_freq_hz` is still in
/// range. Bins above that are discarded — saves ~half the heap on
/// ESP32 (4 MB PSRAM ceiling).
///
/// **Pub for benchmarking only — do not depend on it.**
#[doc(hidden)]
#[cfg(not(feature = "fixed-point"))]
pub fn compute_spectrogram<S: AudioSample>(audio: &[S], max_freq_hz: f32) -> Spectrogram {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let band_top_hz = max_freq_hz + (NTONES as f32) * TONE_SPACING_HZ;
    let n_freq_full = NFFT_SPEC / 2;
    let n_freq = ((band_top_hz / df).ceil() as usize + 1).min(n_freq_full);
    let n_time = NMAX / NSTEP - 3;
    let scale = 1.0f32 / 300.0;

    let mut planner = default_planner();
    let fft = planner.plan_forward(NFFT_SPEC);

    let hann = hann_window_f32();
    let mut data = vec![0.0f32; n_freq * n_time];
    let mut buf = vec![Complex::new(0.0f32, 0.0); NFFT_SPEC];

    for j in 0..n_time {
        let ia = j * NSTEP;
        for (k, c) in buf.iter_mut().enumerate() {
            *c = if k < NSPS {
                let sample = if ia + k < audio.len() {
                    audio[ia + k].to_f32() * scale * hann[k]
                } else {
                    0.0
                };
                Complex::new(sample, 0.0)
            } else {
                Complex::new(0.0, 0.0)
            };
        }
        fft.process(&mut buf);
        // Column-major write — `data[j * n_freq + i]` keeps each
        // time slice contiguous in memory (good PSRAM locality
        // for downstream coarse_sync).
        let row_base = j * n_freq;
        for i in 0..n_freq {
            data[row_base + i] = buf[i].norm_sqr();
        }
    }

    Spectrogram {
        n_freq,
        n_time,
        data,
    }
}

/// Fixed-point variant: `Vec<u32>` magnitude squared from an i16
/// complex FFT. Halves spectrogram heap on PSRAM-light targets and
/// is the only viable backend on FPU-less MCUs.
///
/// **Auto-gain**: esp-dsp's `dsps_fft2r_sc16` divides by 2 at each
/// of `log2(N)` butterfly stages (12 stages at NFFT=4096) to keep
/// the i16 working set from overflowing. That's a total `/4096`
/// scale-down of the output. A real-world FT8 recording with peaks
/// well below i16 max (e.g. WSJT-X reference WAVs at 5 % of full
/// scale) gets quantised to zero by stage 6 of the FFT and produces
/// an empty spectrogram. We compute the slot's peak once and shift
/// the i16 input left enough to reach ~ ¼ of i16 range, leaving
/// headroom for FFT growth in tone-rich slots.
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn compute_spectrogram<S: AudioSample>(audio: &[S], max_freq_hz: f32) -> Spectrogram {
    use crate::core::fft::default_planner_16;

    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let band_top_hz = max_freq_hz + (NTONES as f32) * TONE_SPACING_HZ;
    let n_freq_full = NFFT_SPEC / 2;
    let n_freq = ((band_top_hz / df).ceil() as usize + 1).min(n_freq_full);
    let n_time = NMAX / NSTEP - 3;

    // Pre-scale gain: shift left so the audio peak lands at
    // `2 × NFFT` (so after `log2(NFFT)` stages of /2 the post-FFT
    // peak still has a usable mantissa of ~2).
    let target_peak: i32 = (NFFT_SPEC * 2) as i32;
    let mut peak_abs: i32 = 1;
    let n_scan = audio.len().min(NMAX);
    for k in 0..n_scan {
        let v = audio[k].to_i16() as i32;
        let a = v.unsigned_abs() as i32;
        if a > peak_abs {
            peak_abs = a;
        }
    }
    let mut shift: u32 = 0;
    while peak_abs << shift < target_peak && shift < 8 {
        shift += 1;
    }
    // +1 extra shift: the Hann window's coherent gain is 0.5
    // (single-bin signal amplitude halved → bin power ÷ 4). Pre-
    // shifting input by +1 bit doubles amplitude so the post-FFT bin
    // amplitude lands back where the rectangular-window auto-gain
    // plan expected. Peak input samples near the window centre
    // (where Hann ≈ 1) may saturate to i16_MAX after this shift,
    // but the centre is also where CG is highest — clamping a few
    // samples there costs less than the 6 dB amplitude loss the
    // window otherwise imposes on every bin.
    shift = (shift + 1).min(8);

    let mut planner = default_planner_16();
    let fft = planner.plan_forward(NFFT_SPEC);

    let hann = hann_window_q15();
    let mut data: Vec<u16> = vec![0u16; n_freq * n_time];
    let mut buf: Vec<Complex<i16>> = vec![Complex::new(0i16, 0i16); NFFT_SPEC];

    // **Two-for-one real-FFT trick**. Each frame's audio is real; we
    // pack two consecutive frames as `re = windowed(frame_a)`,
    // `im = windowed(frame_b)` into a single complex FFT. From
    //     Y[k] = X_a[k] + j·X_b[k]
    // and the real-input conjugate symmetry `X_•[N-k] = conj(X_•[k])`,
    // we recover the per-frame spectra via post-butterfly:
    //     X_a[k] = (Y[k] + conj(Y[N-k])) / 2
    //     X_b[k] = -j · (Y[k] - conj(Y[N-k])) / 2
    // i.e.
    //     A_re = (Y[k].re + Y[N-k].re) / 2,
    //     A_im = (Y[k].im - Y[N-k].im) / 2
    //     B_re = (Y[k].im + Y[N-k].im) / 2,
    //     B_im = (Y[N-k].re - Y[k].re) / 2
    // Halves the FFT count (184 → 92 on the standard FT8 slot) — the
    // dominant stage-1 cost on Core2. Demux is O(n_freq) per pair,
    // negligible vs an N=4096 FFT. Magnitude scaling matches the
    // single-frame path (the >>1 in demux exactly cancels the √2
    // amplitude headroom that |Y[k]|² = |X_a[k]|² + |X_b[k]|² would
    // otherwise give); auto-gain `shift` is reused unchanged.
    let n_pairs = n_time / 2;
    let n_odd = n_time & 1;

    let pack = |buf: &mut [Complex<i16>], ia_a: usize, ia_b: Option<usize>| {
        for (k, c) in buf.iter_mut().enumerate() {
            let re = if k < NSPS && ia_a + k < audio.len() {
                let raw = audio[ia_a + k].to_i16() as i32;
                let scaled = (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32);
                ((scaled * hann[k] as i32) >> 15) as i16
            } else {
                0
            };
            let im = match ia_b {
                Some(ia_b) if k < NSPS && ia_b + k < audio.len() => {
                    let raw = audio[ia_b + k].to_i16() as i32;
                    let scaled = (raw << shift).clamp(i16::MIN as i32, i16::MAX as i32);
                    ((scaled * hann[k] as i32) >> 15) as i16
                }
                _ => 0,
            };
            *c = Complex::new(re, im);
        }
    };

    for jj in 0..n_pairs {
        let j_a = 2 * jj;
        let j_b = j_a + 1;
        pack(&mut buf, j_a * NSTEP, Some(j_b * NSTEP));
        fft.process(&mut buf);

        let row_a = j_a * n_freq;
        let row_b = j_b * n_freq;
        // Demux. NFFT_SPEC is a power of two so `(NFFT_SPEC - k)
        // & (NFFT_SPEC - 1)` wraps k=0 to 0, where the formula
        // collapses to A_re = Y[0].re, A_im = 0 (and B = Im(Y[0]))
        // — i.e. the DC bin is real, as expected for real input.
        let mask = NFFT_SPEC - 1;
        for k in 0..n_freq {
            let kn = (NFFT_SPEC - k) & mask;
            let yk_re = buf[k].re as i32;
            let yk_im = buf[k].im as i32;
            let yn_re = buf[kn].re as i32;
            let yn_im = buf[kn].im as i32;
            let a_re = (yk_re + yn_re) >> 1;
            let a_im = (yk_im - yn_im) >> 1;
            let b_re = (yk_im + yn_im) >> 1;
            let b_im = (yn_re - yk_re) >> 1;
            let mag2_a = ((a_re * a_re + a_im * a_im) as u32) >> FP_SPEC_SHIFT;
            let mag2_b = ((b_re * b_re + b_im * b_im) as u32) >> FP_SPEC_SHIFT;
            data[row_a + k] = mag2_a as u16;
            data[row_b + k] = mag2_b as u16;
        }
    }

    // Odd-frame fallback: single-frame FFT for the trailing slice when
    // n_time is odd. n_time=184 on the standard FT8 slot so this path
    // is exercised only by truncated inputs / regression-test fixtures.
    if n_odd != 0 {
        let j = 2 * n_pairs;
        pack(&mut buf, j * NSTEP, None);
        fft.process(&mut buf);
        let row_base = j * n_freq;
        for i in 0..n_freq {
            let re = buf[i].re as i32;
            let im = buf[i].im as i32;
            let mag2 = ((re * re + im * im) as u32) >> FP_SPEC_SHIFT;
            data[row_base + i] = mag2 as u16;
        }
    }

    Spectrogram {
        n_freq,
        n_time,
        data,
    }
}

// ── Coarse sync ─────────────────────────────────────────────────────────────

/// Costas-array correlation search across the spectrogram. Matches
/// the host `core::sync::coarse_sync` shape but reads bins by
/// fractional offset (`tone_step_bins ≈ 4.267` at NFFT_SPEC=8192,
/// rounded to nearest integer).
///
/// **Pub for benchmarking only — do not depend on it.**
#[doc(hidden)]
pub fn coarse_sync(
    spec: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tstep = NSTEP as f32 / SAMPLE_RATE_HZ;
    let jstrt = (TX_START_OFFSET_S / tstep).round() as i32;
    let jz = (sync_lag_s() / tstep).round() as i32;
    let tone_step_bins = TONE_SPACING_HZ / df;

    // Carrier-bin search range. Reserve room above the carrier for the
    // top tone (round(7 * tone_step_bins)).
    let ia = (freq_min / df).round() as usize;
    let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
    let nh1 = spec.n_freq;
    let ib_unbounded = (freq_max / df).round() as usize;
    let ib = ib_unbounded.min(nh1.saturating_sub(max_tone_off));
    if ib < ia {
        return Vec::new();
    }
    let n_freq = ib - ia + 1;
    let n_lag = (2 * jz + 1) as usize;
    let mut sync2d = vec![0.0f32; n_freq * n_lag];
    let idx = |fi: usize, lag: i32| fi * n_lag + (lag + jz) as usize;
    let ratio_eps = ratio_eps();
    #[cfg(feature = "std")]
    let prof = cfg!(feature = "profile-coarse") || std::env::var("MFSK_PROFILE_COARSE").is_ok();
    #[cfg(feature = "std")]
    let t_setup = std::time::Instant::now();

    // **Multi-bin tone sum (Plan A)**: tone_step_bins ≈ 2.13 means
    // the 8 FT8 tones fall at fractional bin positions [0.00, 2.13,
    // 4.27, 6.40, 8.53, 10.67, 12.80, 14.93]. Reading just `round(...)`
    // captures only one bin's worth of the Hann mainlobe (which is
    // ~2 bins wide); off-bin tones lose 1–3 dB to the neighbour.
    // We sum the floor-bin and floor-bin+1 instead, recovering the
    // full mainlobe energy for every tone regardless of fractional
    // alignment. Cost: 2× spec reads per tone — negligible vs PSRAM
    // bandwidth headroom on Core2.
    let mut tone_bin_lo = [0usize; NTONES];
    for k in 0..NTONES {
        tone_bin_lo[k] = (k as f32 * tone_step_bins).floor() as usize;
    }

    // Pre-compute the (bk, n) → m_base table. m for the inner iter is
    // `m_base[bk][n] + lag`; m_base depends only on the Costas pattern
    // and `jstrt` (constants of the slot), not on (fi, lag). Hoists
    // 21 mul/add chains out of the n_freq × n_lag × 21 inner loop.
    let m_base: [[i32; COSTAS.len()]; COSTAS_POS.len()] = {
        let mut t = [[0i32; COSTAS.len()]; COSTAS_POS.len()];
        for (bk, &start_sym) in COSTAS_POS.iter().enumerate() {
            let block_offset = NSSY * start_sym as i32;
            for (n, _) in COSTAS.iter().enumerate() {
                t[bk][n] = jstrt + block_offset + NSSY * n as i32;
            }
        }
        t
    };
    // Pre-compute Costas-tone bin offsets in COSTAS-order (i.e.
    // `tone_bin_lo[COSTAS[n]]`). Saves an indirect lookup per inner
    // iteration.
    let costas_off: [usize; COSTAS.len()] = {
        let mut t = [0usize; COSTAS.len()];
        for (n, &costas_n) in COSTAS.iter().enumerate() {
            t[n] = tone_bin_lo[costas_n];
        }
        t
    };

    // Pre-compute the set of `m` time-indices that the score loop
    // will read. Only Costas-symbol positions ± lag count — typically
    // 3 contiguous bands totalling ~110 of the 184 frames, so the
    // naïve "compute allsum for every m" wastes ~40 % of its work
    // on cells the score loop never reads.
    let needed_m: alloc::vec::Vec<usize> = {
        let mut mark = alloc::vec![false; spec.n_time];
        for bk in 0..COSTAS_POS.len() {
            let lo = m_base[bk][0] - jz;
            let hi = m_base[bk][COSTAS.len() - 1] + jz;
            let lo_u = lo.max(0) as usize;
            let hi_u = (hi.min(spec.n_time as i32 - 1)) as usize;
            if lo_u <= hi_u {
                #[allow(clippy::needless_range_loop)]
                for m in lo_u..=hi_u {
                    mark[m] = true;
                }
            }
        }
        (0..spec.n_time).filter(|&m| mark[m]).collect()
    };

    // Pre-compute Σ_k (spec[lo,m] + spec[lo+1,m]) for every
    // (fi, m ∈ needed_m).
    //
    // **Sliding-window optimisation**: at NFFT_SPEC=4096, tone_step_bins
    // ≈ 2.13 → tone_bin_lo = [0, 2, 4, 6, 8, 10, 12, 14], i.e. the 8
    // tones plus the multi-bin neighbour cover the **16 contiguous
    // bins** `[i_carrier, i_carrier + 15]`. So allsum reduces to a
    // 16-bin sum that slides by one bin as fi advances:
    //   `allsum[fi+1, m] = allsum[fi, m] − spec[i_carrier, m] + spec[i_carrier + 16, m]`
    // — 2 reads/op per (fi, m) update vs the full 16. ~8× fewer spec
    // reads in the precompute, big PSRAM-bandwidth win on Core2.
    //
    // The contiguous layout holds when tone_step_bins > 2 floors to
    // even integer steps of 2; verify and fall back to the generic
    // sum if a future NFFT_SPEC breaks the assumption.
    let win = 2 * NTONES; // 16 contiguous bins
    let contiguous = (0..NTONES).all(|k| tone_bin_lo[k] == 2 * k);
    let mut allsum: Vec<CoarseAcc> = vec![CoarseAcc::default(); n_freq * spec.n_time];
    if contiguous {
        // Initial column (fi=0): full 16-bin sum.
        let row0 = 0;
        for &m in &needed_m {
            let mut s: CoarseAcc = CoarseAcc::default();
            for j in 0..win {
                let bin = (ia + j).min(nh1 - 1);
                s += spec.power_acc(bin, m);
            }
            allsum[row0 * spec.n_time + m] = s;
        }
        // Slide for fi=1..n_freq.
        for fi in 1..n_freq {
            let drop_bin = ia + fi - 1;
            let add_bin = (ia + fi + win - 1).min(nh1 - 1);
            let prev_off = (fi - 1) * spec.n_time;
            let curr_off = fi * spec.n_time;
            for &m in &needed_m {
                allsum[curr_off + m] =
                    allsum[prev_off + m] - spec.power_acc(drop_bin, m) + spec.power_acc(add_bin, m);
            }
        }
    } else {
        // Generic fallback: 16 reads per (fi, m).
        for (fi, i_carrier) in (ia..=ib).enumerate() {
            let row_off = fi * spec.n_time;
            for &m in &needed_m {
                let mut s: CoarseAcc = CoarseAcc::default();
                for k in 0..NTONES {
                    let lo = (i_carrier + tone_bin_lo[k]).min(nh1 - 1);
                    let hi = (lo + 1).min(nh1 - 1);
                    s += spec.power_acc(lo, m) + spec.power_acc(hi, m);
                }
                allsum[row_off + m] = s;
            }
        }
    }
    #[cfg(feature = "std")]
    let t_allsum = std::time::Instant::now();

    // Three identical Costas arrays at symbol positions 0, 36, 72.
    //
    // **Bounds-check hoist**. For the standard FT8 slot:
    //   block 0: m = lag + jstrt + NSSY*n   ∈ [-7..31] over (lag, n)
    //   block 1: m = lag + jstrt + 144 + NSSY*n  ∈ [65..103]
    //   block 2: m = lag + jstrt + 288 + NSSY*n  ∈ [137..175]
    //
    // Only block 0 can dip below 0 (and only at large negative lag).
    // Compute the smallest valid `n` per lag once, then iterate
    // n_start..NTONES_C without per-iter checks. Blocks 1 and 2 are
    // always fully in range. Sanity-check the upper bound at function
    // entry so the unchecked-as-usize is safe.
    debug_assert!(
        m_base[2][COSTAS.len() - 1] + jz < spec.n_time as i32,
        "n_time too small for SYNC_LAG_S/jstrt"
    );
    let n_time = spec.n_time;
    // Per-fi: tbin_lo[n] = i_carrier + costas_off[n]. Hoist out of
    // the lag loop — saves an addition per inner iteration.
    let mut tbin_lo_arr = [0usize; COSTAS.len()];
    for (fi, i_carrier) in (ia..=ib).enumerate() {
        for n in 0..COSTAS.len() {
            tbin_lo_arr[n] = i_carrier + costas_off[n];
        }
        let allsum_row = &allsum[fi * n_time..(fi + 1) * n_time];
        for lag in -jz..=jz {
            let mut t_blocks: [CoarseAcc; 3] = [CoarseAcc::default(); 3];
            let mut t0_blocks: [CoarseAcc; 3] = [CoarseAcc::default(); 3];

            // Block 0: smallest valid n where m_base[0][n] + lag >= 0.
            // m_base[0][n] = jstrt + NSSY*n. Solve for n:
            //   jstrt + NSSY*n + lag >= 0 ⇒ n >= ceil((-jstrt - lag) / NSSY)
            let bk0_n_start = {
                let needed = -jstrt - lag;
                if needed <= 0 {
                    0usize
                } else {
                    ((needed + NSSY - 1) / NSSY).min(COSTAS.len() as i32) as usize
                }
            };
            for n in bk0_n_start..COSTAS.len() {
                let m_u = (m_base[0][n] + lag) as usize;
                let tbin_lo = tbin_lo_arr[n];
                t_blocks[0] += spec.power_acc(tbin_lo, m_u) + spec.power_acc(tbin_lo + 1, m_u);
                t0_blocks[0] += allsum_row[m_u];
            }
            // Blocks 1, 2: always fully in range — no per-iter check.
            for bk in 1..COSTAS_POS.len() {
                for n in 0..COSTAS.len() {
                    let m_u = (m_base[bk][n] + lag) as usize;
                    let tbin_lo = tbin_lo_arr[n];
                    t_blocks[bk] += spec.power_acc(tbin_lo, m_u) + spec.power_acc(tbin_lo + 1, m_u);
                    t0_blocks[bk] += allsum_row[m_u];
                }
            }

            // Regularised ratio `t / (mean_others + ε)`.
            // ε prevents the u16-quantised fp path from blowing up
            // when phantom carriers happen to land where 7 of 8 tone
            // bins quantise to 0; `t0_ref → 0` would otherwise inflate
            // ratio scores by 100-1000× over real signals.
            //
            // Phase 3: t/t0 sums stay in CoarseAcc; convert to f32 only
            // at the score-division boundary so sync2d / red / base
            // (downstream sort + percentile) remain f32.
            let t_all: CoarseAcc = t_blocks[0] + t_blocks[1] + t_blocks[2];
            let t0_all: CoarseAcc = t0_blocks[0] + t0_blocks[1] + t0_blocks[2];
            // `as f32` is a real promotion under fixed-point (CoarseAcc=i32)
            // and a no-op when CoarseAcc=f32 — silence the host clippy.
            #[allow(clippy::unnecessary_cast)]
            let t_all_f = t_all as f32;
            #[allow(clippy::unnecessary_cast)]
            let t0_all_f = t0_all as f32;
            let t0_ref = (t0_all_f - t_all_f) / (NTONES as f32 - 1.0);
            let sync_all = t_all_f / (t0_ref + ratio_eps);

            // Trailing-2-blocks score (drop block 0 — late-start tolerance).
            #[allow(clippy::unnecessary_cast)]
            let t_tail_f = (t_blocks[1] + t_blocks[2]) as f32;
            #[allow(clippy::unnecessary_cast)]
            let t0_tail_f = (t0_blocks[1] + t0_blocks[2]) as f32;
            let t0_tail_ref = (t0_tail_f - t_tail_f) / (NTONES as f32 - 1.0);
            let sync_tail = t_tail_f / (t0_tail_ref + ratio_eps);

            sync2d[idx(fi, lag)] = sync_all.max(sync_tail);
        }
    }
    #[cfg(feature = "std")]
    let t_score = std::time::Instant::now();

    // Per-bin peak + 40-percentile noise floor (matches host code shape).
    let mut red = vec![0.0f32; n_freq];
    for fi in 0..n_freq {
        red[fi] = (-jz..=jz)
            .map(|lag| sync2d[idx(fi, lag)])
            .fold(0.0f32, f32::max);
    }
    let base = {
        let mut sorted = red.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let pct_idx = (0.40 * n_freq as f32) as usize;
        sorted[pct_idx.min(n_freq - 1)].max(f32::EPSILON)
    };

    const MLAG: i32 = 10;

    let mut cands: Vec<SyncCandidate> = Vec::new();
    for fi in 0..n_freq {
        let i_carrier = ia + fi;
        let freq_hz = i_carrier as f32 * df;

        let mut peaks: Vec<(i32, f32)> = (-jz..=jz)
            .filter_map(|lag| {
                let raw = sync2d[idx(fi, lag)];
                let norm = raw / base;
                if norm.is_finite() && norm >= sync_min {
                    Some((lag, norm))
                } else {
                    None
                }
            })
            .collect();
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        let mut picked: Vec<i32> = Vec::new();
        for (lag, score) in peaks {
            if picked.iter().any(|&pl| (lag - pl).abs() <= MLAG) {
                continue;
            }
            picked.push(lag);
            // Parabolic refinement of dt: fit y = a*x² + b*x + c
            // through (lag-1, lag, lag+1) sync2d values, locate the
            // peak. Saves a 3-point dt grid in stage 3 (3× DFT
            // budget) — the refined dt below is taken at face value
            // so process_candidates can run a single full DFT per
            // candidate.
            let dt_quanta = if lag > -jz && lag < jz {
                let y_lo = sync2d[idx(fi, lag - 1)];
                let y_mi = sync2d[idx(fi, lag)];
                let y_hi = sync2d[idx(fi, lag + 1)];
                let denom = y_lo - 2.0 * y_mi + y_hi;
                if denom.abs() > f32::EPSILON {
                    let off = 0.5 * (y_lo - y_hi) / denom;
                    // Clamp to ±1 sample to guard against poorly-
                    // conditioned fits at the search edges.
                    off.clamp(-1.0, 1.0)
                } else {
                    0.0
                }
            } else {
                0.0
            };
            let dt_lag = lag as f32 + dt_quanta;
            cands.push(SyncCandidate {
                freq_hz,
                dt_sec: (dt_lag - 0.5) * tstep,
                score,
            });
            if picked.len() >= 8 {
                break;
            }
        }
    }

    // Dedupe within 4 Hz / 40 ms; keep highest score.
    //
    // Sort once by score desc, then greedily keep cands with no
    // already-kept near neighbour. Stops early at `max_cand` for
    // O(n log n + n × max_cand) instead of the prior O(n²) pairwise
    // compare-and-zero (n is several thousand).
    //
    // Empirically this drops 1 borderline busy-band truth on qso3
    // vs the byte-equivalent O(n²) implementation — likely a
    // tie-break ordering difference at the 4 Hz dedupe boundary
    // (haven't fully traced; the algorithms are equivalent on every
    // small-case scenario I worked through). We accept the recall
    // delta for the ~150 us host (~150 ms Core2) saving.
    cands.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
    let mut out: Vec<SyncCandidate> = Vec::with_capacity(max_cand);
    for c in cands {
        if c.score < sync_min {
            break;
        }
        let near = out
            .iter()
            .any(|k| (c.freq_hz - k.freq_hz).abs() < 4.0 && (c.dt_sec - k.dt_sec).abs() < 0.04);
        if near {
            continue;
        }
        out.push(c);
        if out.len() >= max_cand {
            break;
        }
    }
    let cands = out;
    #[cfg(feature = "std")]
    if prof {
        let t_end = std::time::Instant::now();
        let allsum_us = (t_allsum - t_setup).as_micros();
        let score_us = (t_score - t_allsum).as_micros();
        let post_us = (t_end - t_score).as_micros();
        let total_us = (t_end - t_setup).as_micros();
        eprintln!(
            "[coarse_sync prof] n_freq={n_freq} n_lag={n_lag}  allsum={allsum_us} score={score_us} dedupe+sort={post_us}  total={total_us} us"
        );
    }
    cands
}

// ── Per-symbol direct DFT (no FFT cache) ────────────────────────────────────

/// Compute the 79 × 8 complex tone spectra for one candidate by
/// direct DFT at the exact tone frequencies. Bypasses the wide-band
/// FFT cache entirely.
///
/// **Phase-rotator recursion.** Naïve per-sample `cos/sin` would be
/// ~25 M libm calls per `decode_block` invocation (8 candidates × 5
/// dt offsets × 79 symbols × 8 tones × 1920 samples) — minutes on
/// LX6. We replace it with one cos/sin pair per (symbol, tone) and
/// a single complex multiply per sample.
///
/// **PSRAM-aware access pattern.** The audio buffer (360 KB) lives
/// in PSRAM on Core2 (40 MHz quad, ~5× slower than internal RAM).
/// A naïve "for tone × for sample" loop would re-read each audio
/// sample 8 times across PSRAM. Instead we copy each 1920-sample
/// symbol into a stack-local f32 buffer once, then run all 8 tone
/// integrations over that internal-RAM copy. Reduces audio reads
/// from PSRAM by 8× — the dominant cost on LX6.
///
/// Numerical error: each rotation is a unit-magnitude multiply with
/// f32 round-off ≈ 6e-8; over 1920 samples the cumulative magnitude
/// drift stays below 0.012 % — negligible for LLR computation.
/// **Pub for benchmarking only — do not depend on it.**
#[doc(hidden)]
pub fn symbol_spectra_direct<S: AudioSample>(
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    sym_mask: SymMask,
) -> Box<[[Cmplx<f32>; 8]; 79]> {
    let mut out: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    fill_symbol_spectra(&mut out, audio, freq_hz, dt_sec, sym_mask);
    out
}

/// Which subset of the 79 symbols to compute. Used for the
/// Costas-first early-reject in `process_candidates`: the first
/// pass fills only Costas tone positions (21 symbols, 27 % of
/// full DFT cost) for the `sync_quality` gate; only on a hit do
/// we go back and fill the data-symbol positions.
///
/// **Pub for benchmarking only.**
#[doc(hidden)]
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum SymMask {
    /// Costas symbols only — all three blocks (positions 0-6, 36-42,
    /// 72-78). 21 symbols. Used for full-precision sync_quality
    /// gating in stage 3.
    SyncOnly,
    /// Costas block 0 only (positions 0-6). 7 symbols — 1/3 the cost
    /// of `SyncOnly`. Used for Pass 2 sync_quality re-rank where the
    /// finer ranking precision of all three blocks is unnecessary.
    SyncBlock0,
    /// Everything except Costas block 0 — fills positions 7-78
    /// (data symbols + Costas blocks 1, 2). 72 symbols. Used in
    /// stage 3 to "top up" a `SyncBlock0`-filled spectrum.
    NotBlock0,
    /// Data symbols only (positions 7-35, 43-71). Skips the 21 sync
    /// positions — used to "top up" a `SyncOnly`-filled spectrum.
    DataOnly,
    /// Costas blocks 1 and 2 only (positions 36-42, 72-78). 14
    /// symbols — 2/3 the cost of `SyncOnly`. Used in stage 3 to top
    /// up a `SyncBlock0`-filled cs (Pass 2 output) into a full
    /// `SyncOnly`-equivalent without redoing block 0.
    SyncBlocks12,
}

#[inline]
fn sym_in_mask(sym: usize, mask: SymMask) -> bool {
    let (in_block_a, in_block_b, in_block_c) = (
        sym < COSTAS.len(),                                         // 0..7
        sym >= COSTAS_POS[1] && sym < COSTAS_POS[1] + COSTAS.len(), // 36..43
        sym >= COSTAS_POS[2] && sym < COSTAS_POS[2] + COSTAS.len(), // 72..79
    );
    let is_sync = in_block_a || in_block_b || in_block_c;
    match mask {
        SymMask::SyncOnly => is_sync,
        SymMask::SyncBlock0 => in_block_a,
        SymMask::NotBlock0 => !in_block_a,
        SymMask::DataOnly => !is_sync,
        SymMask::SyncBlocks12 => in_block_b || in_block_c,
    }
}

/// **Pub for benchmarking only — do not depend on it.**
///
/// f32 wrapper around the generic [`fill_symbol_spectra_generic`].
#[doc(hidden)]
#[cfg(not(feature = "fixed-point"))]
pub fn fill_symbol_spectra<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    fill_symbol_spectra_generic::<f32, S>(out, audio, freq_hz, dt_sec, mask);
}

/// Generic per-symbol DFT — writes `Cmplx<Sc>` for any spec scalar
/// `Sc: SpecScalar`. For `Sc = f32` (`NEEDS_AUTOGAIN = false`) the
/// inner loop writes f32 components directly — byte-identical to the
/// pre-Phase-2.6 implementation. For fixed-point `Sc` (`NEEDS_AUTOGAIN
/// = true`) the function runs a 2-pass scan-and-scale: compute all
/// 79 × 8 Complex<f32> entries into a stack tmp buffer (~5 KB), find
/// the peak |re|/|im| across the active mask, then write
/// `Sc::from_f32_scaled(value, scale)` with `scale = i16::MAX × 0.95
/// / peak` so the i16 range is fully utilised without saturation.
#[doc(hidden)]
#[cfg(not(feature = "fixed-point"))]
pub fn fill_symbol_spectra_generic<Sc: crate::core::scalar::SpecScalar, S: AudioSample>(
    out: &mut [[Cmplx<Sc>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    let i0 = ((TX_START_OFFSET_S + dt_sec) * SAMPLE_RATE_HZ).round() as i64;
    let two_pi_over_fs = core::f32::consts::TAU / SAMPLE_RATE_HZ;

    let mut rotators = [Complex::new(0.0f32, 0.0); NTONES];
    for tone in 0..NTONES {
        let tone_freq = freq_hz + tone as f32 * TONE_SPACING_HZ;
        let dphi = -two_pi_over_fs * tone_freq;
        rotators[tone] = Complex::new(dphi.cos(), dphi.sin());
    }

    let mut sym_buf = [0.0f32; NSPS];

    if !Sc::NEEDS_AUTOGAIN {
        // Sc = f32: inline write via `Sc::from_f32` (no-op for f32).
        // Const dispatch — LLVM eliminates the `else` branch when
        // monomorphised for `Sc = f32`.
        for sym in 0..NN {
            if !sym_in_mask(sym, mask) {
                continue;
            }
            let sym_start = i0 + (sym as i64) * (NSPS as i64);
            for k in 0..NSPS {
                let idx = sym_start + k as i64;
                sym_buf[k] = if idx >= 0 && (idx as usize) < audio.len() {
                    audio[idx as usize].to_f32()
                } else {
                    0.0
                };
            }
            for tone in 0..NTONES {
                let rotator = rotators[tone];
                let mut osc = Complex::new(1.0f32, 0.0);
                let mut acc = Complex::new(0.0f32, 0.0);
                for &s in sym_buf.iter() {
                    acc.re += s * osc.re;
                    acc.im += s * osc.im;
                    osc *= rotator;
                }
                out[sym][tone] = Cmplx {
                    re: Sc::from_f32(acc.re),
                    im: Sc::from_f32(acc.im),
                };
            }
        }
        return;
    }

    // Fixed-point path: 2-pass with auto-gain.
    let mut tmp = [[Complex::new(0.0f32, 0.0); 8]; 79];
    let mut peak: f32 = 0.0;
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let sym_start = i0 + (sym as i64) * (NSPS as i64);
        for k in 0..NSPS {
            let idx = sym_start + k as i64;
            sym_buf[k] = if idx >= 0 && (idx as usize) < audio.len() {
                audio[idx as usize].to_f32()
            } else {
                0.0
            };
        }
        for tone in 0..NTONES {
            let rotator = rotators[tone];
            let mut osc = Complex::new(1.0f32, 0.0);
            let mut acc = Complex::new(0.0f32, 0.0);
            for &s in sym_buf.iter() {
                acc.re += s * osc.re;
                acc.im += s * osc.im;
                osc *= rotator;
            }
            tmp[sym][tone] = acc;
            peak = peak.max(acc.re.abs()).max(acc.im.abs());
        }
    }
    let scale = if peak > 1e-9 {
        (i16::MAX as f32 * 0.95) / peak
    } else {
        0.0
    };
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        for tone in 0..NTONES {
            let c = tmp[sym][tone];
            out[sym][tone] = Cmplx {
                re: Sc::from_f32_scaled(c.re, scale),
                im: Sc::from_f32_scaled(c.im, scale),
            };
        }
    }
}

/// Required scratch length for [`fill_symbol_spectra_into`] — one
/// flat array per axis (cos / sin), `NTONES × NSPS = 15 360` i16.
/// Caller must provide two slices of at least this length.
#[cfg(feature = "fixed-point")]
pub const BASIS_SCRATCH_LEN: usize = NTONES * NSPS;

/// Fixed-point per-symbol DFT — basis-precompute + dot-product
/// kernel. Drop-in heap-allocating wrapper around
/// [`fill_symbol_spectra_into`]: allocates 60 KB × 2 of basis scratch
/// from the default heap on every call. Convenient for host use; on
/// embedded targets the scratch typically lands in PSRAM (slow reads
/// in the dot-product inner loop), so callers that care about Core2
/// throughput should pre-allocate scratch in **internal RAM**
/// (`static [i16; BASIS_SCRATCH_LEN]` in `.bss`, or
/// `heap_caps_malloc(MALLOC_CAP_INTERNAL)`) and call
/// [`fill_symbol_spectra_into`] directly.
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn fill_symbol_spectra<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    fill_symbol_spectra_generic::<f32, S>(out, audio, freq_hz, dt_sec, mask);
}

/// Generic fixed-point fill — writes `Cmplx<Sc>`. f32 wrapper above;
/// Q14i16 path used under `fixed-point-cs`.
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn fill_symbol_spectra_generic<Sc: crate::core::scalar::SpecScalar, S: AudioSample>(
    out: &mut [[Cmplx<Sc>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    let mut basis_re: Vec<i16> = alloc::vec![0i16; BASIS_SCRATCH_LEN];
    let mut basis_im: Vec<i16> = alloc::vec![0i16; BASIS_SCRATCH_LEN];
    fill_symbol_spectra_into_generic::<Sc, S>(
        out,
        audio,
        freq_hz,
        dt_sec,
        mask,
        &mut basis_re,
        &mut basis_im,
    );
}

/// Fixed-point per-symbol DFT with caller-provided basis scratch.
///
/// Two phases per call:
/// 1. **Basis precompute** (in `basis_re` / `basis_im`) — 8 tones ×
///    {cos, sin} = 16 vectors of NSPS=1920 i16 samples, generated by
///    a Q15 rotator (one cos+sin pair per tone, then 1920 complex
///    multiplies to fill the vector).
/// 2. **Per-symbol dot products** — for each symbol in `mask`,
///    16 calls to [`crate::core::dotprod::dot_q15_i32`] against the
///    basis. Default is a Rust loop; embedded targets can override
///    via `mfsk_core_dot_q15_i32` to bridge to chip-native asm
///    (e.g. esp-dsp `dsps_dotprod_s16_ae32` on Xtensa LX6).
///
/// **Why caller-provided scratch?** On Core2 the basis is the inner
/// loop's hot data — esp-dsp's asm dot product runs at 1 cycle/sample
/// only when the basis lives in fast internal RAM. Default heap on
/// ESP32 with `CONFIG_SPIRAM_USE_MALLOC` puts a 60 KB allocation in
/// PSRAM (~5–10 cycles/sample read latency), which kills the asm
/// kernel's advantage. Pre-allocating scratch in `.bss` (static
/// arrays land in internal DRAM) lets the dot product reach its
/// theoretical speed.
///
/// Both `basis_re` and `basis_im` must be at least
/// [`BASIS_SCRATCH_LEN`] long — debug-asserted; longer is fine
/// (only the prefix is used).
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn fill_symbol_spectra_into<S: AudioSample>(
    out: &mut [[Cmplx<f32>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) {
    fill_symbol_spectra_into_generic::<f32, S>(
        out, audio, freq_hz, dt_sec, mask, basis_re, basis_im,
    );
}

/// Generic version of [`fill_symbol_spectra_into`] — writes
/// `Cmplx<Sc>` for any spec scalar. f32 wrapper above; Q14i16 path
/// used under `fixed-point-cs`.
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn fill_symbol_spectra_into_generic<Sc: crate::core::scalar::SpecScalar, S: AudioSample>(
    out: &mut [[Cmplx<Sc>; 8]; 79],
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) {
    use crate::core::dotprod::dot_q15_i32;
    debug_assert!(basis_re.len() >= BASIS_SCRATCH_LEN);
    debug_assert!(basis_im.len() >= BASIS_SCRATCH_LEN);
    let i0 = ((TX_START_OFFSET_S + dt_sec) * SAMPLE_RATE_HZ).round() as i64;
    let two_pi_over_fs = core::f32::consts::TAU / SAMPLE_RATE_HZ;

    // ── Phase 1: precompute Q15 basis vectors (cos, sin × 8 tones).
    for tone in 0..NTONES {
        let tone_freq = freq_hz + tone as f32 * TONE_SPACING_HZ;
        let dphi = -two_pi_over_fs * tone_freq;
        let rot_re = (dphi.cos() * 32767.0).round() as i32;
        let rot_im = (dphi.sin() * 32767.0).round() as i32;
        let mut osc_re: i32 = 32767;
        let mut osc_im: i32 = 0;
        let base = tone * NSPS;
        for k in 0..NSPS {
            basis_re[base + k] = osc_re as i16;
            basis_im[base + k] = osc_im as i16;
            let new_re = ((osc_re * rot_re) - (osc_im * rot_im)) >> 15;
            let new_im = ((osc_re * rot_im) + (osc_im * rot_re)) >> 15;
            osc_re = new_re;
            osc_im = new_im;
        }
    }

    // Stack buffer: one symbol of audio as i16. 1920 × 2 = 3.8 KB.
    let mut sym_buf = [0i16; NSPS];

    // ── Phase 2: per-symbol dot products (audio × basis).
    //
    // For `Sc = f32` (no autogain) we write each cell straight into
    // `out` via `Sc::from_f32` (no-op cast). For fixed-point types
    // we collect i32 accumulators into a stack tmp buffer
    // (~40 KB i32×8×79×2), find the peak, and write
    // `Sc::from_f32_scaled(acc as f32, scale)` so the i16 output
    // range is fully utilised. The scratch is a stack array — fits
    // the 32 KB Core2 main task stack with the existing 16 KB used
    // by basis scratch references and the spec.
    let mut tmp_re = [[0i32; 8]; 79];
    let mut tmp_im = [[0i32; 8]; 79];
    let mut peak: i32 = 0;
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let sym_start = i0 + (sym as i64) * (NSPS as i64);
        for k in 0..NSPS {
            let idx = sym_start + k as i64;
            sym_buf[k] = if idx >= 0 && (idx as usize) < audio.len() {
                audio[idx as usize].to_i16()
            } else {
                0
            };
        }
        for tone in 0..NTONES {
            let base = tone * NSPS;
            let basis_re_tone = &basis_re[base..base + NSPS];
            let basis_im_tone = &basis_im[base..base + NSPS];
            let acc_re = dot_q15_i32(&sym_buf, basis_re_tone);
            let acc_im = dot_q15_i32(&sym_buf, basis_im_tone);
            if !Sc::NEEDS_AUTOGAIN {
                // f32 fast path — direct write, identical to the
                // pre-2.6 cast.
                out[sym][tone] = Cmplx {
                    re: Sc::from_f32(acc_re as f32),
                    im: Sc::from_f32(acc_im as f32),
                };
            } else {
                tmp_re[sym][tone] = acc_re;
                tmp_im[sym][tone] = acc_im;
                peak = peak.max(acc_re.unsigned_abs() as i32);
                peak = peak.max(acc_im.unsigned_abs() as i32);
            }
        }
    }

    // 2-pass auto-gain: scale i32 accumulators into i16 range,
    // saturating safe, peak ≈ 95 % of i16::MAX. Skipped on the f32
    // fast path (`out` is already populated above).
    if Sc::NEEDS_AUTOGAIN {
        let scale = if peak > 0 {
            (i16::MAX as f32 * 0.95) / peak as f32
        } else {
            0.0
        };
        for sym in 0..NN {
            if !sym_in_mask(sym, mask) {
                continue;
            }
            for tone in 0..NTONES {
                out[sym][tone] = Cmplx {
                    re: Sc::from_f32_scaled(tmp_re[sym][tone] as f32, scale),
                    im: Sc::from_f32_scaled(tmp_im[sym][tone] as f32, scale),
                };
            }
        }
    }
}

/// Heap-allocating sibling of [`symbol_spectra_direct`] that uses a
/// caller-provided basis scratch (passed through to
/// [`fill_symbol_spectra_into`]). Only the fixed-point variant is
/// exposed — host f32 path doesn't need a scratch (`fill_symbol_spectra`
/// f32 has no basis precompute step).
#[doc(hidden)]
#[cfg(feature = "fixed-point")]
pub fn symbol_spectra_direct_into<S: AudioSample>(
    audio: &[S],
    freq_hz: f32,
    dt_sec: f32,
    sym_mask: SymMask,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Box<[[Cmplx<f32>; 8]; 79]> {
    let mut out: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    fill_symbol_spectra_into(
        &mut out, audio, freq_hz, dt_sec, sym_mask, basis_re, basis_im,
    );
    out
}

// ── Public entry ────────────────────────────────────────────────────────────

/// Embedded FT8 decode for one 15-s slot.
///
/// Runs the same algorithm shape as [`decode_frame`](super::decode::decode_frame)
/// but talks only to power-of-two FFTs (via the
/// [`crate::core::fft::FftPlanner`] trait) and uses the min-sum LDPC
/// kernel to skip per-iteration `tanh` / `atanh`. No
/// `decode_sniper*` paths are involved; no wide-band 192 k FFT cache.
///
/// Sensitivity vs `decode_frame` is characterised on host AWGN
/// sweeps before any embedded port — see
/// `tests/ft8_decode_block_snr_sweep.rs`.
///
/// # Arguments
/// * `audio`     — 12 kHz i16 PCM, length up to NMAX = 180 000.
/// * `freq_min`  — lower edge of carrier search (Hz).
/// * `freq_max`  — upper edge of carrier search (Hz).
/// * `sync_min`  — minimum normalised Costas score (typical 1.0–2.0).
/// * `depth`     — `Bp` / `BpAll` / `BpAllOsd`.
/// * `max_cand`  — cap on Costas candidates evaluated.
pub fn decode_block<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
) -> Vec<DecodeResult> {
    let spec = compute_spectrogram(audio, freq_max);
    let pass1 = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
    drop(spec);
    let pass2 = refine_candidates(audio, pass1, max_cand);
    process_candidates(audio, pass2, depth)
}

/// Variant of [`decode_block`] that accepts caller-provided
/// basis scratch — required on Core2 / Cortex-M for the asm dot
/// product to reach its theoretical 1-cycle/sample speed (default
/// heap allocation routes the 60 KB × 2 basis to PSRAM /
/// non-cacheable RAM and erases the kernel's advantage).
///
/// Both `basis_re` and `basis_im` must be at least
/// [`BASIS_SCRATCH_LEN`] long; place them in **internal-RAM `.bss`**
/// (`static [i16; BASIS_SCRATCH_LEN]` arrays) for max throughput.
/// Same recall / depth / staircase as `decode_block`, just no
/// per-candidate allocation cycles.
#[cfg(feature = "fixed-point")]
pub fn decode_block_into<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<DecodeResult> {
    let spec = compute_spectrogram(audio, freq_max);
    let pass1 = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
    drop(spec);
    let pass2 = refine_candidates_into(audio, pass1, max_cand, basis_re, basis_im);
    process_candidates_into(audio, pass2, depth, basis_re, basis_im)
}

/// Pass-1 candidate cap — coarse_sync emits at most this many
/// candidates regardless of `max_cand`. Pass 2 re-ranks by
/// `sync_quality` (the same metric stage 3 uses to gate decode
/// attempts — much sharper than the per-bin power ratio) and
/// truncates to caller's `max_cand` for stage 3.
///
/// Sweep on real-QSO WAVs (host fp i16, BpAll, with the regularised
/// coarse_sync ratio in `RATIO_EPS_DEFAULT`) showed:
/// - PASS1 ∈ {30, 50}: 14/22 truth (drops one weak qso1 signal)
/// - PASS1 ∈ {75, 100}: 15/22 truth (full recall ceiling)
/// - PASS1=200: same 15/22 (no further gain — qso3's remaining gap
///   is at coarse_sync rank 100+, beyond Pass 2's reach).
///
/// 75 is the smallest PASS1 that keeps the full recall ceiling.
/// 30 is the smallest PASS1 that keeps the qso3 (busy band) truth
/// ceiling — it loses one borderline -17 dB qso1 signal (OH3NIV).
/// Core2 ships with 30 (speed-priority — Pass 2 cost ≈ 0.4 s vs
/// 1.0 s at PASS1=75). Override per-call via `MFSK_PASS1_LIMIT`
/// when std is enabled.
const PASS1_LIMIT_DEFAULT: usize = 30;
fn pass1_limit() -> usize {
    #[cfg(feature = "std")]
    {
        if let Ok(s) = std::env::var("MFSK_PASS1_LIMIT")
            && let Ok(v) = s.parse::<usize>()
        {
            return v;
        }
    }
    PASS1_LIMIT_DEFAULT
}

/// One Pass-2 output: the original candidate, its 79×8 Costas-only
/// spectrum (filled in stage 3 with the data-symbol DFT), and its
/// `sync_quality` score for ranking.
pub type RefinedCandidate = (SyncCandidate, Box<[[Cmplx<f32>; 8]; 79]>, u32);

/// Per-candidate Costas-block-0 DFT + sync_quality_block0 re-rank.
/// Keeps the top `max_cand` by Pass-2 score; **the cs spectrum is
/// retained** (block 0 only at this point) and stage 3 fills the
/// remaining 72 symbols via [`SymMask::NotBlock0`].
///
/// Cost: 7 sync symbols × 8 tones = 56 DFT per candidate vs
/// `SyncOnly`'s 168 — 1/3 the work. On Core2 ~13 ms/cand with the
/// asm dot product. PASS1=75 → Pass 2 ≈ 1.0 s.
///
/// The retained `q` is the **block-0** score (range 0..=7, expected
/// ~6-7 for real signals, ~0.875 for noise). Stage 3 recomputes the
/// full 21-symbol `sync_quality` after filling blocks 1, 2 and uses
/// that for its `q > 6` gate; the per-cand sort here is just for
/// truncating to `max_cand`.
fn refine_candidates<S: AudioSample>(
    audio: &[S],
    cands: Vec<SyncCandidate>,
    max_cand: usize,
) -> Vec<RefinedCandidate> {
    refine_candidates_with(cands, max_cand, |c| {
        symbol_spectra_direct(audio, c.freq_hz, c.dt_sec, SymMask::SyncBlock0)
    })
}

/// Variant of [`refine_candidates`] that uses caller-provided basis
/// scratch (forwards to `symbol_spectra_direct_into`).
///
/// **Pub for benchmarking + manually-staged callers** (e.g.
/// m5stack-core2 main.rs which logs per-stage wall-clock).
#[cfg(feature = "fixed-point")]
#[doc(hidden)]
pub fn refine_candidates_into<S: AudioSample>(
    audio: &[S],
    cands: Vec<SyncCandidate>,
    max_cand: usize,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<RefinedCandidate> {
    refine_candidates_with(cands, max_cand, |c| {
        symbol_spectra_direct_into(
            audio,
            c.freq_hz,
            c.dt_sec,
            SymMask::SyncBlock0,
            basis_re,
            basis_im,
        )
    })
}

/// Common min-heap selection logic used by both `refine_candidates`
/// (heap-allocated basis per call) and `refine_candidates_into`
/// (caller-provided basis scratch). The closure abstracts how each
/// candidate's cs Box is produced.
fn refine_candidates_with<F>(
    cands: Vec<SyncCandidate>,
    max_cand: usize,
    mut cs_for: F,
) -> Vec<RefinedCandidate>
where
    F: FnMut(&SyncCandidate) -> Box<[[Cmplx<f32>; 8]; 79]>,
{
    use alloc::collections::BinaryHeap;
    use core::cmp::{Ordering, Reverse};

    // Min-heap on q so the smallest survivor is at the top — replace
    // it whenever a stronger candidate arrives. Bounds the live heap
    // footprint at `max_cand × cs Box` regardless of PASS1_LIMIT,
    // which is the heap-fragmentation fix Task #2 was opened for.
    // Old code collected all PASS1=30 cs Boxes (240 KB) before the
    // truncate; new code never holds more than max_cand=15 Boxes
    // (120 KB peak, halving again to 60 KB once `fixed-point-cs`
    // ships Cmplx<Q14i16>).
    //
    // The heap stores (q, cand_idx, RefinedCandidate); cand_idx
    // breaks ties deterministically (insertion order) so the
    // truncation result is reproducible across runs.
    struct Slot {
        q: u32,
        idx: u32,
        cand: SyncCandidate,
        cs: Box<[[Cmplx<f32>; 8]; 79]>,
    }
    impl PartialEq for Slot {
        fn eq(&self, other: &Self) -> bool {
            self.q == other.q && self.idx == other.idx
        }
    }
    impl Eq for Slot {}
    impl Ord for Slot {
        fn cmp(&self, other: &Self) -> Ordering {
            self.q.cmp(&other.q).then_with(|| self.idx.cmp(&other.idx))
        }
    }
    impl PartialOrd for Slot {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }

    let mut heap: BinaryHeap<Reverse<Slot>> = BinaryHeap::with_capacity(max_cand + 1);
    for (idx, c) in cands.into_iter().enumerate() {
        let cs = cs_for(&c);
        let q = sync_quality_block0(&cs);
        let slot = Slot {
            q,
            idx: idx as u32,
            cand: c,
            cs,
        };
        if heap.len() < max_cand {
            heap.push(Reverse(slot));
        } else if let Some(Reverse(top)) = heap.peek()
            && slot.q > top.q
        {
            heap.pop();
            heap.push(Reverse(slot));
            // else branch: drop slot.cs immediately when it leaves scope
        }
    }
    let mut out: Vec<RefinedCandidate> = heap
        .into_iter()
        .map(|r| {
            let s = r.0;
            (s.cand, s.cs, s.q)
        })
        .collect();
    out.sort_by_key(|r| core::cmp::Reverse(r.2));
    out
}

/// Hard-decision sync quality on Costas **block 0 only** (symbols
/// 0..7). Cheaper variant of [`sync_quality`] for Pass 2 — checks
/// only one of the three Costas blocks. Range 0..=7.
///
/// Pub-but-doc-hidden so embedded callers (e.g. the m5stack-core2
/// PoC's manual Pass 2) can re-rank coarse_sync candidates by this
/// metric without pulling in the full `decode_block` D-pattern.
#[doc(hidden)]
pub fn sync_quality_block0<S: crate::core::scalar::SpecScalar>(cs: &[[Cmplx<S>; 8]; 79]) -> u32
where
    S::Wide: PartialOrd,
{
    let mut count = 0u32;
    for (t, &expected) in COSTAS.iter().enumerate() {
        let sym = t; // block 0 starts at symbol 0
        let best = (0..NTONES)
            .max_by(|&a, &b| {
                let na = cs[sym][a].norm_sqr_wide();
                let nb = cs[sym][b].norm_sqr_wide();
                na.partial_cmp(&nb).unwrap_or(core::cmp::Ordering::Equal)
            })
            .unwrap_or(0);
        if best == expected {
            count += 1;
        }
    }
    count
}

/// LLR / BP scalar for the embedded RX hot loop. Selected at compile
/// time: `Q11i16` under `fixed-point-bp` (integer-only, target FPU-less
/// MCUs / architectural consistency); `f32` otherwise (host / FPU
/// targets where soft-emu cost is irrelevant). Both routes go
/// through the same generic NMS implementation in `fec::ldpc::bp`.
#[cfg(feature = "fixed-point-llr")]
type LlrT = crate::core::scalar::Q11i16;
#[cfg(not(feature = "fixed-point-llr"))]
type LlrT = f32;

/// Stage 3: take Pass-2 refined candidates (cand + Costas-only cs +
/// sync_quality), fill in the data-symbol spectra, run LLR + BP/OSD
/// staircase. The Costas DFT was already done in Pass 2 — we only
/// add the data-symbol DFT here.
///
/// **Pub for benchmarking only — do not depend on it.**
#[doc(hidden)]
pub fn process_candidates<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
) -> Vec<DecodeResult> {
    process_candidates_with(audio, cands, depth, |cs, cand, mask| {
        fill_symbol_spectra(cs, audio, cand.freq_hz, cand.dt_sec, mask);
    })
}

/// Variant of [`process_candidates`] that uses caller-provided basis
/// scratch — required on Core2 / Cortex-M for the asm dot product
/// to reach its theoretical 1-cycle/sample speed (default heap
/// allocation routes the 60 KB × 2 basis to PSRAM /
/// non-cacheable RAM and erases the kernel's advantage).
///
/// **Pub for benchmarking + manually-staged callers** (e.g.
/// m5stack-core2 main.rs which logs per-stage wall-clock).
#[cfg(feature = "fixed-point")]
#[doc(hidden)]
pub fn process_candidates_into<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<DecodeResult> {
    process_candidates_with(audio, cands, depth, |cs, cand, mask| {
        fill_symbol_spectra_into(
            cs,
            audio,
            cand.freq_hz,
            cand.dt_sec,
            mask,
            basis_re,
            basis_im,
        );
    })
}

/// Common body of `process_candidates` / `process_candidates_into`
/// — the BP staircase logic is identical between the two; only the
/// per-candidate `fill_symbol_spectra(_into)` call differs (heap-
/// allocated basis vs caller-provided).
fn process_candidates_with<S: AudioSample, F>(
    _audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    mut fill: F,
) -> Vec<DecodeResult>
where
    F: FnMut(&mut [[Cmplx<f32>; 8]; 79], &SyncCandidate, SymMask),
{
    // dt is already parabolically refined by coarse_sync; no grid here.

    let mut results: Vec<DecodeResult> = Vec::new();
    let q_thr = q_thresh();
    // BP scratch pool — instantiated once and reused across all
    // candidates × all 5 BP calls per candidate. Eliminates the
    // ~12 KB-per-call `tlsf_malloc` traffic that dominated stage-3
    // non-DFT cost on Core2 (~50–100 ms / qso). See
    // `mfsk_core::fec::ldpc::bp::BpScratch`.
    let mut bp_scratch =
        crate::fec::ldpc::bp::BpScratch::<crate::fec::ldpc::params::Ldpc174_91Params, LlrT>::new();
    for (cand, mut cs, _q_block0) in cands {
        // Two-step fill: sync blocks first, gate by full sync_quality,
        // then fill data symbols only for survivors. Saves the 58 ×
        // 8 = 464 data-symbol DFTs on every candidate that fails the
        // q gate (typically half of `max_cand`). `SyncBlocks12`
        // (instead of `SyncOnly`) skips re-filling block 0 — Pass 2
        // already populated it via `symbol_spectra_direct_into` on
        // `SyncBlock0`, and that data survives in `cs` here. Saves
        // an additional 56 DFTs / candidate.
        fill(&mut cs, &cand, SymMask::SyncBlocks12);
        let q = sync_quality(&cs);
        if q <= q_thr {
            continue;
        }
        fill(&mut cs, &cand, SymMask::DataOnly);
        let refined_dt = cand.dt_sec;

        // ── Staircase: cheap → deeper → OSD ─────────────────────────
        //
        // 1) Bp(llra) on the fast nsym=1 LLR. Most candidates that
        //    decode at all decode here; the rest fall through.
        // 2) Full compute_llr (nsym=1+2+3) → Bp on all 4 variants
        //    (a/b/c/d).
        // 3) OSD-1 / OSD-3 fallback gated on sync_quality.
        //
        // `BpAll` and `BpAllOsd` enable the deeper stages; plain
        // `Bp` stops after step 1.
        let mut accepted: Option<(crate::fec::ldpc::bp::BpResult, u8)> = None;

        // Step 1: fast llra. The LLR / BP scalar is selected at compile
        // time via the `fixed-point-bp` feature (Q11i16 → integer-only
        // hot loop on FPU-less embedded targets; f32 → host path).
        // Both go through the *same* generic NMS implementation —
        // bit-identical AWGN behaviour by design.
        let llr_a_fast: super::llr::LlrSet<LlrT> = super::llr::compute_llr_fast(&cs);
        let bp_step1 = crate::fec::ldpc::bp::bp_decode_nms_with_scratch::<LlrT>(
            &mut bp_scratch,
            &llr_a_fast.llra,
            None,
            BP_MAX_ITER,
            Some(check_crc14),
            NMS_ALPHA,
        );
        if let Some(bp) = bp_step1 {
            accepted = Some((bp, 0));
        }

        // Step 2: deeper-LLR variants. Lazy + LLR-shared with Step 1.
        //
        // **Variant a is skipped** — Step 1 already ran BP on the
        // identical input (`compute_llr_fast` and `compute_llr`
        // produce bit-identical `llra`, since nsym=1 work doesn't
        // depend on `max_nsym`). Re-running it would be guaranteed
        // failure.
        //
        // **Variant d reuses** Step 1's `llr_a_fast.llrd` — same
        // nsym=1 derivation, costs zero LLR work.
        //
        // **Variants b / c are lazy-computed**: only pay the nsym=2
        // work if d failed, and only pay the heavy nsym=3 work
        // (~80 % of `compute_llr`) if both d and b also failed.
        // Order chosen by ascending compute cost — same number of BP
        // calls as the old variant loop in the worst case, far fewer
        // in the typical case where any earlier variant decodes.
        if accepted.is_none() && matches!(depth, DecodeDepth::BpAll | DecodeDepth::BpAllOsd) {
            // Variant d: free reuse of Step 1's llrd.
            let bp_d = crate::fec::ldpc::bp::bp_decode_nms_with_scratch::<LlrT>(
                &mut bp_scratch,
                &llr_a_fast.llrd,
                None,
                BP_MAX_ITER,
                Some(check_crc14),
                NMS_ALPHA,
            );
            if let Some(bp) = bp_d {
                accepted = Some((bp, 3));
            }
            // Variant b: lazy nsym=2 only.
            if accepted.is_none() {
                let llrb_arr: [LlrT; LDPC_N] = super::llr::compute_llr_partial::<LlrT>(&cs, 2);
                let bp_b = crate::fec::ldpc::bp::bp_decode_nms_with_scratch::<LlrT>(
                    &mut bp_scratch,
                    &llrb_arr,
                    None,
                    BP_MAX_ITER,
                    Some(check_crc14),
                    NMS_ALPHA,
                );
                if let Some(bp) = bp_b {
                    accepted = Some((bp, 1));
                }
            }
            // Variant c: lazy nsym=3 (the expensive one).
            if accepted.is_none() {
                let llrc_arr: [LlrT; LDPC_N] = super::llr::compute_llr_partial::<LlrT>(&cs, 3);
                let bp_c = crate::fec::ldpc::bp::bp_decode_nms_with_scratch::<LlrT>(
                    &mut bp_scratch,
                    &llrc_arr,
                    None,
                    BP_MAX_ITER,
                    Some(check_crc14),
                    NMS_ALPHA,
                );
                if let Some(bp) = bp_c {
                    accepted = Some((bp, 2));
                }
            }
        }

        // Step 3: OSD fallback (sync_quality gated; only for BpAllOsd).
        // OSD operates on `&[f32]` directly — independent of the
        // `LlrT` choice — so we compute a fresh f32 LLR bundle here.
        // Only fires when Steps 1+2 BP failed and `q >= 12`, so the
        // extra compute_llr is cheap relative to the OSD work itself.
        if accepted.is_none() && matches!(depth, DecodeDepth::BpAllOsd) && q >= 12 {
            let llr_full_f32: super::llr::LlrSet<f32> = super::llr::compute_llr(&cs);
            for (llr, pid) in [
                (&llr_full_f32.llra, 4u8),
                (&llr_full_f32.llrb, 5),
                (&llr_full_f32.llrc, 6),
                (&llr_full_f32.llrd, 7),
            ] {
                let osd = if q >= 18 {
                    osd_decode_deep(llr, 3, Some(check_crc14))
                } else {
                    osd_decode(llr)
                };
                if let Some(osd) = osd {
                    // Reuse BpResult shape via a synthetic conversion.
                    let bp = crate::fec::ldpc::bp::BpResult {
                        message77: osd.message77,
                        info: osd.info,
                        codeword: vec![0u8; LDPC_N],
                        hard_errors: osd.hard_errors,
                        iterations: 0,
                    };
                    accepted = Some((bp, pid));
                    break;
                }
            }
        }

        let Some((bp, pass_id)) = accepted else {
            continue;
        };
        let Some(text) = unpack77(&bp.message77) else {
            continue;
        };
        // Plausibility filter — reject CRC-passing-but-garbage
        // messages. With max_cand=200 × 4 LLR variants × OSD,
        // CRC-14's 1/16384 false-positive rate produces ~1-2 random
        // strings per slot. Same filter the host wide-band path
        // uses (`decode_frame::process_candidate`).
        if !crate::msg::wsjt77::is_plausible_message(&text) {
            continue;
        }
        if results.iter().any(|r| r.message77 == bp.message77) {
            continue;
        }
        let itone = message_to_tones(&bp.message77);
        let snr_db = super::llr::compute_snr_db(&cs, &itone);
        results.push(DecodeResult {
            message77: bp.message77,
            freq_hz: cand.freq_hz,
            dt_sec: refined_dt,
            hard_errors: bp.hard_errors,
            sync_score: cand.score,
            pass: pass_id,
            sync_cv: 0.0,
            snr_db,
        });
    }

    results
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{MessageCodec, MessageFields};
    use crate::ft8::wave_gen::{message_to_tones, tones_to_f32};
    use crate::msg::Wsjt77Message;

    fn pack_cq() -> [u8; 77] {
        let bits = Wsjt77Message
            .pack(&MessageFields {
                call1: Some("CQ".into()),
                call2: Some("JA1ABC".into()),
                grid: Some("PM95".into()),
                ..Default::default()
            })
            .unwrap();
        let mut out = [0u8; 77];
        out.copy_from_slice(&bits);
        out
    }

    fn synth_clean(msg77: &[u8; 77], freq_hz: f32) -> Vec<i16> {
        let itone = message_to_tones(msg77);
        let pcm = tones_to_f32(&itone, freq_hz, 0.5);
        let mut slot = vec![0.0f32; NMAX];
        let start = (TX_START_OFFSET_S * SAMPLE_RATE_HZ) as usize;
        let n = pcm.len().min(NMAX - start);
        slot[start..start + n].copy_from_slice(&pcm[..n]);
        slot.iter()
            .map(|&s| (s * 25_000.0).clamp(-32_768.0, 32_767.0) as i16)
            .collect()
    }

    #[test]
    fn roundtrip_clean_signal() {
        let msg = pack_cq();
        let audio = synth_clean(&msg, 1500.0);
        let results = decode_block(&audio, 100.0, 3000.0, 1.0, DecodeDepth::BpAll, 30);
        assert!(
            results.iter().any(|r| r.message77 == msg),
            "decode_block should recover clean CQ; got {} results",
            results.len()
        );
    }

    /// Fill a clean CQ signal with `Sc = Q14i16` cs storage and
    /// verify the autogain produces non-saturated output that the
    /// generic `sync_quality_block0` recognises as a Costas hit.
    /// Exercises the Phase 2.6 fill / autogain path end-to-end.
    #[test]
    fn fill_q14i16_autogain_recovers_costas() {
        use crate::core::scalar::Q14i16;
        let msg = pack_cq();
        let audio = synth_clean(&msg, 1500.0);

        let mut cs_q14: alloc::boxed::Box<[[Cmplx<Q14i16>; 8]; 79]> =
            alloc::vec![[Cmplx::<Q14i16>::default(); 8]; 79]
                .try_into()
                .unwrap();
        // Use the host f32 fill on the f32 path; under fixed-point
        // the i16 fill (with autogain) is exercised. Both must yield
        // a Costas-perfect block 0 sync_quality.
        #[cfg(not(feature = "fixed-point"))]
        fill_symbol_spectra_generic::<Q14i16, i16>(
            &mut cs_q14,
            &audio,
            1500.0,
            0.0,
            SymMask::SyncBlock0,
        );
        #[cfg(feature = "fixed-point")]
        fill_symbol_spectra_generic::<Q14i16, i16>(
            &mut cs_q14,
            &audio,
            1500.0,
            0.0,
            SymMask::SyncBlock0,
        );

        // Spot-check: non-zero entries (autogain didn't kill the signal).
        let nonzero = cs_q14.iter().flatten().any(|c| c.re.0 != 0 || c.im.0 != 0);
        assert!(nonzero, "Q14 autogain produced all-zero cs");

        // sync_quality_block0 generic with S=Q14i16 should recognise
        // the Costas pattern at full strength on a clean signal.
        let q = sync_quality_block0(&cs_q14);
        assert_eq!(q, 7, "expected perfect Costas block-0 hit, got q={q}");
    }
}
