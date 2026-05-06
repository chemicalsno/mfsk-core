//! WSJT-X-faithful JT9 demodulator pipeline.
//!
//! Direct port of `lib/softsym.f90` and the subroutines it calls
//! (`downsam9`, `peakdt9`, `afc9`, `twkfreq`, `symspec2`,
//! `interleave9`). Replaces the `baseband.rs` + `demod_bb.rs`
//! box-car path. The decisive step is **`downsam9`**: a single
//! NFFT1=653184-point FFT of the entire 60-s slot, from which we
//! select NFFT2=1512 bins centred at the candidate carrier and
//! IFFT back to a 27.78-Hz complex baseband. That brick-wall band
//! selection rejects the wide-band noise that the box-car path
//! drags into our LLRs.

use num_complex::Complex;
use rustfft::FftPlanner;

use super::interleave::deinterleave_llrs;
use super::sync_pattern::JT9_ISYNC;

/// Big FFT length — covers ~54.43 s of 12 kHz audio. Chosen so that
/// `NFFT1 / NFFT2 = 432 = 8 × 54` is an integer decimation factor and
/// `NFFT2 / NSPSD = 1512 / 16` ≈ 94.5 symbols straddles the 85-symbol
/// JT9 frame plus comfortable pre/post buffer.
pub const NFFT1: usize = 653_184;
pub const NFFT2: usize = 1512;
/// Samples per symbol at the 27.78 Hz downsampled rate.
pub const NSPSD: usize = 16;
/// 85 symbols × 16 samples — the per-candidate signal slice.
pub const NZ3: usize = 1360;
/// Decimation factor: NFFT1 / NFFT2 = 432.
pub const NDOWN: usize = NFFT1 / NFFT2;
/// Sample rate of the downsampled signal: 12000 / 432 = 27.778 Hz.
pub const FSAMPLE_DOWN: f32 = 12_000.0 / NDOWN as f32;
/// JT9 tone spacing in Hz at 12 kHz.
pub const TONE_SPACING: f32 = 12_000.0 / 6912.0;

const SCALE: f32 = 10.0;
/// LLR clamp matching WSJT-X (it uses int8 = ±127).
const LLR_CLAMP: f32 = 127.0;

/// Pre-computed audio FFT, reused across many candidate frequencies.
///
/// The big FFT is the dominant cost; once `c1` is built it can be
/// re-used for every coarse-search candidate without re-FFT.
pub struct AudioFft {
    /// Half-spectrum (NFFT1/2 + 1 complex bins) of the input audio.
    pub c1: Vec<Complex<f32>>,
    /// 1 Hz-resolution power envelope (5000 entries: 0..5000 Hz).
    pub envelope: Vec<f32>,
}

impl AudioFft {
    /// Build the big FFT once for the whole slot.
    pub fn build(audio: &[f32]) -> Self {
        // Pad/truncate to NFFT1; scale to int16-equivalent so noise
        // estimates land in WSJT-X's calibrated regime (downsam9
        // ingests int16 samples directly).
        let n = audio.len().min(NFFT1);
        let mut buf: Vec<Complex<f32>> = vec![Complex::new(0.0, 0.0); NFFT1];
        for i in 0..n {
            buf[i].re = audio[i] * 32_768.0;
        }

        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(NFFT1);
        let mut scratch = vec![Complex::new(0.0, 0.0); fft.get_inplace_scratch_len()];
        fft.process_with_scratch(&mut buf, &mut scratch);
        buf.truncate(NFFT1 / 2 + 1);

        // 1 Hz-resolution power envelope across 0..5 kHz.
        let df1 = 12_000.0 / NFFT1 as f32;
        let nadd = (1.0 / df1).round() as usize;
        let env_len = 5000usize;
        let mut envelope = vec![0.0f32; env_len];
        for i in 0..env_len {
            let j_start = ((i as f32) / df1).round() as usize;
            for n_off in 0..nadd {
                let j = j_start + n_off;
                if j < buf.len() {
                    envelope[i] += buf[j].norm_sqr();
                }
            }
        }

        Self { c1: buf, envelope }
    }

    /// `downsam9`: extract a 27.78 Hz complex baseband centred at `fpk` Hz.
    /// Returns NFFT2 = 1512 complex samples.
    pub fn downsam9(&self, fpk: f32) -> Vec<Complex<f32>> {
        let df1 = 12_000.0 / NFFT1 as f32;
        let i0 = (fpk / df1) as i64;
        let nh2 = (NFFT2 / 2) as i64;

        // 40th-percentile noise floor in a ±100 Hz window around fpk.
        let nf = fpk.round() as i64;
        let ia = (nf - 100).max(1) as usize;
        let ib = (nf + 100).min(self.envelope.len() as i64 - 1) as usize;
        let mut env_slice: Vec<f32> = self.envelope[ia..=ib].to_vec();
        env_slice.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let pcl = env_slice.len() * 40 / 100;
        let avenoise = env_slice[pcl.min(env_slice.len() - 1)].max(1e-6);
        let fac = (1.0 / avenoise).sqrt();

        // Place selected bins into c2 with FFT-shift convention so that
        // the IFFT yields a baseband centred at fpk.
        let mut c2 = vec![Complex::new(0.0, 0.0); NFFT2];
        let c1_len = self.c1.len() as i64;
        for i in 0..NFFT2 as i64 {
            let mut j = i0 + i;
            if i > nh2 {
                j -= NFFT2 as i64;
            }
            if j >= 0 && j < c1_len {
                c2[i as usize] = self.c1[j as usize] * fac;
            }
        }

        // IFFT to time domain.
        let mut planner = FftPlanner::<f32>::new();
        let ifft = planner.plan_fft_inverse(NFFT2);
        let mut scratch = vec![Complex::new(0.0, 0.0); ifft.get_inplace_scratch_len()];
        ifft.process_with_scratch(&mut c2, &mut scratch);
        c2
    }
}

/// `peakdt9`: integrated symbol-power score across 85 sliding-window
/// positions, ratio of sync-symbol power to data-symbol power.
///
/// Returns `(lagpk, score, c3)` — the best lag, the
/// (sync_avg/data_avg − 1) score, and the 1360-sample slice
/// extracted starting from that lag (zero-padded outside the input).
pub fn peakdt9(c2: &[Complex<f32>]) -> (i64, f32, Vec<Complex<f32>>) {
    assert_eq!(c2.len(), NFFT2);

    // Sliding-window coherent sum over `NSPSD` samples → integrated
    // power at each lag. WSJT-X scales by 1e-3 to keep magnitudes
    // away from f32 overflow when downsam9 amplified them.
    let mut p = vec![0.0f32; NFFT2 + 5 * NSPSD];
    let i0 = 5 * NSPSD;
    for i in 0..NFFT2 {
        let lo = (i + 1).saturating_sub(NSPSD);
        let mut z = Complex::new(0.0f32, 0.0);
        for k in lo..=i {
            z += c2[k];
        }
        z *= 1e-3;
        p[i0 + i] = z.norm_sqr();
    }

    // Lag bounds match WSJT-X getlags for nsps8=864 (NSPSD=16):
    //   lag0=123, lag1=39, lag2=291. Search lag1..=lag2.
    let lag0: i64 = 123;
    let lag1: i64 = 39;
    let lag2: i64 = 291;
    let mut smax = f32::NEG_INFINITY;
    let mut lagpk = lag0;
    for lag in lag1..=lag2 {
        let mut sum0 = 0.0f32;
        let mut sum1 = 0.0f32;
        for sym in 0..85usize {
            let idx = (sym * NSPSD) as i64 + lag;
            if idx < 0 || idx as usize >= p.len() {
                continue;
            }
            let v = p[idx as usize];
            if JT9_ISYNC[sym] == 1 {
                sum1 += v;
            } else {
                sum0 += v;
            }
        }
        if sum0 <= 0.0 {
            continue;
        }
        let ss = (sum1 / 16.0) / (sum0 / 69.0) - 1.0;
        if ss > smax {
            smax = ss;
            lagpk = lag;
        }
    }

    // Extract NZ3 samples starting at lagpk (with the WSJT-X offsetting
    // convention: c3(i) = c2(i + lagpk - i0 - NSPSD + 1)).
    let mut c3 = vec![Complex::new(0.0, 0.0); NZ3];
    for i in 0..NZ3 as i64 {
        let j = i + lagpk - i0 as i64 - NSPSD as i64 + 1;
        if j >= 0 && (j as usize) < NFFT2 {
            c3[i as usize] = c2[j as usize];
        }
    }
    (lagpk, smax, c3)
}

/// `twkfreq` (constant-frequency variant): apply a frequency shift of
/// `df_hz` Hz to the complex signal at `FSAMPLE_DOWN` rate. Mirrors
/// WSJT-X's polynomial form with only `a(1)` non-zero.
pub fn twkfreq_const(buf: &mut [Complex<f32>], df_hz: f32) {
    let dphi = -2.0 * std::f32::consts::PI * df_hz / FSAMPLE_DOWN;
    let step = Complex::new(dphi.cos(), dphi.sin());
    let mut w = Complex::new(1.0f32, 0.0);
    for s in buf.iter_mut() {
        *s = w * *s;
        w *= step;
    }
}

/// `symspec2`: tone-shift c5 by 1.736 Hz × i for i=0..8, coherent-sum
/// `NSPSD` samples per symbol, then compute max-log-MAP LLRs from
/// the resulting `ss3[0..7][0..69]` table. Returns 207 LLRs in
/// channel-symbol (interleaved) order.
pub fn symspec2(c5: &[Complex<f32>]) -> [f32; 207] {
    assert_eq!(c5.len(), NZ3);

    let mut ss2 = [[0.0f32; 85]; 9];
    let mut work: Vec<Complex<f32>> = c5.to_vec();
    let dphi = -2.0 * std::f32::consts::PI * TONE_SPACING / FSAMPLE_DOWN;
    let step = Complex::new(dphi.cos(), dphi.sin());

    for i in 0..9usize {
        if i >= 1 {
            // Shift work down by one tone spacing relative to its
            // current state: work[k] *= e^{-j 2π Δf k / fs}.
            let mut w = Complex::new(1.0f32, 0.0);
            for s in work.iter_mut() {
                *s = w * *s;
                w *= step;
            }
        }
        for j in 0..85usize {
            let lo = j * NSPSD;
            let mut z = Complex::new(0.0f32, 0.0);
            for k in 0..NSPSD {
                z += work[lo + k];
            }
            ss2[i][j] = z.norm_sqr();
        }
    }

    // Build ss3[0..7][0..69] = power for data-tone i+1, data-symbol m+1.
    let mut ss3 = [[0.0f32; 69]; 8];
    for i in 1..9usize {
        let mut m = 0usize;
        for j in 0..85usize {
            if JT9_ISYNC[j] == 0 {
                ss3[i - 1][m] = ss2[i][j];
                m += 1;
            }
        }
    }

    // Baseline: average of the seven non-max ss3 entries per symbol.
    let mut ss_total = 0.0f32;
    for j in 0..69 {
        let mut smax = 0.0f32;
        let mut col_sum = 0.0f32;
        for i in 0..8 {
            let v = ss3[i][j];
            if v > smax {
                smax = v;
            }
            col_sum += v;
        }
        ss_total += col_sum - smax;
    }
    let ave = (ss_total / (69.0 * 7.0)).max(1e-9);
    for col in ss3.iter_mut() {
        for v in col.iter_mut() {
            *v /= ave;
        }
    }

    // Max-log-MAP LLRs. WSJT-X convention: positive ⇒ bit=1 likely.
    // We adopt the OPPOSITE sign (positive ⇒ bit=0) to stay
    // consistent with the rest of mfsk-core's FEC pipeline.
    let mut out_207 = [0.0f32; 207];
    let mut k = 0usize;
    for j in 0..69usize {
        for m in (0..3i32).rev() {
            let (r1, r0) = match m {
                2 => (
                    [ss3[4][j], ss3[5][j], ss3[6][j], ss3[7][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                    [ss3[0][j], ss3[1][j], ss3[2][j], ss3[3][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                ),
                1 => (
                    [ss3[2][j], ss3[3][j], ss3[4][j], ss3[5][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                    [ss3[0][j], ss3[1][j], ss3[6][j], ss3[7][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                ),
                _ => (
                    [ss3[1][j], ss3[2][j], ss3[4][j], ss3[7][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                    [ss3[0][j], ss3[3][j], ss3[5][j], ss3[6][j]]
                        .iter()
                        .cloned()
                        .fold(f32::NEG_INFINITY, f32::max),
                ),
            };
            // Flip sign so positive ⇒ bit=0, matching mfsk-core's
            // FEC sign convention.
            let llr = SCALE * (r0 - r1);
            out_207[k] = llr.clamp(-LLR_CLAMP, LLR_CLAMP);
            k += 1;
        }
    }

    out_207
}

/// Full pipeline: feed `c5` into `symspec2`, deinterleave the
/// resulting 207 LLRs in place, drop the padding bit, and return
/// the 206 LLRs ready for `ConvFano232::decode_soft`.
pub fn llrs_from_c5(c5: &[Complex<f32>]) -> [f32; 206] {
    let mut s207 = symspec2(c5);
    // Drop the padding LLR (index 206 in 0-based) and deinterleave
    // the first 206 in place.
    let mut s206 = [0f32; 206];
    s206.copy_from_slice(&s207[..206]);
    deinterleave_llrs(&mut s206);
    let _ = &mut s207[206]; // padding ignored
    s206
}

/// Simple AFC: measure residual frequency offset from sync symbols
/// and return (df_hz). Coarse 1-D parabolic search around 0.
///
/// We test mixing offsets in ±1 tone spacing in 0.1-tone steps,
/// score each by the sync-symbol coherent-sum power, and parabolic-
/// fit the peak. This is much simpler than the full `afc9` 3-D
/// chi-square minimisation but captures the dominant contribution.
pub fn afc_simple(c3: &[Complex<f32>]) -> f32 {
    let mut best_df = 0.0f32;
    let mut best_score = f32::NEG_INFINITY;
    let mut work: Vec<Complex<f32>> = vec![Complex::new(0.0, 0.0); NZ3];
    let trial: Vec<f32> = (-20i32..=20)
        .map(|k| k as f32 * 0.1 * TONE_SPACING)
        .collect();
    for &df in &trial {
        work.copy_from_slice(c3);
        twkfreq_const(&mut work, df);
        let mut s = 0.0f32;
        for j in 0..85usize {
            if JT9_ISYNC[j] == 0 {
                continue;
            }
            let lo = j * NSPSD;
            let mut z = Complex::new(0.0f32, 0.0);
            for k in 0..NSPSD {
                z += work[lo + k];
            }
            s += z.norm_sqr();
        }
        if s > best_score {
            best_score = s;
            best_df = df;
        }
    }
    best_df
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{DecodeContext, FecCodec, FecOpts, MessageCodec};
    use crate::fec::ConvFano232;
    use crate::jt9::tx::synthesize_standard;
    use crate::msg::{Jt72Codec, Jt72Message};

    #[test]
    fn softsym_golden_grid_roundtrips() {
        let cases = &[
            ("CQ", "GM7GAX", "IO75"),
            ("TF3G", "N7MQ", "CN84"),
            ("K1JT", "KF4RWA", "73"),
            ("CQ", "M0WAY", "IO82"),
            ("K1JT", "N5KDV", "EM41"),
        ];
        for &(c1, c2, grid) in cases {
            let audio = synthesize_standard(c1, c2, grid, 12_000, 1346.0, 0.5).expect("synth");
            let mut padded = vec![0f32; 720_000];
            let n = audio.len().min(padded.len());
            padded[..n].copy_from_slice(&audio[..n]);
            let big = AudioFft::build(&padded);
            let c2_buf = big.downsam9(1346.0);
            let (_lag, sc, c3) = peakdt9(&c2_buf);
            assert!(
                sc > 0.5,
                "sync score for {} {} {} too low: {}",
                c1,
                c2,
                grid,
                sc
            );
            let llrs = llrs_from_c5(&c3);
            let res = ConvFano232
                .decode_soft(&llrs, &FecOpts::default())
                .unwrap_or_else(|| panic!("Fano failed for {} {} {}", c1, c2, grid));
            let mut payload = [0u8; 72];
            payload.copy_from_slice(&res.info);
            let msg = Jt72Codec::default()
                .unpack(&payload, &DecodeContext::default())
                .unwrap_or_else(|| panic!("unpack failed for {} {} {}", c1, c2, grid));
            match msg {
                Jt72Message::Standard {
                    call1,
                    call2,
                    grid_or_report,
                } => {
                    assert_eq!(call1, c1);
                    assert_eq!(call2, c2);
                    assert_eq!(
                        grid_or_report, grid,
                        "grid mismatch for {} {} {}",
                        c1, c2, grid
                    );
                }
                other => panic!(
                    "expected Standard for {} {} {}, got {:?}",
                    c1, c2, grid, other
                ),
            }
        }
    }

    /// Encode a message with a CHOSEN Gray-code direction so we can
    /// hand WSJT-X two WAVs and see which one (if any) it decodes.
    /// Returns the 85 channel tones.
    fn encode_with_gray_dir(c1: &str, c2: &str, grid: &str, invert_gray: bool) -> [u8; 85] {
        use crate::core::FecCodec;
        use crate::fec::ConvFano232;
        use crate::jt9::interleave::interleave;
        use crate::jt9::sync_pattern::JT9_ISYNC;
        use crate::msg::jt72::pack_standard;

        // Same forward-gray as production:
        let fwd_gray3 = |n: u8| -> u8 { (n ^ (n >> 1)) & 0x7 };
        // Inverse:
        let inv_gray3 = |g: u8| -> u8 {
            let mut n = g & 0x7;
            n ^= n >> 1;
            n ^= n >> 2;
            n & 0x7
        };

        let words = pack_standard(c1, c2, grid).expect("pack");
        let mut info = [0u8; 72];
        for (i, b) in info.iter_mut().enumerate() {
            *b = (words[i / 6] >> (5 - (i % 6))) & 1;
        }
        let mut cw206 = vec![0u8; 206];
        ConvFano232.encode(&info, &mut cw206);
        let mut bits206 = [0u8; 206];
        bits206.copy_from_slice(&cw206);
        interleave(&mut bits206);
        // Pad to 207 with zero (matches gen9.f90: i1ScrambledBits(207)=0)
        let mut bits207 = [0u8; 207];
        bits207[..206].copy_from_slice(&bits206);

        let mut tones = [0u8; 85];
        let mut j = 0usize;
        for (i, slot) in tones.iter_mut().enumerate() {
            if JT9_ISYNC[i] == 1 {
                *slot = 0;
            } else {
                let b0 = bits207[3 * j];
                let b1 = bits207[3 * j + 1];
                let b2 = bits207[3 * j + 2];
                let raw = (b0 << 2) | (b1 << 1) | b2;
                let gc = if invert_gray {
                    inv_gray3(raw)
                } else {
                    fwd_gray3(raw)
                };
                *slot = gc + 1;
                j += 1;
            }
        }
        tones
    }

    fn synth_audio(tones: &[u8; 85], freq: f32, amp: f32) -> Vec<f32> {
        const NSPS: usize = 6912;
        const SR: f32 = 12_000.0;
        let spacing: f32 = SR / NSPS as f32;
        let mut out = Vec::with_capacity(NSPS * 85);
        let mut phase = 0.0f32;
        for &sym in tones {
            let f = freq + sym as f32 * spacing;
            let dphi = 2.0 * std::f32::consts::PI * f / SR;
            for _ in 0..NSPS {
                out.push(amp * phase.cos());
                phase += dphi;
                if phase > 2.0 * std::f32::consts::PI {
                    phase -= 2.0 * std::f32::consts::PI;
                }
            }
        }
        out
    }

    fn write_wav(audio: &[f32], path: &str) {
        let mut bytes: Vec<u8> = Vec::with_capacity(44 + audio.len() * 2);
        let data_len = (audio.len() * 2) as u32;
        bytes.extend_from_slice(b"RIFF");
        bytes.extend_from_slice(&(36u32 + data_len).to_le_bytes());
        bytes.extend_from_slice(b"WAVE");
        bytes.extend_from_slice(b"fmt ");
        bytes.extend_from_slice(&16u32.to_le_bytes());
        bytes.extend_from_slice(&1u16.to_le_bytes());
        bytes.extend_from_slice(&1u16.to_le_bytes());
        bytes.extend_from_slice(&12_000u32.to_le_bytes());
        bytes.extend_from_slice(&24_000u32.to_le_bytes());
        bytes.extend_from_slice(&2u16.to_le_bytes());
        bytes.extend_from_slice(&16u16.to_le_bytes());
        bytes.extend_from_slice(b"data");
        bytes.extend_from_slice(&data_len.to_le_bytes());
        for &s in audio {
            let v = (s * 32_767.0).round().clamp(-32_768.0, 32_767.0) as i16;
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        std::fs::write(path, &bytes).expect("write WAV");
    }

    /// Emit two WAVs differing only in Gray-code direction at encode.
    /// Whichever decodes in WSJT-X is the correct direction.
    #[test]
    #[ignore]
    fn dump_em41_gray_ab() {
        for (invert, label, fname) in [
            (false, "FORWARD gray (current)", "/tmp/260506_1300.wav"),
            (true, "INVERSE gray (test)", "/tmp/260506_1400.wav"),
        ] {
            let tones = encode_with_gray_dir("K1JT", "N5KDV", "EM41", invert);
            let signal = synth_audio(&tones, 1500.0, 0.3);
            let mut audio = vec![0f32; 12_000 * 60];
            let off = 1_200usize;
            let n = signal.len().min(audio.len() - off);
            audio[off..off + n].copy_from_slice(&signal[..n]);
            write_wav(&audio, fname);
            eprintln!("Wrote {} → {}", label, fname);
            eprintln!("  first 20 tones: {:?}", &tones[..20]);
        }
        eprintln!("\nDecode BOTH in WSJT-X (mode JT9). Whichever produces");
        eprintln!("'K1JT N5KDV EM41' is the WSJT-X-compatible direction.");
    }

    /// Diagnostic: write our `synthesize_standard` output for
    /// "K1JT N5KDV EM41" to a 12 kHz mono PCM-16 WAV. Hand to WSJT-X
    /// jt9 decoder to verify whether our encoder is bit-compatible
    /// with the reference.
    ///
    /// Run with: `cargo test --release --features jt9,fft-rustfft \
    ///   --lib jt9::softsym::tests::dump_synth_em41 -- --ignored --nocapture`
    /// Then point WSJT-X File→Open at `/tmp/mfsk_jt9_em41_synth.wav`.
    #[test]
    #[ignore]
    fn dump_synth_em41() {
        // 60-second slot at 12 kHz: WSJT-X expects 720_000 frames.
        let mut audio = vec![0f32; 12_000 * 60];
        // dt = +1.0 s — matches WSJT-X jt9sim's "k=12000" signal start.
        // The 130418_1742.wav golden signals also live at dt ≈ +0.86..+1.04
        // (WSJT-X's coarse search expects signals near +1 s, not +0.1 s).
        let signal =
            synthesize_standard("K1JT", "N5KDV", "EM41", 12_000, 1500.0, 0.3).expect("synth");
        let off = 12_000usize; // 1.0 s × 12 kHz
        let n = signal.len().min(audio.len() - off);
        audio[off..off + n].copy_from_slice(&signal[..n]);

        // WSJT-X parses the slot timestamp from `YYMMDD_HHMM.wav`.
        // Use a real date so File→Open accepts it and File→Open Next
        // doesn't reject the slot.
        let path = "/tmp/260506_1200.wav";
        let mut bytes: Vec<u8> = Vec::with_capacity(44 + audio.len() * 2);
        let data_len = (audio.len() * 2) as u32;
        bytes.extend_from_slice(b"RIFF");
        bytes.extend_from_slice(&(36u32 + data_len).to_le_bytes());
        bytes.extend_from_slice(b"WAVE");
        bytes.extend_from_slice(b"fmt ");
        bytes.extend_from_slice(&16u32.to_le_bytes()); // fmt chunk size
        bytes.extend_from_slice(&1u16.to_le_bytes()); // PCM
        bytes.extend_from_slice(&1u16.to_le_bytes()); // mono
        bytes.extend_from_slice(&12_000u32.to_le_bytes()); // sample rate
        bytes.extend_from_slice(&24_000u32.to_le_bytes()); // byte rate = 12k * 2
        bytes.extend_from_slice(&2u16.to_le_bytes()); // block align
        bytes.extend_from_slice(&16u16.to_le_bytes()); // bits per sample
        bytes.extend_from_slice(b"data");
        bytes.extend_from_slice(&data_len.to_le_bytes());
        for &s in &audio {
            let v = (s * 32_767.0).round().clamp(-32_768.0, 32_767.0) as i16;
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        std::fs::write(path, &bytes).expect("write WAV");
        eprintln!("Wrote noiseless 'K1JT N5KDV EM41' synth → {}", path);
        eprintln!("  carrier = 1500 Hz, dt = +1.0 s, 60 s slot, mono PCM-16 12 kHz");
        eprintln!("  amplitude = 0.3 (~ -10 dBFS peak)");
        eprintln!("  Decode with WSJT-X (mode=JT9) and report what it says.");
    }

    #[test]
    fn synth_softsym_roundtrip() {
        let freq = 1500.0f32;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        // Pad audio to >= NFFT1 / 8 samples to give downsam9 something to work with.
        let mut padded = vec![0f32; 720_000];
        let n = audio.len().min(padded.len());
        padded[..n].copy_from_slice(&audio[..n]);
        let big_fft = AudioFft::build(&padded);
        let c2 = big_fft.downsam9(freq);
        let (_lag, sc, c3) = peakdt9(&c2);
        eprintln!("synth roundtrip peakdt9 score = {}", sc);
        assert!(
            sc > 0.5,
            "sync score for clean synth should be high; got {}",
            sc
        );

        let llrs = llrs_from_c5(&c3);
        let res = ConvFano232
            .decode_soft(&llrs, &FecOpts::default())
            .expect("Fano must converge on clean synth via WSJT-X-faithful pipeline");
        let mut payload = [0u8; 72];
        payload.copy_from_slice(&res.info);
        let msg = Jt72Codec::default()
            .unpack(&payload, &DecodeContext::default())
            .expect("unpack");
        match msg {
            Jt72Message::Standard {
                call1,
                call2,
                grid_or_report,
            } => {
                assert_eq!(call1, "CQ");
                assert_eq!(call2, "K1ABC");
                assert_eq!(grid_or_report, "FN42");
            }
            other => panic!("expected Standard, got {:?}", other),
        }
    }
}
