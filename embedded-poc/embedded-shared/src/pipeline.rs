//! Streaming RX pipeline messages and queue helpers.
//!
//! The pipeline is wav_sim → stage1_inc → main, connected by two
//! FreeRTOS Queues. Each stage owns its own buffers and transfers
//! ownership through the queues by sending raw `Box::into_raw` pointers.
//! No shared mutable state, no notification-and-out-pointer split.

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::ptr;

use esp_idf_svc::sys::{
    xQueueGenericCreate, xQueueGenericSend, xQueueReceive, QueueHandle_t,
};

const PD_PASS: i32 = 1;
const QUEUE_SEND_TO_BACK: i32 = 0;
const QUEUE_TYPE_BASE: u8 = 0;
const PORT_MAX_DELAY: u32 = u32::MAX;

/// Audio chunk size pushed by wav_sim each tick (100 ms @ 12 kHz).
pub const CHUNK_LEN: usize = 1_200;

/// Message from wav_sim to stage1_inc.
pub enum ChunkMsg {
    /// New audio samples for the current slot. Variable length so the
    /// final chunk of a WAV can be shorter than `CHUNK_LEN`.
    Samples(Vec<i16>),
    /// End of current slot. stage1_inc finalizes the slot and sends it
    /// downstream, then resets internal state for the next slot.
    SlotEnd {
        wav_idx: usize,
        total_samples: usize,
    },
}

/// Completed-slot bundle from stage1_inc to main.
///
/// Contains everything `decode_one_slot` needs to drive stage 2 / pass 2
/// / stage 3. Owned by exactly one task at a time — main drops the Box
/// when the slot's decode is done, freeing all the buffers.
pub struct Slot {
    pub audio: Vec<i16>,
    pub spec: mfsk_core::ft8::decode_block::Spectrogram,
    pub allsum_head: Vec<f32>,
    pub allsum_tail: Vec<f32>,
    pub wav_idx: usize,
    pub inc_total_us: i64,
}

/// Create a depth-N FreeRTOS queue carrying `*mut ChunkMsg` pointers.
pub fn create_chunk_queue(depth: u32) -> QueueHandle_t {
    create_ptr_queue::<ChunkMsg>(depth)
}

/// Create a depth-N FreeRTOS queue carrying `*mut Slot` pointers.
pub fn create_slot_queue(depth: u32) -> QueueHandle_t {
    create_ptr_queue::<Slot>(depth)
}

fn create_ptr_queue<T>(depth: u32) -> QueueHandle_t {
    let q = unsafe {
        xQueueGenericCreate(
            depth,
            core::mem::size_of::<*mut T>() as u32,
            QUEUE_TYPE_BASE,
        )
    };
    assert!(!q.is_null(), "xQueueGenericCreate failed");
    q
}

/// Send a heap-allocated message through a queue, transferring
/// ownership to the receiver. Blocks if the queue is full.
pub fn send_box<T>(q: QueueHandle_t, boxed: Box<T>) {
    let raw: *mut T = Box::into_raw(boxed);
    let r = unsafe {
        xQueueGenericSend(
            q,
            (&raw as *const *mut T) as *const core::ffi::c_void,
            PORT_MAX_DELAY,
            QUEUE_SEND_TO_BACK,
        )
    };
    debug_assert_eq!(r, PD_PASS, "xQueueGenericSend failed: {r}");
}

/// Receive a boxed message from a queue, taking ownership. Blocks
/// until a message is available.
pub fn recv_box<T>(q: QueueHandle_t) -> Box<T> {
    let mut raw: *mut T = ptr::null_mut();
    let r = unsafe {
        xQueueReceive(
            q,
            (&mut raw as *mut *mut T) as *mut core::ffi::c_void,
            PORT_MAX_DELAY,
        )
    };
    debug_assert_eq!(r, PD_PASS, "xQueueReceive failed: {r}");
    debug_assert!(!raw.is_null());
    unsafe { Box::from_raw(raw) }
}
