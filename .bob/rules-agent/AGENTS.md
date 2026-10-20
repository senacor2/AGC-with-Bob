# Coding Rules (Non-Obvious)

## Crate Split — Hard Boundary

- `agc-core`: `no_std`, no heap, no `std` feature. This is the flight-software crate. It must always compile with `cargo build --target thumbv7em-none-eabihf`.
- `agc-sim`: host-only wrapper. `std` is fine here. Integration tests live here.
- **Never add a `std`-gated path into `agc-core`** even behind a feature flag used only in tests.

## Error Handling — Different Rules per Crate

- `agc-core`: `Option` + `alarm::raise(...)`. No `Result`. No `unwrap`. No `expect`.
- `agc-sim`: normal `Result`-based Rust.
- Violations in `agc-core` either panic (triggering GOJAM) or silently corrupt state.

## Shared Mutable State

- Pattern: `static THING: cortex_m::interrupt::Mutex<RefCell<Option<T>>> = ...`
- Access: always inside `cortex_m::interrupt::free(|cs| { THING.borrow(cs).borrow_mut()... })`
- Raw `static mut` is a Clippy error — CI will reject it.

## Interrupt Attribute

- Import `#[interrupt]` from the device PAC crate (e.g. `stm32f7xx_hal::pac`), **not** from `cortex-m-rt`. Wrong import compiles but silently misroutes interrupts.

## HAL Structs (mandatory interface)

Every HAL struct must:
1. Implement `free()` to return ownership of the peripheral (C-FREE).
2. Implement all applicable `embedded-hal` traits (C-HAL-TRAITS).
3. Use typestate generics for operational modes (C-PIN-STATE), e.g. `Uart<Enabled>` / `Uart<Disabled>`.

## Cross-Reference Comment — Not Optional

Every function that maps to an AGC routine must have:
```rust
/// AGC source: `Comanche055/<FILE>.agc`, <ROUTINE_NAME> routine.
```
Search `input/virtualagc/Comanche055/` for the `.agc` source file. The `input/Apollo-11/Comanche055/` copy has annotations that clarify intent.

## Numeric Convention

- Navigation math: `f64` only. Never use `i16`/`i32` for intermediate math.
- `i16`/`u16` only when reading raw CDU angle registers, PIPA counts, or channel words.
- Do **not** reproduce AGC ones-complement scaling in Rust — it was a 1960s hardware constraint.

## Memory Discipline

- No dynamic allocation anywhere in `agc-core`. Fixed-size arrays only.
- The phase-table pattern is the only approved way to make multi-step computations restart-safe.
- `panic = "abort"` — every panic is a GOJAM (full hardware restart). `Drop` will not run.
