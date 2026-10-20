# AGENTS.md

This file provides guidance to agents when working with code in this repository.

## Project Goals

**Scope**: Comanche055 (Command Module), covering Earth-to-Moon-and-back. Lunar landing (Luminary / LM) is out of scope.

**Fidelity principle**: Where Rust idiom and AGC fidelity conflict, fidelity wins. Navigation errors kill people.

**Deliverable**: A `no_std` Rust crate that runs on a Cortex-M7 bare-metal target (STM32 Nucleo-F767ZI) and produces the same navigation, guidance, and control outputs as Comanche055 for the same inputs. A parallel `agc-sim` crate wraps the core for terminal-based simulation.

**Long-term**: Physical DSKY, real IMU, actual embedded target.

## Repository Layout

```
input/
  Apollo-11/          # git submodule — chrislgarry/Apollo-11 (canonical AGC source)
  virtualagc/         # git submodule — virtualagc/virtualagc (assembler, sim, docs)
  AGC Quick Reference.md       # AGC instruction set reference
  AGC Symbolic Listing.md      # Block 2 symbolic listing format guide
  spaceflight_context_index.md # Indexed guide to the PDF reference documents
  The Apollo Guidance Computer.pdf
  Users Guide to Apollo GNCS.pdf
```

The primary AGC source to implement is `input/virtualagc/Comanche055/` (`.agc` files).
The `input/Apollo-11/Comanche055/` submodule contains the same source with annotations.

## Build and Test

```sh
# Validate all changes
cargo fmt && cargo clippy -- -D warnings && cargo test

# Embedded build — must always pass; a failure blocks merge
cargo build --target thumbv7em-none-eabihf -p agc-core

# Run a single test
cargo test <test_name>
cargo test -p agc-core <test_name>

# Simulator / host-only
cargo run -p agc-sim
```

## no_std / Bare-Metal Constraints (agc-core)

- **No heap**: `alloc`, `Vec`, `Box`, `String`, `HashMap` are forbidden. All data structures are statically sized.
- **No `static mut`**: Use `cortex_m::interrupt::Mutex<RefCell<T>>` + `cortex_m::interrupt::free(|cs| ...)`.
- **No blocking** in interrupt handlers or Waitlist tasks. Long work → Executive job.
- **No unwinding**: `panic = "abort"`. Every panic triggers GOJAM. Do not rely on `Drop` for pre-restart cleanup.
- **No interpreter VM**: AGC interpretive-language routines are **not** re-implemented as a VM. Design proper Rust abstractions for the underlying orbital-mechanics and navigation math. Use `input/AGC Symbolic Listing.md` to understand what each routine computes, then implement it as a clean `f64` function.
- **Restart safety**: Multi-step computations that must survive restart use the phase-table pattern.

## Numeric Types

- `f64` for all navigation math. `i16`/`u16` only for raw hardware values (CDU angles, PIPA counts, channel words).
- The AGC's ones-complement fixed-point arithmetic is a hardware artifact — do **not** replicate it.

## Error Handling Split

- `agc-core`: use `Option` + `alarm::raise` — no `Result`, no heap.
- `agc-sim`: use `Result`-based error handling normally.
- Never use `unwrap` / `expect` in `agc-core` flight code.

## Architecture Rules

- Domain logic (navigation, guidance, DAP) is strictly separate from the HAL and the Executive scheduler.
- HAL boundary is the only place peripherals are touched. No register access outside HAL.
- HAL structs must: implement `free()` (C-FREE), implement applicable `embedded-hal` traits (C-HAL-TRAITS), use typestate type parameters for operational modes (C-PIN-STATE).
- `#[interrupt]` must be re-exported from the device PAC crate, **not** from `cortex-m-rt` directly.
- Interrupt handlers receive the narrowest possible view of state.

## Code Style

- Stable Rust only. No nightly features.
- `snake_case` functions/variables/modules · `PascalCase` types/traits · `SCREAMING_SNAKE_CASE` constants.
- Public APIs: prefer `&T`, `&mut T`, owned values. Never expose `RefCell<T>`, `UnsafeCell<T>`, or `Mutex<T>` publicly.
- Prefer concrete types → generics → `dyn Trait` (only where dynamic dispatch is genuinely required).
- Every `unsafe` block must have a comment naming the invariant upheld (not just "safe here").
- Prefer `#[expect(lint, reason = "...")]` over `#[allow(lint)]`.
- No `dbg!`, `println!`, or `hprintln!` in finished changes.
- Memory footprint: minimize it, second priority after safety/stability/maintainability.

## Documentation Requirements

- Public modules, types, and functions must be documented.
- Navigation/guidance functions doc must state: input units & scale, output units & scale, AGC source routine + file.
- Physical-quantity newtypes (`CduAngle`, `Met`, `DeltaV`) doc the unit and scale factor at struct level.
- Non-obvious constants: document what they represent, AGC source, and units.

## AGC Source Cross-References (mandatory)

Every Rust function implementing a specific AGC routine carries a cross-reference:

```rust
/// Brief description.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPRTN routine.
pub fn kepler_step(r0: Vec3, v0: Vec3, dt: f64, mu: f64) -> (Vec3, Vec3) { ... }
```

## Testing Rules

- Unit tests in `agc-core` run on the host (`#[cfg(test)]`); must not use any `std` behind the `sim` feature flag.
- Integration tests use the simulator-hosted HAL (end-to-end scenario testing).
- Math function tests must include at least one case from a VirtualAGC reference run.

## Source Control

- Der Quellcode wird mit **git** verwaltet.
- Änderungen werden in **Tasks** gegliedert. Jeder Task ist eine abgeschlossene, in sich sinnvolle Arbeitseinheit.
- Nach erfolgreichem Abschluss eines Tasks (alle Tests grün, `cargo clippy -- -D warnings` sauber) wird die Änderung mit einem **git commit** ins Repository übernommen.
- Commit-Message im Format: `<Task-Titel>: <kurze Beschreibung was geändert wurde>`
