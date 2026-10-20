# Architecture Constraints (Non-Obvious)

## Two-Crate Split

```
agc-core   →  no_std, bare-metal, flight logic (navigation, guidance, DAP, Executive, HAL traits)
agc-sim    →  std, terminal app, wraps agc-core, hosts HAL implementations, integration tests
```

The split is a hard compile-time boundary — `agc-core` must build with `--target thumbv7em-none-eabihf` at all times.

## Interpretive Language → Rust Design Strategy

The AGC ran a bytecode "interpretive language" on top of its machine code. **Do not** re-implement this as a VM or translate op-codes 1:1 into Rust. Instead, **design proper Rust abstractions** for the underlying orbital-mechanics and navigation computations that the interpretive routines performed. The interpretive layer was a code-density trick for the 1960s hardware; the math is what matters.

Reference `input/AGC Symbolic Listing.md` to understand what each interpretive routine computes, then implement the computation as a clean `f64` Rust function.

## Executive / Scheduler Model

The AGC Executive is a cooperative, priority-based task scheduler with no preemption and no OS. In Rust:
- Tasks are plain functions scheduled through a statically-sized job table.
- Waitlist tasks fire on a T4RUPT (10ms tick) and must complete without blocking.
- Long computations break into phases; phase state persists via the phase-table pattern.

## Restart / GOJAM

`panic = "abort"` means every panic is a hardware restart (GOJAM). The phase-table pattern is the only approved mechanism for multi-step computations to survive restarts. `Drop` cannot be relied upon for cleanup before restart.

## HAL Isolation

- Peripheral registers: only inside HAL structs. Zero register access in domain logic.
- The domain logic calls embedded-hal traits only (e.g. `embedded_hal::serial::Write`).
- HAL structs own the peripheral via typestate: `Uart<Enabled>` vs `Uart<Disabled>` enforced at compile time.

## Memory Budget (Priority Order)

1. Safety / correctness
2. Stability / maintainability
3. **Memory footprint** — static arrays, fixed capacities, no dynamic allocation

## Scope

- In scope: Comanche055 (CM, Earth-to-Moon-and-back)
- Out of scope: Luminary (LM), any other AGC software version
