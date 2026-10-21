# README

A Re-Implementation of the **Apollo Guidance Computer** in Rust.
This version was implemented by [IBM Bob](https://bob.ibm.com).

The goal is a modern rewrite of the AGC target for [Rust Embedded](https://docs.rust-embedded.org/book/)

- Program in idiomatic Rust.
- Use `no_std` for the embedded platform.
- Stick as close to the original semantics as possible, especially when it comes to reliability and stability.
- Use double precision floating point math (`f64`) instead of double precision fixed point math with scaling.
- Reduce memory consumption where possible.

## Components

### AGC Core

This is the AGC core logic. The crate creates a library which can be linked to the simulator or the bare metal version.

### AGC Physics

This crate is used by the simulator to feed the AGC sensors with realistic data.

### AGC Simulator

This crate integrates the core library into a simulator with a DSKY and simulated sensors with the help of the Physics crate.

Launch the simulator by running

```shell
cargo run -p agc-sim
```

