# Documentation Context (Non-Obvious)

## Reference Material Location

All AGC reference documents live under `input/` — **not** in the Rust source tree:

| File | Purpose |
|---|---|
| `input/AGC Quick Reference.md` | AGC instruction set (mnemonic table) |
| `input/AGC Symbolic Listing.md` | Block 2 assembly format & interpretive language guide |
| `input/spaceflight_context_index.md` | Indexed navigation guide to both PDFs (with PDF page offsets) |
| `input/The Apollo Guidance Computer.pdf` | Primary architectural reference (Executive, Waitlist, IMU, DSKY) |
| `input/Users Guide to Apollo GNCS.pdf` | G&N system interface reference |

## Canonical Source Code

- **Use** `input/virtualagc/Comanche055/*.agc` as the ground truth for routine names, structure, and constants.
- **Prefer** `input/Apollo-11/Comanche055/*.agc` when annotation / intent clarity matters — it has human-readable commentary.
- Both are git submodules; they track separate upstream repos. Do not edit them.

## spaceflight_context_index.md

The index file lists PDF page offsets at the top. Book page numbers ≠ PDF page numbers — always apply the offset when navigating the PDFs.

## AGC Interpretive Language

`input/AGC Symbolic Listing.md` is the primary reference for the interpretive op-codes (VLOAD, STORE, TLOAD, etc.). These are **not** implemented as a VM nor re-implement them directly in Rust. The goal is to properly design the necessary abstractions for orbital mechanics and navigation. The listing explains what each interpretive instruction computes.

## Scope Reminder

`input/Apollo-11/Luminary099/` and `input/virtualagc/Luminary*` are out of scope (LM software). Do not reference them when implementing Comanche055 routines.
