# wishbone

Wishbone B4 bindings for the neutral register contract.

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. Two binders attach an IP that only speaks the bus-neutral contract to Wishbone B4 slave pins in pipelined mode:

| Binder | Target | Behaviour |
| :-- | :-- | :-- |
| `mkWbBind` | `RegIf`, answers at once | STALL_O is always low; a transfer starts when CYC_I and STB_I are high, and ACK_O or ERR_O follows in the next cycle |
| `mkWbBindT` | `RegTarget`, may stall | STALL_O is high while a transfer is outstanding, so the master holds the next one |

Both follow the pipelined rules of the specification: a transfer starts on CYC, STB and no STALL (RULE 3.57 and 3.58), acknowledgements come independently of STB (OBSERVATION 3.10), and DAT_O is valid with ACK_O (RULE 3.65). A target error answers with ERR_O instead of ACK_O.

The testbench drives a pipelined master that puts four transfers on the bus back to back without waiting for acknowledgements, against a zero-wait and a stalling target. It checks the order and number of answers, a write followed by a read, an address that always errors, byte strobes, and that STB without CYC starts nothing.

Standard (non-pipelined) masters, RTY_O and tag signals are not implemented.

## Specification sources

The specifications this IP is implemented against, with their links, digests and the clause-by-clause comparison, are kept on the [`spec` branch](https://github.com/Tape-Out/wishbone/tree/spec).

## License

Mulan PSL v2.
