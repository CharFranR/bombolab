# Architecture Decision Records

Architecture Decision Records (ADRs) capture the significant decisions of
bombolab: the problem, the chosen option, and the consequences. They follow the
classic format (Status, Context, Decision, Consequences) and are the primary
mechanism for recording *why* the code is the way it is — the code documents
itself, the ADRs document the decisions.

| ADR | Topic | Status |
|-----|-------|--------|
| [0001](./0001-constrained-drawing-ik.md) | Constrained drawing IK with reduced Jacobian | Accepted |
| [0002](./0002-serial-protocol-microseconds.md) | Serial wire protocol in microseconds | Accepted |
| [0003](./0003-tests-separate-docs-not-comments.md) | Tests in separate files; docs, not comments | Accepted |
| [0004](./0004-tool-frame-roll-only-invariant.md) | Tool-frame roll-only invariant | Accepted |

## How to add an ADR

1. Create `book/src/adr/NNNN-short-title.md` following the format of the
   existing records.
2. Add it to this index and to `SUMMARY.md`.
3. Record the status honestly: `Proposed` while under discussion, `Accepted`
   once adopted, `Superseded` when replaced.
