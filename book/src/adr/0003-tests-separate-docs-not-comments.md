# ADR-0003: Tests in Separate Files; Code Is Documented, Not Commented

- **Status**: Accepted
- **Date**: 2026-08-08

## Context

`ik.rs` had grown to 1,608 lines, roughly half of them tests, and the codebase
carried a large amount of inline comments (many of them AI-generated boilerplate
that restated what the code already showed). Reviewers spent effort on whitespace
and noise instead of behavior. The team also wanted a durable answer to
"where do tests live" as new analysis features (SVD gates, workspace sampling)
are added.

## Decision

1. **Tests live in sibling files** `foo_tests.rs` next to the source module.
   They are declared as child modules of the module under test so they keep
   access to private items:
   ```rust
   #[cfg(test)]
   #[path = "ik_tests.rs"]
   mod ik_tests;
   ```
   Rust edition 2024 requires the `#[path]` attribute because `mod X;` inside
   `foo.rs` resolves to `foo/X.rs` (a subdirectory), not a sibling file.
2. **Binary tests** (`src/bin/...`) cannot use sibling files: Cargo
   auto-discovers every `*.rs` in `src/bin/` as an independent binary. They
   live in `src/bin/<bin>/<bin>_tests.rs`.
3. **Code is documented, not commented — and carries zero doc comments.**
   No inline comments and no rustdoc (`///`, `//!`) in source files. All
   explanation lives in the ADRs (this directory) and the mdbook
   documentation. Non-obvious invariants (e.g. the roll-only constraint in
   ADR-0004) are recorded in ADRs, never as prose inside the code.

## Consequences

- `ik.rs` shrank from 1,608 to 529 lines; the extracted test modules are
  byte-identical in content.
- 161 unit tests + 1 integration test + 6 ws-bridge tests remain green;
  `clippy -D warnings` and `cargo fmt --check` stay clean.
- Test names now include the module path
  (`math::jacobian::jacobian_tests::...`), which makes failure output more
  specific.
- New code must follow the rule: documentation artifacts, not comments.
