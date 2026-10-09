# Add continuous integration (GitHub Actions)

Adds the CI the project already advertises but doesn't have: there is no
`.github/` directory, yet `README.md` claims "CI builds every PR on all three
platforms," `web/README.md` references `.github/workflows/web.yml`, and Issue 3
explicitly flagged "a CI job that runs it on PRs would be a welcome follow-up."

> Note: the uploaded set has five issues; this is a drafted seventh, chosen
> because it's the highest-leverage repo-hygiene gap and the companion Issue 3
> named. Its Python job runs Issue 3's suite, so it **stacks on Issue 3** — merge
> that first (or together). The C++ and GDExtension jobs stand alone.

## What's added

`.github/workflows/ci.yml` — runs on every push to `main` and every PR:

- **`python-tests`** (Python 3.9 / 3.11 / 3.12): installs the package with dev
  extras and runs the offline `pytest` suite against the mock server — no Godot,
  no GPU.
- **`cpp-core`** (Linux + macOS): compiles the standalone physics core with the
  platform toolchain and runs `web/core/test_core.cpp`, then builds and runs any
  `tests/cpp/*.cpp` (so the Issue-2 / Issue-6 C++ tests are picked up
  automatically once merged; a safe no-op until then).
- **`gdextension`** (Linux + macOS + Windows): configures and builds the Godot 4
  extension via CMake (godot-cpp is fetched automatically — mirrors
  `scripts/build.sh`) and uploads the binary as an artifact. This is what backs
  the README's "builds every PR on all three platforms."

`.github/workflows/web.yml` — the Pages deploy `web/README.md` already points to:
sets up Emscripten, runs `web/build_wasm.sh`, and publishes `web/` to GitHub
Pages on push to `main`.

`README.md` — a CI status badge under the title (with an `OWNER/REPO` placeholder
to fill in once the slug is known).

## Validation

Run in this environment (the jobs' exact commands):

- **python-tests**: `pip install -e "./python[dev]"` then `pytest` → `22 passed`.
- **cpp-core**: `c++ -std=c++20 -O2 -Wall -Wextra -I include -I web/core
  web/core/drone_core.cpp web/core/test_core.cpp -o test_core && ./test_core`
  → `CORE OK` (warnings only, no errors — no `-Werror`, so CI stays green).
- **cpp-core unit-test loop**: verified it discovers, compiles, and runs a
  `tests/cpp/*.cpp` file, and is a no-op when the directory is absent.
- Both workflow YAML files parse and have well-formed job graphs.

Not run in this sandbox (no toolchain available here), but faithful to the
existing scripts they mirror: the **gdextension** job (`cmake` + godot-cpp fetch,
identical to `scripts/build.sh`) and the **web** job (`emsdk` + `web/build_wasm.sh`,
identical to the documented local build).

## Acceptance

- [x] Every PR runs the offline Python suite and the C++ core tests.
- [x] The GDExtension builds on Linux, macOS, and Windows (README claim now
      backed by an actual workflow).
- [x] The web tier's promised Pages deploy workflow exists.
- [x] No `-Werror`; jobs use only the toolchains available on standard runners.
