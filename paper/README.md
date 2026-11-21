# SkySim — IEEE Access paper and reproducible evaluation

```
paper/
  tex/    skysim_access.tex, refs.bib, fig_arch.tex, es_section.tex, ieeeaccess.cls (+ images/)
  eval/   eval.cpp (C++ harness), ref_bemt.py (reference BEMT), train_es.py (CPU-only ES),
          train_ppo.py (CPU-only PPO, SB3), make_figs.py
  data/   every CSV/JSON the paper quotes, plus UIUC APC 10x4.7 static + geometry data
  figs/   generated figures (PDF + PNG)
```

## Code changes this paper depends on (branch `paper-eval`)

1. `include/aero/blade_element.hpp` — **axial inflow sign fix** (`v_axial` was negated, so climbing
   gained thrust → heave/rate anti-damping). Section VI-B of the paper.
2. `include/power/battery.hpp` — restored (was missing from `main`; `web/core` and the battery
   tests did not build without it).
3. `web/core/capi.cpp`, `web/build_wasm.sh` — new `skysim_step_n(d, dt, n)` batched step.
4. `python/skysim/core_env.py` — new in-process Gymnasium env over the C API (`ctypes`).

## Reproduce

```bash
# from repo root
S="web/core/drone_core.cpp paper/eval/eval.cpp"
g++ -std=c++20 -O3 -Iinclude -Iweb/core $S -o /tmp/eval -pthread
# original (v3.0) rotor sign, for the axial/step comparison:
mkdir -p /tmp/orig/aero && git show a31864f:include/aero/blade_element.hpp > /tmp/orig/aero/blade_element.hpp
g++ -std=c++20 -O3 -I/tmp/orig -Iinclude -Iweb/core $S -o /tmp/eval_orig -pthread

cd paper/data
/tmp/eval hoveromega
/tmp/eval static     > static_skysim.csv
/tmp/eval static 3.75 > static_skysim_cal.csv         # +3.75 deg collective offset (fitted on CT)
python3 ../eval/ref_bemt.py uiuc_apcsf_10x4.7_geom.txt > static_refbemt.csv
/tmp/eval axial 837.6633 > axial_fixed.csv;  /tmp/eval_orig axial 837.6633 > axial_orig.csv
/tmp/eval step > step_fixed.csv;  /tmp/eval_orig step > step_orig.csv
/tmp/eval ground > ground.csv;  /tmp/eval vrs > vrs.csv;  /tmp/eval battery > battery.csv
/tmp/eval perf > perf_native_gcc.csv
/tmp/eval trace                                          # determinism hash

# WebAssembly (no emsdk needed): pip install ziglang
python3 -m ziglang c++ -target wasm32-wasi -std=c++20 -O3 -fno-exceptions -Iinclude -Iweb/core $S -o /tmp/eval.wasm
node runwasi.mjs /tmp/eval.wasm perf                    # tiny WASI loader, see below

# CPU-only learning
g++ -std=c++20 -O3 -shared -fPIC -Iinclude -Iweb/core web/core/drone_core.cpp web/core/capi.cpp -o /tmp/libskysim_core.so
SKYSIM_CORE_LIB=/tmp/libskysim_core.so ES_GENS=500 python3 paper/eval/train_es.py paper/data/es
# PPO baseline (pip install stable-baselines3 torch; runs on CPU, 1100 s wall budget)
SKYSIM_CORE_LIB=/tmp/libskysim_core.so python3 paper/eval/train_ppo.py paper/data/ppo

python3 paper/eval/make_figs.py paper/data paper/figs
cd paper/tex && pdflatex fig_arch && latexmk -pdf skysim_access
```

`runwasi.mjs`:
```js
import { readFile } from 'node:fs/promises'; import { WASI } from 'node:wasi';
const wasi = new WASI({ version: 'preview1', args: ['eval', ...process.argv.slice(3)], env: {} });
const inst = await WebAssembly.instantiate(await WebAssembly.compile(await readFile(process.argv[2])), wasi.getImportObject());
wasi.start(inst);
```

## Before submitting

- Replace `ieeeaccess.cls` and `images/` with the **official 2024 IEEE Access LaTeX template**
  (ACCESS_latex_template_20240429.zip from ieeeaccess.ieee.org). The class bundled here is an older
  community copy so the draft compiles; fonts fall back to Times/Helvetica substitutes.
- Fill in `[Surname]`, `[email]`, affiliation and the biography (+ photo).
- Merge `paper-eval` into `main` so the public release matches the paper (inflow fix in particular).
- SITL table (Table II) comes from the 2026-10-15 integration run; re-run it on the merged code and
  attach the PX4 Flight Review link / ArduPilot logs as supplementary material.
- Optional but strengthening: an in-browser timing of the Emscripten build (Chrome + Firefox), and
  extra seeds for the ES and PPO runs (each was run once).
- `tex/es_section.tex` is now maintained by hand (it covers ES and PPO); `eval/write_es_section.py`
  only regenerates the original ES-only version.
