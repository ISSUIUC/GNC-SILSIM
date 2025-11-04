<!-- Copilot instructions tailored to the EKF Flight Data Simulator (GNC_SILSIM) -->
# Quick guidance for AI coding assistants

Be concise, factual, and reference repository files/paths. Focus edits on small, well-scoped changes unless asked to implement larger features.

## Big picture (what this repo contains)
- C++ EKF simulator: core algorithm lives under `gnc/` (`ekf.cpp`, `ekf.h`, `rotation.h`). The executable is built as `simulation/test_ekf` from `simulation/test_ekf.cpp` and linked with `gnc/` sources via `build.sh`.
- Simulation harness: `simulation/` contains test runner and sensor abstractions (`Buffer.h`, `sensor_data.h`, `fsm_states.h`).
- Plotting: static and interactive plotters in `plotter/` (Python scripts) and `interactive_plotter/` (web UI + `server.py`). The interactive server serves `output/results.csv` and listens on port 8000.
- Orchestration scripts: `build.sh`, `run_simulation.sh`, and `start_plotter.sh` automate build → run → plot flows.

## Key developer workflows (concrete commands)
- Build (Linux/macOS/bash): `./build.sh` (requires Eigen via `pkg-config eigen3`).
- Full run & plot: `./run_simulation.sh` (options: `-i`, `-o`, `-s`, `--interactive`, `--no-plot`). See `run_simulation.sh` for exact flags.
- Start interactive plotter: `cd interactive_plotter && python3 server.py` (server opens `http://localhost:8000/interactive_plotter/`).
- Python deps: `pip install -r requirements.txt` (packages: numpy, pandas, matplotlib).
- Docker alternative: Dockerfile / docker-compose usage is described in `README.md` (useful on Windows).

## Project-specific conventions & patterns
- C++: code compiles with `-std=c++17` and uses Eigen for linear algebra. Prefer small single-file changes to `ekf.cpp` for algorithm tweaks.
- Sensor / data shape: `output/results.csv` columns are prescribed in `README.md` (timestamp, pos_*, vel_*, acc_*, altitude, fsm, etc.). Many scripts and the plotter expect those exact headers.
- FSM-driven filtering: FSM states live in `simulation/fsm_states.h` and are used by plotting and simulation stop conditions. Use those symbols when adding filters or CLI flags.
- Adding sensors: update `sensor_data.h`, adjust `Buffer.h` parsing, and add CSV parsing in `simulation/test_ekf.cpp`.

## Integration points to watch
- `build.sh` expects pkg-config/libraries for Eigen (native build requirement). On Windows, recommend using WSL, MSVC + vcpkg, or Docker as described in `README.md`.
- `run_simulation.sh` expects `data/*.csv` input and writes `output/results.csv` that `interactive_plotter/server.py` and `plotter/*` read.
- `interactive_plotter/server.py` exposes two endpoints used by the web UI: `GET /api/results` (JSON) and `GET /events` (SSE). It reads `../output/results.csv` (relative to repo root).

## Where to change EKF behavior (examples)
- Tuning parameters: look in `gnc/ekf.cpp` for `spectral_density_`, `s_dt`, and the `R`/`Q` matrices. Small numeric changes are common.
- Measurement model: modify `ekf.cpp` functions that assemble measurement vectors and covariance.
- Parsing / CSV format: update `simulation/test_ekf.cpp` and `plotter/plot_interactive.py` if you add/remove CSV columns.

## What to avoid / common pitfalls
- Don't change CSV column names silently — many tools and the web UI rely on exact header names. If renaming, update `interactive_plotter/server.py`, `plotter/*.py`, and `README.md` accordingly.
- Large edits across C++ and Python at once increase integration risk. Iterate in small commits and validate with `./run_simulation.sh` and by opening the plotter.

## Quick review checklist for PRs
- Build passes locally: `./build.sh` (or Docker alternative).
- Run a short simulation: `./run_simulation.sh -i data/sample_1k.csv --no-plot` and confirm `output/results.csv` appears.
- If UI changes: smoke test interactive plotter via `python3 interactive_plotter/server.py` and confirm `/api/results` returns JSON.

---
If anything here is unclear or you want more specific examples (unit tests, CMake support, or Windows build guidance), tell me which area to expand and I will update this file.
