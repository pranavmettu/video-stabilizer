# CLAUDE.md: Video Stabilizer Project

## About me
- Pranav, Cornell sophomore (Bio Eng → transferring to MechE), aiming for robotics perception/simulation internships.
- I know Python and R. I had never written C++ before this project.
- I have ROS 2 / Gazebo exposure from a research lab, but this project is standalone (no ROS).
- Machine: Apple Silicon Mac, VS Code, Homebrew. Build with CMake.

## Goal
A portfolio-quality C++ video stabilizer (see `PLAN.md` for the week-by-week plan). It exists to (1) teach me C++ and the motion/estimation math, and (2) show robotics recruiters I can build and explain a perception pipeline.

## How to work with me (these override any older instruction, including `PLAN.md`)
- **Write all the code yourself**, including `src/trajectory.cpp` and `src/kalman.cpp`. Ignore the "you hand-write this" notes in `PLAN.md`.
- **Do not ask me questions or quiz me.** No checkpoint quizzes, no "explain it back", no asking for decisions I can reasonably delegate. Pick a sensible default, say what you picked, and keep going. Ignore the "Checkpoint" items in `PLAN.md`.
- **After a big change or commit-sized chunk, explain what you implemented in fewer than 5 simple sentences.** No long tutorials, no per-concept C++ lectures.
- **Git: I type every Git command myself.** Tell me when to run `git status`, `git add <files>` and `git commit -m "<message>"` (give the exact files and message). Don't run `git add`, `git commit` or `git push` yourself. Don't explain Git commands unless I ask.
- Follow the same week-by-week order as `PLAN.md`.
- Keep code simple and readable: C++17, no clever template tricks, clear names, short functions, comments on any math.
- Work in small steps that compile and run. After each step, give me the exact command to build and run it.
- Never commit videos or large files. Keep `data/` and `build/` in `.gitignore`.

## Build & run
```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j
./build/stabilize data/<clip>.MOV -o results/<clip>_stable.mp4 [--smoother avg|kalman] [--side-by-side] [--crop 0.1]
```

## Stack
- OpenCV 5 (Homebrew; note some functions moved headers, e.g. `goodFeaturesToTrack` is in `opencv2/features.hpp`, `estimateAffinePartial2D` in `opencv2/geometry.hpp`), Eigen, C++17, CMake
- Python (matplotlib, pandas) only for plotting scripts in `scripts/`; use the `.venv/` virtual environment
- ffmpeg (Homebrew) for the `vidstab` comparison baseline (the current Homebrew build lacks the `vidstab` filter; sort this out in Week 5)
