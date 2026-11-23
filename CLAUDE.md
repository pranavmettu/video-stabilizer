# CLAUDE.md: Video Stabilizer Project

## About me
- Pranav, Cornell sophomore (Bio Eng → transferring to MechE), aiming for robotics perception/simulation internships.
- I know Python and R. **I have never written C++ before this project.**
- I've never really used Git. Explain Git commands when they come up, and let me type them myself.
- I have ROS 2 / Gazebo exposure from a research lab, but this project is standalone (no ROS).
- Machine: Apple Silicon Mac, VS Code, Homebrew. Build with CMake.

## Goal
A portfolio-quality C++ video stabilizer (see `PLAN.md` for the week-by-week plan). It exists to (1) teach me C++ and the motion/estimation math, and (2) show robotics recruiters I can build and explain a perception pipeline.

## How to work with me
- **You may generate most of the code**: CMake, I/O, CLI parsing, feature tracking, motion estimation, warping, metrics, scripts, README scaffolding.
- **Do NOT write these for me:** `src/trajectory.cpp` (trajectory accumulation + moving-average smoother) and `src/kalman.cpp` (Kalman filter). For these, give hints, explain the math, point out bugs, and review my code. If I ask you to just write them, remind me of this rule once; if I insist, it's my call.
- When you introduce a new C++ concept (references, `const`, classes, headers vs source files, smart pointers, templates, etc.), explain it in 2-3 sentences with a Python comparison where it helps.
- Keep code simple and readable: C++17, no clever template tricks, clear names, short functions, comments on any math.
- Work in small steps that compile and run. After each step, tell me the exact command to build and run it.
- At the end of each week's milestone in `PLAN.md`, **quiz me**: ask me to explain the files built that week and the checkpoint question. Don't move on until I can answer.
- Suggest a Git commit (with a message) after each working step, but let me run the commands.
- Never commit videos or large files. Keep `data/` and `build/` in `.gitignore`.

## Build & run
```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j
./build/stabilize data/<clip>.mp4 -o results/<clip>_stable.mp4 [--smoother avg|kalman] [--side-by-side] [--crop 0.1]
```

## Stack
- OpenCV (Homebrew), Eigen, C++17, CMake
- Python (matplotlib, pandas) only for plotting scripts in `scripts/`
- ffmpeg (Homebrew) for the `vidstab` comparison baseline
