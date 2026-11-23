# Video Stabilizer in C++: Build Plan

**Owner:** Pranav Mettu
**Timeline:** Nov 23, 2026 to ~Jan 17, 2027 (8 weeks, ~5-8 hrs/week; lighter during finals and holidays)
**Setup:** Apple Silicon Mac, VS Code + Claude Code extension
**Working style:** Claude Code writes most of the code. **You hand-write the math core** (trajectory accumulation, moving-average smoother, Kalman filter), and you explain every file back before moving on. See `CLAUDE.md`.

---

## What you're building

A command-line tool that takes a shaky video and outputs a stabilized one:

```
./stabilize data/shaky.mp4 -o results/stable.mp4 --smoother kalman --side-by-side
```

**Pipeline (one pass per frame pair):**

1. **Track features.** Find corners in frame *t* (`cv::goodFeaturesToTrack`) and follow them into frame *t+1* with optical flow (`cv::calcOpticalFlowPyrLK`).
2. **Estimate motion.** Fit a similarity transform (translation + rotation + scale) to the matched points with RANSAC (`cv::estimateAffinePartial2D`). RANSAC throws out bad matches, like points on a person walking through the frame.
3. **Build the camera trajectory.** Pull out `dx`, `dy` and the angle `da = atan2(m10, m00)` from each transform, then take a running sum. That gives the camera's path over time. **(You write this.)**
4. **Smooth the trajectory.** Moving average first, then a Kalman filter. **(You write both.)**
5. **Correct each frame.** correction = smoothed path - raw path. Add the correction to each frame's transform and warp the frame (`cv::warpAffine`).
6. **Handle the borders.** Warping leaves black edges, so zoom in slightly with a fixed crop.
7. **Measure it.** Compute stability metrics, then compare against ffmpeg's `vidstab` filter.

**Why this matters for robotics:** steps 1-2 are the front end of visual odometry (project 4), and step 4 is the same state estimation used in robot localization. You can say this in interviews.

---

## Repo layout (target)

```
video-stabilizer/
├── CLAUDE.md                 # rules for Claude Code
├── PLAN.md                   # this file
├── README.md                 # portfolio write-up (week 7)
├── CMakeLists.txt
├── .gitignore                # ignores build/, data/, large videos
├── src/
│   ├── main.cpp              # CLI parsing, wiring the pipeline together
│   ├── feature_tracker.hpp/.cpp
│   ├── motion_estimator.hpp/.cpp
│   ├── trajectory.hpp/.cpp   # YOU WRITE: accumulation + smoothers
│   ├── kalman.hpp/.cpp       # YOU WRITE
│   ├── frame_warper.hpp/.cpp
│   └── metrics.hpp/.cpp
├── scripts/
│   ├── plot_trajectory.py    # plots raw vs smoothed path from CSV
│   └── compare_vidstab.sh    # runs ffmpeg vidstab for comparison
├── data/                     # input videos (gitignored)
├── results/                  # outputs, CSVs, plots (mostly gitignored)
└── docs/                     # small GIFs and plots for the README
```

---

## Week-by-week

Each week ends with a **checkpoint**: you explain what was built, in your own words, to Claude Code (or a friend, or a notes file). If you can't, stop and ask questions before moving on.

### Week 0 (Nov 23-29): Setup + first C++ program
**Goal:** a C++ program that opens a video and writes a copy.

- [ ] Install Xcode Command Line Tools: `xcode-select --install`
- [ ] Install Homebrew (brew.sh), then: `brew install cmake opencv eigen ffmpeg git`
- [ ] VS Code extensions: **CMake Tools**, **clangd** (better autocomplete/errors than the default C++ extension on Mac), **Claude Code**
- [ ] Make a GitHub account (use your Cornell email for the Student Developer Pack) and set up `git config --global user.name/user.email`
- [ ] Create the repo folder, put `PLAN.md` and `CLAUDE.md` in it, then run `git init`
- [ ] Have Claude Code generate `CMakeLists.txt` (finding OpenCV + exporting `compile_commands.json` for clangd) and a `main.cpp` that reads a video, prints frame count/FPS/resolution, and writes a copy
- [ ] Shoot 3-4 test clips on your phone (10-20 s each): walking, panning, handheld zoom, one with people moving through the frame
- [ ] **Git basics (you type these yourself):** `git status`, `git add`, `git commit -m`, create the GitHub repo, `git remote add`, `git push`
- [ ] **C++ reading:** learncpp.com chapters 0-2 (what compiling is, variables, functions)

**Done when:** `./stabilize data/walk.mp4` prints the video info and writes a copy, and the repo is on GitHub.
**Checkpoint:** explain what CMake does, what `#include` does, and what `cv::Mat` is.

### Week 1 (Nov 30-Dec 6): Feature tracking
**Goal:** see features being tracked between frames.

- [ ] Claude Code builds `feature_tracker`: corners + optical flow between consecutive frames, keeping only successfully tracked points
- [ ] A debug mode that draws the tracked points and their motion lines on the video
- [ ] **C++ reading:** learncpp.com sections on `std::vector`, references (`&`), `const`, and classes (just enough to read the generated code)

**Done when:** a debug video shows green dots with motion trails that follow the scene.
**Checkpoint:** explain why we track *corners* rather than arbitrary pixels, and what optical flow assumes.

### Week 2 (Dec 7-13, finals, keep it light): Motion estimation
**Goal:** per-frame camera motion as numbers.

- [ ] Claude Code builds `motion_estimator`: `estimateAffinePartial2D` with RANSAC, extracting `dx, dy, da` (and scale)
- [ ] Write per-frame motion to `results/motion.csv`
- [ ] Claude Code writes `scripts/plot_trajectory.py` (you already know Python, so read it closely)

**Done when:** you have a plot of dx/dy/angle per frame for a shaky clip.
**Checkpoint:** explain the 2x3 affine matrix and why `atan2(m10, m00)` gives the rotation angle. Write it out on paper.

### Week 3 (Dec 14-20): First working stabilizer ✱ YOU WRITE
**Goal:** end-to-end stabilization with a moving average.

- [ ] **You write** `trajectory.cpp`: running sum of dx/dy/da → trajectory
- [ ] **You write** a moving-average smoother (window radius as a parameter, e.g. 30 frames; decide how to handle the first and last frames)
- [ ] Claude Code builds `frame_warper`: apply correction = smoothed - raw, rebuild the transform, `warpAffine`
- [ ] `--side-by-side` output mode
- [ ] Plot raw vs smoothed trajectory

**Done when:** a side-by-side video where the right half is visibly steadier.
**Checkpoint:** explain why we smooth the *trajectory* (cumulative path) instead of the per-frame motion, and what the window size trades off.

### Week 4 (Dec 21-27, holidays, light): Borders + Kalman filter ✱ YOU WRITE
**Goal:** a clean output and a second, smarter smoother.

- [ ] Claude Code adds a fixed zoom/crop (`--crop 0.1`) to hide black borders
- [ ] **You write** `kalman.cpp`: one 1D Kalman filter per parameter (x, y, angle), each with state = [position, velocity] (a constant-velocity model). Ask Claude Code for *hints and review*, not the code.
- [ ] Expose the tuning: process noise Q and measurement noise R as CLI flags
- [ ] Note: the Kalman filter is **causal** (it only uses past frames). That's what makes real-time mode possible in week 6.

**Done when:** `--smoother kalman` works, and you can show how changing Q/R changes the plot.
**Checkpoint:** explain predict vs update, and what raising Q or R does, using your own plots.

### Week 5 (Dec 28-Jan 3): Metrics + comparison
**Goal:** numbers that prove it works.

- [ ] Claude Code builds `metrics`:
  - **ITF (interframe transformation fidelity):** average PSNR between consecutive output frames. Higher = steadier.
  - **Stability score:** take an FFT of the smoothed path and measure the share of energy in the lowest frequencies. Closer to 1 = smoother camera motion.
  - **Cropping ratio:** how much of the frame you kept.
- [ ] `scripts/compare_vidstab.sh`: run ffmpeg's `vidstabdetect` + `vidstabtransform` on the same clips. First check that your ffmpeg has it: `ffmpeg -filters | grep vidstab`
- [ ] Test on a public benchmark as well as your own clips. The NUS video stabilization dataset (Liu et al., 2013; categories like regular, running, zooming, crowd) is the standard one; check its download page.
- [ ] Results table: raw vs moving average vs Kalman vs vidstab

**Done when:** a results table in `results/` with all four methods on 4+ clips.
**Checkpoint:** explain what each metric captures and where your method wins or loses against vidstab, and why.

### Week 6 (Jan 4-10): Real-time webcam mode
**Goal:** live stabilization from the MacBook camera.

- [ ] `--webcam` mode: Kalman smoother only (moving average needs future frames)
- [ ] Measure and print FPS. Profile the slow parts and try fewer features or a downscaled frame for tracking.
- [ ] macOS will ask for camera permission for Terminal/VS Code on the first run. Allow it.
- [ ] **Optional ML stretch (after mid-Jan if short on time):** run a person-segmentation model (ONNX, via OpenCV DNN or ONNX Runtime) and ignore features on moving people. Compare metrics on the crowd clips with and without it. This turns it into a CV + ML project.

**Done when:** a screen recording of live side-by-side stabilization with an FPS counter.
**Checkpoint:** explain why offline smoothing can look better than real-time smoothing.

### Week 7 (Jan 11-17): Portfolio polish
**Goal:** a repo a recruiter understands in 30 seconds.

- [ ] README: a one-line summary, a before/after GIF at the top, a pipeline diagram, the math (short), the results table, how to build/run, and what you'd do next
- [ ] Clean up: consistent naming, remove dead code, comments on the math
- [ ] (Optional) A few unit tests with GoogleTest for the smoothers. Feed in a known path and check the output.
- [ ] Pin the repo on your GitHub profile; add it to your resume and LinkedIn

**Done when:** a friend who's never seen it can clone, build and run it from the README alone.

---

## Resume bullet template (fill in real numbers in week 7)

> **Real-time Video Stabilizer (C++, OpenCV, Eigen)** — Built a feature-based stabilization pipeline (optical-flow tracking, RANSAC motion estimation, Kalman-filtered camera trajectory) achieving [X]% higher interframe fidelity than raw footage and [comparable/better] stability than ffmpeg vidstab on [N] benchmark clips; runs live at [Y] FPS on a laptop webcam.

---

## If you fall behind

Cut in this order: unit tests → webcam mode → public benchmark (use only your own clips). **Never cut:** the hand-written Kalman filter, the metrics, and the README. Those are what make it a portfolio project instead of a tutorial.
