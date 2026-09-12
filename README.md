# OCULUS: Perception-Guided Adaptive 2.5D LiDAR Mapping
### Autonomous Perception & Hierarchical Spatial Mapping Research Platform

[![Live Web Application](https://img.shields.io/badge/Live%20Demo-Online-success?style=for-the-badge&logo=googlechrome)](https://karanxgojo-cmyk.github.io/oculus-adaptive-lidar-mapping/)
[![GitHub Repository](https://img.shields.io/badge/GitHub-oculus--adaptive--lidar--mapping-blue?style=for-the-badge&logo=github)](https://github.com/karanxgojo-cmyk/oculus-adaptive-lidar-mapping)
[![Continuous Deployment](https://img.shields.io/badge/CD-GitHub%20Pages%20Automated-brightgreen?style=for-the-badge&logo=githubactions)](https://github.com/karanxgojo-cmyk/oculus-adaptive-lidar-mapping/actions)

---

## Live Public Deployment & Repository Links

- **Live Public Web Application**: [https://karanxgojo-cmyk.github.io/oculus-adaptive-lidar-mapping/](https://karanxgojo-cmyk.github.io/oculus-adaptive-lidar-mapping/)
- **GitHub Source Repository**: [https://github.com/karanxgojo-cmyk/oculus-adaptive-lidar-mapping](https://github.com/karanxgojo-cmyk/oculus-adaptive-lidar-mapping)

---

## Executive Overview

**OCULUS** is an autonomous vehicle spatial intelligence system that implements **Perception-Guided Adaptive 2.5D LiDAR Mapping**.

Rather than allocating a uniformly dense, computationally prohibitive 2.5D grid across the full $100\,\text{m} \times 80\,\text{m}$ sensing area (which demands $>3.2 \times 10^6$ grid cells at $5\,\text{cm}$ resolution), OCULUS adaptively allocates spatial cell resolution:
$$R = f(\text{distance}, \text{semantic class}, \text{geometry complexity}, \text{motion}, \text{uncertainty}, \text{navigation corridor})$$

### Core Advantages
1. **$>99\%$ Active Cell Reduction**: Retains $5\,\text{cm}$ resolution only where it matters (pedestrians, vehicles, obstacles, ego navigation path) while coarsening static background to $80\,\text{cm}$.
2. **Elevation-Aware Hover & 2.5D Inspector**: Interactive hover inspection over vehicles, pedestrians, poles, barriers, buildings ("Open Parking"), and terrain cells displaying Base Elevation ($Z_{base}$), Top Elevation ($Z_{top}$), Object Height ($\Delta Z$), Variance ($\sigma_z^2$), and Terrain Slope.
3. **Subtle Elevation Visual Dimensioning**: On-canvas vertical CAD dimensioning line showing base/top markers and live height callouts.
4. **Sub-Millisecond Cache / <50 ms Live Loop**: Strictly optimized rendering engine operating at target rates of 10, 20, 30, 40, and 50 FPS.
5. **Fixed World Consistency**: Permanent global coordinates for static assets (poles, curbs, crosswalks, trees, buildings) with true ego-motion translation and rotation along analytical spline trajectories.

---

## System Architecture

```
                                  [ LiDAR Sensor Stream ]
                                             ¦
                                             ?
                                  [ Preprocessing Stage ]
                             (Ground Removal, Voxel Filtering)
                                             ¦
                                             ?
                             [ Perception & Semantic Engine ]
                         (Bounding Boxes, Velocity, Semantics)
                                             ¦
                                             ?
                             [ Risk & Importance Evaluator ]
                         (Path Relevance, Collision Danger, TTC)
                                             ¦
                                             ?
                             [ Adaptive 2.5D Quadtree Grid ]
             Level 0 (5cm, RED) -- Critical Path, Pedestrians, Obstacles
             Level 1 (10cm, ORANGE) -- Dynamic Vehicles, Curbs
             Level 2 (20cm, YELLOW) -- Mid-range Roadway (25-50m)
             Level 3 (40cm, LIGHT BLUE) -- Distant Roadway (50-80m)
             Level 4 (80cm, DEEP BLUE) -- Distant Static Background (>80m)
                                             ¦
                                             ?
                               [ Web Cockpit Visualizer ]
                     FastAPI Backend (Local) / GitHub Pages (Public)
```

---

## Running Locally

### Prerequisites
- Python 3.10+
- `pip install -r requirements.txt`

### 1. Launch Interactive Cockpit (FastAPI)
```bash
python main.py --mode web --port 8050
```
*Or double-click `run_demo.bat` (Windows). The cockpit will open automatically in your browser at `http://127.0.0.1:8050`.*

### 2. Run Headless Performance Benchmark
```bash
python main.py --mode cli --frames 10
```

### 3. Run Automated Verification Tests
```bash
python test_pipeline.py
```

---

## How Public Deployment Works

The live application is hosted on **GitHub Pages** with an automated **Continuous Deployment (CD)** pipeline powered by **GitHub Actions** (`.github/workflows/deploy.yml`):

1. **Static Pre-Rendering & Resilient Fallback**: All 180 sequential 3-minute driving keyframes are pre-rendered into `app/visualization/static/api/frames/`.
2. **Dual-Stack Architecture**: The frontend code (`dashboard.js`) automatically detects whether it is running on the FastAPI local server (`http://127.0.0.1:8050`) or static CDN hosting (`https://...`). It resolves routes dynamically with fallback to static precomputed JSON payloads.
3. **Automated CD Trigger**: Any `git push` to branch `main` triggers `.github/workflows/deploy.yml`, which packages the static application and deploys it to GitHub Pages in under 30 seconds.

---

## Instructions for Updating the Live Site

To update the public application in the future:

1. **Make your code edits** in the project directory (e.g., modifying `dashboard.js`, `style.css`, `index.html`, or pipeline models).
2. If simulation parameters or frames were changed, re-export static data:
   ```bash
   python export_static_data.py
   ```
3. Commit and push the changes:
   ```bash
   git add .
   git commit -m "feat: description of your update"
   git push origin main
   ```
4. **GitHub Actions will automatically build and redeploy the live application** at:
   [https://karanxgojo-cmyk.github.io/oculus-adaptive-lidar-mapping/](https://karanxgojo-cmyk.github.io/oculus-adaptive-lidar-mapping/)

---

## Technical Disclosures & Integrity
- **Perception Mode**: Deterministic geometric clustering and spatial proximity are used for real-time edge execution without requiring external heavy GPU weights.
- **Measured Metrics**: All frame latency, memory reduction percentages, and FPS figures reflect actual computational operations.
- **Memory Comparison**: Reductions are benchmarked against an equivalent uniform $5\,\text{cm}$ grid covering the identical $100\,\text{m} \times 80\,\text{m}$ spatial boundary.
