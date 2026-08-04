# Obsidian Notebooks

_A personal knowledge base for visual SLAM, robotics, autonomous navigation, and geometric control._

---

This repository contains detailed study reports written for [Obsidian](https://obsidian.md/). The notes combine mathematical explanations, key-term definitions, worked intuition, diagrams, formulas, extracted figures, and links between related topics.

The current material is centered on two learning tracks:

- *Introduction to Visual SLAM: From Theory to Practice*
- MIT 16.485 — Visual Navigation for Autonomous Vehicles

## 📚 Repository contents

| Location | Purpose | Status |
| --- | --- | --- |
| [`1. Book and Paper Research`](<1. Book and Paper Research/>) | Book chapters, lecture reports, paper notes, source PDFs, and extracted figures | Active |
| `2. Projects/` | Organized project documentation | Reserved |
| `3. WorkingProjects/` | Work in progress and experimental material | Reserved |
| [`4. Rules`](<4. Rules/>) | Standards for producing research and chapter reports | Active |
| [`5. Extras`](<5. Extras/>) | Reusable note templates and supporting material | Active |
| `References/` | General reference material | Reserved |

> [!NOTE]
> Git does not preserve empty directories. Reserved folders may only appear after they contain a note or placeholder file.

## 🧭 Suggested learning path

```mermaid
flowchart LR
    accTitle: Visual Navigation Learning Path
    accDescr: Recommended progression from rigid-body geometry and camera models through visual odometry, optimization, complete SLAM systems, and quadrotor control

    geometry[📐 Rigid-body geometry] --> lie_theory[🔄 Lie groups and algebras]
    lie_theory --> cameras[📷 Camera models]
    cameras --> odometry[🔍 Visual odometry]
    odometry --> estimation[⚙️ State estimation]
    estimation --> slam[🗺️ Complete SLAM systems]
    slam --> control[🚁 Quadrotor control]

    classDef foundation fill:#dbeafe,stroke:#2563eb,stroke-width:2px,color:#1e3a5f
    classDef perception fill:#ede9fe,stroke:#7c3aed,stroke-width:2px,color:#3b0764
    classDef system fill:#dcfce7,stroke:#16a34a,stroke-width:2px,color:#14532d

    class geometry,lie_theory foundation
    class cameras,odometry perception
    class estimation,slam,control system
```

## 📖 Introduction to Visual SLAM

The [`book_intro_to_vslam`](<1. Book and Paper Research/book_intro_to_vslam/>) collection follows the major stages of a visual SLAM system from mathematical foundations to practical implementation.

| Part | Topics |
| --- | --- |
| Foundations | SLAM structure, rigid-body motion, coordinate transformations, quaternions, $SO(3)$, and $SE(3)$ |
| Perception | Camera projection, distortion, stereo vision, RGB-D sensing, features, matching, and optical flow |
| Motion estimation | Epipolar geometry, triangulation, PnP, ICP, feature-based methods, and direct methods |
| Estimation | Least squares, Gauss–Newton, Levenberg–Marquardt, Kalman filters, bundle adjustment, and robust kernels |
| Backend | Sliding-window optimization, marginalization, pose graphs, and loop closure |
| Mapping | Sparse maps, dense reconstruction, depth filters, point clouds, OctoMap, and TSDF |
| Systems | Stereo visual odometry, ORB-SLAM, LSD-SLAM, SVO, visual-inertial SLAM, and semantic SLAM |

The collection includes reports for Chapters 1–13 and an appendix covering Gaussian distributions and matrix derivatives.

## 🚁 MIT Visual Navigation

The [`MIT16.485 - Visual Navigation for Autonomous Vehicles`](<1. Book and Paper Research/MIT16.485 - Visual Navigation for Autonomous Vehicles/>) collection currently covers:

- [Lecture 4 — Lie Groups](<1. Book and Paper Research/MIT16.485 - Visual Navigation for Autonomous Vehicles/Lecture 4 - Lie Groups.md>)
- [Lecture 6 — Quadrotor Model](<1. Book and Paper Research/MIT16.485 - Visual Navigation for Autonomous Vehicles/Lecture 6 - Quadrotor Model.md>)
- [Geometric Tracking Control of a Quadrotor UAV on SE(3)](<1. Book and Paper Research/MIT16.485 - Visual Navigation for Autonomous Vehicles/References/Geometric Tracking Control of a Quadrotor UAV on SE(3)/Geometric Tracking Control of a Quadrotor UAV on SE(3).md>)

Together, these notes connect Lie-group geometry with rigid-body dynamics, differential flatness, nonlinear tracking control, and stability analysis.

## 📝 Note format

Research reports follow the conventions in [Research-reports.md](<4. Rules/Research-reports.md>). A typical report contains:

- Obsidian YAML frontmatter with topic-specific tags
- A chapter or document overview
- Detailed explanations of important concepts
- Mathematical expressions using inline and block LaTeX
- Definitions and study checklists
- Obsidian callouts for summaries, warnings, and tips
- Internal links between related chapters
- Extracted figures placed beside the relevant explanation
- Mermaid diagrams for workflows and conceptual relationships

## 🛠️ Open the vault

Clone the repository:

```bash
git clone https://github.com/NguyenNhaKhanh/Obsidian_Notebooks.git
```

Then open Obsidian and select **Open folder as vault**, choosing the cloned `Obsidian_Notebooks` directory.

The repository includes its `.obsidian` configuration and the Translate community plugin. Review community-plugin settings before enabling them in a new environment.

## 📦 Repository size

The source-material directory includes PDFs and extracted figures. In particular, `slambook-en.pdf` is approximately 70 MB. This is accepted by GitHub, but it makes the initial clone larger than a text-only notes repository.

## 📌 Purpose

This vault is intended to be a long-term learning resource rather than a collection of short summaries. The reports emphasize understanding, mathematical intuition, connections between topics, and material that can be reviewed later without reopening every source document.
