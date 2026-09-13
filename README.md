<div align="center">

# Hand Gesture Recognition With Uncertainty Awareness<br>via FMCW Radar Sensing and Deep Learning

[![IEEE Sensors Journal](https://img.shields.io/badge/IEEE-Sensors_Journal_2025-00629B?style=for-the-badge&logo=ieee&logoColor=white)](https://ieeexplore.ieee.org/document/11023089/)
[![DOI](https://img.shields.io/badge/DOI-10.1109%2FJSEN.2025.3573743-0288D1?style=for-the-badge)](https://doi.org/10.1109/JSEN.2025.3573743)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg?style=for-the-badge)](Uncertainty-Aware/LICENSE)

[![Python](https://img.shields.io/badge/Python-3.8_|_3.9_|_3.10-3776AB?style=flat-square&logo=python&logoColor=white)](https://www.python.org/)
[![TensorFlow](https://img.shields.io/badge/TensorFlow-2.13.0-FF6F00?style=flat-square&logo=tensorflow&logoColor=white)](https://tensorflow.org/)
[![PyQt5](https://img.shields.io/badge/GUI-PyQt5_|_PyQtGraph-41CD52?style=flat-square&logo=qt&logoColor=white)](https://riverbankcomputing.com/software/pyqt/)
[![Hardware](https://img.shields.io/badge/Hardware-TI_AWR1243BOOST_|_DCA1000EVM-CC0000?style=flat-square&logo=circuitverse&logoColor=white)](https://www.ti.com/)
[![Robotics](https://img.shields.io/badge/Robotics-Universal_Robots_UR3-005B94?style=flat-square)](https://www.universal-robots.com/)

<br>

**Official repository for the research paper:**  
*"Hand Gesture Recognition With Uncertainty Awareness via FMCW Radar Sensing and Deep Learning"*  
Published in **IEEE Sensors Journal**, Vol. 25, No. 13, pp. 24517–24524, 2025.

[**The Tuan Trinh**](https://github.com/thetuantrinh)$^1$ · [**Hien Vu Pham**](https://orcid.org/0009-0003-3392-7590)$^1$ · [**Tien Dat Le**](https://orcid.org/0009-0009-8063-4866)$^1$ · [**Minhuy Le**](https://orcid.org/0000-0001-6152-6215)$^1$$^,$$^*$
*$^1$ Intelligent Communication System Laboratory (ICSLab), Phenikaa School of Engineering, Phenikaa University, Hanoi, Vietnam*  
$^*$*Corresponding author*: [huy.leminh@phenikaa-uni.edu.vn](mailto:huy.leminh@phenikaa-uni.edu.vn)

---

### [📄 Paper (IEEE Xplore)](https://doi.org/10.1109/JSEN.2025.3573743) • [🎬 Video Demos](#-demonstration-videos) • [🦾 UR3 Control](#-gesture-vocabulary--robot-action-mapping) • [📊 Dataset](#-dataset-access) • [⚡ Quick Start](#-quick-start) • [📖 Citation](#-citation)

---

</div>

## 🌟 Overview & Key Highlights

Frequency-modulated continuous-wave (FMCW) radar provides non-contact, privacy-preserving, and illumination-invariant sensing for human-machine interaction. While conventional deep learning (DL) models achieve high accuracy on benchmark radar datasets, they are typically **deterministic**—producing point estimates without conveying confidence reliability. In mission-critical robotics and autonomous systems, **uncalibrated predictions can cause catastrophic failures**.

This repository provides the complete, end-to-end framework introduced in our paper:

- 🎯 **High Precision**: Achieves **>99% recognition accuracy** across 10 dynamic gesture types from 10+ human subjects.
- 🛡️ **Uncertainty Quantification**: Seamlessly integrates:
  - **Monte Carlo Dropout (MCD)** for epistemic uncertainty estimation.
  - **Deep Ensemble Learning (DEL)** for variance mitigation across distinct parameter basins.
  - **Spectral-Normalized Neural Gaussian Process (SNGP)** for distance-aware, out-of-distribution (OOD) detection.
- 🦾 **Closed-Loop Robot Manipulation**: Directly controls an industrial **Universal Robots (UR3)** arm with a real-time fail-safe mechanism: when predictive uncertainty exceeds a safety threshold, robot motion is automatically halted.
- 📡 **Full Real-Time Hardware Pipeline**: Complete acquisition and signal processing workflow using TI 77 GHz mmWave radar ([AWR1243BOOST](https://www.ti.com/tool/AWR1243BOOST)) + [DCA1000EVM](https://www.ti.com/tool/DCA1000EVM) streaming over UDP to an interactive PyQt5 telemetry suite.

---

## 🎥 Demonstration Videos

| Application / Testbed | Video Preview | System Description |
| :--- | :---: | :--- |
| **Real-time Radar HGR System** | [![Radar HGR Demo](https://img.shields.io/badge/YouTube-Watch%20System%20Demo-red?style=for-the-badge&logo=youtube)](https://youtu.be/sJVkmNhxBvc) | Real-time ADC data streaming, Range-Doppler heatmaps, live class probabilities, and epistemic uncertainty gauge. |
| **Physical UR3 Robotic Control** | [![UR3 Physical Demo](https://img.shields.io/badge/YouTube-Watch%20UR3%20Demo-red?style=for-the-badge&logo=youtube)](https://youtu.be/mBBL0jrHSRk) | Non-contact gesture steering of physical UR3 manipulator with safety-interlocked Cartesian and gripper actuation. |
| **URSim Simulation Testbed** | [![URSim Simulation](https://img.shields.io/badge/YouTube-Watch%20URSim%20Demo-red?style=for-the-badge&logo=youtube)](https://youtube.com/shorts/XkRe66ik0ME?feature=share) | Hardware-in-the-loop (HIL) digital twin validation via Universal Robots URSim simulator over TCP/IP sockets. |

---

## 🤖 Gesture Vocabulary & Robot Action Mapping

The system classifies 10 dynamic gesture modes. In autonomous robotic mode, recognized gestures are translated into deterministic Cartesian / tool poses. The mapping is declared as data in [`robot/actions.py`](Uncertainty-Aware/src/radar_hgr/robot/actions.py):

| # | Gesture Class | Motion Pattern | UR3 Robotic Execution | Safety / Interlock Policy |
| :-: | :--- | :--- | :--- | :--- |
| **0** | `Empty` | Background / idle environment | Standby | No motion |
| **1** | `Counter-clockwise` | Circular arc (CCW) | Reserved trajectory | Safety verified |
| **2** | `Clockwise` | Circular arc (CW) | Reserved trajectory | Safety verified |
| **3** | `Push-down` | Downward hand thrust | **Lower TCP ($Z - 30\text{ mm}$)** | Auto-brake if uncertainty > threshold |
| **4** | `Pull-up` | Upward hand lift | **Lift TCP ($Z + 30\text{ mm}$)** | Auto-brake if uncertainty > threshold |
| **5** | `Zoom-out` | Expanding two-hand/palm spread | **Tool Release (Open Gripper)** | Auto-brake if uncertainty > threshold |
| **6** | `Zoom-in` | Contracting palm squeeze | **Tool Clamp (Close Gripper)** | Auto-brake if uncertainty > threshold |
| **7** | `To-left` | Lateral swipe to the left | **Translate TCP ($X + 30\text{ mm}$)** | Auto-brake if uncertainty > threshold |
| **8** | `To-right` | Lateral swipe to the right | **Translate TCP ($X - 30\text{ mm}$)** | Auto-brake if uncertainty > threshold |
| **9** | `Unknown` / OOD | Random clutter / unfamiliar motion | **Immediate Hold / Fail-Safe** | **Interlock triggered** |

---

## 🏛️ System Architecture

```
   ┌────────────────────────┐         ┌─────────────────────────┐
   │ Texas Instruments      │  Raw IQ │ Texas Instruments       │
   │ AWR1243BOOST Radar     ├────────►│ DCA1000EVM Capture Card │
   │ (77 GHz FMCW mmWave)   │  LVDS   │ (High-Speed UDP Stream) │
   └────────────────────────┘         └────────────┬────────────┘
                                                   │ Ethernet
                                                   ▼
   ┌────────────────────────────────────────────────────────────┐
   │ Real-time Processing & Uncertainty Engine (Host Workstation)│
   │  ├─ Digital Signal Processing: Range-Doppler / Micro-Doppler│
   │  ├─ Uncertainty-Aware Deep CNN (MCD / DEL / SNGP)           │
   │  ├─ Real-Time Telemetry Dashboard (PyQt5 & PyQtGraph)       │
   │  └─ Dynamic Safety Gate: Uncertainty Evaluation & Filtering │
   └──────────────────────────────┬─────────────────────────────┘
                                  │ TCP/IP Socket (Port 30003)
                                  ▼
   ┌────────────────────────────────────────────────────────────┐
   │ Industrial Manipulator: Universal Robots UR3 / URSim        │
   │  ├─ Real-time Tool Center Point (TCP) Position Translation │
   │  ├─ Pneumatic / Electric Gripper Actuation (Clamp/Release)  │
   │  └─ Instant Safety Hold whenever Uncertainty is Elevated   │
   └────────────────────────────────────────────────────────────┘
```

---

## 📁 Repository Layout

```text
Hand-Gesture-Recognition/
├── main.py                            # Convenience launcher for a fresh clone (--model, --verbose)
├── (2+1)D CVCNN/                      # Git submodule: Complex-Valued (2+1)D CVCNN model
├── Uncertainty-Aware/                 # Real-time application (installable package `radar-hgr`)
│   ├── pyproject.toml                 # Packaging, dependencies, pytest & ruff configuration
│   ├── LICENSE                        # MIT License
│   ├── README.md                      # Application documentation & developer notes
│   ├── models/                        # Pre-trained uncertainty-aware checkpoints
│   ├── Datasets/README.md             # Dataset specification & acquisition protocol
│   ├── tests/                         # Hardware-free unit tests
│   └── src/radar_hgr/
│       ├── cli.py                     # Argument parsing & console-script entry point
│       ├── config/                    # Frozen dataclasses: radar, network, robot, features, paths
│       ├── dsp/                       # Range-Doppler chain + packet-to-frame reassembly
│       ├── radar/                     # DCA1000EVM wire protocol & UDP client
│       ├── inference/                 # Vote smoothing, model loading & gesture predictor
│       ├── robot/                     # Gesture vocabulary, controllers, safety & vendored UR SDK
│       ├── workers/                   # Qt threads for acquisition & telemetry
│       ├── ui/                        # Window layout, presentation views & assets
│       └── app/                       # Hardware sessions & main-window wiring
├── .gitignore                         # Standard Python exclusion rules
├── .gitmodules                        # Submodule mapping
└── README.md                          # Primary project documentation
```

The application is layered with dependencies pointing strictly downwards:
`config` depends on nothing, the four domain packages (`dsp`, `radar`,
`inference`, `robot`) depend only on `config`, and `app` is the sole layer aware
of both the widgets and the hardware. See
[Uncertainty-Aware/README.md](Uncertainty-Aware/README.md) for the module map
and maintainer notes.

> 🔗 **Submodule Link:** `(2+1)D CVCNN` is directly linked to the companion repository [Complex-Valued-FMCW-Radar-Hand-Gesture-Recognition](https://github.com/thetuantrinh/Complex-Valued-FMCW-Radar-Hand-Gesture-Recognition).

---

## ⚡ Quick Start

### 1. Clone with Submodules

Clone this repository and recursively initialize all linked submodules:

```bash
git clone --recurse-submodules https://github.com/thetuantrinh/Hand-Gesture-Recognition.git
cd Hand-Gesture-Recognition
```

*(If cloned previously without `--recurse-submodules`, run `git submodule update --init --recursive`)*.

### 2. Environment Setup

We recommend Python 3.8–3.10 with a clean conda or virtual environment:

```bash
# Create and activate conda environment
conda create -n radar_hgr python=3.9 -y
conda activate radar_hgr

# Install the application (editable) together with TensorFlow
pip install -e 'Uncertainty-Aware[inference]'
```

TensorFlow is an optional extra: `pip install -e Uncertainty-Aware` alone gives
radar acquisition, the live micro-Doppler display and manual robot control,
with gesture classification disabled.

### 3. Run Real-Time Controller & GUI

Ensure your host machine is connected to:
- The **DCA1000EVM** card via Ethernet (Default IP: `192.168.33.30`).
- The **UR3 Controller** or **URSim** host (via TCP/IP).

Launch with the installed console script, or directly from the repository root:

```bash
radar-hgr                                  # after pip install
radar-hgr --model Uncertainty-Aware/models/model_1.h5 --verbose

python3 main.py                            # without installing
```

Inside the GUI:
1. **START RADAR** to view the live micro-Doppler feature map.
2. Tick **Predict Hand Gesture**, optionally selecting another checkpoint from
   `Uncertainty-Aware/models/` with the file browser.
3. **START UR3** and tick **Control UR3 by Hand Gesture** to close the loop.
   The jog pad and gripper selector operate independently of the classifier.

---

## 📊 Dataset Access

The dataset consists of raw radar time-domain ADC data and processed Doppler-range feature cubes collected from **10+ volunteers** under clean and controlled noise scenarios:
- `hand_gesture_recognition_dataset.mat`
- `hand_gesture_recognition_corrupted_dataset.mat` (with varying SNR levels)

> [!NOTE]  
> Due to hosting size constraints, the full datasets are hosted on dedicated institutional servers and available upon academic request.  
> Please contact **Dr. Minhuy Le** ([huy.leminh@phenikaa-uni.edu.vn](mailto:huy.leminh@phenikaa-uni.edu.vn)) or refer to [Uncertainty-Aware/Datasets/README.md](Uncertainty-Aware/Datasets/README.md).

---

## 📖 Citation

If this paper, codebase, pre-trained models, or dataset contribute to your research, please cite our publication:

```bibtex
@article{trinh2025hand,
  author={Trinh, The Tuan and Pham, Hien Vu and Le, Tien Dat and Le, Minhuy},
  journal={IEEE Sensors Journal}, 
  title={Hand Gesture Recognition With Uncertainty Awareness via FMCW Radar Sensing and Deep Learning}, 
  year={2025},
  volume={25},
  number={13},
  pages={24517--24524},
  doi={10.1109/JSEN.2025.3573743}
}
```

---

## 📄 License

This project is licensed under the **MIT License** — see the [LICENSE](Uncertainty-Aware/LICENSE) file for complete details.

---

## 🏛️ Acknowledgements

This research was conducted at the **Intelligent Communication System Laboratory (ICSLab)**, Phenikaa School of Engineering, Phenikaa University, Hanoi, Vietnam. We thank all study volunteers who contributed to the radar gesture dataset.
