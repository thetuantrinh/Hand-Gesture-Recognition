# Hand Gesture Recognition With Uncertainty Awareness via FMCW Radar Sensing and Deep Learning

[![IEEE Sensors Journal](https://img.shields.io/badge/IEEE-Sensors_Journal_2025-00629B.svg)](https://ieeexplore.ieee.org/document/11023089/)
[![DOI](https://img.shields.io/badge/DOI-10.1109%2FJSEN.2025.3573743-blue.svg)](https://doi.org/10.1109/JSEN.2025.3573743)
[![Python 3.8+](https://img.shields.io/badge/python-3.8%2B-brightgreen.svg)](https://www.python.org/)
[![TensorFlow](https://img.shields.io/badge/TensorFlow-2.13.0-orange.svg)](https://tensorflow.org/)
[![Hardware](https://img.shields.io/badge/Hardware-TI_AWR1243_%7C_DCA1000_%7C_UR3-lightgrey.svg)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Official repository for the article:  
**"Hand Gesture Recognition With Uncertainty Awareness via FMCW Radar Sensing and Deep Learning"**, published in **IEEE Sensors Journal**, Vol. 25, No. 13, pp. 24517–24524, 2025.  
DOI: [10.1109/JSEN.2025.3573743](https://doi.org/10.1109/JSEN.2025.3573743)

---

## 👥 Authors & Affiliation

**The Tuan Trinh**, **Hien Vu Pham**, **Tien Dat Le**, and **Minhuy Le**$^*$  
*Intelligent Communication System Laboratory (ICSLab), Phenikaa School of Engineering, Phenikaa University, Hanoi, Vietnam*  
$^*$*Corresponding Author*: [huy.leminh@phenikaa-uni.edu.vn](mailto:huy.leminh@phenikaa-uni.edu.vn) | [Website](https://icslab.phenikaa-uni.edu.vn/)

---

## 📖 Abstract

Frequency-modulated continuous-wave (FMCW) radar is a promising sensor technology for hand gesture recognition in autonomous control systems due to its privacy preservation, robustness to ambient lighting conditions, and non-contact operation. While deep learning (DL) models have demonstrated significant success in classifying radar gesture signals, traditional deterministic DL models lack the ability to convey **predictive uncertainty**. In mission-critical and human-robot interaction (HRI) applications, knowing *when a model does not know* is paramount for safety.

This work introduces an **uncertainty-aware deep convolutional neural network (CNN)** framework for FMCW radar-based gesture recognition. The framework incorporates:
1. **Monte Carlo Dropout (MCD)** for sampling-based epistemic uncertainty quantification.
2. **Deep Ensemble Learning (DEL)** for robust variance estimation across diverse model initializations.
3. **Spectral-Normalized Neural Gaussian Process (SNGP)** for distance-aware uncertainty calibration.

Evaluated on a comprehensive dataset of 10 distinct gestures collected from 10+ volunteers, the proposed model achieves **over 99% recognition accuracy** and demonstrates superior robustness against environmental clutter and additive noise compared to conventional deterministic architectures. Furthermore, the system is integrated into a real-time hardware-in-the-loop control pipeline for a **Universal Robots (UR3)** industrial robotic arm with safety-aware fail-safe gating.

---

## 🎥 Demonstration Videos

| Demonstration | Preview / Video Link | Description |
| :--- | :---: | :--- |
| **Radar Hand Gesture Recognition System** | [![Demo Video](https://img.shields.io/badge/YouTube-Watch%20Demo-red?logo=youtube)](https://youtu.be/sJVkmNhxBvc) | Real-time radar data capture, range-Doppler processing, and gesture prediction with live uncertainty estimation. |
| **UR3 Robotic Arm Real-World Control** | [![UR3 Video](https://img.shields.io/badge/YouTube-Watch%20UR3%20Demo-red?logo=youtube)](https://youtu.be/mBBL0jrHSRk) | Physical Universal Robots UR3 arm manipulation driven by real-time gesture commands via Ethernet interface. |
| **URSim Digital Twin Simulation** | [![URSim Video](https://img.shields.io/badge/YouTube-Watch%20Simulation-red?logo=youtube)](https://youtube.com/shorts/XkRe66ik0ME?feature=share) | Hardware-in-the-loop validation using Universal Robots URSim software environment before physical deployment. |

---

## 🛠️ System Architecture & Hardware Setup

```
   ┌──────────────────────┐         ┌─────────────────────────┐
   │  Texas Instruments   │  Raw IQ │ Texas Instruments       │
   │  AWR1243BOOST Radar  ├────────►│ DCA1000EVM Capture Card │
   │  (77 GHz mmWave)     │ LVDS    │ (High-speed UDP Stream) │
   └──────────────────────┘         └────────────┬────────────┘
                                                 │ Ethernet
                                                 ▼
   ┌──────────────────────────────────────────────────────────┐
   │ Real-time Processing & Uncertainty-Aware Engine (Host)   │
   │  • Range-Doppler / Micro-Doppler FFT Preprocessing       │
   │  • Deep CNN (Monte Carlo Dropout / DEL / SNGP)           │
   │  • PyQt5 Dashboard: Confidence, Uncertainty & Telemetry  │
   └──────────────────────────────┬───────────────────────────┘
                                  │ Safety-gated commands
                                  ▼
   ┌──────────────────────────────────────────────────────────┐
   │ Universal Robots UR3 Industrial Robotic Arm (or URSim)   │
   │  • End-effector Cartesian & Joint-space Trajectory       │
   │  • Automatic fail-safe hold when uncertainty is elevated │
   └──────────────────────────────────────────────────────────┘
```

- **Radar Sensor**: Texas Instruments [AWR1243BOOST](https://www.ti.com/tool/AWR1243BOOST) mmWave sensor (76–81 GHz).
- **Data Capture Card**: Texas Instruments [DCA1000EVM](https://www.ti.com/tool/DCA1000EVM) providing real-time raw ADC data streaming over UDP.
- **Robotic Manipulator**: [Universal Robots UR3](https://www.universal-robots.com/products/ur3-robot/) 6-DOF robotic arm.
- **Gesture Alphabet**: 10 hand gestures performed by 10+ participants under various angles and noise perturbations.

---

## 📂 Repository Structure

```text
Hand-Gesture-Recognition/
├── (2+1)D CVCNN/              # Git Submodule: Complex-Valued (2+1)D CVCNN architecture
├── Uncertainty-Aware/
│   ├── Datasets/              # Dataset metadata, specifications, and access information
│   │   └── README.md
│   └── scripts/
│       ├── main_control.py    # Main PyQt5 application: Radar DSP, inference & UR3 control
│       ├── requirements.txt   # Python dependency specifications
│       ├── LICENSE            # MIT License
│       ├── models/            # Pretrained uncertainty-aware model checkpoints (.h5)
│       │   ├── model_1.h5
│       │   └── model_2.h5
│       └── src/
│           ├── DSP/           # Radar signal processing & feature extraction routines
│           ├── radar/         # mmWave sensor configuration & DCA1000 interface
│           ├── UI/            # PyQt5 Graphical User Interface components
│           ├── UR/            # Universal Robots TCP/IP socket client & motion commands
│           ├── use_case/      # Prediction engines and robot control state machines
│           ├── thread_fn/     # Multi-threaded acquisition and worker implementations
│           └── utils/         # Plotting, metrics, and visualization utilities
├── .gitmodules                # Git submodule configuration
└── README.md
```

> **Submodule Note:** `(2+1)D CVCNN` is linked directly to [Complex-Valued-FMCW-Radar-Hand-Gesture-Recognition](https://github.com/thetuantrinh/Complex-Valued-FMCW-Radar-Hand-Gesture-Recognition).

---

## 🚀 Getting Started

### 1. Clone the Repository (with Submodules)

Clone the repository and recursively fetch all nested submodules:

```bash
git clone --recurse-submodules https://github.com/thetuantrinh/Hand-Gesture-Recognition.git
cd Hand-Gesture-Recognition
```

If you have already cloned without `--recurse-submodules`, initialize the submodule using:

```bash
git submodule update --init --recursive
```

### 2. Environment Setup

It is recommended to use Python 3.8–3.10 and create a virtual environment:

```bash
# Using conda
conda create -n radar_hgr python=3.9 -y
conda activate radar_hgr

# Or using venv
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies

Install the required packages:

```bash
cd Uncertainty-Aware/scripts
pip install -r requirements.txt
```

### 4. Running the Real-Time Application

Launch the real-time GUI application for radar gesture prediction and robot control:

```bash
python3 main_control.py
```

The GUI allows you to:
- Connect to the DCA1000EVM / AWR1243BOOST radar over Ethernet/USB.
- Connect to the Universal Robots UR3 controller (or local URSim IP).
- Visualize real-time radar Range-Doppler heatmaps and gesture classification probability distributions.
- Monitor predictive uncertainty in real time with safety interlocks.

---

## 📊 Dataset Access

The dataset contains raw radar ADC data and processed feature cubes corresponding to **10 gesture types** collected from **10+ volunteers** under clean and noise-corrupted environments (`hand_gesture_recognition_dataset.mat` and `hand_gesture_recognition_corrupted_dataset.mat`).

* Due to storage size considerations, full raw datasets are available upon request.
* Please contact **Dr. Minhuy Le** at [huy.leminh@phenikaa-uni.edu.vn](mailto:huy.leminh@phenikaa-uni.edu.vn) or refer to [Uncertainty-Aware/Datasets/README.md](Uncertainty-Aware/Datasets/README.md) for details.

---

## 📝 Citation

If you find this work, codebase, or dataset useful in your research, please cite our IEEE paper:

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

## 📜 License

This project is licensed under the **MIT License** - see the [LICENSE](Uncertainty-Aware/scripts/LICENSE) file for details.

---

## 🙏 Acknowledgements

This research was conducted at the **Intelligent Communication System Laboratory (ICSLab)**, Phenikaa School of Engineering, Phenikaa University, Hanoi, Vietnam.
