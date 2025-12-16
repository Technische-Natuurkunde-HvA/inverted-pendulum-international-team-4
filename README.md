# Reaction Wheel Inverted Pendulum – Team 4
This project was carried out by an international team of students at the Technische Natuurkunde program (HvA) and Instituto Superior de Engenharia de Lisboa (ISEL). The objective is to stabilize an inverted pendulum using a reaction wheel driven by an electric motor.
---
## 1. Project Motivation

An inverted pendulum is a system where a weight is balanced upside down, making it naturally unstable. We think of it as an interesting project due to how tough of a challenge it is in physic's and engineering. How to keep something unstable from falling over? The solution comes from control theory, which uses math to create a system that can mantain the balance of a pendulum. But this time, it just so happens to be upside down, trying to balance at the very top. It's all about making very tiny and quick adjustments to keep things stable.

---
## 2. System Overview

<img width="472" height="602" alt="oef3inverted2" src="https://github.com/user-attachments/assets/50021137-85a7-4959-b0f2-3bd480b67d1e" />

- Pendulum arm with angle sensor
- Reaction wheel attached to a DC motor
- Arduino (or compatible microcontroller)
- Power electronics and safety features
---
## 3. Control Principle
Explain in words first (for non-technical people), then more mathematically
(for technical readers).
- The pendulum is naturally unstable in the upright position.
- By accelerating or braking the reaction wheel, we generate torque.
- The controller reads the angle and angular velocity and chooses the motor
command.
(You can show block diagrams as images from `visuals/`.)
![Control block diagram](../visuals/control_block_diagram.png)
---
## 4. Implementation
### 4.1 Arduino Control Software (folder `code/`)
Explain:
- Main control loop frequency (e.g., 100 Hz)
- Which `.ino` file is the main entry point
- How sensor readings and motor outputs are handled
### 4.2 Python Tools (also in `code/`)
Explain:
- Reading measurement files from `data/`
- Generating plots saved into `visuals/`
- Any analysis (e.g., identification, performance metrics)
---
## 5. Experiments and Data
Link to measurement files in `data/`:
- [Step response data](../data/step_response.csv)
- [Stabilization log](../data/stabilization_run1.csv)
Include images created from those data:
![Angle over time](../visuals/angle_vs_time.png)
If you have videos in `visuals/`:
- [Watch the pendulum balancing](../visuals/stabilization_demo.mp4)
Or link to YouTube, if you prefer.
---
## 6. Results
Summarize:
- Did the pendulum stabilize?
- Typical settling time
- Sensitivity to disturbances
- Limitations of the design
Include relevant plots:
![Control effort over time](../visuals/control_effort.png)
---
## 7. Project Timeline (from `/progress`)
Explain that the weekly reports in `progress/` document the evolution.
Optionally link to them:
- [Week 1 report](../progress/week01.md)
- [Week 2 report](../progress/week02.md)
- ...
---
## 8. Team and Credits
- Student names and universities
- Supervisors
- Collaborating institutions
---
## 9. Repository
Project repository:
- [GitHub: inverted-pendulum-international-team-x](https://github.com/TechnischeNatuurkunde-HvA/inverted-pendulum-international-team-x)
