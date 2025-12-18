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

- The pendulum is naturally unstable in the upright position.
- By accelerating or braking the reaction wheel, we generate torque.
- The controller reads the angle and angular velocity and chooses the motor
command.

<img width="1024" height="370" alt="1715776245-pid-block-diagram" src="https://github.com/user-attachments/assets/6fbe90b9-d3e6-4e79-8d7c-aaf1deb1b7b5" />


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
- [Measurements with no load](data/csv_data/motor_data_no_load.csv)
- [Measurements with the wheel and 8 bolts/screws](data/csv_data/motor_data_8screws_8bolts_evenly_distributed.csv)

![RPM as a function of PWM](data/graphs/RPM_as_a_function_of_PWM/RPM_Comparison.jpg)

- [Watch the pendulum balancing](https://github.com/user-attachments/assets/33b4c6e5-de3b-42a6-be7a-60adbd040c9a)
- [Watch the pendulum balancing again](https://github.com/user-attachments/assets/a8724a11-2e41-4fb6-b088-a9859eba4981)
- [Watch the pendulum balancing from upside down](https://github.com/user-attachments/assets/08ec3768-02b8-4cd1-acb5-1238e4f26114)
---
## 6. Results
Summarize:
- The pendulum did stabilize.
- With bearings, the pendulum resisted disturbances as far as 10 degrees, as long as it wasn't too fast. 
- The design of pendulum heavily limits its precision due to how precise the setpoint measurement is. It is not only very difficult to measure as it also changes over time and during the stabilizing process. 
---
## 7. Project Timeline (from `/progress`)
The Weekly Progress Reports describe the work we've done on the project each week. They detail the results we achieved, the challenges we encountered, our solutions, and our goals for the following week. These weekly reports provide a clear overview of our project progress. If we want to do a similar project later on, we can easily read in these reports what steps we need to take and what problems we need to think about.
## 8. Team and Credits
- Technische Natuurkunde program (HvA)
- Instituto Superior de Engenharia de Lisboa (ISEL)
- Cisca Wams
- Splinter Kohlbrugge
- Thijs Ritter
- David Varela
- Gabriel Marques
- Supervisors
- Collaborating institutions
---
## 9. Repository
Project repository:
- [GitHub: inverted-pendulum-international-team-4](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-4)
