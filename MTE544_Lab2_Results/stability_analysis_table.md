# Section 2: Stability and Performance Analysis Table (Point Controller)

| Controller Configuration | Agility: Rise Time $t_r$ [s] | Agility: Settling Time $t_s$ [s] | Overshoot [% OS] | Accuracy: $e_{ss}$ [m] | Final $\theta$ [rad] | Oscillations | Total Time [s] |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **pcontroller_baseline_point(1 1)** | 11.86 | 14.9 | 0.0% | 0.046 | 1.013 | 0 | 15.06 |
| **pcontroller_klp025kap09_point(1 1)GOOD** | 9.6 | 12.59 | 0.0% | 0.047 | 1.026 | 0 | 12.71 |
| **pcontroller_klp06kap04_point(1 1)BAD** | 6.8 | 27.5 | 6.7% | 0.05 | 1.877 | 29 | 27.6 |
| **pidcontroller_baseline_point(1 1)** | 13.11 | 16.11 | 0.0% | 0.047 | 1.05 | 0 | 16.21 |
| **pidcontroller_klp02klv05kli04kap08kav06kai02_point(1 1)** | 11.41 | 14.3 | 0.0% | 0.049 | 1.129 | 0 | 14.4 |
