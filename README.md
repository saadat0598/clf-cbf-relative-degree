# Relative Degree and Barrier Design in CLF–CBF Quadratic Programs

Simulation code for the MSc dissertation *Relative Degree and Barrier Design in
CLF–CBF Quadratic Programs: A Comparative Simulation Study of a
Differential-Drive Robot and a Kinematic-Bicycle Vehicle*, submitted to The
University of Manchester for the degree of MSc in Advanced Control and Systems
Engineering, 2026.

The dissertation compares three safety-critical controllers built on control
barrier functions, and studies how the relative degree of a barrier with respect
to the available actuation channels determines what a designer can guarantee:

- **Case Study I** — pointwise CLF–CBF quadratic program on a differential-drive
  robot, with the barrier written on a lookahead point (dissertation Chapter 5)
- **Case Study II** — pointwise CLF–CBF quadratic program on a kinematic bicycle,
  with an exponential barrier on the rear disc (Chapter 6)
- **Case Study III** — receding-horizon MPC-CBF on the same bicycle, in both a
  sequential-convex linearised form and a nonlinear form, over a pointwise inner
  safety filter (Chapter 7)

## Requirements

<!-- TODO: confirm the version you actually ran and delete this comment -->

- MATLAB R2024b or later
- Simulink
- Optimization Toolbox (`quadprog` for the pointwise programs and the
  sequential-convex outer loop, `fmincon` for the nonlinear outer loop)
- Simscape Multibody — only if you run the Case Study I plant model

## Repository layout

<!-- TODO: replace with your actual filenames once the files are uploaded -->

```
case1_differential_drive/    Case Study I  (Chapter 5)
case2_kinematic_bicycle/     Case Study II (Chapter 6)
case3_predictive/            Case Study III (Chapter 7)
common/                      shared barrier and QP helpers
plotting/                    figure generation scripts
```

## Reproducing the figures

Each entry gives the script to run and the figure it produces in the
dissertation. Run the simulation first, then the plotting script, which expects
the logged signals in the base workspace.

<!-- TODO: fill in the script names. Delete rows that do not apply. -->

| Dissertation figure | Run this | Then this |
|---|---|---|
| Figure 1 — trajectory at nominal lookahead offset | | |
| Figure 2 — commanded inputs, nominal run | | |
| Figure 3 — trajectory at *a* = 0.10 m | | |
| Figure 4 — commanded inputs, stalled run | | |
| Figure 5 — parking scenario and two-circle model | | `plot_parking_handout.m` |
| Figure 6 — switching signal σ(*t*) | | |
| Figure 7 — diagnostics, nominal linearised run | | |
| Figure 8 — minimum barrier for four decay rates | | |
| Figure 9 — nonlinear MPC-CBF closed-loop track | | |
| Figure 10 — nonlinear MPC-CBF minimum barrier | | |

Parameter sweeps reported in the dissertation:

| Dissertation table | Swept parameter | Script |
|---|---|---|
| Table 1 | lookahead offset *a* (Section 5.6) | |
| Table 2 | ECBF pole magnitude *g* (Section 6.5.1) | |
| Table 3 | DCBF decay rate *γ*<sub>d</sub> (Section 7.6) | |
| Table 4 | prediction horizon *N*<sub>p</sub> (Section 7.6) | |

## Notes on the results

The barrier is written as a squared distance,
*h*(*x*) = *D*<sub>x</sub>² + *D*<sub>y</sub>² − *r*²,
so barrier values in the logs and figures are in m², not metres. A negative
value of magnitude |*h*| corresponds to a penetration of roughly |*h*|/2*r* of
the enforced circle.

Metrics reported in the dissertation are single-run rather than statistical, and
solve times were not logged. The tuning values used throughout
(*γ*<sub>d</sub> = 0.3, *N*<sub>p</sub> = 5, *g* = 2) were selected by simulation
and are not claimed to be optimal.

## Third-party code

<!-- TODO: list anything you did not write yourself, with its source and licence.
     If everything here is your own work, replace this section with:
     "All code in this repository was written by the author." -->

## Citing

If you use this code, please cite the dissertation:

> S. Karimova, "Relative degree and barrier design in CLF–CBF quadratic
> programs: a comparative simulation study of a differential-drive robot and a
> kinematic-bicycle vehicle," MSc dissertation, Dept. of Electrical and
> Electronic Engineering, Univ. of Manchester, Manchester, U.K., 2026.

## Licence

Released under the MIT Licence. See [LICENSE](LICENSE).
