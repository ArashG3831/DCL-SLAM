# Literature mapping

| Project mechanism | Closest anchor | Disposition | Exact project adaptation |
|---|---|---|---|
| Equal task utility, travel cost, utility reduction | Burgard et al. 2005, Algorithm 1 | Keep/replace | Canonical tasks and Nav2 path lengths replace frontier cells/value iteration. |
| Information value minus travel cost | Zlot et al. | Keep | The implementation uses `U_t - beta*C_i,t`; beta is 1.0. |
| Sensor-range redundancy reduction | Burgard et al. 2005 | Keep/adapt | `P(d)=1-d/R` when LOS is clear, otherwise zero. |
| Canonical same-task prevention | Project protocol | Keep | Physical equivalence is resolved before bids; one canonical ID can be assigned once. |
| Nav2 path feasibility | Project navigation layer | Keep | Local `ComputePathToPose`, Euclidean polyline metres, 18 m admissibility gate. |
| Route overlap scalar | Project heuristic | Move | Geometry is no longer exploration utility; selected-route conflict is traffic scheduling. |
| Hard failure | Project failure memory | Keep as gate | Suppression/expiry excludes robot-task pairs; no magic negative score. |
| Workload balance | Project heuristic | Remove from production | Fairness is not an exploration objective or safety gate. |
| Doorway ordering | Chandra et al. 2023 architectural precedent | Adapt | Cooperative deterministic pre-dispatch ETA ordering, not their full optimizer. |

Primary references: Burgard, Moors, Stachniss, Schneider, IEEE TRO 2005, DOI 10.1109/TRO.2004.839232; Zlot et al., *Market-Driven Multi-Robot Exploration*; Chandra et al., arXiv:2306.08815.
