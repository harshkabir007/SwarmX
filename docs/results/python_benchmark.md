# SwarmX benchmark

Generated 2026-09-27 21:11 · seeds=6 · 270 runs · 332s wall

## Success criteria

* Zero inter-robot collisions: **PASS** (0 collisions in 90 SwarmX runs)
* All tasks completed by SwarmX in every run: **yes**
* Aisle-lock safety (ground truth): 0 incompatible co-occupancies, 0 entries without a lock
* Total task completion time reduction on overlapping paths (crossing) vs stop-and-wait: 3 robots: 8.3%, 5 robots: 18.8%, 8 robots: 77.5% -> **NOT MET for every fleet size** (target >= 20%)

## SwarmX vs stop-and-wait

| scenario | robots | total completion time | makespan | mean task time | baseline finished all tasks |
|---|---:|---:|---:|---:|:---:|
| blocked_aisle | 3 | 14.2% | 9.5% | 14.2% | yes |
| blocked_aisle | 5 | 13.3% | 6.6% | 13.3% | yes |
| blocked_aisle | 8 | 8.4% | 5.3% | 8.3% | yes |
| crossing | 3 | 8.3% | 11.1% | 8.3% | yes |
| crossing | 5 | 18.8% | 21.4% | 18.8% | yes |
| crossing | 8 | 77.5% | 80.5% | 16.6% | **no (gridlock)** |
| hot_aisles | 3 | 13.1% | 11.5% | 13.1% | yes |
| hot_aisles | 5 | 13.0% | 10.9% | 13.0% | yes |
| hot_aisles | 8 | 12.4% | 8.4% | 12.4% | yes |
| random | 3 | 13.6% | 11.7% | 13.6% | yes |
| random | 5 | 14.6% | 15.0% | 14.6% | yes |
| random | 8 | 9.4% | 8.7% | 9.4% | yes |
| robot_failure | 3 | 15.3% | 11.0% | 15.3% | yes |
| robot_failure | 5 | 9.3% | 10.1% | 9.3% | yes |
| robot_failure | 8 | 7.3% | -3.9% | 7.3% | yes |

Reductions are relative to the stop-and-wait baseline (positive = SwarmX faster). Unfinished tasks are charged the time limit.

## All runs (mean ± std over seeds)

| scenario | robots | method | done | total completion [s] | makespan [s] | collisions | min sep [m] | zone wait [s] | stop time [s] | reroutes | msgs/robot/s | kB/s/robot |
|---|---:|---|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| blocked_aisle | 3 | ghost | ✓ | 3068.4 ± 229.3 | 344.2 ± 25.3 | 9 ± 1 | 0.01 ± 0.01 | 0.0 | 0.0 | 0 | 10.4 | 6.43 ± 0.17 |
| blocked_aisle | 3 | stopwait | ✓ | 3771.8 ± 217.7 | 396.4 ± 13.7 | 0 | 0.93 | 13.2 ± 3.0 | 53.1 ± 22.2 | 0 | 10.2 | 7.00 ± 0.09 |
| blocked_aisle | 3 | swarmx | ✓ | 3234.7 ± 247.7 | 358.8 ± 30.6 | 0 | 0.69 | 23.9 ± 12.2 | 0.0 | 0 | 10.4 | 6.54 ± 0.21 |
| blocked_aisle | 5 | ghost | ✓ | 2051.4 ± 175.6 | 225.0 ± 30.3 | 14 ± 3 | 0.01 ± 0.01 | 0.0 | 0.0 | 0 | 10.5 ± 0.1 | 6.65 ± 0.32 |
| blocked_aisle | 5 | stopwait | ✓ | 2609.6 ± 153.8 | 257.2 ± 12.4 | 0 | 0.90 ± 0.04 | 22.2 ± 3.9 | 132.6 ± 49.5 | 0 | 10.2 | 6.95 ± 0.16 |
| blocked_aisle | 5 | swarmx | ✓ | 2263.4 ± 118.5 | 240.2 ± 25.2 | 0 | 0.69 | 49.6 ± 18.0 | 0.0 | 0 | 10.5 ± 0.1 | 6.82 ± 0.30 |
| blocked_aisle | 8 | ghost | ✓ | 3226.9 ± 261.3 | 239.8 ± 43.0 | 50 ± 5 | 0.00 | 0.0 | 0.0 | 0 | 10.6 ± 0.1 | 8.53 ± 0.56 |
| blocked_aisle | 8 | stopwait | ✓ | 4446.8 ± 311.4 | 273.4 ± 9.4 | 0 | 0.78 ± 0.11 | 54.5 ± 21.6 | 381.1 ± 68.8 | 0 | 10.1 | 7.98 ± 0.16 |
| blocked_aisle | 8 | swarmx | ✓ | 4075.4 ± 242.2 | 258.8 ± 15.3 | 0 | 0.69 | 151.4 ± 50.6 | 0.0 | 0 ± 1 | 10.6 ± 0.1 | 9.27 ± 0.17 |
| crossing | 3 | ghost | ✓ | 813.7 ± 26.2 | 108.0 ± 6.0 | 6 ± 2 | 0.01 ± 0.01 | 0.0 | 0.0 | 0 | 10.5 | 6.23 ± 0.08 |
| crossing | 3 | stopwait | ✓ | 1015.0 ± 54.1 | 137.0 ± 6.0 | 0 | 0.96 ± 0.02 | 19.9 ± 5.1 | 62.2 ± 12.9 | 0 | 10.2 | 6.37 ± 0.09 |
| crossing | 3 | swarmx | ✓ | 930.9 ± 28.2 | 121.8 ± 3.0 | 0 | 0.70 ± 0.01 | 45.8 ± 13.6 | 0.0 | 0 ± 0 | 10.5 | 6.59 ± 0.07 |
| crossing | 5 | ghost | ✓ | 1359.6 ± 22.8 | 113.2 ± 2.9 | 23 ± 6 | 0.00 | 0.0 | 0.0 | 0 | 10.6 ± 0.1 | 7.74 ± 0.14 |
| crossing | 5 | stopwait | ✓ | 2142.9 ± 175.9 | 179.8 ± 15.2 | 0 | 0.95 ± 0.02 | 54.8 ± 9.2 | 259.9 ± 43.0 | 0 | 10.2 ± 0.1 | 7.08 ± 0.07 |
| crossing | 5 | swarmx | ✓ | 1740.4 ± 33.7 | 141.3 ± 4.9 | 0 | 0.68 ± 0.01 | 91.2 ± 16.2 | 0.0 | 2 ± 1 | 10.5 ± 0.0 | 8.06 ± 0.06 |
| crossing | 8 | ghost | ✓ | 2225.6 ± 50.5 | 117.3 ± 4.1 | 65 ± 11 | 0.00 | 0.0 | 0.0 | 0 | 10.8 ± 0.1 | 10.47 ± 0.17 |
| crossing | 8 | stopwait | ✗ | 14386.2 ± 13560.2 | 861.7 ± 638.8 | 0 | 0.94 ± 0.02 | 524.7 ± 489.6 | 5633.9 ± 4908.6 | 0 | 10.1 | 7.98 ± 0.99 |
| crossing | 8 | swarmx | ✓ | 3236.6 ± 251.7 | 167.8 ± 8.0 | 0 | 0.68 | 108.3 ± 33.1 | 0.0 | 10 ± 8 | 10.7 ± 0.1 | 10.72 ± 0.28 |
| hot_aisles | 3 | ghost | ✓ | 3084.7 ± 83.0 | 303.2 ± 6.7 | 8 ± 2 | 0.01 | 0.0 | 0.0 | 0 | 10.4 | 6.71 ± 0.04 |
| hot_aisles | 3 | stopwait | ✓ | 3820.8 ± 197.3 | 362.3 ± 15.3 | 0 | 0.95 ± 0.03 | 19.9 ± 2.8 | 57.0 ± 15.3 | 0 | 10.2 | 7.11 ± 0.04 |
| hot_aisles | 3 | swarmx | ✓ | 3319.6 ± 170.3 | 320.6 ± 11.9 | 0 | 0.69 | 41.1 ± 10.2 | 0.0 | 0 | 10.4 | 6.88 ± 0.06 |
| hot_aisles | 5 | ghost | ✓ | 2036.2 ± 37.7 | 181.7 ± 3.2 | 19 ± 6 | 0.00 | 0.0 | 0.0 | 0 | 10.4 ± 0.1 | 7.19 ± 0.04 |
| hot_aisles | 5 | stopwait | ✓ | 2607.2 ± 85.7 | 234.7 ± 8.4 | 0 | 0.92 ± 0.01 | 32.0 ± 5.9 | 133.6 ± 25.1 | 0 | 10.1 | 7.06 ± 0.07 |
| hot_aisles | 5 | swarmx | ✓ | 2268.9 ± 83.7 | 209.2 ± 8.7 | 0 | 0.69 | 64.2 ± 11.5 | 0.0 | 0 | 10.4 ± 0.0 | 7.18 ± 0.06 |
| hot_aisles | 8 | ghost | ✓ | 3240.8 ± 55.5 | 188.9 ± 4.2 | 47 ± 4 | 0.00 | 0.0 | 0.0 | 0 | 10.6 ± 0.0 | 9.37 ± 0.12 |
| hot_aisles | 8 | stopwait | ✓ | 5015.6 ± 488.7 | 283.0 ± 19.2 | 0 | 0.91 | 91.3 ± 24.5 | 508.0 ± 97.2 | 0 | 10.1 | 8.09 ± 0.11 |
| hot_aisles | 8 | swarmx | ✓ | 4393.6 ± 400.2 | 259.3 ± 12.1 | 0 | 0.68 | 202.1 ± 48.0 | 0.0 | 0 | 10.6 ± 0.1 | 9.42 ± 0.40 |
| random | 3 | ghost | ✓ | 3052.9 ± 232.8 | 333.6 ± 14.7 | 9 ± 2 | 0.01 ± 0.01 | 0.0 | 0.0 | 0 | 10.4 | 6.47 ± 0.07 |
| random | 3 | stopwait | ✓ | 3739.0 ± 229.3 | 393.4 ± 15.8 | 0 | 0.93 ± 0.01 | 14.8 ± 1.2 | 50.3 ± 21.4 | 0 | 10.2 | 7.00 ± 0.08 |
| random | 3 | swarmx | ✓ | 3230.6 ± 258.7 | 347.4 ± 17.2 | 0 | 0.69 | 20.4 ± 11.9 | 0.0 | 0 | 10.4 | 6.61 ± 0.09 |
| random | 5 | ghost | ✓ | 2029.8 ± 169.7 | 204.3 ± 10.5 | 14 ± 1 | 0.01 ± 0.01 | 0.0 | 0.0 | 0 | 10.4 | 6.86 ± 0.10 |
| random | 5 | stopwait | ✓ | 2601.6 ± 175.8 | 261.8 ± 19.1 | 0 | 0.91 ± 0.01 | 23.5 ± 2.5 | 128.9 ± 33.1 | 0 | 10.1 | 6.90 ± 0.10 |
| random | 5 | swarmx | ✓ | 2222.1 ± 133.8 | 222.6 ± 9.3 | 0 | 0.69 | 45.4 ± 18.2 | 0.0 | 0 | 10.4 | 6.91 ± 0.10 |
| random | 8 | ghost | ✓ | 3139.5 ± 234.5 | 210.4 ± 7.6 | 46 ± 5 | 0.00 | 0.0 | 0.0 | 0 | 10.6 ± 0.1 | 8.83 ± 0.11 |
| random | 8 | stopwait | ✓ | 4445.6 ± 341.3 | 282.0 ± 17.3 | 0 | 0.84 ± 0.10 | 64.2 ± 16.4 | 407.9 ± 68.5 | 0 | 10.1 | 7.87 ± 0.11 |
| random | 8 | swarmx | ✓ | 4028.1 ± 256.6 | 257.4 ± 19.3 | 0 | 0.68 ± 0.01 | 170.0 ± 61.1 | 0.0 | 0 ± 0 | 10.5 ± 0.1 | 9.06 ± 0.33 |
| robot_failure | 3 | ghost | ✓ | 3648.2 ± 229.8 | 373.3 ± 14.3 | 7 ± 1 | 0.02 ± 0.01 | 0.0 | 0.0 | 0 | 9.3 ± 0.0 | 5.79 ± 0.09 |
| robot_failure | 3 | stopwait | ✓ | 4594.3 ± 388.8 | 442.9 ± 24.7 | 0 | 0.92 ± 0.02 | 11.6 ± 1.6 | 55.4 ± 27.9 | 0 | 9.3 ± 0.1 | 6.40 ± 0.09 |
| robot_failure | 3 | swarmx | ✓ | 3890.4 ± 323.9 | 394.0 ± 18.3 | 0 | 0.69 | 21.6 ± 8.4 | 0.0 | 0 | 9.3 ± 0.1 | 5.92 ± 0.10 |
| robot_failure | 5 | ghost | ✓ | 2338.3 ± 200.7 | 242.5 ± 15.4 | 15 ± 4 | 0.01 ± 0.01 | 0.0 | 0.0 | 0 | 9.4 ± 0.1 | 6.04 ± 0.06 |
| robot_failure | 5 | stopwait | ✓ | 2913.6 ± 176.5 | 290.3 ± 14.2 | 0 | 0.83 ± 0.10 | 18.6 ± 4.3 | 115.9 ± 40.2 | 0 | 9.3 ± 0.0 | 6.30 ± 0.08 |
| robot_failure | 5 | swarmx | ✓ | 2644.0 ± 182.9 | 261.0 ± 15.6 | 0 | 0.69 | 42.0 ± 11.4 | 0.0 | 0 | 9.4 ± 0.1 | 6.22 ± 0.11 |
| robot_failure | 8 | ghost | ✓ | 3429.1 ± 274.2 | 231.5 ± 10.9 | 44 ± 3 | 0.00 | 0.0 | 0.0 | 0 | 9.9 ± 0.1 | 8.23 ± 0.14 |
| robot_failure | 8 | stopwait | ✓ | 5115.2 ± 747.4 | 313.0 ± 27.1 | 0 ± 0 | 0.66 ± 0.30 | 74.0 ± 43.7 | 447.4 ± 152.0 | 0 | 9.6 ± 0.1 | 7.53 ± 0.12 |
| robot_failure | 8 | swarmx | ✓ | 4743.2 ± 478.7 | 325.1 ± 91.0 | 0 | 0.69 | 188.7 ± 108.5 | 0.0 | 0 ± 1 | 10.0 ± 0.1 | 8.29 ± 0.57 |

Methods: `swarmx` = CBBA + intent-aware routing + zone locks + ORCA/RSS; `stopwait` = traditional block reservation (stop-and-wait) with one-way lanes and greedy task claiming; `ghost` = robots ignore each other (unachievable lower bound); others are ablations.
