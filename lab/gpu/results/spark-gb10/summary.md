# GPU lab results: spark-gb10

```
machine spark-gb10
platform Linux-6.11.0-1014-nvidia-aarch64-with-glibc2.39
processor aarch64
primary compiler gcc
twin compilers clang
quick False
machine spark-gb10
platform Linux-6.11.0-1014-nvidia-aarch64-with-glibc2.39
processor aarch64
primary compiler gcc
twin compilers -
quick False
```

Each twin's hash against reference.json (E11's Windows hashes); each GPU's mismatching words against the twin of
its own run. Positive controls must differ. ms/step at the largest N.

| log | twin | twin vs reference | threads agree | GPUs (mismatching words; ms/step) |
|---|---|---|---|---|
| control_F_fast | gcc 13.3.0, aarch64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 5295/54038/532751 (4.73 ms); llvmpipe: 4954/51099/501623 (51.77 ms) |
| control_F_fma | gcc 13.3.0, aarch64, contraction ON (pos | 10000: differs (control ok) | yes | - |
| control_rows_V1_fast | gcc 13.3.0, aarch64, contraction off | ok | | nvidia: 649040 rows (2.85 ns/row); llvmpipe: 188120 rows (5.61 ns/row) |
| control_rows_V1_fma | gcc 13.3.0, aarch64, contraction ON (pos | differs (control ok) | | - |
| rows_V1 | gcc 13.3.0, aarch64, contraction off | ok | | nvidia: 0 rows (2.84 ns/row); llvmpipe: 0 rows (4.87 ns/row) |
| rows_V1_pszinp | gcc 13.3.0, aarch64, contraction off | ok | | - |
| rows_V1_rtesz | gcc 13.3.0, aarch64, contraction off | ok | | nvidia: 0 rows (2.88 ns/row); llvmpipe: 0 rows (5.00 ns/row) |
| rows_V2 | gcc 13.3.0, aarch64, contraction off | ok | | nvidia: 0 rows (4.79 ns/row); llvmpipe: 0 rows (32.88 ns/row) |
| rows_V3 | gcc 13.3.0, aarch64, contraction off | ok | | nvidia: 0 rows (2.77 ns/row); llvmpipe: 0 rows (64.89 ns/row) |
| rows_V4 | gcc 13.3.0, aarch64, contraction off | ok | | nvidia: 0 rows (2.76 ns/row); llvmpipe: 0 rows (89.50 ns/row) |
| rows_V4_dxc | gcc 13.3.0, aarch64, contraction off | ok | | nvidia: 0 rows (2.76 ns/row); llvmpipe: 0 rows (88.61 ns/row) |
| rows_V4_glslang | gcc 13.3.0, aarch64, contraction off | ok | | nvidia: 0 rows (2.76 ns/row); llvmpipe: 0 rows (82.97 ns/row) |
| rows_V4b | gcc 13.3.0, aarch64, contraction off | ok | | nvidia: 0 rows (2.75 ns/row); llvmpipe: 0 rows (83.41 ns/row) |
| solve_F | gcc 13.3.0, aarch64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 0/0/0 (4.81 ms); llvmpipe: 0/0/0 (52.22 ms) |
| solve_F_any | gcc 13.3.0, aarch64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 0/0/0 (4.80 ms); llvmpipe: 0/0/0 (52.38 ms) |
| solve_F_ieee | gcc 13.3.0, aarch64, contraction off |  | yes | ; **exit -11** after GPU 1 (NVIDIA Tegra NVIDIA GB10) |
| solve_Fdisc | gcc 13.3.0, aarch64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 4222/44465/432644 (4.74 ms); llvmpipe: 0/0/0 (49.74 ms) |
| solve_Fdisc_ieee | gcc 13.3.0, aarch64, contraction off |  | yes | ; **exit -11** after GPU 1 (NVIDIA Tegra NVIDIA GB10) |
| solve_Fnocap | gcc 13.3.0, aarch64, contraction off | 1000: differs (NaN bits, expected off x64), 10000: differs (NaN bits, expected off x64) | yes | nvidia: 12/228 (1.07 ms); llvmpipe: 0/0 (7.10 ms) |
| solve_I32 | gcc 13.3.0, aarch64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 0/0/0 (5.15 ms); llvmpipe: 0/0/0 (55.61 ms) |
| solve_I64 | gcc 13.3.0, aarch64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 0/0/0 (8.65 ms); llvmpipe: 0/0/0 (88.10 ms) |
| solve_V4 | gcc 13.3.0, aarch64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 0/0/0 (6.55 ms); llvmpipe: 0/0/0 (63.00 ms) |
| twin_F.clang | clang 18.1.3 (1ubuntu1), aarch64, contra | 1000: ok, 10000: ok, 100000: ok | yes | - |
| twin_Fdisc.clang | clang 18.1.3 (1ubuntu1), aarch64, contra | 1000: ok, 10000: ok, 100000: ok | yes | - |
| twin_I32.clang | clang 18.1.3 (1ubuntu1), aarch64, contra | 1000: ok, 10000: ok, 100000: ok | yes | - |
| twin_I64.clang | clang 18.1.3 (1ubuntu1), aarch64, contra | 1000: ok, 10000: ok, 100000: ok | yes | - |
| twin_V4.clang | clang 18.1.3 (1ubuntu1), aarch64, contra | 1000: ok, 10000: ok, 100000: ok | yes | - |
| twin_rows_V1.clang | clang 18.1.3 (1ubuntu1), aarch64, contra | ok | | - |
| twin_rows_V2.clang | clang 18.1.3 (1ubuntu1), aarch64, contra | ok | | - |
| twin_rows_V3.clang | clang 18.1.3 (1ubuntu1), aarch64, contra | ok | | - |
| twin_rows_V4.clang | clang 18.1.3 (1ubuntu1), aarch64, contra | ok | | - |
| twin_rows_V4b.clang | clang 18.1.3 (1ubuntu1), aarch64, contra | ok | | - |

**Failures:** none
