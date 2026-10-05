# GPU lab results: mac-m5

```
machine mac-m5
platform macOS-27.0.1-arm64-arm-64bit
processor arm64
primary compiler clang
twin compilers -
quick False
```

Each twin's hash against reference.json (E11's Windows hashes); each GPU's mismatching words against the twin of
its own run. Positive controls must differ. ms/step at the largest N.

| log | twin | twin vs reference | threads agree | GPUs (mismatching words; ms/step) |
|---|---|---|---|---|
| control_F_fast | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 1000: ok, 10000: ok, 100000: ok | yes | apple: 5280/54007/532813 (4.55 ms) |
| control_F_fma | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 10000: differs (control ok) | yes | - |
| control_F_mvkfast | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 1000: ok, 10000: ok, 100000: ok | yes | apple: 0/0/0 (6.70 ms) |
| control_rows_V1_fast | clang 21.0.0 (clang-2100.3.34.2), aarch6 | ok | | apple: 641680 rows (3.56 ns/row) |
| control_rows_V1_fma | clang 21.0.0 (clang-2100.3.34.2), aarch6 | differs (control ok) | | - |
| control_rows_V1_mvkfast | clang 21.0.0 (clang-2100.3.34.2), aarch6 | ok | | apple: 28 rows (6.46 ns/row) |
| rows_V1 | clang 21.0.0 (clang-2100.3.34.2), aarch6 | ok | | apple: 28 rows (6.42 ns/row) |
| rows_V1_pszinp | clang 21.0.0 (clang-2100.3.34.2), aarch6 | ok | | - |
| rows_V1_rtesz | clang 21.0.0 (clang-2100.3.34.2), aarch6 | ok | | apple: 28 rows (7.05 ns/row) |
| rows_V2 | clang 21.0.0 (clang-2100.3.34.2), aarch6 | ok | | apple: 0 rows (8.87 ns/row) |
| rows_V3 | clang 21.0.0 (clang-2100.3.34.2), aarch6 | ok | | apple: 0 rows (4.98 ns/row) |
| rows_V4 | clang 21.0.0 (clang-2100.3.34.2), aarch6 | ok | | apple: 0 rows (5.23 ns/row) |
| rows_V4_dxc | clang 21.0.0 (clang-2100.3.34.2), aarch6 | ok | | apple: 0 rows (5.07 ns/row) |
| rows_V4_glslang | clang 21.0.0 (clang-2100.3.34.2), aarch6 | ok | | apple: 0 rows (4.92 ns/row) |
| rows_V4b | clang 21.0.0 (clang-2100.3.34.2), aarch6 | ok | | apple: 0 rows (4.03 ns/row) |
| solve_F | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 1000: ok, 10000: ok, 100000: ok | yes | apple: 0/0/0 (7.01 ms) |
| solve_F_any | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 1000: ok, 10000: ok, 100000: ok | yes | apple: 0/0/0 (7.13 ms) |
| solve_F_ieee | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 1000: ok, 10000: ok, 100000: ok | yes | apple: 0/0/0 (7.49 ms) |
| solve_Fdisc | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 1000: ok, 10000: ok, 100000: ok | yes | apple: 825/8056/83452 (7.29 ms) |
| solve_Fdisc_ieee | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 1000: ok, 10000: ok, 100000: ok | yes | apple: 825/8056/83452 (7.31 ms) |
| solve_Fnocap | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 1000: differs (NaN bits, expected off x64), 10000: differs (NaN bits, expected off x64) | yes | apple: 0/0 (1.89 ms) |
| solve_I32 | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 1000: ok, 10000: ok, 100000: ok | yes | apple: 0/0/0 (5.34 ms) |
| solve_I64 | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 1000: ok, 10000: ok, 100000: ok | yes | apple: 0/0/0 (14.93 ms) |
| solve_V4 | clang 21.0.0 (clang-2100.3.34.2), aarch6 | 1000: ok, 10000: ok, 100000: ok | yes | apple: 0/0/0 (6.26 ms) |
| twin_F.rosetta | clang 21.0.0 (clang-2100.3.34.2), x64, c |  | yes | - |
| twin_Fdisc.rosetta | clang 21.0.0 (clang-2100.3.34.2), x64, c |  | yes | - |
| twin_I32.rosetta | clang 21.0.0 (clang-2100.3.34.2), x64, c |  | yes | - |
| twin_I64.rosetta | clang 21.0.0 (clang-2100.3.34.2), x64, c |  | yes | - |
| twin_V4.rosetta | clang 21.0.0 (clang-2100.3.34.2), x64, c |  | yes | - |
| twin_rows_V1.rosetta | clang 21.0.0 (clang-2100.3.34.2), x64, c | ok | | - |
| twin_rows_V2.rosetta | clang 21.0.0 (clang-2100.3.34.2), x64, c | ok | | - |
| twin_rows_V3.rosetta | clang 21.0.0 (clang-2100.3.34.2), x64, c | ok | | - |
| twin_rows_V4.rosetta | clang 21.0.0 (clang-2100.3.34.2), x64, c | ok | | - |
| twin_rows_V4b.rosetta | clang 21.0.0 (clang-2100.3.34.2), x64, c | ok | | - |

**Failures:** none
