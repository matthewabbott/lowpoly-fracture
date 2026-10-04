# GPU lab results: win-laptop

```
machine win-laptop
platform Windows-11-10.0.22621-SP0
processor AMD64
primary compiler msvc
twin compilers clang-cl
quick False
```

Each twin's hash against reference.json (E11's Windows hashes); each GPU's mismatching words against the twin of
its own run. Positive controls must differ. ms/step at the largest N.

| log | twin | twin vs reference | threads agree | GPUs (mismatching words; ms/step) |
|---|---|---|---|---|
| control_F_fast | MSVC 194234435, x64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 5310/54048/534154 (5.57 ms); intel: 5344/54800/539943 (31.66 ms) |
| control_F_fma | MSVC 194234435, x64, contraction ON (pos | 10000: differs (control ok) | yes | - |
| control_rows_V1_fast | MSVC 194234435, x64, contraction off | ok | | nvidia: 664175 rows (1.52 ns/row); intel: 770171 rows (41.03 ns/row) |
| control_rows_V1_fma | MSVC 194234435, x64, contraction ON (pos | differs (control ok) | | - |
| rows_V1 | MSVC 194234435, x64, contraction off | ok | | nvidia: 0 rows (1.53 ns/row); intel: 0 rows (39.34 ns/row) |
| rows_V1_pszinp | MSVC 194234435, x64, contraction off | ok | | intel: 0 rows (38.79 ns/row) |
| rows_V1_rtesz | MSVC 194234435, x64, contraction off | ok | | nvidia: 0 rows (1.54 ns/row); intel: 0 rows (37.65 ns/row) |
| rows_V2 | MSVC 194234435, x64, contraction off | ok | | nvidia: 0 rows (3.83 ns/row); intel: 0 rows (192.00 ns/row) |
| rows_V3 | MSVC 194234435, x64, contraction off | ok | | nvidia: 0 rows (2.63 ns/row); intel: 0 rows (51.01 ns/row) |
| rows_V4 | MSVC 194234435, x64, contraction off | ok | | nvidia: 0 rows (3.02 ns/row); intel: 0 rows (53.59 ns/row) |
| rows_V4_dxc | MSVC 194234435, x64, contraction off | ok | | nvidia: 0 rows (2.52 ns/row); intel: 0 rows (55.09 ns/row) |
| rows_V4_glslang | MSVC 194234435, x64, contraction off | ok | | nvidia: 0 rows (2.52 ns/row); intel: 0 rows (54.49 ns/row) |
| rows_V4b | MSVC 194234435, x64, contraction off | ok | | nvidia: 0 rows (2.28 ns/row); intel: 0 rows (74.49 ns/row) |
| solve_F | MSVC 194234435, x64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 0/0/0 (5.65 ms); intel: 0/0/1 (34.23 ms) |
| solve_F_any | MSVC 194234435, x64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 0/0/0 (5.67 ms); intel: 0/0/1 (34.23 ms) |
| solve_F_ieee | MSVC 194234435, x64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 0/0/0 (5.67 ms); intel: 0/0/1 (34.36 ms) |
| solve_Fdisc | MSVC 194234435, x64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 4222/44465/432644 (5.67 ms); intel: 4114/43975/426541 (34.08 ms) |
| solve_Fdisc_ieee | MSVC 194234435, x64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 3815/41690/402417 (5.36 ms); intel: 4114/43975/426541 (31.21 ms) |
| solve_Fnocap | MSVC 194234435, x64, contraction off | 1000: ok, 10000: ok | yes | nvidia: 886/23157 (0.89 ms); intel: 874/22945 (2.87 ms) |
| solve_I32 | MSVC 194234435, x64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 0/0/0 (6.38 ms); intel: 0/0/0 (41.05 ms) |
| solve_I64 | MSVC 194234435, x64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 0/0/0 (9.73 ms); intel: 5911/89650/899797 (316.95 ms) |
| solve_V4 | MSVC 194234435, x64, contraction off | 1000: ok, 10000: ok, 100000: ok | yes | nvidia: 0/0/0 (7.04 ms); intel: 0/0/0 (42.66 ms) |
| twin_F.clang-cl | clang 23.1.2 (https://github.com/llvm/ll | 1000: ok, 10000: ok, 100000: ok | yes | - |
| twin_Fdisc.clang-cl | clang 23.1.2 (https://github.com/llvm/ll | 1000: ok, 10000: ok, 100000: ok | yes | - |
| twin_I32.clang-cl | clang 23.1.2 (https://github.com/llvm/ll | 1000: ok, 10000: ok, 100000: ok | yes | - |
| twin_I64.clang-cl | clang 23.1.2 (https://github.com/llvm/ll | 1000: ok, 10000: ok, 100000: ok | yes | - |
| twin_V4.clang-cl | clang 23.1.2 (https://github.com/llvm/ll | 1000: ok, 10000: ok, 100000: ok | yes | - |
| twin_rows_V1.clang-cl | clang 23.1.2 (https://github.com/llvm/ll | ok | | - |
| twin_rows_V2.clang-cl | clang 23.1.2 (https://github.com/llvm/ll | ok | | - |
| twin_rows_V3.clang-cl | clang 23.1.2 (https://github.com/llvm/ll | ok | | - |
| twin_rows_V4.clang-cl | clang 23.1.2 (https://github.com/llvm/ll | ok | | - |
| twin_rows_V4b.clang-cl | clang 23.1.2 (https://github.com/llvm/ll | ok | | - |

**Failures:** none
