# Local patches to sokol

Re-apply these when `sokol_app.h` is refreshed; each is marked `Local patch` in the source.

- **High-DPI window size** (`_sapp_win32_create_window`): with `high_dpi` set, `desc.width` and `desc.height` are
  framebuffer pixels instead of 100%-DPI logical pixels.
- **A window that does not take focus** (`sapp_win32_desc.no_activate`, `_sapp_win32_create_window`): shown with
  `SW_SHOWNOACTIVATE`, for the sandbox's `--background` (agent-driven co-op tests open windows while someone works).
