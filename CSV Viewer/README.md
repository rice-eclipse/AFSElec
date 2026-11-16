# AFS Flight Data Viewer

Interactive viewer for the flight computer's SD-card logs (`flights/flight_NNN.csv` sensor logs and
`flights/nav_NNN.csv` navigation logs). It also opens any other CSV that has a time column.

```
pip install -r requirements.txt
python flight_viewer.py                       # then File > Open, or drop CSVs on the window
python flight_viewer.py nav_003.csv flight_003.csv
python flight_viewer.py --layout my_layout.json
```

On Windows you can also double-click `run_viewer.bat` or drag CSVs onto it.

## What it does

| Feature | How |
|---|---|
| Graphs over time | Opening a file builds standard graphs (altitude, velocity, acceleration, rates, attitude). Drag columns from the browser onto a graph, or onto empty space for a new graph. Double-click a column to add it to the active graph (blue outline). |
| Scrolling & graph sizes | Graphs sit in a scrollable column. Use the mouse wheel or PgUp/PgDn to scroll through them. Drag a graph's bottom-edge grip (or use its −/+ buttons) to make it taller or shorter. Double-click the grip to reset the height. `Ctrl+0` fits every graph into the window. Saved layouts keep the heights. |
| Zoom | Ctrl+wheel zooms time, Ctrl+Shift+wheel zooms Y. View ▸ "Mouse wheel zooms graphs" swaps wheel and Ctrl+wheel if you prefer zoom-on-wheel. **Zoom select** (`Z`): drag across a span. `+`/`-`, `F` fits all, double-click fits all, `Backspace` goes back to the previous view. |
| Pan | Drag (Pan mode), middle-drag in any mode, `Ctrl+←/→`, or drag the window in the overview strip. |
| Crop | **Crop select** (`C`): drag across the span to keep. You can also crop to the current view (`Ctrl+K`) or to the A–B cursors (`Ctrl+Shift+K`). `Ctrl+Z` undoes a crop and `Ctrl+R` resets it. The crop applies to the graphs, statistics and CSV export. |
| Measure | `M` turns on the A/B cursors (drag them, or press `A`/`B` to place one at the mouse). The readout shows values at the cursor, at A and B, the B−A difference, Δt, and min/max/mean/std over A–B (or over the visible range). |
| Flight states | Powered ascent, cruising and descent are shaded from `flight_state`, with labeled transitions (`S` toggles). Data ▸ Jump to event centers the view on launch, burnout, apogee, landing or peak acceleration. |
| Derived signals | `D` or right-click a column: `mag(imu_ax_g, imu_ay_g, imu_az_g)`, `ddt(baro_alt_m)`, `smooth(x, 25)`, `lowpass(x, 5)`, `integ(x)`, `-vd_mps`, plus any numpy math. |
| Multiple files | Open the sensor and nav logs together. Use the per-file **Offset** to line them up in time. |
| XY plot | Tools ▸ XY plot, e.g. `east_m` vs `north_m` ground track with equal aspect and color by time. |
| Summary | Tools ▸ Flight summary: duration, sample rate, gaps, peaks with times, state timeline, dropped rows. Tools ▸ Column statistics gives stats for every column. |
| Export | `Ctrl+P` saves a PNG of all graphs. `Ctrl+E` saves a CSV of the crop, the visible range or A–B, with all columns or only the plotted ones. Right-click a single graph for pyqtgraph's own export (SVG, etc.). |
| Layouts | `Ctrl+S` / `Ctrl+L` save and restore files, graphs, colors, derived signals and the crop as JSON. This lets you reuse one layout for every flight. |
| Theme | `T` switches between dark and light. |

Press `F1` in the app for the full list of mouse and keyboard controls.

## Notes

- The time column is detected automatically (`t_us`, `time_ms`, `time_s`, ...). The unit comes from the
  suffix and the time starts at 0 by default. You can change any of these in the **Time axis** panel.
- Rows that don't parse are skipped, so a log cut off mid-row by a power loss still opens.
- Large logs stay responsive because drawing is downsampled per pixel (peak mode), so spikes are not lost.
