# Perception — Image Filtering & LiDAR Line Extraction

This section covers two foundational perception techniques for mobile robots:

1. **Image filtering by spatial correlation** — implementing 2D convolution-style image filters from scratch and applying them to a grayscale photo. This is the building block beneath edge detection, blurring, sharpening, and most classical computer-vision pipelines.
2. **LiDAR line extraction by Split-and-Merge** — fitting straight line segments to 2D range-scan data so a mobile robot can recover the geometric structure of its surroundings (walls, table edges) from raw range measurements.

The two problems come from different sensing modalities (camera, LiDAR) but share the same theme: turning raw sensor pixels or points into something the rest of the autonomy stack can reason about.

## Files

```text
perception/
├── README.md
├── image_filtering_and_line_extraction.ipynb
└── data/
    ├── parrot.png                       ← grayscale test image (150 × 200)
    ├── rangeData_4_9_360.xlsx           ← scan from (4, 9), 360 beams
    ├── rangeData_5_5_180.xlsx           ← scan from (5, 5), 180 beams
    └── rangeData_7_2_90.xlsx            ← scan from (7, 2), 90 beams
```

The notebook's plot outputs are committed and render directly on GitHub.

---

## Part A — Image Filtering

### Background — Correlation as a dot product

For a grayscale image `I ∈ ℝ^(m×n)` and a filter `F ∈ ℝ^(k×ℓ)`, the **correlation** `G = I ⊗ F` is defined pixelwise as:

```text
G(i, j) = Σ_u Σ_v F(u, v) · Ī(i + u, j + v)
```

where `Ī` is `I` zero-padded around the edges so the filter window doesn't fall off the image at the boundary.

The naive way to implement this is a quadruple loop (image rows × image cols × filter rows × filter cols), which for a `200 × 150` image and a `3 × 3` filter is 270,000 multiply-adds inside Python's interpreter — slow.

A faster equivalent: at each pixel `(i, j)`, extract the `k × ℓ` neighborhood patch and flatten both the patch and the filter into 1D vectors. Correlation reduces to a single dot product:

```text
G(i, j) = f · t_ij
```

where `f = vec(F)` and `t_ij = vec(Ī[i:i+k, j:j+ℓ])`. This pushes the inner multiply-add into NumPy's C implementation and is dramatically faster than the nested-loop form. The notebook uses this dot-product form.

### Problem 1 — Manual filter calculations

The assignment includes a Problem 1 with six hand-worked `3 × 3` filter applications on a small `3 × 3` test image — identity, shift, horizontal edge `[[1,1,1],[0,0,0],[-1,-1,-1]]`, its vertical counterpart, a Gaussian-like `(1/16) · [[1,2,1],[2,4,2],[1,2,1]]`, and a box-blur `(1/9) · [[1,1,1],[1,1,1],[1,1,1]]`. The manual calculations were done on paper and are not included in this repository (they're rote correlation arithmetic). The conceptual takeaway:

- The `[[1,1,1],[0,0,0],[-1,-1,-1]]` filter computes a vertical gradient — top minus bottom — and so highlights **horizontal edges**.
- Its column-wise counterpart `[[-1,0,1],[-1,0,1],[-1,0,1]]` does the symmetric operation (left minus right) and highlights **vertical edges**.
- The Gaussian-like filter is a weighted average that blurs more gently than the box blur, preserving slightly more structure.
- The box blur `(1/9) · 𝟙_(3×3)` is the simplest possible low-pass filter — every pixel becomes the mean of its 3×3 neighborhood.

### Problem 2b) — Correlation operator on the parrot image

The notebook implements the dot-product correlation operator described above and applies it to `data/parrot.png` using the **box-blur filter** (Problem 1f):

```python
F = (1/9) · np.ones((3, 3))
```

The committed plot output shows the parrot image after one pass of the 3×3 box blur — visibly smoothed but still recognizable. Fine feather detail is reduced, the high-frequency texture in the background foliage is softened, and the overall shape is preserved. This is exactly the expected behavior of a low-pass filter.

The same code path would produce the other five filter outputs (sharpening, the two edge-detection results, the Gaussian-like blur, the identity) by swapping the `F` matrix at the top of the cell and re-running cells [3]–[7]. The notebook only retains the box-blur run.

---

## Part B — LiDAR Line Extraction

### Background — Line fitting to range data

A 2D LiDAR sensor casts equally-spaced laser beams from the robot's position and returns the range `ρ_i` at each beam angle `θ_i`. The result is a list of polar coordinates `{(ρ_i, θ_i)}` describing the contour of whatever the beams hit.

A useful processing step is to fit straight-line segments to runs of those points that lie along the same physical edge (a wall, the side of a table). In polar form, a line through space is described by `(r, α)` where `r` is the perpendicular distance from the sensor origin to the line and `α` is the orientation of that perpendicular relative to the x-axis. The defining equation is:

```text
ρ_i · cos(θ_i − α) = r
```

The point-to-line residuals are `d_i = ρ_i · cos(θ_i − α) − r`, and the polar least-squares fit minimizes `S = Σ_i d_i²` over all `n` points in a candidate segment. Closed-form solutions for `(r, α)` exist (the assignment gives the formula explicitly).

### Background — The Split-and-Merge algorithm

Split-and-Merge is a recursive top-down line extraction technique. Starting from the full ordered list of scan points, it:

1. **Fit** a single line to all points in the current segment.
2. **Find** the point with the largest perpendicular distance to that line.
3. **If** that distance exceeds a threshold and the point isn't at either endpoint, **split** the segment at that point and recursively repeat on each half.
4. **Else** accept the current segment as a single line.
5. After all splits, **merge** adjacent segments whose joint fit error is below threshold (this step trims over-splitting).

Practical implementations expose four tuning parameters:

| Parameter | Purpose |
|---|---|
| `LINE_POINT_DIST_THRESHOLD` | The max perpendicular distance allowed before splitting |
| `MIN_POINTS_PER_SEGMENT` | The minimum point count to accept a segment |
| `MIN_SEG_LENGTH` | The minimum geometric length (in meters) of a kept segment |
| `MAX_P2P_DIST` | The maximum allowed gap between adjacent points in a segment — prevents segments from "jumping" between unrelated objects |

### Problem 3a) — Implementation

The notebook implements the recursive split portion of the algorithm. For each dataset:

1. Read the xlsx into a NumPy array. Row 0 stores the robot's world position `(x_r, y_r)`; rows 1..N store `(θ_i, ρ_i)`.
2. Convert the polar scan to Cartesian: `x = ρ·cos(θ)`, `y = ρ·sin(θ)`. (Note: this gives points in the sensor's local frame, not translated to world coordinates — see [Known Limitations](#known-limitations).)
3. Fit a line to all points with `np.polyfit(x, y, 1)`.
4. Compute the perpendicular distance from each point to that line, find the max-distance point.
5. If the max distance exceeds `LINE_POINT_DIST_THRESHOLD` and isn't at the endpoints, split there and recurse on both halves.
6. Else stop. After recursion completes, filter the candidate segments by `MIN_POINTS_PER_SEGMENT` and `MIN_SEG_LENGTH`.

Parameter choices used in the committed run:

```python
LINE_POINT_DIST_THRESHOLD = 0.2  # meters
MIN_POINTS_PER_SEGMENT    = 5
MIN_SEG_LENGTH            = 0.5  # meters
MAX_P2P_DIST              = 0.3  # meters  (declared but unused — see Known Limitations)
```

### Numerical results

The committed plot shows the extracted segments for all three datasets side by side, with each segment drawn in a different color:

- **Dataset 1 (`4_9_360`)** — scan from `(4, 9)` with 360 beams. Two segments retained after thresholding. The blue segment captures the wall/ceiling region near the top of the room cleanly; the orange segment traces the right-side objects. A long blue diagonal cuts across the plot — a classic "jumping" segment that connects two unrelated regions because the `MAX_P2P_DIST` gate isn't being applied during the split.
- **Dataset 2 (`5_5_180`)** — scan from `(5, 5)` with 180 beams. Four segments retained. The red segment covers most of the room outline; the green, orange, and blue segments capture smaller features. Several short "spoke" lines radiate from the center — also a symptom of missing point-to-point gap enforcement.
- **Dataset 3 (`7_2_90`)** — scan from `(7, 2)` with 90 beams. Three segments retained. Similar pattern: a dominant red segment plus two smaller ones.

The assignment notes explicitly that "there is not one correct answer" and that the goal is to "smoothly fit the actual contours of the objects in the room and minimize the number of false lines." The committed implementation captures the broad geometry but leaves visible false lines — see Known Limitations for the specific gaps and how they'd be fixed.

---

## How to Run

```bash
cd perception
pip install -r ../requirements.txt
jupyter notebook image_filtering_and_line_extraction.ipynb
```

Run the cells top-to-bottom. The committed notebook already has plot outputs embedded, so the section renders fully on GitHub without execution.

**Dependencies:** This section adds `opencv-python` (for `cv2.imread` grayscale loading), `pandas` (for `pd.read_excel`), and `openpyxl` (the Excel reader engine `pandas` uses for `.xlsx`). All three are listed in the repository's top-level `requirements.txt`.

**File paths:** The notebook was originally developed on Google Colab and references `/content/parrot.png` / `/content/rangeData_*.xlsx`. To run locally from the `perception/` folder, change the paths in cells [2] and [11] to `data/parrot.png` and `data/rangeData_*.xlsx` respectively.

## Known Limitations

- **Problem 1 calculations not in repo.** The six manual filter applications on the 3×3 test image (Problem 1a–f) were done on paper. The notebook implements only Problem 2b — the correlation operator applied to the parrot image — and only for the box-blur filter (Problem 1f). The implementation is correct and runs for any 3×3 filter; swap the `F` matrix in cell [3] and re-execute to see the other filter outputs.

- **Problem 3a — only the "Split" half of Split-and-Merge is implemented.** The recursive split is in place but the **merge** step that recombines adjacent over-split segments is not. With the current parameters this isn't catastrophic — over-splitting is bounded by `MIN_POINTS_PER_SEGMENT` — but a complete implementation would re-fit adjacent segment pairs after the recursion and keep the merge if the joint residual is below threshold.

- **`MAX_P2P_DIST` is declared but unused.** The parameter is passed into `split_and_merge` but never referenced inside the recursive splitter. This parameter is what prevents segments from "jumping" across gaps between unrelated objects (e.g., from the wall behind one object to the next object over). Its absence is visible in the Dataset 1 plot as the long diagonal blue line and in Datasets 2 and 3 as the spoke-like rays radiating from interior points. The fix: add a pre-pass that splits any segment wherever the gap between consecutive points exceeds `MAX_P2P_DIST`, then run the recursive split on each gap-bounded sub-segment.

- **Line fit is Cartesian, not polar.** The assignment specifies a polar least-squares solution `(r, α)` for the line parameters (eq. 3 in the problem statement). The implementation uses `np.polyfit(x, y, 1)` on the converted Cartesian coordinates, which solves a different objective and is numerically unstable as a segment approaches vertical (slope `m → ∞`). For near-vertical wall segments this can throw warnings or give nonsensical fits. The fix: implement the closed-form polar LSQ from the assignment's equation (3) using the `(ρ_i, θ_i)` data directly, without converting to Cartesian first.

- **Sensor-local frame, not world frame.** Each xlsx file stores the robot's world position in row 0 (`(4, 9)`, `(5, 5)`, `(7, 2)` for the three files, matching their filenames). The current code reads but discards row 0 — the line plot is in the sensor's local coordinates with the robot at the origin. To map the extracted lines into the world frame of the room, translate the Cartesian point cloud by `(x_r, y_r)` before line fitting. This wouldn't change which lines are found, but it would let all three scans be overlaid on a single shared world map — which is the natural next step toward building an occupancy grid.

- **Recursion-depth cap is a workaround.** The recursive splitter caps depth at 100 to guard against pathological cases. With the current data and parameters this cap is never hit, but its presence suggests a worry about edge cases (e.g. failed split that recurses on the same segment) that a polar-LSQ fit would eliminate.

## License

MIT — see [LICENSE](../LICENSE) in the repository root.
