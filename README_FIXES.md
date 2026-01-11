# ASD Assignment - Implementation & Fixes

**Status:** ✅ Fully Functional
**Final Accuracy:** < 0.2m error (Stops at ~9.91, 9.91 for Goal 10,10)

---

## 1. Core Logic Implementations

### Planner Node (A*)
*   **Standard A*:** Implemented correct `f = g + h` logic with a `closed_set` to prevent infinite loops.
*   **8-Connectivity:** Enabled diagonal movement with correct cost (`1.414`) for smoother, optimal paths.
*   **Weighted Cost:** Added logic to penalize high-cost cells (inflation zones), causing the robot to curve around obstacles rather than grazing them.
*   **Robustness:** Relaxed start/goal checks to ensure a path is always generated even if the robot starts or ends slightly within a safety buffer.

### Control Node (Pure Pursuit)
*   **Stability:** Normalized steering angles to `[-pi, pi]` to prevent erratic spinning.
*   **Stop-on-Fail:** Added safety logic to immediately stop the robot if the path becomes invalid or empty.
*   **Precision:** Tightened goal tolerance to `0.2m` for precise stops.

### Costmap & Map Memory
*   **Safety:** Increased inflation radius to `1.0m` to provide a wide safety margin around walls.
*   **Efficiency:** Implemented distance-based map updates (only updates when robot moves > 1.5m).

---

## 2. Visualization & Frame Fixes

### The "Missing Transform" Issue
The provided simulation environment had a disconnection between the visual frame (`sim_world`) and the navigation data frame (`odom`). This caused:
1.  "Missing Transform" errors in Foxglove.
2.  Visual offset (Path appearing 6m away from the robot).

### The Fix
Added a **Static Transform Publisher** to `robot.launch.py` to bridge the gap:
*   Linked `sim_world` → `odom` with a **-6.0m offset**.
*   This correctly aligns the navigation data (starting at 0,0) with the visual robot (spawning at -6,0).

**Result:** The Map, Path, and Robot now align correctly in Foxglove when viewing the `sim_world` frame.

---

## 3. How to Run
1.  **Build:** `./watod build`
2.  **Run:** `./watod up`
3.  **Visualize:** Open Foxglove (Fixed Frame: `sim_world`).
4.  **Test:** Publish a point to `/goal_point`.
