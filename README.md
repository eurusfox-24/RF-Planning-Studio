# RF Planning Studio: Wireless Link Evaluation

## Objective
This project evaluates a wireless link at **868 MHz** by modifying key design parameters in a MATLAB environment. The goal is to observe how antenna height, gain, environmental factors, and obstacle positioning affect link feasibility.

---

## 1. Plot Screenshot
![RF Planning Plot](Screenshot_27-3-2026_165651_matlab.mathworks.com.jpeg) 

---

## 2. Experiment Results

### 1️⃣ Increase Gateway Height (+5 m)
* **Does maximum LOS distance change?** Yes. Increasing the gateway height from 20 m to 25 m increases the maximum LOS distance from approximately **18.2 km to 20.3 km**.
* **Does link feasibility improve significantly?** For the specific 4 km link, the change is minor, but it provides a better safety margin against ground-level obstacles.
* **Why?** Antenna height defines the radio horizon; a higher elevation allows the signal to travel further before the Earth's curvature or terrain blocks the path.

### 2️⃣ Increase Antenna Gain to 5 dBi
* **How does the received power curve change?** The curve shifts **upward** on the Y-axis. Increasing gain from 2 dBi to 5 dBi at both ends adds **6 dB** to the total link budget.
* **How much additional range is achieved?** Based on the plot, the distance where the signal hits the -120 dBm sensitivity line extends significantly (roughly doubling the coverage area).
* **Why does antenna gain extend coverage?** It focuses the radio energy into a tighter beam, increasing the power density at the receiver without requiring more battery power at the transmitter.

### 3️⃣ Change Environment Exponent ($n$)
* **How does increasing $n$ affect coverage?** It causes a much steeper drop in received power over distance. Coverage shrinks drastically as $n$ moves from 2.0 (open space) to 3.5 or 4.0 (urban).
* **Which has a stronger impact?** The **environment change ($n$)** has a significantly stronger impact on range than antenna gain.
* **Physical Explanation:** $n$ represents the power loss rate. While gain adds a fixed amount of power, $n$ determines how fast that power is consumed by the environment (buildings, trees, and ground reflections).

### 4️⃣ Move Gateway Location (Fresnel Study)
* **Where is Fresnel radius largest?** At the **midpoint** ($0.5D$ or 50% of the distance).
* **Why is midpoint usually critical?** The Fresnel zone is shaped like an ellipsoid (a football). It is widest in the center and narrows toward the antenna tips.
* **What happens if the 60% clearance rule is violated?** Even with a visual line of sight, the signal will suffer from "diffraction loss." Physical objects entering this zone reflect waves that can cancel out the main signal.

---

## 3. Final Conclusion
In practical wireless deployment, the **environment exponent ($n$)** has the strongest impact on coverage. While hardware improvements like antenna gain provide a steady "boost," the environment dictates the **rate of decay**. In a dense urban area ($n=4$), even high-gain antennas cannot overcome the rapid signal loss compared to a rural setting ($n=2$). Therefore, site selection and understanding the terrain are more critical for link success than hardware specifications alone.
