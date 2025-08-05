<img width="221" height="378" alt="{31113221-7220-4430-98CB-CC8F5DADEAFF}" src="https://github.com/user-attachments/assets/e8a5046f-5450-4afe-b887-9c411795025e" />------------------

Features

- Time-Expanded A* for multi-agent path planning  
- Rerouting logic to avoid deadlocks & long waits  
- Wireless direction communication over WiFi using Python + Arduino Uno WiFi  
- Line-following with IR sensors for physical execution  
- Synchronized execution** of multiple robots  
- Simulation + Real-world testing with video results  


------------------

How It Works

1. User selects start and end positions for each robot in a grid GUI.
2. The system runs Time-Expanded A* with reservation tables to compute collision-free paths.
3. Paths are translated into direction strings and sent via WiFi to each robot.
4. Robots wait for user confirmation, then begin simultaneous execution on a taped floor grid.
5. Real-world testing scenarios include:
   - Head-to-Head Collision
   - Crossing Paths
   - In-Series Movement


------------------

Demo Videos

<img width="2255" height="2255" alt="qrVideo" src="https://github.com/user-attachments/assets/4c3bf5c0-c61d-410c-9b18-e0712bb3df78" />


------------------

Hardware

- Arduino Uno WiFi Rev2 × 2  
- IR sensor array × 5 per robot (for line following)  
- Taped floor grid (6×10 cells, ~1ft per cell)  
- Central control via Python on PC (WiFi hotspot)


---

Limitations & Future Work

- Reliance on line-following limits long-term reliability
- Hardware lacks encoders, affecting accuracy
- Battery life constraints
- Future work: Add encoder feedback, dynamic replanning, and support for 3+ robots


---

License

This project is part of a Final Year Dissertation at UNIVERSITI TEKNOLOGI PETRONAS by Muhammad Adam bin Kahar (Student ID: 21001952) . For academic reference and educational use only.


---

