# N_DOF_simulation
Disclaimer: D-H parameter sequence
          <p>  |alpha(i-1) |d(i) |theta(i) or a(i)|</p>

# 🤖 Robotic Arm Manipulator Visualizer using Processing3

A powerful tool for visualizing and comparing the solutions of robotic arm inverse kinematics (IK) using **Processing3**. This project allows users to input a **Denavit-Hartenberg (DH) matrix table** and visualize the solution path of the manipulator according to a given parametric path function. It also supports **real-time Arduino integration**, enabling live performance comparison between calculated IK solutions and actual robotic arm movements.

---

## 📽️ Demo Visualizations

### 🔧 6-DOF Manipulator Visualization
<div align="center">
  <img src="https://github.com/AbrarMahmud/N_DOF_simulation/blob/main/DH_matrix_to_arm_visualizer/6_DOF_arm.gif" alt="github-small" width="50%">
</div>
---

### ⚙️ SCARA Robot Visualization
<div align="center">
  <img src="https://github.com/AbrarMahmud/N_DOF_simulation/blob/main/DH_matrix_to_arm_visualizer/SCARA_arm.gif" alt="github-small" width="50%">
</div>
---

### 🔌 Real-Time Performance Comparison (Arduino)
This visual shows the comparison between the calculated IK solution and the real robotic arm connected through the Arduino serial interface.  
<div align="center">
  <img src="https://github.com/AbrarMahmud/N_DOF_simulation/blob/main/DH_matrix_to_arm_visualizer/IMG_0497.gif" alt="github-small" width="50%">
</div>
---

## 🧩 Features

- 🎯 **DH Parameter Input**: Easily define any robotic manipulator via its DH matrix.
- 📊 **Path Visualization**: Simulate the manipulator's movement across a parametric trajectory.
- 🔁 **Multiple Configurations Supported**: Works with various arm types like 6-DOF, SCARA, and others.
- ⏱️ **Real-Time Serial Communication**: Connect and compare actual vs. calculated movement using Arduino.
- 🔍 **IK Solution Comparison**: Overlay different IK solutions to observe and compare accuracy.

---

## 🚀 Getting Started

### 📥 Prerequisites

- **Processing3** - [Download here](https://processing.org/download/)
- Basic understanding of DH parameters
- *(Optional)* Arduino with a compatible robotic arm for real-time comparison

### 🛠️ How to Use

1. **Clone this repo**:
   ```bash
   git clone https://github.com/yourusername/robotic-arm-visualizer.git
   ```
2. **Open the sketch**  in Processing3.
3. **Edit the DH Table** inside the code or via UI (if supported) to define your manipulator.
   Disclaimer: D-H parameter sequence
          <pre>  || Theta  ||  Alpha  ||  r  ||  d  || </pre>
4. **Define the path function** for your manipulator to follow.
```c++
void pathGeneration(float coordinate[])
{
  if(t_param >= 4*PI)
    t_param = 0.0;
  float t;
  noFill();
  beginShape();
  for(t=0.0;t<t_param;t += 0.01)
    {
      coordinate[0] = 5*cos(1*t)+5;
      coordinate[1] = 7*sin(1*t)+5;
      coordinate[2] = 3*cos(5*t)+2;
      
      //2DOF planner robot (considering end-efector rotation)      
      //coordinate[0] = 2*cos(t)+4;
      //coordinate[1] = 6*sin(t)+4;
      //coordinate[2] = 0*6*sin(t);  
      curveVertex(coordinate[0]*cm,-1*coordinate[1]*cm,coordinate[2]*cm);
    }
  endShape();
  fill(200);
}
   ```
5.**Run the sketch** to see the visualization.

---
## 🧪 Use Cases
1. Visual learning of robotic kinematics and DH parameters
2. Testing and validating IK algorithms
3. Comparing simulated vs. physical manipulator performance
4. Educational demos and robotics research

---
## 🛠️ Customization
You can modify the following to suit your project needs:
1. DH parameters for different manipulator types
2. Parametric path functions
3. Serial port settings for Arduino communication
4. Visualization styles and performance logs

---
## 🧑‍💻 Contributing
Feel free to fork the repo, suggest features, or submit pull requests. Your ideas can help improve this tool for everyone interested in robotics.
