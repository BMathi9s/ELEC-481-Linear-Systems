# ELEC 481 Lab Analysis and Complete Guide
## ECP Model 205 Torsional Control System

---

## 📋 Lab Overview

This lab focuses on the **ECP Model 205/205a Torsional Control System**, a sophisticated educational apparatus designed for hands-on control systems education. The system consists of rotating disks connected by flexible shafts, creating a multi-degree-of-freedom torsional system that exhibits complex dynamic behavior.

### What is the ECP Model 205 Apparatus?

The **ECP Model 205 Torsional Spring/Inertia Apparatus** is an educational control system that simulates real-world torsional vibration problems found in:
- Automotive drivetrains
- Industrial machinery
- Rotating equipment
- Power transmission systems

**Key Components:**
- **Rotating Disks (Inertias):** Multiple aluminum disks that provide rotational inertia
- **Flexible Shafts:** Steel shafts that provide torsional spring characteristics
- **DC Motor:** Provides controlled torque input
- **Encoders:** Measure angular position and velocity
- **Control Hardware:** ECP interface box with A/D and D/A converters

### System Configurations

Based on the provided MATLAB script, the apparatus supports three main configurations:

1. **Plant #1:** Single disk system (simplest configuration)
2. **Plant #2:** Two-disk system connected by flexible shaft
3. **Plant #3:** Three-disk system with two flexible connections

---

## 🎯 Lab Questions Analysis

### Questions You Can Complete WITHOUT the Physical Apparatus (9 out of 14)

#### ✅ **Question 1: State and Output Equations**
- **Requirement:** Derive state-space representation from input-output equations
- **Approach:** Use system parameters from the provided MATLAB script
- **Data Source:** Parameter values in Appendix A.1i

#### ✅ **Question 2: Transfer Function**
- **Requirement:** Find open-loop transfer function
- **Approach:** Convert from state-space or derive directly from system equations
- **Data Source:** Numerator and denominator polynomials from MATLAB script

#### ✅ **Question 3: Canonical Forms**
- **Requirement:** Controllable, observable, and Jordan canonical forms
- **Approach:** Mathematical transformations using identified parameters
- **Data Source:** State matrices from MATLAB script

#### ✅ **Question 4: Time Responses**
- **Requirement:** Impulse and step responses with arbitrary initial conditions
- **Approach:** Simulation using identified system models
- **Data Source:** Complete system models provided in script

#### ✅ **Question 5: Frequency Analysis**
- **Requirement:** Bode plots and root locus
- **Approach:** Use transfer functions from identified models
- **Data Source:** System parameters and transfer functions

#### ✅ **Question 6: Controller Design**
- **Requirement:** Lead-lag or PID controller design
- **Approach:** Classical control design techniques
- **Data Source:** Open-loop system characteristics

#### ✅ **Question 9: State Feedback Design**
- **Requirement:** Full state feedback control design
- **Approach:** Pole placement or LQR techniques
- **Data Source:** State-space models from script

#### ✅ **Question 12: Transfer Function Analysis**
- **Requirement:** Find observer-controller transfer function
- **Approach:** Mathematical analysis of combined system
- **Data Source:** Designed controllers and observers

#### ✅ **Question 13: Comparative Study**
- **Requirement:** Classical vs. modern control comparison
- **Approach:** Theoretical analysis and literature review
- **Data Source:** Design results from previous questions

### Questions Requiring Physical Apparatus (5 out of 14)

#### ❌ **Question 7: Experimental Responses**
- **Requirement:** Step, square wave, and sinusoidal responses
- **Need:** Physical system for actual measurements

#### ❌ **Question 8: Robustness Testing**
- **Requirement:** Test with noise and parameter variations
- **Need:** Real-time implementation and measurement

#### ❌ **Question 10: Observer-Controller Testing**
- **Requirement:** Experimental validation of observer-based control
- **Need:** Physical implementation

#### ❌ **Question 11: Observer Design and Testing**
- **Requirement:** Full-order and reduced-order observer responses
- **Need:** Physical system for validation

#### ❌ **Question 14: Experimental Validation**
- **Requirement:** Numerical simulations vs. experimental results
- **Need:** Actual experimental data

---

## 📊 How to Obtain System Data

### 1. **From Provided MATLAB Script (Primary Source)**
The Appendix A.1i contains complete system identification data:

```matlab
% Hardware parameters
kc = 10/32768;          % Control gain
kaktkp = 0.70;          % Amplifier gains
ke = 16000/2/pi;        % Encoder gain
ks = 32;                % System gain
khw = kc*kaktkp*ke*ks;  % Overall hardware gain

% Plant #2 Parameters (most common configuration)
J1 = 0.0108;            % Inertia 1 (kg⋅m²)
J2 = 0.0103;            % Inertia 2 (kg⋅m²)
c1 = 0.007;             % Damping 1 (N⋅m⋅s/rad)
c2 = 0.001;             % Damping 2 (N⋅m⋅s/rad)
k1 = 1.37;              % Spring constant (N⋅m/rad)
```

### 2. **From System Identification (If Available)**
- **Frequency Response Data:** Use experimental Bode plot data
- **Step Response Data:** Extract parameters from step response measurements
- **Least Squares Identification:** Fit models to input-output data

### 3. **From Literature Values (Backup)**
- Use typical values for similar torsional systems
- Scale parameters based on physical dimensions
- Validate through simulation consistency

---

## 🔧 System Parameters Summary

### Configuration 1 (Single Disk)
```
J = 0.0109 kg⋅m²
c = 0.948 N⋅m⋅s/rad
Hardware gain = khw = 0.85
```

### Configuration 2 (Two Disks) - **Most Common**
```
J1 = 0.0108 kg⋅m²    J2 = 0.0103 kg⋅m²
c1 = 0.007 N⋅m⋅s/rad  c2 = 0.001 N⋅m⋅s/rad
k1 = 1.37 N⋅m/rad
Hardware gain = khw = 0.85
```

### Configuration 3 (Three Disks)
```
J1 = 0.0025 kg⋅m²    J2 = 0.0018 kg⋅m²    J3 = J2
c1 = 0.007 N⋅m⋅s/rad  c2 = 0.001 N⋅m⋅s/rad  c3 = c2
k1 = 2.7 N⋅m/rad      k2 = 2.6 N⋅m/rad
Hardware gain = khw = 0.85
```

---

## 📝 Step-by-Step Lab Execution Plan

### Phase 1: Theoretical Analysis (Questions 1-6, 9, 12-13)
1. **Week 1:** System modeling and transfer functions (Q1-2)
2. **Week 2:** Canonical forms and time responses (Q3-4)
3. **Week 3:** Frequency analysis and classical design (Q5-6)
4. **Week 4:** Modern control design (Q9)
5. **Week 5:** Observer design and comparative analysis (Q12-13)

### Phase 2: Simulation Validation
1. Implement all designs in MATLAB/Python
2. Compare theoretical and simulated results
3. Prepare for experimental validation

### Phase 3: Experimental Work (If Apparatus Available)
1. System identification and parameter verification
2. Controller implementation and testing (Q7-8, 10-11, 14)

---

## 🎓 Learning Objectives

By completing this lab, you will:

1. **Master System Modeling:** Convert physical systems to mathematical models
2. **Understand Control Design:** Apply both classical and modern techniques
3. **Gain Practical Experience:** Bridge theory with real-world implementation
4. **Develop Analysis Skills:** Compare different control approaches
5. **Learn Professional Tools:** Use industry-standard software

---

## 📚 Required Background Knowledge

- **Mathematics:** Linear algebra, differential equations, Laplace transforms
- **Control Theory:** Classical and modern control techniques
- **Programming:** MATLAB or Python for simulations
- **System Analysis:** Time and frequency domain analysis

---

## ⚠️ Important Notes

1. **Parameter Accuracy:** Use the specific values from your lab manual for best results
2. **System Configuration:** Clarify which configuration (1, 2, or 3) you're analyzing
3. **Safety:** Follow all safety procedures when using the physical apparatus
4. **Documentation:** Maintain detailed records of all calculations and results

---

## 🔗 Additional Resources

- **ECP Manual:** Detailed hardware specifications and safety procedures
- **Control Theory Textbooks:** Franklin, Powell & Emami-Naeini; Ogata; Dorf & Bishop
- **Software Documentation:** MATLAB Control System Toolbox, Python Control Library
- **Academic Papers:** Search for "torsional control systems" and "educational control laboratories"

---

This guide provides a comprehensive roadmap for successfully completing your ELEC 481 lab work, whether you have access to the physical apparatus or not. Focus on the theoretical foundation first, as it forms the basis for all practical implementations.