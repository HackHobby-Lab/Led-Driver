# Led-Driver
This project aims to select MCU and make it to drive the LED driver circuit. 


## Microcontroller Comparison for LED Driver Project

Comparison for a **LED Driver Project**. We evaluate **CH32V203**, **STM32C0**, and **STM32F1** based on key features like **PWM frequency**, **ADC resolution**, and **price** to help you select the best fit for your needs.

## Microcontroller Comparison

### 1. **CH32V203 (RISC-V Core)**
- **Core**: 32-bit **RISC-V**
- **Max Clock Speed**: **72 MHz**
- **PWM Frequency**: Up to **1 MHz** (no prescaler)
- **PWM Resolution**: 16-bit
- **ADC**: 12-bit ADC
- **Price**: **Very Low** (Cost-effective, ideal for budget-constrained projects)

---

### 2. **STM32C0 (Cortex-M0+)**
- **Core**: **ARM Cortex-M0+** (Low Power)
- **Max Clock Speed**: **48 MHz**
- **PWM Frequency**: Up to **1 MHz** (depending on prescaler)
- **PWM Resolution**: 16-bit
- **ADC**: 12-bit ADC
- **Price**: **Low** (Affordable, ideal for cost-sensitive applications)

---

### 3. **STM32F1 (Cortex-M3)**
- **Core**: **ARM Cortex-M3** (Mid-Performance)
- **Max Clock Speed**: **72 MHz**
- **PWM Frequency**: Up to **1.1 MHz** (no prescaler)
- **PWM Resolution**: 16-bit
- **ADC**: 12-bit ADC
- **Price**: **Mid-range** (Affordable for most applications, but more expensive than C0)

---

## Summary of Key Features
*PWM initial Frequency that I have achieved. Can be tweaked further. (Not tested on C0)

| Feature                 | **CH32V203**        | **STM32C0**          | **STM32F1**          |
|-------------------------|---------------------|----------------------|----------------------|
| **Core**                | RISC-V (32-bit)     | Cortex-M0+           | Cortex-M3            |
| **Max Clock Speed**     | 72 MHz              | 48 MHz               | 72 MHz               |
| **PWM Frequency**       | Up to 1 MHz         | -----                | Up to 1.1 MHz        |
| **PWM Resolution**      | 16-bit              | 16-bit               | 16-bit               |
| **ADC**                 | 12-bit              | 12-bit               | 12-bit               |
| **Price**               | Very Low            | Low                  | Mid-range            |


