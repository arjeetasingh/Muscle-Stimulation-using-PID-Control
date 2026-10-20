# Safety

This project drives electrical current through the human body. It is a student research prototype.
It is **not a medical device**, it has **not been tested on hardware or people by the author of this
repository**, and it must not be used for therapy or diagnosis.

## Rules

1. **Dummy load first.** Build and test the stimulator on a resistor with a capacitor in series
   (for example 1 k to 2.2 k in series with 10 uF, plus a parallel 100 k) while watching the output on an
   oscilloscope. Confirm: two equal phases of opposite sign, no DC offset, current equal to the command,
   current limit trips, kill switch cuts the output, the output is off at power-up.
2. **Only on a healthy adult who has agreed**, with someone else present who can reach the kill switch,
   and with approval from your supervisor or institution. Never on the head, neck, chest, across the
   chest, or on anyone with a pacemaker or other implant, epilepsy, pregnancy, or broken or irritated skin.
3. **Battery power only.** No mains-powered part may be connected to the person. Use a USB isolator
   between the ESP32 and a computer while electrodes are on a person.
4. **Hardware limits do not depend on software.** The firmware limits (`config.h`) are a second layer.
   The first layer is a hardware current limit, a fuse, a kill switch and a dead-man button. The firmware
   cannot detect an open-circuit current sensor or rail sensor (the reading just goes to zero), so the
   hardware limit must be correct on its own.
5. **Start low.** Keep `I_MAX_MA` at or below 30 mA (the compile checks refuse more than 50 mA).
   Begin a new person at a small intensity, raise slowly, stop at the first discomfort.
6. **Electrodes:** at least about 5 cm x 5 cm for the larger electrode, fresh hydrogel, firm contact, never
   re-use wet or dried-out pads. Check the skin before and after.
7. **Do not touch the electrodes or the output wires with the stimulator armed.**

## What the firmware does

| Layer | Behaviour |
|---|---|
| Boot | All outputs low, enable pin low (external pull-down required) |
| ARM | Needs a calibrated force sensor and a tare. Enables the high-voltage stage only after the command |
| START | Needs the dead-man button held. Releasing it for 30 ms stops the run |
| Faults (latching) | Over-current, over-voltage, over-temperature or +4 C rise, heart-rate out of range or missing, force above limit, force-sensor fault, EMG lead-off (mirror mode), session longer than 300 s, continuous stimulation budget (15 s leaky bucket), commanded current but none measured, control loop late (> 50 ms), temperature sensor fault |
| On a fault | Stimulation off, enable low, state FAULT. Only `RESET` in IDLE clears it |

All of this is exercised by `firmware/test/test_core.cpp`. That tests the logic, **not** the hardware.

## What is not covered

* The analogue high-voltage stage is described in `docs/hardware.md` by its function and parts class only.
  There is no verified schematic in this repository. Whoever builds it is responsible for its design,
  for the creepage and clearance of the high-voltage section, and for its tests.
* Heart rate comes from a finger optical sensor. It is a plausibility check, not a medical monitor.
* Skin-temperature sensing with an NTC under an electrode is an indication, not a burn guard.
* Force is measured on a pad, not inside the muscle.

If anything feels wrong, press the kill switch first and ask questions afterwards.
