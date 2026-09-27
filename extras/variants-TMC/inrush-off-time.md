# DRV8874 – OFF time for the inrush protection signal

The purpose of the DRV8874’s inrush protection is to charge any capacitors in locomotives and carriages as quickly as possible, without triggering the OCP. As the OCP intervenes after 3 µs, the charging pulses last only 2.5 µs. The question now is how quickly these pulses can be repeated – in other words, how long the OFF time must be – without the DRV8874 overheating or failing.

## Thermal protection
If a standard 4-layer JLCPCB board is chosen (outer = 1 OZ, inner = 0.5 OZ), with the top copper layer having a minimum area of 2 cm² and sufficient vias, the temperature rises by roughly 35 degrees per Watt (see Figure 21 of the datasheet). However, if a standard 2-layer JLCPCB board (outer = 1 OZ) is chosen, the temperature rises by roughly 75 degrees per Watt (see Figure 23). We will therefore base the analysis below on a 4-layer board.

If the temperature rises by roughly 35 degrees per Watt, and we assume an ambient temperature of 20 degrees, then the total power dissipated by the DRV8874 should be limited to 3 Watts.


Although the supply voltage is known (for example, 16 V), the maximum current that can flow for a brief period during the 2.5 µs cannot really be determined. The RDS(on) value is 200 mΩ, which theoretically means a maximum current of 80 A could flow. However, the datasheet also states that “an analogue current limit circuit on each MOSFET limits the peak current out of the device even in hard short-circuit events”. Values are given for the current thresholds above which the OCP intervenes after 3 µs (minimum 6 A, typical 10 A), but not the maximum current that can be reached within those 3 µs. We must therefore assume that the current during the 2.5 µs lies somewhere between 6 and 80 A.


As the peak current (and therefore the exact power dissipation per pulse) during a hard short circuit cannot be deduced from the datasheet, and the range of possible values is very wide, we cannot base the safe minimum repetition time (OFF time) on an average dissipation of 3 watts. An analysis based on the integrated thermal protection (TSD) is also inconclusive, as hard parameters regarding the chip’s internal thermal conductivity are not available. Consequently, we do not know whether the TSD can respond quickly enough.

## Practical values
As a theoretical analysis does not yield a clear conclusion, we derive the minimum OFF time from the practical values used within the DCC-Ex project. Within that project, a setting of 3 µs ON followed by 13 µs OFF was chosen. If we stay slightly below that – i.e. 2.5 µs ON and 17.5 µs OFF – there is a good chance that the inrush protection will function correctly in our case as well.
