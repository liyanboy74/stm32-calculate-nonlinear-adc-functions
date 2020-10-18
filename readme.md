# Stm32 Calculate Nonlinear ADC Functions

### Step One:
1. Compile and Run `Get_Sample`
2. Open `UART1` 
3. Connect `Potensiometer` to pin `A0`
4. Connect pin `A0` to `Voltmeter`
5. Restart the MCU
6. in `UART1` Terminal Enter `S` to Start Sampling from ADC
7. use `Enter` or `G` to record sample And Enter the Value of the voltmeter Show! (float in Volt)
8. Rotate `Potensiometer` and enter the value from `Voltmeter` several times
9. Enter `P` to print result.
10. copy result and use that in `ADC_Calculate`

### Step Two

1. open `ADC_Calculate`
2. Paste or Replace result from `Step One`
3. Use `float Calculate_ADC(uint16_t Adc)` for convert `ADC` to `Voltage` or something else !
