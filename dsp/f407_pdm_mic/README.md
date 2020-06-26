## PDM Microphone to I2S class-D Amplifier

Demonstrates use of PDM microphone as input and I2S class-D amplifier as output. This is a clone of 
the [YetAnotherElectronicsChannel project](https://github.com/YetAnotherElectronicsChannel/STM32_PDM_Microphone).
The only changes are the components used. I am using an STM MP45DT02 PDM microphone, and MAX98357A I2S Class-D amplifier. The MAX98357A does not require an MCLK input.  By default, if using 5V as a power supply, the module is set up to average the L and R channels if you leave the SD pin floating. I connected the SD input to 3.3V to configure the MAX98357A to use the L channel as input to the amplifier.

I2S2 is configured as a slave for timing, it receives BCLK from I2S3. The PDM2PCM library is used to convert the PDM data stream into PCM data.


<img src="wiring_schematic.jpg">


See [this Youtube video](https://www.youtube.com/watch?v=JuXKeyFraF4) for an explanation of the data flow, STM32CubeIDE project setup and code review.



<img src="i2s_data_format.jpg">
