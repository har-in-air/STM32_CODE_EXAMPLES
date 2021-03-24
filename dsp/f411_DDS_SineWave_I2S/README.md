# Direct Digital Synthesis of periodic waveform 
* Outputs I2S digital audio stream with 24bit resolution @ sampling frequency Fs = 48KHz
* In this example, the periodic waveform is a sine wave  with a harmonic
* Configurable inputs are 
	* Fundamental frequency = `WAVE_FREQ_HZ`
	* Fundamental amplitude = `VOLUME`
* The example is built to generate a 440Hz (musical A note) with harmonic at 880Hz
* I2S peripheral configured in half-duplex master transmit mode
* Circular DMA transmit from sample buffer to I2S peripheral with half-complete and full-complete
  callbacks to generate new samples
* Implemented on WeAct STM32F411CEU6 Black Pill dev board. Will also  work with trivial mods on 
  the cheaper STM32F401CCU6 Black Pill board.
  
### Credits
* https://www.youtube.com/watch?v=YDC5zaEZGhM
* https://github.com/dimtass/stm32f407_dds_dac,
* https://github.com/YetAnotherElectronicsChannel/STM32_PDM_Microphone
