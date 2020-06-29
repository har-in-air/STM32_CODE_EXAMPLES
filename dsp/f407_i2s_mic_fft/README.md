## Spectral analysis of audio stream from I2S microphone

* I2S2 configured as I2S master half-duplex receive, 24bit data in 32bit frame,
Fs = 32kHz
* Only upper 16bits of sample data used
* 128 point sample buffer
* Hanning window applied to input buffer before FFT

## Fixed point versus floating point FFT

Tried ARM CMSIS fixed point Q15 and floating point F32 fft.

Execution time was logged on a logic analyzer - onboard pin PA4 
pulsed high for the FFT execution duration, pin PA5 toggled every time
we read 128 data samples from the I2S DMA receive buffer.

On the STM32F407, it looks like there is no advantage to using
fixed point for 128point FFTs (Q15 fft = 128uS, F32 fft = 120uS).

### Fixed point Q15 FFT

<img src="fft_128pt_q15.png"/>

### Floating point F32 FFT

<img src="fft_128pt_f32.png"/>

## FFT of test audio data from an external audio signal generator

* Android signal generator app used to generate 600Hz sine wave, 4650Hz sine wave and 
800Hz square wave tones on a phone placed next to microphone.
* Onboard PA1 button press used to trigger dump of FFT magnitude array to serial terminal.
* Serial terminal data log opened in spreadsheet for plotting
* 128 point FFT, spectral resolution is 32kHz/128 = 250Hz = array spacing.
     
<img src="generator_600Hz_sine_wave.png"/>
<img src="generator_4650Hz_sine_wave.png"/>
<img src="generator_800Hz_square_wave.png"/>
<img src="cutecom_screenshot.png"/>
<img src="600Hz_sine_wave.png"/>
<img src="4650Hz_sine_wave.png"/>
<img src="800Hz_square_wave.png"/>

