# Design-of-a-USB-HID-device-with-data-logger-and-pulse-oximeter
Desing of a USB HID device for psychological tests using a STM32F407 microcontroller (ARM processor). 
To get a correct analysis It is necesary to record time, when different events happen, with great accuracy and precision. 
Microcontroller works as a USB HID keyboard wich send a character when buttons are pressed. Microcontroller was programed 
to implement a control console wich work via serial port, It's possible interact with the console using Matlab or another 4
serial port program. It's possible store, read and delete information events in the microcontroller Flash memory. 
Capable to Synchronize external watches with microcontroller watch. Adapt the signal from a standard pulse oximeter
sensor to microcontroller voltage and current level. Microcontroller samples pulse-oximeter signal after to get filter
and amplify. Using DSP library provided by ARM, microcontroller carry out FFT (Fast Fourier Transform) of the sampled 
signal and get the heart rate.  
Full information (Spanish): https://riunet.upv.es/bitstream/handle/10251/56704/JimenezJerez_JuanDomingo.pdf
