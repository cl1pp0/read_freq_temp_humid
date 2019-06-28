# read_freq_temp_humid
An Arduino UNO based frequency, temperature, humidity meter with LCD display and serial output

* Implementation for Arduino UNO
* Frequency is measured via a photo transistor and some analog pre-processing (eliminate DC voltage, amplify AC voltage)
* Temperature and humidity is measured via DHT11
* Output on 2x16 LCD and via serial port

It can be used for e.g. measuring the frequency of the power grid with a simple LED light bulb (emits double frequency due to rectification) above the photo transistor.

Output via serial port looks like:
```
Humidity = 34.0%, Temperature = 27.0°C, Frequency = 99.971Hz, Voltage = 502mV
Humidity = 33.0%, Temperature = 27.0°C, Frequency = 99.976Hz, Voltage = 507mV
Humidity = 34.0%, Temperature = 27.0°C, Frequency = 99.978Hz, Voltage = 517mV
Humidity = 34.0%, Temperature = 27.0°C, Frequency = 99.974Hz, Voltage = 536mV
Humidity = 33.0%, Temperature = 27.0°C, Frequency = 99.975Hz, Voltage = 551mV
Humidity = 34.0%, Temperature = 27.0°C, Frequency = 99.977Hz, Voltage = 566mV
Humidity = 34.0%, Temperature = 27.0°C, Frequency = 99.984Hz, Voltage = 590mV
Humidity = 34.0%, Temperature = 27.0°C, Frequency = 99.983Hz, Voltage = 614mV
Humidity = 34.0%, Temperature = 27.0°C, Frequency = 99.984Hz, Voltage = 629mV
``` 
Output on 2x16 LCD looks like:

```
+----------------+
|U/mV=502        |
|f/Hz=99.976     |
+----------------+
```
