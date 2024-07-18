# CPE_audio_vis
Circuit Playground Express audio visualizer.

Improves on the `Adafruit_CircuitPlayground` library example audio visualizer through:

- Correct FFT implementation
  - Replace the Adafruit_ZeroFFT library in your `Arduino/libraries` directory with donmccoy's [bugfix](https://github.com/donmccoy/Adafruit_ZeroFFT.git)
  - Use the ZeroFFTMagnitude interface
- Log-space mapping of FFT bins to pixels to better approximate human perception
- Smoothing values with EMA to reduce flicker
- FastLED HSV mapping from bin value to pixel color
