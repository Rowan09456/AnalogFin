// config.h - all user-tunable settings in one place
#pragma once
// ---------------- Pins ----------------
#define PWM_OUT_GPIO        16      // any GPIO works; 16 = PWM slice 0, channel A
#define ADC_IN_GPIO         26      // must be 26, 27 or 28 (ADC0..ADC2). Don't use 29 on a Pico 2 W.
#define IIR_OUT_GPIO        14      // any GPIO works; 14 = PWM slice 7, channel A


#define CARRIER_FREQ 100000 // 100 kHz high-frequency PWM carrier
#define SAMPLE_RATE 2000    // Update duty cycle 2000 times per second

// ---------------- Input (ADC) ----------------
#define ADC_SAMPLE_HZ       20000   // 48 MHz ADC clock / 2400 = 20 kHz exactly
#define ADC_VREF            3.3f    // ADC reference voltage (3.3 V on a Pico 2 W)

// ---------------- FFT ----------------
// Power of 2, and no larger than 16384 (microFFT uses 16-bit loop counters).
// 16384 samples @ 20 kHz = 0.82 s of signal, 1.22 Hz per bin.

#define FFT_N               16384
#define FFT_MIN_PEAK_V      .01250f  // ignore peaks smaller than 12.5 mV (1/4 LSB of 12-bit ADC)

