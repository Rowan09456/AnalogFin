
// Console keys: 1 = triangle, 2 = sawtooth, 3 = square, 4 = composite sine

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h"

#include "config.h"
#include "rms.h"
#include "iir.h"
#include "gfft.h"





static float wave_freq = 50.0; // Default sine wave frequency
static uint32_t sample_idx = 0;

uint16_t pwm_wrap_val;
uint16_t iir_pwm_wrap_val;
uint slice_num;
uint chan;

float vReal[FFT_N], vImag[FFT_N];

uint iir_slice_num;
uint iir_chan;
enum pwm_output {
    PWM_SINE = 0,
    PWM_SQUARE,
    PWM_SAWTOOTH,
    PWM_TRIANGLE,
    PWM_COMPOSITE

} this_pwm_output = PWM_SINE;

enum adc_op {
    IDLE = 0,
    RMS,
    FFT,
    IIR

} this_adc = IDLE;

// Conversion factor for 12-bit ADC (0 to 4095) with a 3.3V reference
 const float conversion_factor = 3.3f / (1 << 12);
 const float scale_factor = 2*120.0f*sqrt(2.0f) / 3.3f; // Peak to peak voltage of 120V RMS sine wave, scaled to 0..3.3V range 
 uint16_t adc_buffer[FFT_N]; // Buffer to hold ADC samples for FFT

uint16_t adc_LastIdx = 0; // Index for the last sample in the ADC buffer
uint16_t adc_MaxIdx = 0; // Max samples to capture

bool adc_InProgress = false; // Flag to indicate if ADC capture is in progress
bool RMS_complete = false; // Flag to indicate if RMS calculation is complete 
float rms_value = 0.0f; // Variable to hold the calculated RMS value
bool FFT_complete = false; // Flag to indicate if FFT processing is complete

// Timer callback to update the duty cycle
bool timer_callback(repeating_timer_t* rt) {

    float sine_val, square_val, sawtooth_val, triangle_val, composite_val;
    // Calculate current angle
    float time = (float)sample_idx / SAMPLE_RATE;
    float pwm_output = 0.0f;
    sine_val = sinf(2.0f * M_PI * wave_freq * time);
    if (sine_val < 0.0f) {
        square_val = 1.0f; // Ensure non-negative for PWM
    }
    else { square_val = 0.0f; }
    sawtooth_val = 2.0f * (time * wave_freq - floorf(time * wave_freq + 0.5f)); // Sawtooth from -1 to 1
    triangle_val = 2.0f * fabsf(sawtooth_val) - 1.0f; // Triangle from -1 to 1
    composite_val = sine_val + 0.1f * (sinf(4.0f * M_PI * wave_freq * time) +
        sinf(6.0f * M_PI * wave_freq * time) +
        sinf(10.0f * M_PI * wave_freq * time) +
        sinf(14.0f * M_PI * wave_freq * time) +
        sinf(50.0f * M_PI * wave_freq * time) +
        sinf(70.0f * M_PI * wave_freq * time) +
        sinf(100.0f * M_PI * wave_freq * time));

    // Scale sine wave (-1.0 to 1.0) to PWM duty cycle (0 to pwm_wrap_val)
    switch (this_pwm_output) {
    case PWM_SINE:
        pwm_output = (sine_val + 1.0f) * 0.5f; // Scale to 0..1
        break;
    case PWM_SQUARE:
        pwm_output = square_val; // Already 0 or 1
        break;
    case PWM_SAWTOOTH:
        pwm_output = (sawtooth_val + 1.0f) * 0.5f; // Scale to 0..1
        break;
    case PWM_TRIANGLE:
        pwm_output = (triangle_val + 1.0f) * 0.5f; // Scale to 0..1
        break;
    case PWM_COMPOSITE:
        pwm_output = (composite_val + 1.0f) * 0.5f; // Scale to 0..1
        break;
    }

    uint16_t level = (uint16_t)(pwm_output * pwm_wrap_val);

    // Update the hardware channel level
    pwm_set_chan_level(slice_num, chan, level);


    sample_idx++;
    if (sample_idx >= SAMPLE_RATE) sample_idx = 0; // Prevent overflow

    return true; // Keep timer running
}

// Timer callback to update the duty cycle
bool adc_capture(repeating_timer_t* rt) 
{
    //printf("Timer 2 fired at %lld us\n", time_us_64());
    if(this_adc==IIR)
    {
        
        uint16_t adc_val = adc_read(); // Read ADC value 
        float adc_voltage = (float)adc_val * conversion_factor; // Convert to voltage
        float filtered_voltage = applyfilter(adc_voltage); // Apply IIR filter
        uint16_t iir_level = (uint16_t)((filtered_voltage / 3.3f) * iir_pwm_wrap_val); // Scale to PWM level
        pwm_set_chan_level(iir_slice_num, iir_chan, adc_val); // Update the hardware channel level with filtered value
        return(true); // Keep timer running
    }

    if(this_adc== IDLE)
        return false; //    Stop timer!

    if(adc_LastIdx >= adc_MaxIdx)
    {
        adc_InProgress = false; // Capture complete
        return false; // Stop timer!
    }
    else
    {
        // Read ADC value and store in buffer
        uint16_t adc_val = adc_read();
        adc_buffer[adc_LastIdx++] = adc_val;
        return true; // Keep timer running
    }    
    
}

int main() 
{
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    //1. Initialize ADC subsystem
    adc_init();   // Initialize the ADC hardware block
    adc_gpio_init(ADC_IN_GPIO); // Configure GPIO 26 to act as an analog input pin (High-impedance, no pull-ups)
    adc_select_input(ADC_IN_GPIO - 26);  // Select ADC channel 0 (which is hardwired to GPIO 26)
     

    // 2. Setup PWM Pin & Slice
    gpio_set_function(PWM_OUT_GPIO, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(PWM_OUT_GPIO);    
    chan = pwm_gpio_to_channel(PWM_OUT_GPIO);

    // 2a. Calculate Clock Divider and Wrap for 100 kHz carrier frequency
    uint32_t sys_clk = clock_get_hz(clk_sys); 
    pwm_wrap_val = 1023;  // Let's use a standard wrap of 1024 for good duty cycle resolution
    float clk_div = (float)sys_clk / (CARRIER_FREQ * (pwm_wrap_val + 1));
    pwm_set_clkdiv(slice_num, clk_div);
    pwm_set_wrap(slice_num, pwm_wrap_val);
    pwm_set_enabled(slice_num, true);


    // 3. Setup a Repeating Timer for changing the duty cycle and ADC
    repeating_timer_t timer;
    repeating_timer_t adc_timer;

    int32_t delay_us2 = -(50); // Negative value ensures strict timing
    add_repeating_timer_us(delay_us2, adc_capture, NULL, &adc_timer); // 20 kHz ADC capture
 
    int32_t delay_us = -(1000000 / SAMPLE_RATE); // Negative value ensures strict timing
    add_repeating_timer_us(delay_us, timer_callback, NULL, &timer);
 
    //3b. Setup IIR PWM Pin & Slice
    gpio_set_function(IIR_OUT_GPIO, GPIO_FUNC_PWM);
    iir_slice_num = pwm_gpio_to_slice_num(IIR_OUT_GPIO);
    iir_chan = pwm_gpio_to_channel(IIR_OUT_GPIO);
    iir_pwm_wrap_val = 1023;  // Let's use a standard wrap of 1024 for good duty cycle resolution
    float iir_clk_div = (float)sys_clk / (ADC_SAMPLE_HZ * (iir_pwm_wrap_val + 1));
    pwm_set_clkdiv(iir_slice_num, iir_clk_div);
    pwm_set_wrap(iir_slice_num, iir_pwm_wrap_val);
    pwm_set_enabled(iir_slice_num, true);

    
    

    printf("Generating %g Hz sine wave on pin %d\n", wave_freq, PWM_OUT_GPIO);
    printf("PWM Output: [1] Sine, [2] Square, [3] Sawtooth, [4] Triangle, [5] Composite\n");
    printf("ADC Proc:   [6] RMS,  [7] FFT ,   [8] IIR Filter(GP15-Pin 20) [9] IDLE\n");
    printf("Press 0 to change frequency (enter new frequency in Hz)\n");
    int i = 0;

    while (1) 
    {
        int ch = getchar_timeout_us(0);
        if (ch >= '1' && ch <= '5') 
        {
            switch (ch) {
            case '1': this_pwm_output = PWM_SINE; printf("Generating %g Hz sine wave on pin %d\n", wave_freq, PWM_OUT_GPIO); break;
            case '2': this_pwm_output = PWM_SQUARE; printf("Generating %g Hz square wave on pin %d\n", wave_freq, PWM_OUT_GPIO); break;
            case '3': this_pwm_output = PWM_SAWTOOTH; printf("Generating %g Hz sawtooth wave on pin %d\n", wave_freq, PWM_OUT_GPIO); break;
            case '4': this_pwm_output = PWM_TRIANGLE; printf("Generating %g Hz triangle wave on pin %d\n", wave_freq, PWM_OUT_GPIO); break;
            case '5': this_pwm_output = PWM_COMPOSITE; printf("Generating %g Hz composite wave on pin %d\n", wave_freq, PWM_OUT_GPIO); break;
            }
            printf("Press 1 for sine, 2 for square, 3 for sawtooth, 4 for triangle, 5 for composite\n");
            printf("Press 6 for RMS [1 cycle], 7 for FFR [1 cycle], 8 for Butterworh Filtering (output GPIO 15 - Pin 20\n");
            printf("Press 0 to change frequency (enter new frequency in Hz)\n");

        }
        else if (ch == '0') 
        {
            printf("Enter new frequency in Hz: ");
            float new_freq;
            scanf("%f", &new_freq);
            wave_freq = new_freq;
            printf("Frequency changed to %g Hz\n", wave_freq);
            printf("Press 1 for sine, 2 for square, 3 for sawtooth, 4 for triangle, 5 for composite\n");
            printf("Press 6 for RMS [1 cycle], 7 for FFR [1 cycle], 8 for Butterworh Filtering (output GPIO 15 - Pin 20\n");
            printf("Press 0 to change frequency (enter new frequency in Hz)\n");
            

        }
        else if(ch >= '6' && ch<='9')
        {
            this_adc = IDLE; // Stop any ongoing ADC processing
            switch (ch) {
                case '6': 
                this_adc = RMS; 
                printf("ADC Processing: RMS [1 cycle]\n"); 
                adc_MaxIdx = ADC_SAMPLE_HZ/wave_freq; // Calculate the number of samples to capture for 1 cycle
                if(adc_MaxIdx > FFT_N)
                {
                    printf("Wave frequency is too low. Increasing to 25 Hz.\n");
                    wave_freq = 25.0f;
                    adc_MaxIdx = ADC_SAMPLE_HZ/wave_freq;
                }
                if(adc_MaxIdx < 100)
                {
                    printf("Wave frequency is too high. Decreasing to 200 Hz.\n");
                    wave_freq = 200.0f;
                    adc_MaxIdx = ADC_SAMPLE_HZ/wave_freq;
                }
                break;
                case '7': 
                this_adc = FFT; 
                printf("ADC Processing: FFT [1 cycle]\n"); 
                adc_MaxIdx = FFT_N; // Capture max. samples for FFT processing
                break;
                case '8': this_adc = IIR; printf("ADC Processing: Butterworth Filtering (output GPIO 15 - Pin 20)\n"); break;
                case '9': this_adc = IDLE; printf("ADC Processing: IDLE\n"); break;
            }
                        
            
            adc_LastIdx = 0; // Reset the index for new ADC processing
            //ReEnable ADC Timer here!
            add_repeating_timer_us(-50, adc_capture, NULL, &adc_timer); // 20 kHz ADC capture
            if((this_adc != IIR)&&(this_adc != IDLE))
                adc_InProgress = true; // Set the flag to indicate ADC capture is in progress
            while(adc_InProgress) 
            {
                 
                sleep_ms(250); // Wait for ADC capture to complete
                
                putchar('.'); // Indicate waiting
            }
            printf("\n");
        }
        if(this_adc == RMS)
        {
            rms_value = calculate_rms(adc_buffer, adc_MaxIdx, conversion_factor)*scale_factor; // Convert to voltage
            printf("\nRMS Value: %.4f V\n", rms_value);
            this_adc = IDLE; // Reset to IDLE after processing
        }
        else if(this_adc == FFT)
        {
            // Perform FFT on the captured ADC samples
            for (int i = 0; i < FFT_N; i++) {
                vReal[i] = (float)(adc_buffer[i] * conversion_factor); // Convert to voltage
                vImag[i] = 0.0f; // Imaginary part is zero for real signals
            }
            fft(vReal, vImag, FFT_N); // Call the FFT function
          
            for (int i = 0; i < FFT_N / 2; i++) 
            {
                float magnitude = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
                float bin_frequency = (float)i * ADC_SAMPLE_HZ / FFT_N;
                if (magnitude > FFT_MIN_PEAK_V) // Only print peaks above the threshold
                {
                    printf("%5d | %14.2f | %9.3f\n", i, bin_frequency, magnitude);
                }
            }
                
           
            this_adc = IDLE; // Reset to IDLE after processing
        }

        sleep_ms(100);
        i++;
        if (i == 10) 
        {
            i = 0;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
        }

    }
}



