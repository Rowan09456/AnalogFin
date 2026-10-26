
#include <math.h>
#define PI 3.14159265358979323846

// In-place Radix-2 Cooley-Tukey FFT
void fft(float *real, float *imag, int n) 
{
    // Bit-reversal permutation
    int j = 0;
    for (int i = 0; i < n; i++) {
        if (i < j) {
            float temp_r = real[i];
            float temp_i = imag[i];
            real[i] = real[j];
            imag[i] = imag[j];
            real[j] = temp_r;
            imag[j] = temp_i;
        }
        int m = n >> 1;
        while (m >= 1 && j >= m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Butterfly stages
    for (int size = 2; size <= n; size <<= 1) {
        float angle = -2.0 * PI / size;
        float w_real = cos(angle);
        float w_imag = sin(angle);
        
        for (int i = 0; i < n; i += size) {
            float cur_w_real = 1.0;
            float cur_w_imag = 0.0;
            int half_size = size >> 1;
            
            for (int k = 0; k < half_size; k++) {
                int match = i + k + half_size;
                // Complex multiplication
                float t_real = cur_w_real * real[match] - cur_w_imag * imag[match];
                float t_imag = cur_w_real * imag[match] + cur_w_imag * real[match];
                
                real[match] = real[i + k] - t_real;
                imag[match] = imag[i + k] - t_imag;
                real[i + k] += t_real;
                imag[i + k] += t_imag;
                
                // Update weight
                float next_w_real = cur_w_real * w_real - cur_w_imag * w_imag;
                cur_w_imag = cur_w_real * w_imag + cur_w_imag * w_real;
                cur_w_real = next_w_real;
            }
        }
    }
}