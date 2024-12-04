#include <mbed.h>
#include <vector>
#include <cmath>
#include "Drivers/LCD_DISCO_F429ZI.h"
#include "Drivers/TS_DISCO_F429ZI.h"
#include <thread.h>
#include <limits.h>
#include <float.h>

// Flags
#define SPI_FLAG (1 << 0)

// Registers
#define OUT_X_L 0x28
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0x6F
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0x00
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0x08

// Configuration constants
const uint32_t GYRO_SPI_FREQ = 1000000; // 1MHz
const float SENSITIVITY_125DPS = 0.0000763582f;
const float SENSITIVITY_250DPS = 0.0001527163f;
const float SENSITIVITY_500DPS = 0.0003054326f;
const float SENSITIVITY = SENSITIVITY_250DPS;
const uint8_t SPI_DATA_SIZE = 8;
const uint8_t SPI_MODE = 3;
const int SEQUENCE_DELAY_MS = 100;
const int MAX_SEQUENCE = 100;
const float DEFAULT_NOISE_THRESHOLD = 0.0001;

// Sensor data buffers
static uint8_t tx_buffer[8];
static uint8_t rx_buffer[8];
static int16_t gyro_raw[3];
static float gyro_data[3];
float ground_truth_gesture_sequence[MAX_SEQUENCE][3];
std::vector<std::vector<float>> ground_truth_sequences;
float test_sequence[MAX_SEQUENCE][3];
std::vector<std::vector<float>> test_sequences;

// Objects
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
DigitalOut led1(LED2);
EventFlags flags;

// Functions to setup gyro and read value

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
};

void execute_spi_transfer(uint8_t length)
{
    spi.transfer(tx_buffer, length, rx_buffer, length + 1, spi_cb, SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);
}

void toggle_status_led()
{
    led1 = !led1;
    wait_us(100);
}

void configure_gyroscope()
{
    // Configure SPI settings
    spi.format(SPI_DATA_SIZE, SPI_MODE);
    spi.frequency(GYRO_SPI_FREQ);

    // Register configurations
    const uint8_t register_configs[][2] = {
        {CTRL_REG1, CTRL_REG1_CONFIG},
        {CTRL_REG4, CTRL_REG4_CONFIG},
        {CTRL_REG3, CTRL_REG3_CONFIG}};

    // Initialize each register
    for (const auto &config : register_configs)
    {
        tx_buffer[0] = config[0];
        tx_buffer[1] = config[1];
        execute_spi_transfer(2);
    }
}

void read_gyroscope()
{
    // Prepare for sequential read
    tx_buffer[0] = OUT_X_L | 0x80 | 0x40; // Auto-increment and read flags
    execute_spi_transfer(7);

    // Process each axis
    for (int axis = 0; axis < 3; axis++)
    {
        // Combine high and low bytes
        gyro_raw[axis] = (static_cast<int16_t>(rx_buffer[2 * axis + 2]) << 8) |
                         static_cast<int16_t>(rx_buffer[2 * axis + 1]);

        // Convert to angular velocity
        gyro_data[axis] = gyro_raw[axis] * SENSITIVITY;
    }

    // Visual feedback
    toggle_status_led();
}

void record_gesture_sequence() {
    
    for(int i = 0; i < MAX_SEQUENCE; i++) {
        read_gyroscope();  // Using the new sample_gyroscope function
        
        // Store values in array and vector
        ground_truth_gesture_sequence[i][0] = gyro_data[0];
        ground_truth_gesture_sequence[i][1] = gyro_data[1];
        ground_truth_gesture_sequence[i][2] = gyro_data[2];

        ThisThread::sleep_for(chrono::milliseconds(SEQUENCE_DELAY_MS));
        
        // Add to sequence collection
        ground_truth_sequences.push_back({
            gyro_data[0],
            gyro_data[1],
            gyro_data[2]
        });
        
    }
    
}

void attempt_sequence() {
    for(int i = 0; i < MAX_SEQUENCE; i++) {
        read_gyroscope();
        
        // Store values in array and vector
        test_sequence[i][0] = gyro_data[0];
        test_sequence[i][1] = gyro_data[1];
        test_sequence[i][2] = gyro_data[2];
        
        ThisThread::sleep_for(chrono::milliseconds(SEQUENCE_DELAY_MS));

        test_sequences.push_back({
            gyro_data[0],
            gyro_data[1],
            gyro_data[2]
        });

    }
    
}


// preprocessing stage - 1 (Calibrating to initial position and denoising the data)
void calibrate_gyro_using_initial_position(std::vector<std::vector<float>>& sequence)
{
    if(sequence.empty())
    {
        return;
    }

    std::vector<float> initial_position = sequence[0];

    for(auto& pos : sequence)
    {
        for(size_t i = 0; i < pos.size(); i++)
        {
            pos[i] = pos[i] - initial_position[i];
        }
    }
}

void remove_noise_from_datapoints(std::vector<std::vector<float>>& sequence, float noise_threshold = DEFAULT_NOISE_THRESHOLD)
{
    if (sequence.empty()) {
        return;
    }

    // Calculate mean values
    std::vector<float> means(sequence[0].size(), 0.0f);
    std::vector<float> max_values(sequence[0].size(), -FLT_MAX);
    std::vector<float> min_values(sequence[0].size(), FLT_MAX);

    // Gather statistics
    for (const auto& data_point : sequence) {
        for (size_t i = 0; i < sequence[0].size(); i++) {
            means[i] += data_point[i];
            max_values[i] = std::max(max_values[i], data_point[i]);
            min_values[i] = std::min(min_values[i], data_point[i]);
        }
    }

    // Calculate final means
    for (auto& mean : means) {
        mean /= sequence.size();
    }

    // Apply calibration
    int noise_removals = 0;
    for (auto& data_point : sequence) {
        for (size_t i = 0; i < sequence[0].size(); i++) {
            // Remove mean (DC offset)
            data_point[i] -= means[i];
            
            // Apply noise threshold
            if (std::abs(data_point[i]) < noise_threshold) {
                data_point[i] = 0.0f;
                noise_removals++;
            }
        }
    }
}


// helper - TODO delete these later from source code


int main()
{
    

    while (1)
    {
       
    }
}