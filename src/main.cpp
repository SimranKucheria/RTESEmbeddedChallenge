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

void record_gesture_sequence()
{

    for (int i = 0; i < MAX_SEQUENCE; i++)
    {
        read_gyroscope(); // Using the new sample_gyroscope function

        // Store values in array and vector
        ground_truth_gesture_sequence[i][0] = gyro_data[0];
        ground_truth_gesture_sequence[i][1] = gyro_data[1];
        ground_truth_gesture_sequence[i][2] = gyro_data[2];

        ThisThread::sleep_for(chrono::milliseconds(SEQUENCE_DELAY_MS));

        // Add to sequence collection
        ground_truth_sequences.push_back({gyro_data[0],
                                          gyro_data[1],
                                          gyro_data[2]});
    }
}

void validate_sequence()
{
    for (int i = 0; i < MAX_SEQUENCE; i++)
    {
        read_gyroscope();

        // Store values in array and vector
        test_sequence[i][0] = gyro_data[0];
        test_sequence[i][1] = gyro_data[1];
        test_sequence[i][2] = gyro_data[2];

        ThisThread::sleep_for(chrono::milliseconds(SEQUENCE_DELAY_MS));

        test_sequences.push_back({gyro_data[0],
                                  gyro_data[1],
                                  gyro_data[2]});
    }
}

// preprocessing stage - 1 (Calibrating to initial position and denoising the data)

void normalize_sequence(std::vector<std::vector<float>> &sequence)
{
    if (sequence.empty())
    {
        return;
    }

    const int num_points = sequence.size();
    const int num_dimensions = sequence[0].size();

    // Initialize statistical vectors
    std::vector<float> mean(num_dimensions, 0.0f);
    std::vector<float> stddev(num_dimensions, 0.0f);

    // Calculate mean
    for (const auto &point : sequence)
    {
        for (int dim = 0; dim < num_dimensions; ++dim)
        {
            mean[dim] += point[dim];
        }
    }

    for (int dim = 0; dim < num_dimensions; ++dim)
    {
        mean[dim] /= num_points;
    }

    // Calculate standard deviation
    for (const auto &point : sequence)
    {
        for (int dim = 0; dim < num_dimensions; ++dim)
        {
            float diff = point[dim] - mean[dim];
            stddev[dim] += diff * diff;
        }
    }

    for (int dim = 0; dim < num_dimensions; ++dim)
    {
        stddev[dim] = std::sqrt(stddev[dim] / num_points);
    }

    // Perform normalization
    for (auto &point : sequence)
    {
        for (int dim = 0; dim < num_dimensions; ++dim)
        {
            if (std::abs(stddev[dim]) != 0)
            {
                point[dim] = (point[dim] - mean[dim]) / stddev[dim];
            }
        }
    }
}

void calibrate_gyro_using_initial_position(std::vector<std::vector<float>> &sequence)
{
    if (sequence.empty())
    {
        return;
    }

    std::vector<float> initial_position = sequence[0];

    for (auto &pos : sequence)
    {
        for (size_t i = 0; i < pos.size(); i++)
        {
            pos[i] = pos[i] - initial_position[i];
        }
    }
}

void remove_noise_from_datapoints(std::vector<std::vector<float>> &sequence, float noise_threshold = DEFAULT_NOISE_THRESHOLD)
{
    if (sequence.empty())
    {
        return;
    }

    // Calculate mean values
    std::vector<float> means(sequence[0].size(), 0.0f);
    std::vector<float> max_values(sequence[0].size(), -FLT_MAX);
    std::vector<float> min_values(sequence[0].size(), FLT_MAX);

    // Gather statistics
    for (const auto &data_point : sequence)
    {
        for (size_t i = 0; i < sequence[0].size(); i++)
        {
            means[i] += data_point[i];
            max_values[i] = std::max(max_values[i], data_point[i]);
            min_values[i] = std::min(min_values[i], data_point[i]);
        }
    }

    // Calculate final means
    for (auto &mean : means)
    {
        mean /= sequence.size();
    }

    // Apply calibration
    int noise_removals = 0;
    for (auto &data_point : sequence)
    {
        for (size_t i = 0; i < sequence[0].size(); i++)
        {
            // Remove mean (DC offset)
            data_point[i] -= means[i];

            // Apply noise threshold
            if (std::abs(data_point[i]) < noise_threshold)
            {
                data_point[i] = 0.0f;
                noise_removals++;
            }
        }
    }
}

// Preprocessing stage - 2 (Adding a filter for denoising, drift compensation and improving the data quality)
void moving_average_filter(std::vector<std::vector<float>> &sequence, int window_size = 3)
{

    if (sequence.empty() || window_size <= 1)
    {
        return;
    }

    int num_points = sequence.size();
    int num_dimensions = sequence[0].size();
    std::vector<std::vector<float>> filtered_sequence(num_points, std::vector<float>(num_dimensions, 0.0f));

    for (int i = 0; i < num_points; ++i)
    {
        for (int j = 0; j < num_dimensions; ++j)
        {
            int window_start = std::max(0, i - window_size / 2);
            int window_end = std::min(num_points - 1, i + window_size / 2);

            float sum = 0;
            for (int k = window_start; k <= window_end; ++k)
            {
                sum += sequence[k][j];
            }
            filtered_sequence[i][j] = sum / (window_end - window_start + 1);
        }
    }
    sequence = filtered_sequence;
}

void exponential_moving_average(std::vector<std::vector<float>> &sequence, float alpha = 0.2f)
{

    if (sequence.empty() || alpha < 0 || alpha > 1)
    {
        return;
    }

    int num_points = sequence.size();
    int num_dimensions = sequence[0].size();
    std::vector<std::vector<float>> filtered_sequence = sequence;

    for (int i = 1; i < num_points; ++i)
    {
        for (int j = 0; j < num_dimensions; ++j)
        {
            filtered_sequence[i][j] = alpha * sequence[i][j] + (1.0f - alpha) * filtered_sequence[i - 1][j];
        }
    }
    sequence = filtered_sequence;
}

void median_filter(std::vector<std::vector<float>> &sequence, int window_size = 3)
{

    if (sequence.empty() || window_size <= 1)
    {
        return;
    }

    int num_points = sequence.size();
    int num_dimensions = sequence[0].size();
    std::vector<std::vector<float>> filtered_sequence(num_points, std::vector<float>(num_dimensions, 0.0f));
    std::vector<float> window;
    window.reserve(window_size);

    for (int i = 0; i < num_points; ++i)
    {
        for (int j = 0; j < num_dimensions; ++j)
        {
            window.clear();
            int window_start = std::max(0, i - window_size / 2);
            int window_end = std::min(num_points - 1, i + window_size / 2);

            for (int k = window_start; k <= window_end; ++k)
            {
                window.push_back(sequence[k][j]);
            }
            std::sort(window.begin(), window.end());
            filtered_sequence[i][j] = window[window.size() / 2];
        }
    }
    sequence = filtered_sequence;
}

void kalman_filter(std::vector<std::vector<float>> &sequence, float process_noise = 0.001f, float measurement_noise = 0.1f)
{

    if (sequence.empty())
    {
        return;
    }

    int num_points = sequence.size();
    int num_dimensions = sequence[0].size();
    std::vector<std::vector<float>> filtered_sequence = sequence;

    // Initialize Kalman filter parameters
    std::vector<float> predicted_state(num_dimensions, 0.0f);
    std::vector<float> prediction_error(num_dimensions, 1.0f);

    for (int i = 0; i < num_points; ++i)
    {
        for (int j = 0; j < num_dimensions; ++j)
        {
            // Predict step
            float predicted_error = prediction_error[j] + process_noise;

            // Update step
            float kalman_gain = predicted_error / (predicted_error + measurement_noise);
            filtered_sequence[i][j] = predicted_state[j] + kalman_gain * (sequence[i][j] - predicted_state[j]);
            prediction_error[j] = (1 - kalman_gain) * predicted_error;

            // Update prediction for next iteration
            predicted_state[j] = filtered_sequence[i][j];
        }
    }

    sequence = filtered_sequence;
}

void weighted_moving_average(std::vector<std::vector<float>> &sequence, int window_size = 3)
{

    if (sequence.empty() || window_size <= 1)
    {    
        return;
    }

    int num_points = sequence.size();
    int num_dimensions = sequence[0].size();
    std::vector<std::vector<float>> filtered_sequence(num_points, std::vector<float>(num_dimensions, 0.0f));

    // Create weights (triangular window)
    std::vector<float> weights(window_size);
    float weight_sum = 0;
    for (int i = 0; i < window_size; ++i)
    {
        weights[i] = 1.0f + std::min(i, window_size - 1 - i);
        weight_sum += weights[i];
    }

    for (int i = 0; i < num_points; ++i)
    {
        for (int j = 0; j < num_dimensions; ++j)
        {
            float weighted_sum = 0;
            float actual_weight_sum = 0;

            int window_start = std::max(0, i - window_size / 2);
            int window_end = std::min(num_points - 1, i + window_size / 2);

            for (int k = window_start; k <= window_end; ++k)
            {
                int weight_index = k - window_start;
                weighted_sum += sequence[k][j] * weights[weight_index];
                actual_weight_sum += weights[weight_index];
            }

            filtered_sequence[i][j] = weighted_sum / actual_weight_sum;
        }
    }

    sequence = filtered_sequence;
    
}
// helper - TODO delete these later from source code

int main()
{

    while (1)
    {
    }
}