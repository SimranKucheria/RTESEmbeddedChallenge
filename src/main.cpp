//Group 29 Princy Doshi(pd2672), Ansh Sarkar(as20363) and Simran Kucheria(sk11645)
// We have used the gyroscope to define what a gesture sequence will be and use that data to validate as well.
// The approach we have taken uses preprocessing steps such as normalising and calibrating the gyro data and then passing it through a filter to smoothen it out.
// After this we had to compare the sensor data using some sort of distance metric as we can consider the gyro data as vectors.
// After a bit of research we decided to implement the DTW algorithm which is usually effective for such scenarios.
// For filters we tried different variations whose implmentations are present in the code, but we decided to go ahead with moving average as that was giving us the best results.

// Inclding libraries
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
std::vector<std::vector<std::vector<float>> > all_user_GT;


// Objects
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
DigitalOut led1(LED2);
DigitalIn button(BUTTON1); // Button input
EventFlags flags;
LCD_DISCO_F429ZI lcd;
Mutex lcd_mutex;
std::string ui_background_color = "BLUE";
std::string header = "Menu";

volatile int user_profiles = 0;
volatile int current_user = 0;
volatile int new_user = 0;
volatile int button_routine = 0;

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
    ground_truth_sequences.clear();

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
    test_sequences.clear();
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

// Apply the pre-processing steps
void preprocess_recorded_sequence(std::vector<std::vector<float>> &recorded_sequence)
{
    normalize_sequence(recorded_sequence);
    // normalize_sequence(validation_sequence);

    calibrate_gyro_using_initial_position(recorded_sequence);
    remove_noise_from_datapoints(recorded_sequence, 0.0001);

    // calibrate_gyro_using_initial_position(validation_sequence);
    // remove_noise_from_datapoints(validation_sequence, 0.0001);

    moving_average_filter(recorded_sequence, 5);
    // moving_average_filter(validation_sequence, 5);
}

void execute_preprocessing_steps(std::vector<std::vector<float>> &recorded_sequence, std::vector<std::vector<float>> &validation_sequence)
{
    // normalize_sequence(recorded_sequence);
    normalize_sequence(validation_sequence);

    // calibrate_gyro_using_initial_position(recorded_sequence);
    // remove_noise_from_datapoints(recorded_sequence, 0.0001);

    calibrate_gyro_using_initial_position(validation_sequence);
    remove_noise_from_datapoints(validation_sequence, 0.0001);

    // moving_average_filter(recorded_sequence, 5);
    moving_average_filter(validation_sequence, 5);
}

// Validation function for previously recorded gesture and new gesture
// Helper function to calculate the distance
float euclidean_distance(const std::vector<float> &a, const std::vector<float> &b)
{
    float sum = 0;
    for (size_t i = 0; i < a.size(); i++)
    {
        float diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

float validate_using_dtw(std::vector<std::vector<float>> &recorded_sequence, std::vector<std::vector<float>> &validation_sequence)
{
    execute_preprocessing_steps(recorded_sequence, validation_sequence); 

    int n = recorded_sequence.size();
    int m = validation_sequence.size();
    std::vector<std::vector<float>> dtw_matrix(n + 1, std::vector<float>(m + 1, std::numeric_limits<float>::max()));

    // Initialize first element
    dtw_matrix[0][0] = 0;

    // Fill DTW matrix
    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= m; ++j) {
        float cost = 0;
        for (int k = 0; k < 3; ++k)
        {
            cost += std::abs(recorded_sequence[i - 1][k] - validation_sequence[j - 1][k]);
        }
        dtw_matrix[i][j] = cost + std::min({dtw_matrix[i - 1][j], dtw_matrix[i][j - 1], dtw_matrix[i - 1][j - 1]});
        }
  }

    float final_distance = dtw_matrix[n][m];
    printf("DTW Distance: %.4f\n", final_distance);
    return final_distance;
}
//UI and Button code for the user to interact with the device
void ButtonLoop(){
    bool button_pressed = false;  // button state
    int button_hold_time = 0;  // button time
    while (true){
        if(button_routine){
            bool current_state = button.read();
            if (current_state && !button_pressed) {   // Button was just pressed
                button_pressed = true;
                button_hold_time = 0;
                
            } else if (current_state && button_pressed) {  // Button is being held down
                button_hold_time++;
            }    
            else if (!current_state && button_pressed) {  // Button was just released
                if (button_hold_time < 20 && !new_user) {  // button released under 4 seconds so enter unlock mode
                    //Evaluate with Recorded GT
                    header = "Recording";
                    validate_sequence();
                    header = "Validating";
                    float deviation = validate_using_dtw(all_user_GT[current_user-1],test_sequences);
                    if(deviation <= 110.0f){
                        header = "Unlocked";
                        ui_background_color = "GREEN";
                        ThisThread::sleep_for(3000ms);
                        header = "Menu";
                        ui_background_color = "BLUE";
                        button_hold_time = 0;
                        button_pressed = false;
                        button_routine = 0;
                    }
                    else{
                        header = "Failed";
                        ui_background_color = "RED";
                        ThisThread::sleep_for(3000ms);
                        header = "Menu";
                        ui_background_color = "BLUE";
                        button_hold_time = 0;
                        button_pressed = false;
                        button_routine = 0;
                
                    }
                    

                }
                else{  // if button held down for 4 seconds then enter record mode      
                
                    //Record GT
                    header = "Recording";
                    record_gesture_sequence();
                    preprocess_recorded_sequence(ground_truth_sequences);
                    header = "Recorded Successfully";
                    all_user_GT.push_back(ground_truth_sequences);
                    ThisThread::sleep_for(3000ms);
                    if(new_user){
                        new_user = 0;
                    }
                    header = "Menu";
                    ui_background_color = "BLUE";
                    button_hold_time = 0;
                    button_pressed = false;
                    button_routine = 0 ;
                }
            }
            else{
                continue;
            } 
        }    
        ThisThread::sleep_for(100ms);
    }
}

void DisplayLoop() {
    while (true) {
        if (ui_background_color == "GREEN") {
            lcd.Clear(LCD_COLOR_GREEN);
            lcd.SetBackColor(LCD_COLOR_GREEN);
        } else if (ui_background_color == "RED"){
            lcd.Clear(LCD_COLOR_RED);
            lcd.SetBackColor(LCD_COLOR_RED);
        }
        else{
            lcd.Clear(LCD_COLOR_BLUE);
            lcd.SetBackColor(LCD_COLOR_BLUE);
        } 

        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.DisplayStringAt(0, LINE(1), (uint8_t *)header.c_str(), CENTER_MODE);
        if (header == "Menu") {
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"CREATE USER", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"USER PROFILES", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"DELETE USER", CENTER_MODE);
            
            // Draw horizontal lines
            lcd.DrawHLine(0, 0, lcd.GetXSize());
            lcd.DrawHLine(0, lcd.GetYSize() - 1, lcd.GetXSize());
            lcd.DrawHLine(0, 40, lcd.GetXSize());
            lcd.DrawHLine(0, 290, lcd.GetXSize());
        }
        else if (header == "Create User") {
            if (user_profiles == 4) {
                lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"CANNOT CREATE MORE", CENTER_MODE);
                lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"PLEASE DELETE", CENTER_MODE);
                lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"BACK TO MENU", CENTER_MODE);
                ThisThread::sleep_for(1000ms);
                header = "Menu";
            }
            else {
                std::string message = "CREATED USER " + std::to_string(user_profiles + 1);
                lcd.DisplayStringAt(0, LINE(5), (uint8_t *)message.c_str(), CENTER_MODE);
                user_profiles++;
                
                current_user = user_profiles;
                new_user = 1;
                ThisThread::sleep_for(1000ms);
                header = "New User Recording";
                button_routine = 1;

            }

        }
        else if (header == "Delete User") {
            if (user_profiles <= 0) {
                lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"NO USERS TO DELETE", CENTER_MODE);
                lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"GOING BACK TO MENU", CENTER_MODE);
                ThisThread::sleep_for(2000ms);
                header = "Menu";
            }
            else {
                std::string message = "DELETED USER " + std::to_string(user_profiles);
                lcd.DisplayStringAt(0, LINE(10), (uint8_t *)message.c_str(), CENTER_MODE);
                all_user_GT.pop_back();
                user_profiles--;
                lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"GOING BACK TO MENU", CENTER_MODE);
                ThisThread::sleep_for(2000ms);
                header = "Menu";
            }
        }
        else if (header == "User Profiles") {
            if (user_profiles == 0) {
                lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"PLEASE ADD USER", CENTER_MODE);
                ThisThread::sleep_for(2000ms);
                header = "Menu";
            }
            else {
                for (int8_t i = 1; i <= user_profiles; i++) {
                    std::string profile = "USER PROFILE " + std::to_string(i);
                    lcd.DisplayStringAt(0, LINE((i * 3) + 1), (uint8_t *)profile.c_str(), CENTER_MODE);
                }
                lcd.DisplayStringAt(0, LINE(19), (uint8_t *)"MENU", CENTER_MODE);
                
            }
        }
        else if (header == "User") {
            std::string userprofile = "HELLO USER" + std::to_string(current_user);
            lcd.DisplayStringAt(0, LINE(1), (uint8_t *)userprofile.c_str(),CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"PRESS BUTTON",CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"TO VALIDATE GESTURE", CENTER_MODE);
            button_routine = 1;
            
        }
        else if (header == "New User Recording") {
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"LONG PRESS BUTTON",CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"TO RECORD GESTURE", CENTER_MODE);
            
                
        }
        ThisThread::sleep_for(100ms);
    }
}


void DynamicLoop() {
    uint8_t status = BSP_TS_Init(lcd.GetXSize(), lcd.GetYSize());
    
    if (status != TS_OK) {
        lcd_mutex.lock();
        lcd.SetBackColor(LCD_COLOR_WHITE);
        lcd.SetTextColor(LCD_COLOR_RED);
        lcd.DisplayStringAt(0, lcd.GetYSize() - 95, (uint8_t *)"ERROR", CENTER_MODE);
        lcd.DisplayStringAt(0, lcd.GetYSize() - 80, (uint8_t *)"Touchscreen cannot be initialized", CENTER_MODE);
        lcd_mutex.unlock();
        return;
    }

    TS_StateTypeDef ts_state;
    while (true) {
        BSP_TS_GetState(&ts_state);
        if (ts_state.TouchDetected) {
            
            if (header == "Menu") {
                if (ts_state.Y < 80 && ts_state.Y >= 15) {
                    header = "Delete User";
                } else if (ts_state.Y < 164 && ts_state.Y >=85) {
                    header = "User Profiles";
                } else if (ts_state.Y < 250 && ts_state.Y >= 165) {
                    header = "Create User";
                }
            }
            else if(header == "User Profiles" && user_profiles > 0){
                if (ts_state.Y < 250 && ts_state.Y >= 200 && user_profiles >=1) {
                    current_user = 1;
                    header = "User";
                    button_routine = 1;
                } else if (ts_state.Y < 200 && ts_state.Y >= 150 && user_profiles >=2) {
                    current_user = 2;
                    header = "User";
                    button_routine = 1;
                } else if (ts_state.Y < 150 && ts_state.Y >= 100 && user_profiles >=3) {
                    current_user = 3;
                    header = "User";
                    button_routine = 1;
                }
                else if (ts_state.Y < 100 && ts_state.Y >= 20 && user_profiles ==4) {
                    current_user = 4;
                    header = "User";
                    button_routine = 1;
                }
                else if (ts_state.Y < 20) {
                    header = "Menu";
                }
                else{
                    continue;
                }
            }
            
        }
        ThisThread::sleep_for(100ms);
    }
    
}

int main()
{
    configure_gyroscope();
    Thread display_thread;
    Thread dynamic_thread;
    Thread button_thread;
    
    display_thread.start(callback(DisplayLoop));
    dynamic_thread.start(callback(DynamicLoop));
    button_thread.start(callback(ButtonLoop));

    while(1)
    {
    }
}
