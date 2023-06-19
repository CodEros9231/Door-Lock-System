// Rolling Average Test Code by Pi Ko
// Compile and test with
// g++ -std=c++11 rolling-average-test.cpp -o rolling-average-test
// Generate the test data with
// ./rolling-average-test>data.csv
// Then plot the data with the matlab code given in the same directory (plottest.m)
// -------------------------------------------------
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <cmath>

#define LED_PORT PORTC
#define LED_DDR DDRC
#define LED_PIN PORTC7

#define MAX_TIME 5000 //Maximum recording time in milliseconds
#define POLLING_DELAY 10 //Delay between consecuitive pollings of the accelerometer in milliseconds

#define THREASHOLD 0 //Threshold for determining comparision match

//Struct to store data from the three axis accelerometer
struct axis_data {
  float x;
  float y;
  float z;
};

//Struct to store three axis accelerometer data
struct axis_array {
  float x_arr [(int) (MAX_TIME / POLLING_DELAY)] = {0};
  float y_arr [(int) (MAX_TIME / POLLING_DELAY)] = {0};
  float z_arr [(int) (MAX_TIME / POLLING_DELAY)] = {0};
  int size = (int) (MAX_TIME / POLLING_DELAY);
};

//Static struct to hold current accelerometer reading
static axis_data acc_data;

//Static struct to hold unlock code
static axis_array raw_unlock_code;

//Static struct to hold filtered unlock code
static axis_array filtered_unlock_code;

//Static struct to hold filtered normalized unlock code
static axis_array norm_unlock_code;

//Static struct to hold attempted unlock pattern
static axis_array raw_attempted_pattern;

//Static struct to hold filtered attempted unlock pattern
static axis_array filtered_attempted_pattern;

//Static struct to hold filtered normalized attempted unlock pattern
static axis_array norm_attempted_pattern;



// Function to calculate the moving average of accelerometer data
void calculate_moving_average(const axis_array& input_data, axis_array& output_data, int window_size) {
  // Calculate moving average for X-axis
  for (int i = 0; i < input_data.size - window_size + 1; i++) {
    float sum_x = 0;
    for (int j = i; j < i + window_size; j++) {
      sum_x += input_data.x_arr[j];
    }
    output_data.x_arr[i] = sum_x / window_size;
  }

  // Calculate moving average for Y-axis
  for (int i = 0; i < input_data.size - window_size + 1; i++) {
    float sum_y = 0;
    for (int j = i; j < i + window_size; j++) {
      sum_y += input_data.y_arr[j];
    }
    output_data.y_arr[i] = sum_y / window_size;
  }

  // Calculate moving average for Z-axis
  for (int i = 0; i < input_data.size - window_size + 1; i++) {
    float sum_z = 0;
    for (int j = i; j < i + window_size; j++) {
      sum_z += input_data.z_arr[j];
    }
    output_data.z_arr[i] = sum_z / window_size;
  }

  // Set the size of the output array to be equal to the size of the input array minus window size plus 1
  output_data.size = input_data.size - window_size + 1;
}


float noise(float amplitude) {
  return amplitude * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 - 1);
}

int main() {
  srand(time(0)); // Seed the random number generator

  // Generate random initial data for raw_unlock_code
  const float noise_amplitude = 0.1;
  for (int i = 0; i < raw_unlock_code.size; i++) {
    float t = static_cast<float>(i) / static_cast<float>(raw_unlock_code.size - 1);
    raw_unlock_code.x_arr[i] = std::cos(2 * M_PI * t) + noise(noise_amplitude);
    raw_unlock_code.y_arr[i] = std::sin(2 * M_PI * t) + noise(noise_amplitude);
    raw_unlock_code.z_arr[i] = t + noise(noise_amplitude);
  }

  // Calculate the moving average of the random initial data
  int window_size = 5;
  calculate_moving_average(raw_unlock_code, filtered_unlock_code, window_size);

  // Output the data in CSV format
  std::cout << "Index,Raw_X,Raw_Y,Raw_Z,Filtered_X,Filtered_Y,Filtered_Z\n";
  for (int i = 0; i < filtered_unlock_code.size; i++) {
    std::cout << std::fixed << std::setprecision(3)
              << i << ","
              << raw_unlock_code.x_arr[i] << ","
              << raw_unlock_code.y_arr[i] << ","
              << raw_unlock_code.z_arr[i] << ","
              << filtered_unlock_code.x_arr[i] << ","
              << filtered_unlock_code.y_arr[i] << ","
              << filtered_unlock_code.z_arr[i] << std::endl;
  }

  return 0;
}
