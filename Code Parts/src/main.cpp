// Door Lock System by
// Eros Kuikel <ek3296@nyu.edu>, 

// Outline of the Code
// -------------------------------
// Red LED State:
// - Off if not recording code
// - On if recording code

// Neo Pixels State:
// - Static blue when device is ready to be unlocked
// - Static yellow while entering unlock code
// - Static red if failed unlock
// - Static green if successful unlock

// Recording new code:
// - Start recording by pressing left button

// Attempting to unlock:
// - Start unlock attempt by pressing right button

#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>

#define LED_PORT PORTC
#define LED_DDR DDRC
#define LED_PIN PORTC7

#define R_BTN_PIN PINF
#define R_BTN_DDR DDRF
#define R_BTN_BIT PF6

#define L_BTN_PIN PIND
#define L_BTN_DDR DDRD
#define L_BTN_BIT PD4

#define MAX_TIME 3500     // Maximum recording time in milliseconds
#define POLLING_DELAY 70 // Delay between consecutive pollings of the accelerometer in milliseconds

#define THRESHOLD 2 // Threshold for determining comparision match

#define BLUE 0x0000FF
#define TEAL 0x008080
#define GREEN 0x00FF00
#define RED 0xFF0000
#define YELLOW 0x808000

// Struct to store data from the three axis accelerometer
struct axis_data
{
  float x;
  float y;
  float z;
};

// Struct to store three axis accelerometer data
struct axis_array
{
  float x_arr[(int)(MAX_TIME / POLLING_DELAY)] = {0};
  float y_arr[(int)(MAX_TIME / POLLING_DELAY)] = {0};
  float z_arr[(int)(MAX_TIME / POLLING_DELAY)] = {0};
  int size = (int)(MAX_TIME / POLLING_DELAY);
};

// Static struct to hold current accelerometer reading
static axis_data acc_data;

// Static struct to hold unlock code
static axis_array raw_unlock_code;

// Static struct to hold attempted unlock pattern
static axis_array raw_attempted_pattern;


// Static variable to store state of device
/*
State 0: No unlock code set (Goes to State 1)
State 1: Recording unlock code (Goes to State 2)
State 2: Unlock code set (Goes State 1 or 3)
State 3: Recording input attempt (Goes to State 4 or 5)
State 4: Failed attempt (Goes to State 2)
State 5: Successful attempt (Goes to State 2)
*/
static uint8_t state;
static uint8_t next_state;

void setup_io()
{
  // Setup LED_PORT as an output
  LED_DDR |= (1 << LED_PIN);

  // Setup Buttons as an input
  R_BTN_DDR |= ~(1 << R_BTN_BIT);
  L_BTN_DDR |= ~(1 << L_BTN_BIT);
}

// Function to read data from the three axis accelerometer
void get_acc_data()
{
  acc_data.x = CircuitPlayground.motionX();
  acc_data.y = CircuitPlayground.motionY();
  acc_data.z = CircuitPlayground.motionZ();
}

void record_data(size_t index, axis_array &data)
{
  get_acc_data();
  data.x_arr[index] = acc_data.x;
  data.y_arr[index] = acc_data.y;
  data.z_arr[index] = acc_data.z;
}

void record_code(axis_array &data, int i)
{
  record_data(i, data);
  delay(POLLING_DELAY);
}

void clear_buffer(axis_array &data, int &i)
{
  data.x_arr[i] = 0.0;
  data.y_arr[i] = 0.0;
  data.z_arr[i] = 0.0;
}

void clear_all_buffers(int &i){
  clear_buffer(raw_unlock_code,i);
  clear_buffer(raw_attempted_pattern,i);
}

// Function to calculate the moving average of accelerometer data in-place
void calculate_moving_average(axis_array& data, int window_size) {
  // Check if window size is larger than data size. If so, return without any calculation.
  if (window_size > data.size) return;

  // Create three variables to store the sum of the window for x, y, and z
  float sum_x = 0, sum_y = 0, sum_z = 0;

  // Calculate the initial sum for the first window
  for (int i = 0; i < window_size; i++) {
    sum_x += data.x_arr[i];
    sum_y += data.y_arr[i];
    sum_z += data.z_arr[i];
  }

  // Calculate the average for the first window and assign it to the middle element
  data.x_arr[window_size / 2] = sum_x / window_size;
  data.y_arr[window_size / 2] = sum_y / window_size;
  data.z_arr[window_size / 2] = sum_z / window_size;

  // Traverse the rest of the array
  for (int i = window_size; i < data.size; i++) {
    // Update the sums by subtracting the first element of the last window and adding the next element
    sum_x = sum_x - data.x_arr[i - window_size] + data.x_arr[i];
    sum_y = sum_y - data.y_arr[i - window_size] + data.y_arr[i];
    sum_z = sum_z - data.z_arr[i - window_size] + data.z_arr[i];

    // Calculate the average and assign it to the middle element of the current window
    data.x_arr[i - window_size / 2] = sum_x / window_size;
    data.y_arr[i - window_size / 2] = sum_y / window_size;
    data.z_arr[i - window_size / 2] = sum_z / window_size;
  }

  // At the end of the array, the remaining elements do not have a full window
  // Assign them the average of the last full window
  for (int i = data.size - window_size / 2; i < data.size; i++) {
    data.x_arr[i] = sum_x / window_size;
    data.y_arr[i] = sum_y / window_size;
    data.z_arr[i] = sum_z / window_size;
  }
}


// Function to truncate the initial and final part of the array for window_size
void truncate(axis_array& data, int window_size) {
  // Check if window size is larger than half of the data size.
  // If so, return without doing anything, as we cannot truncate more than half of the data.
  if (window_size * 2 > data.size) return;

  // Calculate the new size of the data
  int new_size = data.size - window_size * 2;

  // Shift the data from the positions after the window to the start of the arrays
  for (int i = 0; i < new_size; i++) {
    data.x_arr[i] = data.x_arr[i + window_size];
    data.y_arr[i] = data.y_arr[i + window_size];
    data.z_arr[i] = data.z_arr[i + window_size];
  }

  // Update the size of the data
  data.size = new_size;
}

bool compare(axis_array& data1,const axis_array& unlock_pattern, float threshold) {
  // Check if the sizes of both arrays are equal. If not, return false.
  if (data1.size != unlock_pattern.size) {
    Serial.println("Sizes of the two arrays are not equal. Cannot compare.");
    return false;
  }
  // Calculate the square of the Euclidean distance
  float sum = 0;
  // Iterate over each element in the array
  for (int i = 0; i < data1.size; i++) {
    // Calculate the square of the difference for each axis
    float diff_x = data1.x_arr[i] - unlock_pattern.x_arr[i];
    float diff_y = data1.y_arr[i] - unlock_pattern.y_arr[i];
    float diff_z = data1.z_arr[i] - unlock_pattern.z_arr[i];
    sum += sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
  }

  // Calculate the average distance
  float avg_dist = sum / data1.size;

  // If the square of the Euclidean distance is more than the square of the threshold, return false.
  // We compare the squares instead of the actual values to avoid the computational cost of taking square roots.
  // Print the difference
  Serial.print("AVG Difference: ");
  Serial.print(avg_dist);
  if (avg_dist > threshold) {
    return false;
  }

  // If none of the distances exceed the threshold, return true.
  Serial.println("\nUnlocked.");
  return true;
}

void state_control()
{
  int window_size = 10; //This is for filter function
  bool l_press = L_BTN_PIN & (1 << L_BTN_BIT);
  bool r_press = R_BTN_PIN & (1 << R_BTN_BIT);
  Serial.print(state);
  // Print a new line
  Serial.println();
  switch (state)
  {

  // No unlock code set. Goes to State 1 upon left button press
  case 0:
  {

    // Turn LED off
    LED_PORT &= ~(1 << LED_PIN);

    // Turn NeoPixels off
    CircuitPlayground.clearPixels();

    // Check for left button press to record code
    if (l_press)
    {
      CircuitPlayground.playTone(1000, 100);
      next_state = 1;
      delay(500);
    }
  }
  break;

  // Recording unlock code. Goes to State 2 after recording
  case 1:
  {
    // Clear all buffers
    int i = 0;
    while ((i < (int)(MAX_TIME / POLLING_DELAY)))
    {
      clear_all_buffers(i);
      i++;
    }
    
    // Turn LED On
    LED_PORT |= (1 << LED_PIN);

    // Turn NeoPixels off
    CircuitPlayground.clearPixels();

    // Record Code
    i = 0;
    while ((i < (int)(MAX_TIME / POLLING_DELAY)) & (!l_press))
    {
      record_code(raw_unlock_code, i);
      l_press = L_BTN_PIN & (1 << L_BTN_BIT);
      i++;
    }
    raw_unlock_code.size = i;

    // ADD CODE TO FILTER DATA
    calculate_moving_average(raw_unlock_code, window_size);
    // ADD CODE TO TRUNCATE DATA
    truncate(raw_unlock_code, window_size);
    
    next_state = 2;
  }
  break;

  // Unlock code set Goes to State 1 if left button is pressed. Goes to State 3 if right button is pressed
  case 2:
  {

    // Turn LED Off
    LED_PORT &= ~(1 << LED_PIN);

    // Turn NeoPixels Blue
    for (int i = 0; i < 10; i++)
    {
      CircuitPlayground.setPixelColor(i, BLUE);
    }

    // Go to State 1 if left button is pressed
    if (l_press)
    {
      next_state = 1;
      delay(500);
    }

    // Go to State 3 if right button is pressed
    else if (r_press)
    {
      next_state = 3;
      delay(500);
    }

    // Stay in State 2 if no button is pressed
    else
      next_state = 2;
  }
  break;

  // Recording input attempt. Goes to State 4 if unlock fails. Goes to State 5 if unlock is successful
  case 3:
  {

    // Turn LED Off
    LED_PORT &= ~(1 << LED_PIN);

    // Turn NeoPixels Yellow
    for (int i = 0; i < 10; i++)
    {
      CircuitPlayground.setPixelColor(i, YELLOW);
    }
    int i = 0; // added the declaration of i
    while ((i < (int)(MAX_TIME / POLLING_DELAY)) & (!r_press))
    {
      record_code(raw_attempted_pattern, i);
      r_press = R_BTN_PIN & (1 << R_BTN_BIT);
      i++;
    }
    raw_attempted_pattern.size = i;
    // ADD CODE TO FILTER DATA
    calculate_moving_average(raw_attempted_pattern, window_size);
    // ADD CODE TO TRUNCATE DATA
    truncate(raw_attempted_pattern, window_size);
    static bool match = 0; 
    // ADD COMPARISON OUTPUT HERE
    match = compare(raw_attempted_pattern, raw_unlock_code, THRESHOLD);
    if (match)
      next_state = 5;
    else
      next_state = 4;
  }
  break;

  // Failed attempt. Goes to State 2 after 5 seconds.
  case 4:
  {
    // Turn LED Off
    LED_PORT &= ~(1 << LED_PIN);

    // Turn NeoPixels Red
    for (int i = 0; i < 10; i++)
    {
      CircuitPlayground.setPixelColor(i, RED);
    }
    delay(5000);
    next_state = 2;
  }
  break;

  // Successful attempt. Goes to State 2 after 5 seconds.
  case 5:
  {
    // Turn LED Off
    LED_PORT &= ~(1 << LED_PIN);

    // Turn NeoPixels Green
    for (int i = 0; i < 10; i++)
    {
      CircuitPlayground.setPixelColor(i, GREEN);
    }

    delay(5000);
    next_state = 2;
  }
  break;
  }
}

void setup()
{
  CircuitPlayground.begin();
  state = 0;
  next_state = 0;
}

void loop()
{
  state = next_state;
  state_control();
}
