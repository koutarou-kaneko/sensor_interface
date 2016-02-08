// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 57600

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON false  // true or false

boolean output_errors = false;  // true or false


unsigned long timestamp;
unsigned long timestamp_old;

boolean output_stream_on;
boolean output_single_on;


// setup for com
void com_setup()
{
  Serial.begin(OUTPUT__BAUD_RATE);
 
}

// loop func for com
void com_loop()
{
  // Read incoming control messages
  if (Serial.available() >= 2)
    {
    if (Serial.read() == '#') // Start of new control message
    {
      int command = Serial.read(); // Commands
      if (command == 'f') // request one output _f_rame
        output_single_on = true;
      else if (command == 's') // _s_ynch request
      {
        // Read ID
        byte id[2];
        id[0] = readChar();
        id[1] = readChar();
        
        // Reply with synch message
        Serial.print("#SYNCH");
        Serial.write(id, 2);
        Serial.println();
      }
      else if (command == 'o') // Set _o_utput mode
      {
        char output_param = readChar();
        if (output_param == 'n')  // n: calibration for magnetic
        {
          if(!f.ARMED) f.CALIBRATE_MAG = 1;
        }
        else if (output_param == 't') // Output angles as _t_ext
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_TEXT;
        }
        else if (output_param == 'b') // Output angles in _b_inary format
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == 'c') //c: calibration for acceleromet and gyro
        {
          if(!f.ARMED) calibratingA=512;
        }
        else if (output_param == '0') // Disable continuous streaming output
        {
          turn_output_stream_off();
 
        }
        else if (output_param == '1') // Enable continuous streaming output
        {
 
          turn_output_stream_on();
        }
      }
    }
    else
    { } // Skip character
  }

  // Time to read the sensors again?
  if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = millis();

    if (output_mode == OUTPUT__MODE_ANGLES)  // Output angles
    {
      if (output_stream_on || output_single_on) output_angles();
    }
    
    output_single_on = false;
    
  }
}


// Blocks until another byte is available on serial port
char readChar()
{
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}


void turn_output_stream_on()
{
  output_stream_on = true;

}

void turn_output_stream_off()
{
  output_stream_on = false;

}


void output_angles()
{
  if (output_format == OUTPUT__FORMAT_BINARY)
  {
    float ypr[3];  
    ypr[0] = (float)att.heading;
    ypr[1] =  (float)0.1*att.angle[1];
    ypr[2] =  (float)0.1*att.angle[0];
    Serial.write((byte*) ypr, 12);  // No new-line
  }
  else if (output_format == OUTPUT__FORMAT_TEXT)
  {
    Serial.print("#YPR=");
    Serial.print((float)att.heading); Serial.print(",");
    Serial.print((float)0.1*att.angle[1]); Serial.print(",");
    Serial.print((float)0.1*att.angle[0]); Serial.println();
  }
}

