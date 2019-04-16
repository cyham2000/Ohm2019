int low_thresh = 696; // 696 is roughly 3.4 volts for threshold for low voltage on a LiPo cell. When scaled down, byte 174 will be the threshold
int high_thresh = 921; // Corresponds to about 4.5V

// Drive Mode manual or auto
bool automode;

// For monitoring pause, kill, e-stop
int estop= 6; // at com terminal of normal relay ESTOP
int kill = 5; // at the power+ of the normal relay KILL
int pause = 7; // at the power+ of the pause relay (must find this!) PAUSE
//pins for lights
int normal_light = 8; //tower light for normal operating conditions
int pause_light = 9;  //tower light for pause state
int kill_light = 10;  //tower light for kill state
int blink_pin = 12;   //connection for blinking lights

const double scale_factor[6] =  {
  1.00, 2.52, 3.28, 5.52, 5.50, 5.49
}; // Determined by voltage monitor voltage dividers. Scale these back up to 22.2V scale

int tempPin = A6; // Temperature readings

unsigned long past_time = 0;// These track the time between reading and sending diagnostics via serial communication
unsigned long safety_light_time = 0;
unsigned long curr_time = 1;

// *** UNFINISHED
int swap; // Tells if the batteries have been swapped. Put two limit switches in series, so only when both batteries are inserted,
// *** UNFINISHED


void setup()
{
  Serial.begin(9600);
  delay(1000);
  pinMode(cell_voltage_LED, OUTPUT);
  pinMode(cell_voltage_BUZZER, OUTPUT);
  pinMode(Mode_LED, OUTPUT);
  pinMode(Mode_LED_Power,OUTPUT);
  pinMode(estop, INPUT);
  pinMode(kill, INPUT);
  pinMode(pause, INPUT);
  pinMode(estop_LED, OUTPUT);
  pinMode(kill_LED, OUTPUT);
  pinMode(pause_LED, OUTPUT);
  pinMode(tempPin, INPUT);
  pinMode(normal_light, OUTPUT);
  pinMode(pause_light, OUTPUT);
  pinMode(kill_light, OUTPUT);
  pinMode(blink_pin, OUTPUT);
  
  for (int i = 0; i < 3; i++)
  {
    pinMode(GPS_status[i], OUTPUT);
  }

  digitalWrite(Mode_LED_Power,HIGH); // always power 4 module relay (sainsmart)
  digitalWrite(Mode_LED,HIGH); // Initialize safety light to MANUAL mode
  automode = true;
}



// Prints on serial bus in the following form: STX (1 Byte), STATES (1 Byte), CELLS (6 * 2 Bytes, 12 total. High byte, then low byte), TEMP (2 Bytes, high byte, then low byte), ETX (1 Byte)
void loop()
{
  char command;
  if (Serial.available() > 0) // check for
  {
    command = Serial.read();
    command = command - ('a' - 'A'); // Convert letter sent to uppercase
    if (command == 'M' || command == 'A')
    {
      arduino_read_state(command); // These commands have to do with the state of the robot
        if (command == 'M')
        {
           if (digitalRead(estop))
           {
              digitalWrite(pause_light,LOW);
              digitalWrite(normal_light,LOW);
              digitalWrite(kill_light,HIGH);
           }
           if(digitalRead(kill))
           {
              digitalWrite(pause_light,LOW);
              digitalWrite(normal_light,LOW);
              digitalWrite(kill_light,HIGH);
           }
           if(digitalRead(pause))
           {
              digitalWrite(normal_light,LOW);
              digitalWrite(kill_light,LOW);
              digitalWrite(pause_light,HIGH);
           }
           else
           {
              digitalWrite(pause_light,LOW);
              digitalWrite(kill_light,LOW);
              digitalWrite(normal_light,HIGH);
           }
        }
        if (command == 'A')
        {
           if(digitalRead(estop))
           {
              digitalWrite(pause_light,LOW);
              digitalWrite(normal_light,LOW);
              digitalWrite(kill_light,HIGH);
           }
           if(digitalRead(kill))
           {
              digitalWrite(pause_light,LOW);
              digitalWrite(normal_light,LOW);
              digitalWrite(kill_light,HIGH);
           }
           if(digitalRead(pause))
           {
              digitalWrite(normal_light,LOW);
              digitalWrite(kill_light,LOW);
              digitalWrite(pause_light,HIGH);
              digitalWrite(blink_pin,HIGH);
           }
           else
           {
              digitalWrite(pause_light,LOW);
              digitalWrite(kill_light,LOW);
              digitalWrite(normal_light,HIGH);
              digitalWrite(blink_pin,HIGH)
           }
        }
    }
    else if (command == 'F' || command == 'S' || command == 'O') // F = fixed (GPS has heading), S = searching (looking for heading), O == Off
    {
      GPS_Read(command);
    }
  }

    Serial.write(0x02); // 1 Byte
    check_emergency_system(); // Read and print state of emergency system. E-stop, pause, kill
    read_cells(); // Read and print cell voltages, manage warning light & buzzer
    check_temp(); // Read and print temperature
    past_time = curr_time;
    Serial.write(0x03); // 1 Byte
    delay(75);
  
  curr_time = millis();
}


// Read and report over Serial USB if robot is Off, Auto, or Manual mode. Control LED
void arduino_read_state(char cmd)
{
  if (cmd == 'm' || cmd == 'M')
  {
    digitalWrite(Mode_LED, HIGH);
    automode = false;
    Serial.println("Manual");
  }
  else if (cmd == 'a' || cmd == 'A')
  {
    Serial.println("Auto");
    automode = true;
    digitalWrite(Mode_LED, LOW);
  }
}


// Reads state of the GPS from the computer and drives RGB Led
void GPS_Read(char cmd)
{
  if (cmd == 'O')
  {
    digitalWrite(GPS_status[0], HIGH), digitalWrite(GPS_status[1], LOW), digitalWrite(GPS_status[2], LOW);
  }
  else if (cmd == 'S')
  {
    digitalWrite(GPS_status[0], LOW), digitalWrite(GPS_status[1], HIGH), digitalWrite(GPS_status[2], LOW);
  }
  else if (cmd == 'F')
  {
    digitalWrite(GPS_status[0], LOW), digitalWrite(GPS_status[1], LOW), digitalWrite(GPS_status[2], HIGH);
  }
}


// Checks and sends status of E-stop, pause and kill in the form of a byte. Pause is least-significant, then kill, then E_stop
void check_emergency_system()
{
  byte states = 0; // 00000000

  if (!digitalRead(estop)) // E-stop button
  {
    bitSet(states, 2); // 00000100
    Serial.println("Estopped");
    digitalWrite(estop_LED, LOW);
  }
  else
  {
    digitalWrite(estop_LED, HIGH);
    Serial.println("Not Estopped");
  }
  if (digitalRead(kill)) // If robot is killed
  {
    digitalWrite(kill_LED, LOW);
    bitSet(states, 1); // 00000010
    Serial.println("Killed");
  }
    else
  {
    digitalWrite(kill_LED, HIGH);
    Serial.println("Not Killed");
  }
  if (digitalRead(pause)) // If robot is paused
  {
    bitSet(states, 0); // 00000001
    Serial.println("Paused");
    digitalWrite(pause_LED, LOW);
  }
    else
  {
    Serial.println("Not Paused");
    digitalWrite(pause_LED, HIGH);
  }

  // Ex: If robot e-stop is hit, it is not killed, and it is paused, we will have 101
  Serial.write(states); // 1 byte
}

// Function reads cell voltages on the LiPo battery and sends results as a byte to laptop with serial communication
// Format of Tx communication: C6 Volt, C5 Volt, ... C1 Volt, t (termination)
// To decode cell voltages to unit Volts, Decode from hex to decimal, then multiply by 5/1024
void read_cells()
{
  int cell_readings[6] =  {
    analogRead(A8), analogRead(A1), analogRead(A2), analogRead(A3), analogRead(A4), analogRead(A5)
  };
  // Holds raw analog readings from arduino 0-1023. Pin A0 reads Cell 1, pin A1 reads Cell 2, ... pin A5 reads Cell 6

  int vLimit = 0; // How many times a lower or upper threshold has been crossed

  for (int i = 0; i < 6; i++)
  {
    cell_readings[i] = cell_readings[i] * scale_factor[i]; // Scale back up voltages (since voltage divider scales down). Units are in Volts*5/1023
  }

  for (int i = 5; i >= 0; i--) // Calculate voltages of each cell, check if they are too low or too high
  {
    if (i > 0)
    {
      cell_readings[i] = cell_readings[i] - cell_readings[i - 1]; // Each should be roughly 695 (3.7V)
    }

    if (cell_readings[i] < low_thresh || cell_readings[i] > high_thresh) // Check if cell voltage is too low or too high
    {
      vLimit++;
    }
  }

  for (int i = 0; i < 6; i++)
  {
    //Serial.write(cell_readings[i]/256); // High Byte
    //Serial.write(cell_readings[i]%256); // Low Byte

    Serial.print("Cell Difference: ");
    Serial.print(cell_readings[i], ); 
  }

  if (vLimit > 0) // Sound the alarm
  {
    digitalWrite(cell_voltage_LED, HIGH), digitalWrite(cell_voltage_BUZZER, HIGH);
  }
  else
  {
    digitalWrite(cell_voltage_LED, LOW), digitalWrite(cell_voltage_BUZZER, LOW);
  }
}

// Checks and sends temperature in the form of an int, must be converted to celsius double using:   temp_C = (((temp_C * 5000)/1024)-500)/10
void check_temp()
{
  int temp = analogRead(tempPin);
  //Serial.write(temp/256); // High Byte
  //Serial.write(temp%256); // Low Byte
  Serial.print("Temp: ");
  Serial.println(temp);
}
