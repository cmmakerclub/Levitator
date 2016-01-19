/*
  magnetic levitation
  Chiangmai Maker Club
  modified 05-12-2014
  by wasin wongkum
  sensor hall sensor A1302
  mosfet irf 540n
  supply 12v
  coil 10 ohm
*/

#define hall        A0
#define magnetic    15
#define filter      0.70f
#define period_pid    5.0f
#define period_debug  1000.0f

float state = 0;
float state_f = 0;
float prev_state = 0;
float prev_state_f = 0;
float ref = 7;
float error = 0;
float prev_error = 0;
float error_dot = 0;
float error_sum = 0;

float kp = 500*4;
float ki = 30;
float kd = 3.65*50;
float output  = 0;

uint32_t time_now, time_prev_pid, time_prev_print;

void setup()
{
  Serial.begin(115200);
  pinMode(hall, INPUT);
  pinMode(output, OUTPUT);
  pinMode(16, OUTPUT);
  time_now = millis();
  time_prev_pid = time_now;
  time_prev_print = time_now;
}

void loop()
{
  time_now = millis();
  if (time_now - time_prev_pid >= period_pid)
  {
    time_prev_pid = time_now;

    PID();
  }
  if (time_now - time_prev_print >= period_debug)
  {
    time_prev_print = time_now;

    Debug();
  }


}
void Debug(void)
{
  Serial.print("Ref ");
  Serial.print(ref);
  Serial.print("     ");
  Serial.print("State ");
  Serial.print(state);
  Serial.print("     ");
  Serial.print("Error ");
  Serial.print(error);
  Serial.print("     ");
  Serial.print("Error_sum ");
  Serial.print(error_sum);
  Serial.print("     ");
  Serial.print("Error_dot ");
  Serial.print(error_dot);
  Serial.print("     ");
  Serial.print("Output ");
  Serial.println(output);
}
void PID(void)
{
  digitalWrite(16, 0);
  /* if fix referent */
  ref = 0.8  ;

  /*  Update Kd gain  */
  kd = 3.65 + ref * 3.6f ;
  ref = 6.3f + ref;

  prev_state = state ;
  state = prev_state + (filter * (((float)(analogRead(hall)) / 100.0f) - prev_state));

  prev_error = error;
  error = ref - state;
  error_dot = (error - prev_error) * 200.0f;
  error_sum = error_sum + error / 200.0f;

  output = kp * error + ki * error_sum + kd * error_dot + 220;

  if (output < 0)   output = 0;
  if (output > 1023) output = 1023;

  if (state < 6)
  {
    output = 0;
    error_sum = 0;
  }
  //  output = 0;
  analogWrite(magnetic, output);
  digitalWrite(16, 1);
}
