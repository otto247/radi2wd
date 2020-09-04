#include <Servo.h>
#include <PS4Controller.h>

#define CH_ENABLE 3
#define PIN_ENABLE 27
#define PIN_PHASE 33
#define PIN_SERVO 32
#define PIN_TOF 34
#define PIN_TURN 16
#define PIN_TAIL 17

#define THROTTLE_MARGIN 16
#define TARGET_MIN 155
#define LIMITED_MAX 190
#define TARGET_MAX 256

#define STEP_FORWARD 2
#define STEP_BACKWARD 1
#define STEER_MAX 200
#define STEER_MIN -200
#define STEER_MARGIN 1

#define FREQUENCY 60
#define TOF_LOW  1500
#define TOF_HIGH 2500

const char* BLE_ADDRESS = "FC:F5:C4:45:83:EE";

struct control
{
  int16_t throttle = 0;
  int16_t steer = 0;
  struct
  {
    bool head = false;
    bool tail = false;
    bool hazard = false;
  } light;
  int tof = 0;
  bool boost = false;
};

struct environment
{
  int64_t steps = 0;
} g_env;

struct context
{
  int16_t center = 0;
  int16_t prev_steer = 0;
  int16_t prev_power = 0;
} g_ctx;

void debug_print( const struct control& ctl
                , const struct environment& env
                , const struct context& ctx)
{
  Serial.print("ctl:");
  Serial.printf("%4d", ctl.throttle);
  Serial.printf("%4d", ctl.steer);
  Serial.print(" ");
  Serial.printf("%c", ctl.light.tail?'T':' ');
  Serial.printf("%c", ctl.light.hazard?'Z':' ');
  Serial.printf("%c", ctl.boost?'B':' ');

  Serial.print(" env:");
  Serial.printf("%4d", ctl.tof);

  Serial.print(" ctx:");
  Serial.printf("%4d", ctx.center);
  Serial.printf("%4d", ctx.prev_steer);
  Serial.printf("%4d", ctx.prev_power);
  
  Serial.println("");
}

Servo steer;

void setup() 
{
  Serial.begin(115200);
  PS4.begin(const_cast<char*>(BLE_ADDRESS));

  ledcSetup(CH_ENABLE, 12800, 8); // frequency = 12.8kHz , precison = 8bit
  ledcAttachPin(PIN_ENABLE, CH_ENABLE);

  pinMode(PIN_PHASE, OUTPUT);
  pinMode(PIN_TAIL, OUTPUT);
  pinMode(PIN_TURN, OUTPUT);
  pinMode(PIN_TOF, INPUT);
  
  steer.attach( PIN_SERVO
              , Servo::CHANNEL_NOT_ATTACHED
              , STEER_MIN
              , STEER_MAX
              );

  Serial.println("ready");
}
 
void loop() 
{
  control ctl;

  // make input
  
  if ( !PS4.isConnected() )
  {
    ctl.throttle = 0;
    ctl.steer = 0;
    ctl.light.tail = true;
    ctl.light.hazard = true;
  }
  else
  {
    // read tof
    ctl.tof = analogRead( PIN_TOF );

    // read controller
    ctl.throttle = (int16_t)PS4.data.analog.button.r2
                   - (int16_t)PS4.data.analog.button.l2 ;
    ctl.steer = PS4.data.analog.stick.lx;

    if ( !PS4.data.button.l1 )
    {
      ctl.steer *= 0.8;
    }

    auto tof = ctl.tof - TOF_LOW;

    if ( tof > 0 && ctl.throttle >= 0 )
    {
      auto range = TOF_HIGH - TOF_LOW;
      if (tof > range)
      {
        tof = range;
      }
      auto throttle_limit = 256 - 512 * tof/range;
      if ( ctl.throttle > throttle_limit )
      {
        ctl.throttle = throttle_limit;
      }
    }
    
    if ( ctl.throttle == 0 )
    {
      ctl.light.tail = true;
    }

    ctl.boost = PS4.data.button.cross;

    // steer
  
    // steering offset
    if ( PS4.data.button.left )
    {
      g_ctx.center --;    
      if ( g_ctx.center < -20 )
        g_ctx.center = -20;
    }
    if ( PS4.data.button.right )
    {
      g_ctx.center ++;
      if ( g_ctx.center > 20 )
        g_ctx.center = 20;
    }
  }

  // suppress waving
  if (abs(ctl.steer-g_ctx.prev_steer)<=STEER_MARGIN)
  {
    ctl.steer = g_ctx.prev_steer;
  }
  else
  {
    g_ctx.prev_steer = ctl.steer;
  }

  ctl.steer += g_ctx.center;

  // drive

  int16_t target = 0;
  int16_t power = g_ctx.prev_power;

  // map throttle to power
  int16_t target_max = LIMITED_MAX;
  if ( ctl.boost )
  {
    target_max = TARGET_MAX; // range of 8bit PMW is 0 to 256. max is not 255.
  }

  if ( ctl.throttle < -THROTTLE_MARGIN )
  {
    target = (target_max-TARGET_MIN) * (ctl.throttle+THROTTLE_MARGIN) / (TARGET_MAX-THROTTLE_MARGIN) - TARGET_MIN;
    if (target < power)
    {
      if (power > -TARGET_MIN)
      {
        power = -TARGET_MIN;
      }
      else
      {
        power -= STEP_BACKWARD;
      }
    }
    else
    {
      power = target;
    }
  }
  else if ( ctl.throttle > THROTTLE_MARGIN )
  {
    target = (target_max-TARGET_MIN) * (ctl.throttle-THROTTLE_MARGIN) / (TARGET_MAX-THROTTLE_MARGIN) + TARGET_MIN;    
    if (target > power)
    {
      if (power < TARGET_MIN)
      {
        power = TARGET_MIN;
      }
      else
      {
        power += STEP_FORWARD;
      }
    }
    else
    {
      power = target;
    }
  }
  else
  {
    target = 0; 
    power = 0;
  }

  g_ctx.prev_power = power;

  // debugging
  
  debug_print(ctl, g_env, g_ctx);

  // do & interval

  // driver
  digitalWrite(PIN_PHASE, power>=0?HIGH:LOW);
  ledcWrite(CH_ENABLE, abs(power));

  // steer
  steer.write( -1*ctl.steer*0.65 );

  // tail light
  digitalWrite(PIN_TAIL, ctl.light.tail?HIGH:LOW);

  // turn light
  if ( ctl.light.hazard )
  {
    digitalWrite(PIN_TURN, (g_env.steps/(FREQUENCY/2))%2?HIGH:LOW);
  }
  else
  {
    digitalWrite(PIN_TURN, LOW);    
  }

  // interval in ms
  delay(1000/FREQUENCY);
  g_env.steps ++;
}
