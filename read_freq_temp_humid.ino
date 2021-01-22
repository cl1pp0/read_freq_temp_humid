/* An Arduino UNO based frequency, temperature, humidity meter with LCD display output */

#include <LiquidCrystal.h>

#define CYCLE_TIME_1        2       /* ms */
#define CYCLE_TIME_2        100     /* ms */
#define CYCLE_TIME_3        500     /* ms */
#define MEASURE_PERIOD      1000    /* ms */
#define NUM_AVG_SAMPLES     8
#define BUF_SIZE            NUM_AVG_SAMPLES /* only 2^N allowed! */
#define BUF_MASK            (BUF_SIZE-1)
#define BUF_OK              false
#define BUF_FAIL            true
#define DEBUG_PIN           4       /* used for any runtime error indication */
#define DHpin               5       /* used for temperature and humidity reading */
#define CAPTURE_PIN         8       /* used for freq measurement (capture edges) */
#define ANALOG_PIN          A0      /* used for measuring analog voltage of photo transistor */
#define F_CPU_CORR          8000    /* compensate xtal tolerance (device specific!) */

/* 2^N ring buffer */
typedef struct {
    uint16_t data[BUF_SIZE];
    uint16_t sum;
    uint16_t last;
    uint8_t  read;
    uint8_t  write;
} ringbuf_t;

typedef enum {
    MEAS_WAIT = 0,
    MEAS_READ,
    MEAS_EVAL
} meas_state_t;

static volatile uint16_t timer1_ovf_cnt = 0;
static volatile uint32_t timer1_capt_cnt = 0;
static uint8_t timer2_ovf_cnt = 0;
static bool led_state = LOW;
static bool debug_pin_state = LOW;
static volatile bool loop_end = false; /* must be volatile! */
static volatile meas_state_t meas_state = MEAS_WAIT;
static volatile uint32_t t_first, t_last;
static volatile bool sync;

const char text1[] = "U/mV=";
const char text2[] = "f/Hz=";

static ringbuf_t sampleBuf = {{}, 0, 0, 0, 0};

LiquidCrystal lcd(6, /*RS*/
                  7, /*EN*/
                  9, /*D4*/
                  10,/*D5*/
                  11,/*D6*/
                  12 /*D7*/);

byte dat [5];

void setup()
{
    lcd.begin(16, 2);
    lcd.print(text1);
    lcd.setCursor(0, 1);
    lcd.print(text2);
    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CAPTURE_PIN, INPUT);
    pinMode(DEBUG_PIN, OUTPUT);
    pinMode (DHpin, OUTPUT);

    cli();
    /* Timer 2 (8-bit) setup */
    TCCR2A = 0;
    TCCR2B = (1 << CS22) | (1 << CS21); /* Prescaler: 256 */
    TIMSK2 = (1 << TOIE2);  /* Overflow interrupt enable */

    /* Timer 1 (16-bit) setup */
    TCCR1A = 0;
    TCCR1B = (1 << CS10)   /* No prescaler */
           | (1 << ICNC1)  /* Input noise cancelling */
           | (1 << ICES1); /* Rising edge triggered */
    TCCR1C = 0;

    TIMSK1 = (1 << ICIE1)  /* Input capture interrupt enable */
           | (1 << TOIE1); /* Overflow interrupt enable */

    TIFR1 = (1 << TOV1) | (1 << ICF1); /* clear pending interrupts */

    sei();
}

static byte read_data () {
  byte data = 0;
  for (int i = 0; i < 8; i ++) {
    if (digitalRead (DHpin) == LOW) {
      while (digitalRead (DHpin) == LOW); // wait for 50us
      delayMicroseconds (30); // determine the duration of the high level to determine the data is '0 'or '1'
      if (digitalRead (DHpin) == HIGH)
        data |= (1 << (7-i)); // high front and low in the post
      while (digitalRead (DHpin) == HIGH); // data '1 ', wait for the next one receiver
    }
  }
  return data;
}

static void start_test () {
  digitalWrite (DHpin, LOW); // bus down, send start signal
  delay (30); // delay greater than 18ms, so DHT11 start signal can be detected
  digitalWrite (DHpin, HIGH);
  delayMicroseconds (40); // Wait for DHT11 response
  pinMode (DHpin, INPUT);
  while (digitalRead (DHpin) == HIGH);
  delayMicroseconds (80); // DHT11 response, pulled the bus 80us
  //if (digitalRead (DHpin) == LOW); // ???
  while (digitalRead (DHpin) == LOW);
  delayMicroseconds (80); // DHT11 80us after the bus pulled to start sending data
  for (int i = 0; i < 4; i ++) // receive temperature and humidity data, the parity bit is not considered
    dat[i] = read_data ();
  pinMode (DHpin, OUTPUT);
  digitalWrite (DHpin, HIGH); // send data once after releasing the bus, wait for the host to open the next Start signal
}

/* ring buffer write */
static bool bufWrite(ringbuf_t *buf, uint16_t data_in)
{
    uint8_t next = ((buf->write + 1) & BUF_MASK);

    if (buf->read == next)
        return BUF_FAIL;

    buf->sum -= buf->last;
    buf->last = buf->data[next];
    buf->data[buf->write] = data_in;
    buf->sum += buf->data[buf->write];
    buf->write = next;

    return BUF_OK;
}

/* ring buffer read */
static uint8_t bufRead(ringbuf_t *buf, uint16_t *data_out)
{
    if (buf->read == buf->write)
        return BUF_FAIL;

    *data_out = buf->data[buf->read];
    buf->read = (buf->read + 1) & BUF_MASK;

    return BUF_OK;
}

/* toggles digital output pin */
static inline void toggle_pin(uint8_t pin_number, bool *pin_state)
{
    if (*pin_state == LOW)
        *pin_state = HIGH;
    else
        *pin_state = LOW;

    digitalWrite(pin_number, *pin_state);
}

/* exec time exceeded -> handle error */
static inline void loop_end_error()
{
    cli();
    PORTD = 0;
    digitalWrite(LED_BUILTIN, HIGH);
    while (1) {}
}

/* wait for loop end to have a fixed frequency execution of main loop code */
static inline void loop_end_wait()
{
    /* wait for loop end */
    if (!loop_end)
    {
        while (!loop_end)
        {
            __asm__("nop\n\t");
        }
        loop_end = false;
    }
    else
    {
        loop_end_error();
    }
}

/* evaluate and print result on LCD or serial */
static void eval_print()
{
    uint16_t sensor_mV;
    uint32_t t_diff;
    float freq;
    char str[11];

    cli();
    sensor_mV = ((sampleBuf.sum / BUF_SIZE) * 39 + 4) >> 3;
    sei();

    t_diff = t_last - t_first;

    if (t_diff)
        freq = (float)timer1_capt_cnt / t_diff * (F_CPU - F_CPU_CORR);
    else
        freq = 0.0;

    lcd.setCursor(sizeof(text1)-1, 0);
    lcd.print("      ");
    lcd.setCursor(sizeof(text1)-1, 0);
    lcd.print(sensor_mV);

    lcd.setCursor(sizeof(text2)-1, 1);
    lcd.print("                     ");
    lcd.setCursor(sizeof(text2)-1, 1);
    if (t_diff)
    {
        dtostrf(freq, -1, 3, str);
        lcd.print(str);
    }
    else
        lcd.print("---");

    start_test ();
    Serial.print ("Humidity = ");
    Serial.print (dat [0], DEC); // display the humidity-bit integer;
    Serial.print ('.');
    Serial.print (dat [1], DEC); // display the humidity decimal places;
    Serial.print ("%, ");
    Serial.print ("Temperature = ");
    Serial.print (dat [2], DEC); // display the temperature of integer bits;
    Serial.print ('.');
    Serial.print (dat [3], DEC); // display the temperature of decimal places;
    Serial.print ("°C, ");
    Serial.print ("Frequency = ");
    Serial.print (str);
    Serial.print ("Hz, ");
    Serial.print ("Voltage = ");
    Serial.print (sensor_mV);
    Serial.println ("mV");
}

/* called by timer 2 overflow ISR */
static void func1_cyclic()
{
    /* currently nothing to do here */
}

/* called by main loop */
static void measure_cyclic()
{
    uint16_t adc_data;
    uint16_t buf_data;

    static uint8_t wait_cnt = 0;

    cli();
    adc_data = analogRead(ANALOG_PIN);
    sei();

    bufWrite(&sampleBuf, adc_data);
    bufRead(&sampleBuf, &buf_data); /* dummy read, avoids buffer overrun */

    switch (meas_state)
    {
        case MEAS_WAIT:
            meas_state = MEAS_READ;
            t_first = 0;
            t_last = 0;
            timer1_capt_cnt = 0;
            sync = 1;
            break;

        case MEAS_READ:
            /* wait MEASURE_PERIOD ms */
            if (++wait_cnt == (MEASURE_PERIOD/CYCLE_TIME_2))
            {
                meas_state = MEAS_EVAL;
                wait_cnt = 0;
            }
            break;

        case MEAS_EVAL:
            eval_print();
            meas_state = MEAS_WAIT;
            break;

        default:
            break;
    }
}

/* called by main loop */
static void led_blink_cyclic()
{
    toggle_pin(LED_BUILTIN, &led_state);
}

/* timer 2 is used for fixed frequency program execution */
ISR(TIMER2_OVF_vect, ISR_NOBLOCK)
{
    /* F_CPU / prescaler / n_ticks = 16 MHz / 256 / 125 = 500 Hz = 2 ms */
    TCNT2 = 131; /* 131...255 -> 125 ticks */

    if (++timer2_ovf_cnt == (CYCLE_TIME_2/CYCLE_TIME_1))
        timer2_ovf_cnt = 0;
    else if (timer2_ovf_cnt == ((CYCLE_TIME_2/CYCLE_TIME_1)-1))
        loop_end = true;

    func1_cyclic();
}

/* timer 1 input capture handling for frequency measurement */
ISR(TIMER1_CAPT_vect)
{
    toggle_pin(DEBUG_PIN, &debug_pin_state);

    if (meas_state == MEAS_READ)
    {
        if (sync)
        {
            sync = 0;
            t_first = ICR1;
            t_first += ((uint32_t)timer1_ovf_cnt << 16);
            t_last = t_first;
            timer1_capt_cnt = 0;
        }
        else
        {
            timer1_capt_cnt++;
            t_last = ICR1;
            t_last += ((uint32_t)timer1_ovf_cnt << 16);
        }

        /* handle interrupts which occur at the same time */
        if (TIFR1 & (1 << TOV1) && ((t_last & 0xffff) < 0x8000))
        {
            t_last += ((uint32_t)1 << 16);
        }

        /* catch events which come while ISR is executed */
        /*
        if ((TIFR1 & (1 << ICF1)))
        {
            timer1_capt_cnt++;
            TIFR1 = 1 << ICF1;
        }
        */
    }
}

/* timer 1 overflow handling */
ISR(TIMER1_OVF_vect, ISR_NOBLOCK)
{
    timer1_ovf_cnt++;
}

/* cyclic main loop */
void loop()
{
    static uint16_t loop_cnt = (CYCLE_TIME_3/CYCLE_TIME_2)-1;

    measure_cyclic();

    if (++loop_cnt == (CYCLE_TIME_3/CYCLE_TIME_2))
    {
         led_blink_cyclic();
         loop_cnt = 0;
    }

    loop_end_wait();    /* wait for cycle time 2 end */
}
