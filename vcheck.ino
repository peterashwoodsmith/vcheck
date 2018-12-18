
/*
 * vcheck: Engine flywheel/propellor balancer. 
 * ==========================================
 * V1.0 (C) April 2018 Peter Ashwood-Smith
 * 
 * This little Arduino program will measure the angle and amplitude of imbalance in an engine's flywheel and / or propellor.
 * It does this by tracking the position of the flywheel relative to a spark plug while simultaneously reading accelleration data 
 * from a sensor mounted on the engine just behind the flywheel. This is intended for use with an aircraft engine like a Lycoming
 * 360/540 or other 4 stroke engine. It will display the results of the analysis on a small LCD screen. The user can then place
 * weights 180 degrees oposite to the point of imballance and try until the major component of vibration is acceptable or no 
 * longer at the frequency of the RPM of the flywheel or propellor (indicating that the vibration is higher frequency and likely
 * some other internal component).
 * 
 * The Arduino board has a small auxiliary analog circuit which converts induced current on a small wire wrapped around the spark
 * plug lead of the number one cylinder on the engine into a square wave which then drives the interrupt pin on the Arduino.
 * This is accomplished by using a transistor to invert the signal from the spark plug into a 555 timer which is configured as a one 
 * shot with an output duration of 5 or so milliseconds (to allow the spark to settle before returning to zero). The output of the
 * 555 drives the Arduino interrupt pin. An optical sensor to track fly wheel position would work equally well but requires additional
 * sensor mounting to the engine and some reflective tape etc. The spark plug wire is simpler for the user.
 * 
 * An isr() on the interrupt pin times the sparks, computes RPM and allows comutation of the position of the flywheel between 
 * sparks. It also has some debouncing and sanity checking becuase occasional spurious interrupts seem to still plague the design.
 * 
 * A polling loop will continuously read an accellerometer's analog input, each time it samples it will compute the 
 * position of the flywheel in degrees 0..359 relative to the last spark and populate acceleration average data in an array for that
 * angle. Digital accelerometers of sufficient bandwidth via I2C could also be used as can much higher bandwidht/resolution 
 * analog accellerometers. Normally vibration measurements are 0-2G with 1G being quite rough. So 16g accellerometers and 3 axis 
 * accelerometers are not required although with a 3 axis accelerometer you could allow for arbitrary mounting or could track x and y 
 * to get even more precise position of the heavy spot.
 * 
 * After several seconds of sampling, which should be at a steady RPM, the polling is stopped, the ISR is disabled and an analysis
 * is performed. The analysis consists of a Descrete Fourier Transform to extract the power and phase of each spectral component of
 * the vibration. We extract the frequency, phase and power of the most sigificant harmonic. This is the major vibration source.
 * If that frequency is the same as the rotational frequency of the flywheel then the flywheel/propelor is the source of the major
 * X axis vibration and we will display it on the LCD. We also keep track of the largest of these that we have seen since start up and 
 * dsplay the largest at all times on the LCD.
 */

#include <math.h>                                                      // We use sin() cos() and PI
#include <LiquidCrystal.h>                                             // Great little LCD dirver object.

const    int  DEBUG                    = 0;                            // Set to non zero and serial debugging occurs

const    byte interrupt_pin            = 18;                           // pin that is receiving RISING EDGE spark interrupts
const    byte accellerationPin         = A0;                           // pin that is receiving analog X-axis accelleration levels
        float MAXANALOG;                                               // maximum analog voltage we can measure (analog reference)
        float MAXDIGITAL;                                              // max A to D output (usually 1023).
        float VOLTSPERG;                                               // analog accelerometer volts per 1G, must be calibrated
        float ZEROGVOLTS;                                              // analog accelerometer volts at 0G, must be calibrated
const    int  NUM_BIN                  = 120;                          // How many accelleration samples, 2 degree resolution
long     int  acc[NUM_BIN];                                            // total accelleration measures at angle i
         int  hit[NUM_BIN];                                            // how many times we measured at this angle i
volatile int  interrupt_count          = 0;                            // how many times we've seen a spark interrupt so far
volatile int  valid_interrupt_count    = 0;                            // how many spark interrupts seemed sensible
volatile long last_interrupt_time      = 0;                            // time of last sensible spark interrupt
volatile int  rpm                      = 0;                            // current RPM as computed by isr()
volatile long dt                       = 0;                            // delta time between sparks                   
volatile long rpm_total                = 0;                            // We try to average the RPM
volatile int  max_rpm                  = 0;                            // We track max RPM we saw
volatile int  min_rpm                  = 6000;                         // We track min RPM we saw
volatile int  invalid_interrupts_long  = 0;                            // We track interrupts that are too infrequent 
volatile int  invalid_interrupts_short = 0;                            // We track interrupts that are too frequent
volatile int  zero_time_interrupts     = 0;                            // We track how often we had to reset last interrupt time to 0
volatile long short_dt                 = 0;                            // This is the last too short dt

LiquidCrystal lcd(50,48,46,44,42,40);                                  // Create LCD object at the appropriate pins

/*
 * This is the interrupt service routine. It gets called every time the spark plug fires.
 * We do have some debouncing here because occasionally the hardware will see some spurious interrupts. Need to
 * work on this in the external hardware.
 * 
 * Essentially we are keeping track of the time of the last interrupt and the current interrupt in microseconds.
 * This then allows us to compute the RPM. Note that the last_interrupt_time is volatile because we will
 * use it while polling to determine for a given accelleration what angle that occurred relative to the spark.
 */
void isr()  {                                                          // Spark plug just fired                                      
    long interrupt_time = micros();                                    // what is current time in microseconds
    interrupt_count += 1;                                              // Keep track of stats for debugging
    if (last_interrupt_time != 0) {                                    // if not first interrupt
         dt = interrupt_time - last_interrupt_time;                    // calculate time since last 
         if (dt < 0) dt = -dt;                                         // handle wrap around
         if (dt > (long) 165*100) {                                    // if more than 16.5ms and
             if (dt < (long) 60*1000) {                                // less than 60ms its a valid DT.
                rpm = (long) 60000*1000 / dt;                          // so calculate RPM
                if (rpm > max_rpm) max_rpm = rpm;                      // track the max we saw
                if (rpm < min_rpm) min_rpm = rpm;                      // track the min we saw
                rpm_total += rpm;                                      // track totals for average RPM values
                valid_interrupt_count += 1;                            // track interrupt count of valid signals
                last_interrupt_time = interrupt_time;                  // remember this time as last for next dt calc
             } else {
                invalid_interrupts_long += 1;                          // dt too long, invalid interrupt
                last_interrupt_time = 0;                               // so reset, next interrupt will start cycle again
             }
          } else {
            invalid_interrupts_short += 1;                             // dt too short, invalid interrupt
            last_interrupt_time = 0;                                   // so reset, next interrupt starts cycle again
            short_dt = dt;
          }
    } else {
          last_interrupt_time = interrupt_time;
          zero_time_interrupts += 1;
    }
}

/*
 * This is the workhorse polling loop. It will read the accellerometer input for duration milliseconds repeatedly.
 * The trick here is that the isr() is humming away in the background resetting the last_interrupt_time every time
 * a spark occurrs. We use the time of the last_interrupt and the time 'dt' between interrupts to comptue how far the
 * fly wheel will have rotated since the spark. We then convert that to a number 0..NUM_BIN (NUM_BIN often 360) and add
 * the accelleration we are reading into an average we are keeping at that slot/angle. After a many seconds of polling we
 * will have an array of acc values 0..359 and hit values 0..359. From these we can produce average accelleration at
 * each angle 0..359 for subsequence spectral analysis. To increase the dynamic range of the A2D conversion we use a 
 * 2.56 volte internal analog reference. We really only need to read 1G deviations so the range should be as tight as
 * possible. Note that the accelerometer can lag by several milliseconds which must be calibrated and corrected for
 * prior to the final output. It returns 0 for a successful poll with no out of range values, or N > 0 if there were
 * errors in the accellerometer (over/under G readings).
 */
int  poll(unsigned int duration)
{
     unsigned long now = millis();                                        // what is current time.
     unsigned long end_t = now  + duration;                               // Compute time we should stop polling.
     int overG = 0;
     if (end_t < now) {  delay(duration); return; }                       // millis() will wrap just skip this sample.               
     /* MICROS WRAP CHECK GOES HERE */                                    // micros() will wrap just skip this sample.
     while(millis() < end_t) {                                          
        if ((rpm > 0) && (last_interrupt_time > 0)&&(dt > NUM_BIN)) {     // if the isr() is giving us RPM and valid last spark
             int  accel   = analogRead(accellerationPin);                 // Lets get the acell value and then find its slot 
             int  rot     = ((micros()-last_interrupt_time)*NUM_BIN)/dt;  // simply conversion to rotation in 0..NUM_BIN units  
             if ((accel > 0)&&(accel < 1023))  {  
                 if ((rot >= 0) && (rot < NUM_BIN)) {                     // sanity checks
                     acc[rot] += accel;                                   // average in the accel voltage into this rotation slot
                     hit[rot] += 1;                                       // into running average at this rotation angle (0..259).
                 }
             } else
                 overG += 1;                                             // Accelerometer out of range, over/under G.
        }
     }
     return(overG);
}

/*
 * Just reset all the counters that we use during polling and interrupts. 
 * Each polling time we start fresh.
 */
void reset_counters()
{
     rpm = 0;
     min_rpm = 6000;
     max_rpm = 0;
     rpm_total = 0;
     last_interrupt_time = 0;
     interrupt_count = 0;
     valid_interrupt_count = 0;
     invalid_interrupts_short = 0;
     invalid_interrupts_long = 0;
     zero_time_interrupts = 0;
     short_dt = 0;
     memset(acc, 0, sizeof(acc));
     memset(hit, 0, sizeof(hit));
}

/*  Now we do a DFT on the accelleration data for a given frequency slot. Usually we will only this for
 *  k=1, but we could do the full DFT by calling it using k=0..NUM_BIN. k=0 is the DC component, k=1 is
 *  the full cycle component etc. 
 */
void descreteFourierTransform(int k, float *this_a, float *this_ang)
{  
     float sum_r = 0.0; float sum_i = 0.0;                         // initialize running real/imaginary tallies
     for(int t = 0; t < NUM_BIN; t++) {                            // standard slow Fourier iteration for harmonic k
          int hit_t = hit[t];
          if (hit_t == 0) continue;                                // protect againt zero samples at this angle
          float angle = (2.0*M_PI)*t*k/NUM_BIN;                    // compute angle relative to frequency and iter
          float in = (float) acc[t] / (float) hit_t;               // use average of accelleraion at this slot
          sum_r = sum_r + in*cos(angle);                           // tally the real component
          sum_i = sum_i - in*sin(angle);                           // tally the imaginary
     }
    *this_a   = sqrt(sum_r*sum_r + sum_i*sum_i)/NUM_BIN;           // return amplitude of this harmonic
    *this_ang = atan2(sum_i, sum_r);                               // and phase of this harmonic
}

/*
 * Extra LCD characters as bit maps. The first is an r/m or revolutions per minute.
 * The second is a small 'o' to indicate degrees and the last is i/s meaning inches
 * per second. Each is described as a array of 8 bytes. If you draw out the bit 
 * maps , or align each byte one per line you can see the characters that I've drawn.
 */
uint8_t c_rpm[8] = { B01100, B01010, B01100, B01010, B00000, B01010, B10101, B10101 };
#define C_RPM 1
//
uint8_t c_deg[8] = { B01000, B10100, B10100, B01000, B00000, B00000, B00000, B00000 };
#define C_DEG 2
//
uint8_t c_vel[8] = { B10000, B00000, B10011, B10100, B10100, B00010, B00001, B00110 };
#define C_VEL 3

/*
 * This function will write "%04d r/m %03d o %03d i/s" to the LCD at the row we have currently set.
 * The r/m, o and i/s are special single characters that are created using the above pixel maps.
 * This saves us a few characters in the display.
 */
void updateLcdLine(int steady, int rpm, int angle, float vel)
{    char temp[10]; 
     sprintf(temp,"%04d", rpm); lcd.print(temp); lcd.write(C_RPM); lcd.print(" ");        // eg: "2500r/m "
     if (steady) {
         sprintf(temp,"%03d", angle); lcd.print(temp); lcd.write(C_DEG); lcd.print(" ");  // eg: "060o ."
         if (vel < 9.0) {
             int v = vel*100;
             int velHunths =  v % 10; v = v/10;
             int velTenths =  v % 10; v = v/10;
             int velOnes   =  v;
             sprintf(temp,"%d.%d%d",velOnes, velTenths, velHunths); 
             lcd.print(temp);                                                             // eg: "1.22i/s"
         } else {
             lcd.print("> 9");
         }
         lcd.write(C_VEL);
     } else {
         lcd.print("not steady");                                                         // unsteady RPM
     }
}

/*
 * Update the LCD with an error message 'msg'. Just clear everything and write the error.
 */
void updateLcdError(char *msg)
{
    lcd.clear();                                                    
    lcd.setCursor(0,0);                                             
    lcd.print("ERR ");
    lcd.print(msg);
}
         
/*
 * This is the LCD output function. We have a 2x16 LCD. We write the most recent RPM/ANGLE/VEL to the top row.
 * The bottom row we keep track of the best estimate. This we do by looking for the longest steady RPM/ANG/VEL
 * and always displaying the longest steady one to the second row. RPM's below 1700 won't update the best so
 * the best will stay after the engine is slowed and shut down.
 */
void updateLcd(int steady, int rpm, int angle, float vel)
{    static int   curr_hits= 0, curr_rpm= 0, curr_angle=0;           // This the upper display value
     static int   best_hits= 0, best_rpm= 0, best_angle=0;           // this is lower display value
     static float curr_vel = 0.0, best_vel = 0.0;
     angle = (angle + 180) % 360;                                    // We have accelerometer inverted so phase is 180 off.
     lcd.clear();                                                    // We refresh LCD every time
     lcd.setCursor(0,0);                                             // start drawing upper row
     if (rpm < 1700) {                                               // below 1700 RPM no estimate
         if (rpm > 0) {
             lcd.print(rpm);                                         // Display the low RPM value
             lcd.write(C_RPM);
             lcd.print(" TOO LOW");                                  // and indicate its too low
         } else {                                                    // No RPM detected yet
             lcd.print("ZERO OR NO RPM");   
         }
     } else {                                                        // > 1700 RPM so show the
         updateLcdLine(steady, rpm, angle, vel);                     // estimate, good bad or ugly
         if (steady) {                                               // if RPM was steady over sample
             if ((abs(rpm - curr_rpm) < 100) &&                      // if sample is "similar" to current 
                 (abs(angle - curr_angle) < 10) &&                   // sample then
                 (abs(vel - curr_vel) < 0.1)) { 
                 curr_hits += 1;                                     // add one to weight of this estimate
                 if (curr_hits > best_hits) {                        // if estimate is now better than 2nd line
                     best_hits  = curr_hits;                         // we update what will be shown on
                     best_rpm   = rpm;                               // 2nd line.
                     best_angle = angle;
                     best_vel   = vel;
                 }
             } else {                                                // the top display changed so 
                 curr_hits   = 0;                                    // its weight drops back to zero
                 curr_rpm    = rpm;                                  // and we remember what was displayed.
                 curr_angle  = angle;
                 curr_vel    = vel;
             }
         }
     }
     lcd.setCursor(0,1);                                             // time to update the best estimate
     if (best_hits == 0) {                                           // if we have no estimate then
         lcd.print("NO BEST ESTIMATE");                              // just say we have no best estimate
     } else {
         updateLcdLine(1, best_rpm, best_angle, best_vel);           // Otherwise update best estimate so far.
     }
}

/*
 * Just return the average of two acceleration samples. The number of hits for a sample can be zero, in which
 * case the contribution is zero.
 */

int avgof(int i, int j)
{
    int v1 = 0, v2 = 0;
    if (i < 0) i = NUM_BIN + i;
    j = j % NUM_BIN;
    if (hit[i] > 0) v1 = acc[i] / hit[i];
    if (hit[j] > 0) v2 = acc[j] / hit[j];
    if (v1 == 0) return v2;
    if (v2 == 0) return v1;
    return((v1+v2)/2);
}

/*
 *  Infill() will go through the acc vector and look for any zero values. This means no samples were taken at 
 *  that angle. If this happens we will try to average this value based on the next and previous values. To
 *  do this we have to wrap around the beginning of the array properly since any zeros can have  a signifcant 
 *  impact on the accelleration measurements because they artifically introduce high frequency 
 *  components in the signal.
 */
int infill()
{   int i, nz = 0;
    for(i = 0; i < NUM_BIN; i++) {                            
        if (hit[i] == 0) {
            acc[i] = avgof(i-1, i+1);
            hit[i] = 1;
            nz += 1;
        }
     }
     return(nz);
}

/* The angle measured by the FFT will be in error because the accelerometer lags by different amounts at different 
 * RPMs. Here we will search for the angle in an array of angle, correction values and extrapolate linearly between
 * the two bounding RPM correction values. This array is determined by calibration where we run the system with a
 * known 0 angle imballance and then measure the error at different RPMs.
 */
int correctAngleForRpm(int angle, int rpm)
{   //
    // Correction vector. Amount of angle error at each RPM as measured during calibration.
    // Must be sorted smallest RPM to largest RPM since we search linearly.
    //
    static int corr[][2] = { { 1700,  5 }, 
                             { 3600,  70 } };
                             
    //
    // Walk through the correction vector and find the pair of RPM's that bound the input RPM.
    // Then do a linear extrapolation to find a resonable correction for this input angle.
    //   
    static int maxi = sizeof(corr) / sizeof(corr[0]);
    for(int i = 0; i < maxi - 1; i++) {
        if ((rpm >= corr[i][0]) && (rpm < corr[i+1][0]))  {    // Find bounding correction pair
            int   dx = corr[i+1][0] - corr[i][0];              // Compute slop between this pair
            int   dy = corr[i+1][1] - corr[i][1];              // as correction change / rpm change
            float m  = (float) dy / (float) dx;
            float x  = rpm - corr[i][0];                       // apply linear interpolation
            int   y  = x * m + corr[i][1];                     // to find correction between pair
            angle = angle - y;                                 // apply correction to angle
            if (angle < 0) angle = 360 + angle;                // if we went negative then wrap
            break;                                    
        }
    }
    return(angle);
}

/*
 * And this is the main loop. It executes repeatedly by the Arduino OS. We start a cycle by just resetting all 
 * counters. Then we enable the interrupt on the spark plug circuit to trigger the isr() on a rising edge.
 * After that we just enter the polling loop for some small number of seconds to accumulate all our accelleration
 * data. As mentioned abouve the polling loop can know at what angle a given accelleration occurred because it
 * has access to the time, the expected time of the next spark and the previous spark and can produce the acc and
 * hit arrays as a result. Usually this will be an array of 360 accelleration values. After we have accumulated
 * many rotations of accelleration data we can then run a spectral anaysis of the signal. Essentially we run a
 * Descrete Fourier Transform (DFT) but are not interested in all the frequency components, we only want the one
 * with the maximum energy. When the flywheel/prop is out of balance the lowest frequency component will be the 
 * highest energy. When the flywheel/prop is in good balance, there will be some other frequeny that is 
 * contriubting more energy to the vibration than the flywheel/prop rotation frequency.
 */
void loop() {
     int   overG, avg_rpm, steady, nz, corrected_angle;
     float accel, accelv, accelN, accelone, angle, freq=0, vel=0, gs=0;
     reset_counters();                                                    // zero all counters 
     attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr, RISING);  // setup interrupts for spark isr
     overG = poll(2000);                                                  // continuous poll of accell for 4000ms
     detachInterrupt(digitalPinToInterrupt(interrupt_pin));               // stop interrupts
     if (overG > 0) {                                                     // If during polling the acceleratomer went full scale
        updateLcdError("OVER G");                                         // then we generate ERROR: "OVER G" on LCD.
     } else { 
        nz = infill();                                                    // average out any zero data points (nz is count of zeros)
        avg_rpm = rpm_total/valid_interrupt_count;                        // average RPM over the sample
        descreteFourierTransform(NUM_BIN-1, &accelN,   &angle);           // find acceleration and phase for N-1st harmonic (this is at RPM frequency).
        descreteFourierTransform(1,         &accelone, &angle);           // find acceleration and phase for 1st harmonic (this is at RPM frequency).
        accel   = accelN + accelone;                                      // RPM harmonic accel in bits is sum of 1st and N-1st spectra in bits
        accelv  = (accel * MAXANALOG) / MAXDIGITAL;                       // digital accelleration convert to analog relative to reference voltage
        gs      = accelv / VOLTSPERG;                                     // amplitude in G is (accelleration volts / 0.330 mv/G) 
        angle   = (angle * 180.0) / M_PI;                                 // and lets convert phase angle to degrees for output.
        freq    = avg_rpm / 60;                                           // Convert RPM to Hz
        vel     = (386.0 * gs) / (2.0*M_PI * freq);                       // vibration velocity as inches per second 
        steady = (max_rpm - min_rpm) < 100;                               // If RPM is not steady we don't count a maximum 
        if (angle > 0)                                                    // We convert from FFT phase angle to peak position
            angle = 360 - angle;                                          // positive angles are 1..180 before spark 
        else if (angle < 0)                                               // -1..-180 mean peak is 1..180 after spark
                 angle = -angle;
        corrected_angle = correctAngleForRpm((int) angle, avg_rpm);       // accelerometer lag effect varies at RPM, fix it              
        updateLcd(steady, avg_rpm, corrected_angle, vel);                 // update the current and maximum vibration velocity
     }
     
     /*  Debugging 
      *  Print interrupt related information to see how well the debouncing is working. 
      *  Print computed RPM, averages etc. frequency etc.
      *  Print the computed Vibration velocity and angle.
      *  Finally print a curve of the sampled accelleration data as a graph, one sample per line.    
      */
     if (DEBUG) {
         int i,j; int max_acc = 0; int min_acc;
         Serial.println("----------------------------------------------------------");
         Serial.print("     Interrupts= ");     Serial.println(interrupt_count); 
         Serial.print("          Valid= ");     Serial.println(valid_interrupt_count);
         Serial.print("           Zero= ");     Serial.println(zero_time_interrupts);
         Serial.print("          Short= ");     Serial.println(invalid_interrupts_short);
         Serial.print("           ShDT= ");     Serial.println(short_dt);
         Serial.print("           Long= ");     Serial.println(invalid_interrupts_long);
         Serial.println("----------------------------------------------------------");
         Serial.print("      Last RPM = ");     Serial.println(rpm);
         Serial.print("           MIN = ");     Serial.println(min_rpm);
         Serial.print("           MAX = ");     Serial.println(max_rpm);
         Serial.print("           AVG = ");     Serial.println(avg_rpm);
         Serial.print("       AVG FRQ = ");     Serial.println(freq); 
         Serial.print("     AVG 1/FRQ = ");     Serial.println((float) 1.0/freq);  
         Serial.print("            NZ = ");     Serial.println(nz);
         Serial.println("----------------------------------------------------------");
         Serial.print(" Vibration IPS = ");     Serial.println(vel); 
         Serial.print("     bits acc1 = ");     Serial.println(accelone);    
         Serial.print("     bits accN = ");     Serial.println(accelN); 
         Serial.print("     volts acc = ");     Serial.println(accelv); 
         Serial.print("           Gs  = ");     Serial.println(gs);     
         Serial.print("       RAW ANG = ");     Serial.println(angle);
         Serial.print("       COR ANG = ");     Serial.println(corrected_angle);
         Serial.println("----------------------------------------------------------");

         
         for(i = 0; i < NUM_BIN; i++) {                     // Figure out the maximum accelleration sample
             if (hit[i] > 0) {
                 int temp = (acc[i] / hit[i]);
                 if (temp > max_acc) max_acc = temp;
             }
         }
         min_acc = max_acc;
         for(i = 0; i < NUM_BIN; i++) {                     // Figure out the minimum non zero accelleration sample
             if (hit[i] > 0) {
                 int temp = (acc[i] / hit[i]);
                 if (temp < min_acc) min_acc = temp;
             }
         }
         if (max_acc > 0) {                                 // Now print a little graph, each line is one sample
             int range = max_acc - min_acc;                 // scale the dynamic range.
             for(i = 0; i < NUM_BIN; i++) {                 // This should look like the oscilliscope accelleration data
                 if (hit[i] > 0) {
                     long int acci = (acc[i] / hit[i]);
                     long int temp = acci - min_acc;                   // we remove the minimum portion of range
                     j = (temp * (long int) 100) / range;              // and scale so that 50 characters is full range
                     while(j-- > 0) Serial.print(" ");
                     Serial.print("* ");
                     if ((acci == max_acc)||(acci == min_acc)) {
                         long int angle = (i * (long int) 360) / NUM_BIN;                // We print data about the peak
                         Serial.print(acci); Serial.print("@"); Serial.print(angle);     // but may not be same as Fourier
                     }
                 }
                 Serial.println("");
             }
         }
         Serial.println("==========================================================");
         Serial.println("");
     }
}

/*
 *  We should be seeing -1G on the accelerometer. If its much different then there is a problem with the sensor. 
 *  It may be the wrong way up, it may be disconnected etc. We try to see and tell the operator. We look for within 5%
 *  This requires that when powered up the accelerometer is properly positioned with X pointing up and with the motor 
 *  stopped. 
 */
int selftest()  
{    float minus1Gvolts  = ZEROGVOLTS + VOLTSPERG;                            // This is where -1G should be
     float plus1Gvolts   = ZEROGVOLTS - VOLTSPERG;                            // This is where +1G should be
     float analogVoltSum = 0;
     int   samples;
     float averageAccelVolts;
     long int averagebits = 0;
     for(samples = 0; samples < 100; samples++) {                             // For 1 second, sample analog every 10ms (i.e. 100 times).
          int v = analogRead(accellerationPin);   
          averagebits += v;  
          analogVoltSum += (v * MAXANALOG) / MAXDIGITAL;                      // accumulate the voltage value for averaging.
          delay(10);
     }
     averageAccelVolts = analogVoltSum / samples;                             // Lets see what we got
     averagebits = averagebits / samples;
     if (DEBUG) {
         Serial.println("--------------- SELF TEST -----------------------------");
         Serial.print("      volts per G = ");     Serial.println(VOLTSPERG);
         Serial.print("       max analog = ");     Serial.println(MAXANALOG);
         Serial.print("      max digital = ");     Serial.println(MAXDIGITAL);
         Serial.print("       zeroGvolts = ");     Serial.println(ZEROGVOLTS); 
         Serial.print("     minus1Gvolts = ");     Serial.println(minus1Gvolts); 
         Serial.print("      plus1Gvolts = ");     Serial.println(plus1Gvolts); 
         Serial.print("  avg accel volts = ");     Serial.println(averageAccelVolts); 
         Serial.print("  avg accel bits  = ");     Serial.println(averagebits);
     }
     if (abs(averageAccelVolts - plus1Gvolts)/plus1Gvolts < 0.10)   return(0);  // All ok
     if (abs(averageAccelVolts - minus1Gvolts)/minus1Gvolts < 0.10) return(1);  // Looks upside down
     if (averageAccelVolts < (plus1Gvolts / 10.0))                  return(2);  // Not connected?
     return(3);                                                                 // Orientation strange
} 

/*
 * Initialization - setup up the interrupt Pin & configure the LCD. Not much else to do. Note
 * Serial port speed options are: 9600, 14400, 19200, 28800, 38400, 57600, or 115200.
 */
void setup() {
     int rc;
     analogReference(INTERNAL2V56);                                           // analogRead dynamic range is 0..2.56 volts 0..1023
     MAXANALOG  = 2.56;                                                       // maximum analog voltage we can measure (will be 1023)
     MAXDIGITAL = 1023.0;                                                     // Dynamic range is 0..1023 
     VOLTSPERG  = 0.2375;                                                     // Calibrated volts/G shows  0.2375.
     ZEROGVOLTS = 1.2275;                                                     // Calibrated 0G volts shows 1.2275
     pinMode(interrupt_pin, INPUT_PULLUP);                                    // Sparks drive interrupts, accelleration is at A0.
     lcd.begin(16,2);                                                         // Configure a 16 x 2 LCD display
     lcd.createChar(C_RPM, c_rpm);                                            // create a custom 8x5 r/m character
     lcd.createChar(C_DEG, c_deg);                                            // create a custom 8x5 'o' degree character
     lcd.createChar(C_VEL, c_vel);                                            // create a custom 8x5 i/s (inches / second) character
     lcd.clear();
     lcd.setCursor(0,0);                                                      // Print the banner
     if (DEBUG) {
        Serial.begin(9600);                                                   // For serial port we use a different banner
        lcd.print("V-CHECK: SERIAL");
        lcd.setCursor(0,1);                                                   // and will have lots of debug going to it at 9600
        lcd.print("DEBUG ENABLED");
     } else {
        lcd.print("V-CHECK VER 1.00");                                         // Otherwise normal banner 
        lcd.setCursor(0,1);
        lcd.print(__DATE__[0]);                                                // Just print a nice version and the time/date
        lcd.print(__DATE__[2]);                                                // of compilation.
        lcd.print('-');
        lcd.print(__DATE__[4]);
        lcd.print(__DATE__[5]);
        lcd.print('-');
        lcd.print(__DATE__[7]); 
        lcd.print(__DATE__[8]);
        lcd.print(__DATE__[9]);
        lcd.print(__DATE__[10]); 
        lcd.print('-');
        lcd.print(__TIME__[0]);
        lcd.print(__TIME__[1]);
        lcd.print(__TIME__[2]);
        lcd.print(__TIME__[3]);
        lcd.print(__TIME__[4]);     
     } 
     delay(2000);                                                   
     rc = selftest();                                                         // Do a quick self test of accellerometer sanity
     if (rc != 0) {                                                           // If all ok, let the loop() run we just return
          char        *msg = (char *) "UKNOWN CAUSE";                         // Otherwise blink the LCD with the sanity error
          if (rc == 1) msg = (char *) "ACCEL INVERTED?";                      // Accel shows ~ +1G (so its upside down)
          if (rc == 2) msg = (char *) "ACCEL NO INPUT";                       // Accel shows ~ 0 volts (so its perhaps not connected)
          if (rc == 3) msg = (char *) "ACCEL NOT 1G";                         // Accel not showing 1G , strange orientation
          lcd.clear();
          while(1) {                                                          // Blink the LCD forever
             int   accd = analogRead(accellerationPin);                       // And for debugging show digital version of analog
             float accv = (accd * MAXANALOG )/ MAXDIGITAL;                    // what that analog is converted.
             float accg = accv * VOLTSPERG;                                   // and what that converted to G is. 
             lcd.setCursor(0,0);
             lcd.print(msg);
             lcd.setCursor(0,1);
             lcd.print(accd); lcd.print("/"); 
             lcd.print(accv); lcd.print("/"); 
             lcd.print(accg);
             delay(1000);
             lcd.clear();
             delay(500);
          }
     }
}

  

