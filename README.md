
vcheck: Engine flywheel/propellor balancer. 
==========================================
V1.0 (C) April 2018 Peter Ashwood-Smith
  
This little Arduino program will measure the angle and amplitude of imbalance in an engine's flywheel and / or propellor.
It does this by tracking the position of the flywheel relative to a spark plug while simultaneously reading accelleration data 
from a sensor mounted on the engine just behind the flywheel. This is intended for use with an aircraft engine like a Lycoming
360/540 or other 4 stroke engine. It will display the results of the analysis on a small LCD screen. The user can then place
weights 180 degrees oposite to the point of imballance and try until the major component of vibration is acceptable or no 
longer at the frequency of the RPM of the flywheel or propellor (indicating that the vibration is higher frequency and likely
some other internal component).
  
The Arduino board has a small auxiliary analog circuit which converts induced current on a small wire wrapped around the spark
plug lead of the number one cylinder on the engine into a square wave which then drives the interrupt pin on the Arduino.
This is accomplished by using a transistor to invert the signal from the spark plug into a 555 timer which is configured as a one 
shot with an output duration of 5 or so milliseconds (to allow the spark to settle before returning to zero). The output of the
555 drives the Arduino interrupt pin. An optical sensor to track fly wheel position would work equally well but requires additional
sensor mounting to the engine and some reflective tape etc. The spark plug wire is simpler for the user.
  
An isr() on the interrupt pin times the sparks, computes RPM and allows comutation of the position of the flywheel between 
sparks. It also has some debouncing and sanity checking becuase occasional spurious interrupts seem to still plague the design.
  
A polling loop will continuously read an accellerometer's analog input, each time it samples it will compute the 
position of the flywheel in degrees 0..359 relative to the last spark and populate acceleration average data in an array for that
angle. Digital accelerometers of sufficient bandwidth via I2C could also be used as can much higher bandwidht/resolution 
analog accellerometers. Normally vibration measurements are 0-2G with 1G being quite rough. So 16g accellerometers and 3 axis 
accelerometers are not required although with a 3 axis accelerometer you could allow for arbitrary mounting or could track x and y 
to get even more precise position of the heavy spot.
  
After several seconds of sampling, which should be at a steady RPM, the polling is stopped, the ISR is disabled and an analysis
is performed. The analysis consists of a Descrete Fourier Transform to extract the power and phase of each spectral component of
the vibration. We extract the frequency, phase and power of the most sigificant harmonic. This is the major vibration source.
If that frequency is the same as the rotational frequency of the flywheel then the flywheel/propelor is the source of the major
X axis vibration and we will display it on the LCD. We also keep track of the largest of these that we have seen since start up and 
dsplay the largest at all times on the LCD.

Testing was done using a bench grider modified with a wooden wheel with marked angles and holes to accept imbalancing weights. 
Likewise a spark plug and coil were driven using a small cam and points mounted on the other end of the bench grinder. 
Testing indicated that accuracy of about 5% is possible within 1600 to 3500 RPM. Since cheap analog accelerometers have lag
at higher requencies they can not easily be used at higher RPMs however its possible to compensate for the lag if you calibrate
for different RPMs. The code has been modified to incorporate a running correction factor which varies from 50 degrees down to
about 0 degrees dependent on RPM and to interpolate between points for RPMs where the correction is not known. This seems to
work quite well. The best solution would be a more expensive piezo accellerometer but the purpose of this project was to try to 
do this as cheaply as possible. 
