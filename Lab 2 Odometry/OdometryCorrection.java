/* 
 * OdometryCorrection.java
 */
package odometry;

import lejos.hardware.Sound;
import lejos.hardware.Sounds;
import lejos.hardware.ev3.LocalEV3;
//Import colour sensor library.
import lejos.hardware.sensor.EV3ColorSensor;

public class OdometryCorrection extends Thread {
	//Declare all variables that we will need.
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private EV3ColorSensor colorSensor;
	private float[] rgbArray = new float [1];
	private String rgbString = "";
	private int countx = 0;
	private int county = 0;

	// constructor
	//Pass odmeter and colorSensor objects into OdometryCorrection constructor.
	public OdometryCorrection(Odometer odometer, EV3ColorSensor colorSensor) {
		//Set odometer and colorSensor variables to be the values of the passed in objects.
		this.odometer = odometer;
		this.colorSensor = colorSensor;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		while (true) {
			correctionStart = System.currentTimeMillis();

			/*put your correction code here
			Get the light sensor reading using the red light. This monitors the change
			in contrast appearing on the board, thus allowing us to detect the black lines 
			when we cross them. This value gets put into the 0th position in rgbArray.
			Get value of light sensor reading and put in rgbString.*/
			colorSensor.getRedMode().fetchSample(rgbArray, 0);
			rgbString = String.valueOf(rgbArray[0]*100);	
			
			/*county and countx are either incremented or decremented depending on which direction
			the robot is traveling. This allows us to keep track of what line we are currently hitting.
			We then use the present value of county and countx to set the the correct values of x and y, 
			depending on which line it is at and in which direction it is heading.*/
			
			/*If county is 0 and we hit a line and the angle is close to 0, we know we want to 
			set the value of Y to be 15 (since it is the first line) - 6 (to account for the gap 
			between the sensor and the center of the robot. Then increment county.*/
			if(county == 0 && ((int)(rgbArray[0]*100)) < 26 && odometer.getTheta() < 0.09 ) {
				odometer.setY(15-6);
				Sound.buzz();
				county += 1;
			}
			
			//Once it crosses its first line in the x direction, set x value accordingly.
			else if(countx == 1 && ((int)(rgbArray[0]*100)) < 26 && odometer.getTheta() > 1.50 && odometer.getTheta() < 1.66) {
				odometer.setX(15-6);
				Sound.beep();
				countx += 1;
			} 
		    
			//If you hit another line in the positive y direction, set odometer value accordingly, and increment counts.
			else {
			if(odometer.getTheta() < 0.09 ){
				if(((int)(rgbArray[0]*100)) < 26) {
					odometer.setY(30*county + 15 - 6);
					county += 1;
					countx += 1;
					Sound.buzz();
				}
			}
			
			//If angle is around 90 degrees (positive x) and it hits a line, set x accordingly and increment count.
			else if(odometer.getTheta() > 1.50 && odometer.getTheta() < 1.66){
				if(((int)(rgbArray[0]*100)) < 26) {
					odometer.setX(30*(countx-1) + 15 - 6);
					countx += 1;
					Sound.beep();
				}
			}
			
			//If angle is around 180 degrees (negative y) and it hits a line, decrement county and set y value accordingly.
			else if(odometer.getTheta() > 3.05 && odometer.getTheta() < 3.23){
				if(((int)(rgbArray[0]*100)) < 26) {
					county -= 1;
					odometer.setY(30*county + 15 + 6);
					Sound.buzz();
					
				}
			}
			
			//If angle is arount 270 degrees (negative x) and it hits a line, decrement countx and setx accordingly.
			else if(odometer.getTheta() > 4.63 && odometer.getTheta() < 4.80){
				if(((int)(rgbArray[0]*100)) < 26) {
					countx -= 1;
					odometer.setX(30*(countx-1)+ 15 + 6);
					Sound.beep();
				}
			}
			}
			
			
			
			
			

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
	
	//Have getRgbString method to get String value of light sensor recording. This will allow us to display 
	//the light sensor reading on the screen.
	public String getRgbString() {
		return rgbString;
	}
	
	
	
}