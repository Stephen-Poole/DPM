// Lab2.java

package odometry;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;

public class Lab2 {
	
	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	//Instantiate colorSensor.
	private static EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	
	//Constants
	//Set the wheel_radius and track (distance between wheels) variables based on our robot.
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 14.95;
	
	public static void main(String[] args) {
		int buttonChoice;

		// some objects that need to be instantiated
		
		final TextLCD t = LocalEV3.get().getTextLCD();
		
		//Create Odometer instance and pass WHEEL_RADIUS and TRACK into our Odometer constructor so that we can use these values
		//in the Odometer class calculations.
		Odometer odometer = new Odometer(leftMotor, rightMotor, WHEEL_RADIUS, TRACK);
		
		//Create OdometryCorrection instance and pass in odometer and colorSensor objects to the constructor.
		OdometryCorrection odometryCorrection = new OdometryCorrection(odometer, colorSensor);
		
		//Instantiate the OdometryDisplay.
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t, odometryCorrection);
		
		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString(" Float | Drive  ", 0, 2);
			t.drawString("motors | in a   ", 0, 3);
			t.drawString("       | square ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);
		
		//If the user hits the left button, run "Float motors"
		if (buttonChoice == Button.ID_LEFT) {
			
			leftMotor.forward();
			leftMotor.flt();
			rightMotor.forward();
			rightMotor.flt();
			
			//Start threads for odometer, odometryDisplay and odometryCorrection.
			odometer.start();
			odometryDisplay.start();
//			odometryCorrection.start();
			
		//If the user hits the right button, run "Drive in a square"
		} else {
			//Start the odometer, the odometry display and (possibly) the
			//odometry correction
			
			//Start threads for odometer, odometryDisplay and odometryCorrection.
			odometer.start();
			odometryDisplay.start();
			odometryCorrection.start();
			
			// spawn a new Thread to avoid SquareDriver.drive() from blocking
			(new Thread() {
				public void run() {
					SquareDriver.drive(leftMotor, rightMotor, WHEEL_RADIUS, WHEEL_RADIUS, TRACK);
				}
			}).start();
		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}