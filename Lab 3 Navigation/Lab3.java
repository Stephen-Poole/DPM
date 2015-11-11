package navigation;

//imports.
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import navigation.Odometer;
import navigation.OdometryDisplay;

public class Lab3 {// Static Resources:
	
   	// Left motor connected to output A
    // Right motor connected to output D
    private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port port = LocalEV3.get().getPort("S4");
	

	// Constants
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 15;
	
	public static void main(String[] args) {
		//Initialize US sensor and SampleProvider (to get distances).
		SensorModes usSensor = new EV3UltrasonicSensor(port);
		SampleProvider usdistance = usSensor.getMode("Distance");
		float[] sample = new float[usdistance.sampleSize()];
		int buttonChoice;

		//Some objects that need to be instantiated (Odometer, OdometryDisplay, Navigation,
		//usnavigation, and UltrasonicPoller).
		final TextLCD t = LocalEV3.get().getTextLCD();	
		Odometer odometer = new Odometer(leftMotor, rightMotor, WHEEL_RADIUS, TRACK);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t);
		
		//For navigation without the US sensor.
		Navigation robotNavigation = new Navigation(odometer, t, leftMotor, rightMotor, 30, 60);
		
		//For navigation with the US sensor.
		usnavigation usrobotNavigation = new usnavigation(odometer, t, leftMotor, rightMotor, 30, 60);
		
		//Needed to continuously get readings.
		UltrasonicPoller us = new UltrasonicPoller(usdistance,sample, usrobotNavigation);
		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString(" usNavi| Navi  ", 0, 2);
			t.drawString("       |        ", 0, 3);
			t.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			
			//Start all our threads needed.
			//Notice here we use usrobotNavigation and us in order to use the US sensor to detect obstacles.
			odometer.start();
			odometryDisplay.start();
			usrobotNavigation.start();
			us.start();
			
		} else {
			//Start all necessary threads.
			odometer.start();
			odometryDisplay.start();
			robotNavigation.start();
		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

}
