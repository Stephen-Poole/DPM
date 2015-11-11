package localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class USLocalizer {
	
	//Set all variables that we will need in this class.
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static double ROTATION_SPEED = 50;
	final static int ACCELERATION = 1000;
	private double deltaTheta;
	
	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private Navigation nav;
	private double leftRadius = 2.1;
	private double rightRadius = 2.1;
	private double width = 14.4;
	
	//Constructor.
	public USLocalizer(Odometer odo, SampleProvider usSensor, float[] usData, Navigation nav, LocalizationType locType) {
		
		//Set all variables from constructor input to be the variables defined above.
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		this.nav = nav;
		
		//Get access to motors.
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		
		//Set acceleration.
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
		
		//Set speed.
		leftMotor.setSpeed((int) ROTATION_SPEED);
		rightMotor.setSpeed((int) ROTATION_SPEED);
	}
	
	//US localization method.
	public void doLocalization() {
		double [] pos = new double [3];
		double angleA, angleB;
		
		//Falling Edge case.
		if (locType == LocalizationType.FALLING_EDGE) {
			
			//Rotate the robot until it sees no wall (~40cm).
			//The + 2 acts as a buffer to avoid confusion between going to and leaving a wall.
			//While the robot does not see wall, turn right.
			while(getFilteredData() < 42){
				leftMotor.forward();
				rightMotor.backward();
			}
			Sound.beep();
			
			//Keep rotating until the robot sees a wall, then latch the current
			//odometer angle; set it to angleA.
			while(getFilteredData() > 40){
				leftMotor.forward();
				rightMotor.backward();
			}
			Sound.beep();
			angleA = odo.getAng();
			
			//Switch direction and turn until it sees no wall.
			while(getFilteredData() < 40 + 2){
				leftMotor.backward();
				rightMotor.forward();
			}
			Sound.beep();
			
			//Keep rotating until the robot sees a wall, then latch the current
			//odometer angle; set it to AngleB
			while(getFilteredData() > 40){
				leftMotor.backward();
				rightMotor.forward();
			}
			Sound.beep();
			angleB = odo.getAng();
			
			//Stop both motors
			leftMotor.stop(true);
			rightMotor.stop(true);
			
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			
			//Set deltaTheta depending on how angles worked out.
			if(angleA < angleB) {
				deltaTheta = 45 - (angleA + angleB)/2;
			}
			else if(angleA > angleB) {
				deltaTheta = 225 - (angleA + angleB)/2;
			}
			
			//Get total new angle.
			double newAng = deltaTheta + odo.getAng();
			
			Sound.buzz();
			
			// update the odometer position (example to follow:)
			odo.setPosition(new double [] {0.0, 0.0, newAng}, new boolean [] {true, true, true});
			
			//Turn to 0, so that it faces the right direction.
			nav.turnTo(0, true);
			
			//Rising Edge case.
		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall.
			 * This is very similar to the FALLING_EDGE routine, but the robot
			 * will face toward the wall for most of it.
			 */
			
			//While robot does not see a wall (US reading > 40), rotate left.
			while(getFilteredData() > 40){
				leftMotor.backward();
				rightMotor.forward();
			}
			Sound.beep();
			
			//Keep rotating until the robot sees no wall, then latch the angle and set it to angleA.
			//The 40cm is the distance at which it will stop seeing the wall. The 1.5 is to avoid
			//confusion between leaving and returning to the wall.
			while(getFilteredData() < 41.5){
				leftMotor.backward();
				rightMotor.forward();
			}
			Sound.beep();
			angleA = odo.getAng();
			
			//Switch direction and wait until it sees a wall.
			while(getFilteredData() > 40){
				leftMotor.forward();
				rightMotor.backward();
			}
			Sound.beep();
			
			//Keep rotating until the robot sees no wall, then latch the angle by setting
			//it to angleB.
			while(getFilteredData() < 40 + 1.5){
				leftMotor.forward();
				rightMotor.backward();
			}
			Sound.beep();
			angleB = odo.getAng();
			
			//Stop both motors.
			leftMotor.stop(true);
			rightMotor.stop(true);
			
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			
			//Set deltaTheta depending on how angles worked out.
			if(angleA > angleB) {
				deltaTheta = 45 - (angleA + angleB)/2 + 180;
			}
			else if(angleA < angleB) {
				deltaTheta = 225 - (angleA + angleB)/2 + 180;
			}
			
			//Get total new angle.
			double newAng = deltaTheta + odo.getAng();
			
			Sound.buzz();
			
			// update the odometer position (example to follow:)
			odo.setPosition(new double [] {0.0, 0.0, newAng}, new boolean [] {true, true, true});
			
			//Turn to 0, so that it faces the right direction.
			nav.turnTo(0, true);

		}
	}
	
	//Get US data.
	private float getFilteredData() {
		//usdata = nothing
		usSensor.fetchSample(usData, 0);
		//usdata contains the distance data from the US sensor
		
		float distance = 100*usData[0];
				
		return distance;
	}
	
	//convertDistance and converAngle methods for helping the robot travel.
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
