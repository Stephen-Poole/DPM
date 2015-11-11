package localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
	
	//Set variables that we will need.
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	private double leftRadius = 2.1;
	private double rightRadius = 2.1;
	public static double ROTATION_SPEED = 80;
	final static int ACCELERATION = 1000;
	private double width = 15.0;
	private double phi;
	private Navigation nav;
	
	private double x1;
	private double y1;
	private double x2;
	private double y2;
	
	private double lightToMid = 12.0;
	private boolean seenLine;
	private double finalx;
	private double finaly;
	
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	
	//Constructor.
	public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData, Navigation nav) {
		
		//Set variables from constructor to be those defined above.
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.nav = nav;
		
		//Get access to motors
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		
		//Set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
				
		//Set speed
		leftMotor.setSpeed((int) ROTATION_SPEED);
		rightMotor.setSpeed((int) ROTATION_SPEED);
	}
	
	
	//Light localization method. 
	public void doLocalization() {
		
		//Set boolean variable seenLine to help the robot avoid beeping multiple times when crossing lines.
		seenLine = false;
		
		//drive to location listed in tutorial
		//While the robot is not crossing a line, drive forward.
		while(getFilteredData() > 26){
			leftMotor.forward();
			rightMotor.forward();	
		}
		
		//Once it sees the line, back up by 17 cms.
		leftMotor.rotate(convertDistance(leftRadius, -17), true);
		rightMotor.rotate(convertDistance(rightRadius, -17), false);
		
		//Turn 90 degrees to the left.
		leftMotor.rotate(-convertAngle(leftRadius, width, 90), true);
		rightMotor.rotate(convertAngle(rightRadius, width, 90), false);
		
		//Drive forward until it sees a line.
		while(getFilteredData() > 26){
			leftMotor.forward();
			rightMotor.forward();		
		}
		
		//When it sees a line, back up 15 cms.
		leftMotor.rotate(convertDistance(leftRadius, -15), true);
		rightMotor.rotate(convertDistance(rightRadius, -15), false);

		
		//Start rotating and clock all 4 gridlines.
		
		//While robot spins (roughly) 360 degrees:
		while(Math.abs(odo.getAng() - 86) > 3) {
			leftMotor.backward();
			rightMotor.forward();
			
			//If it sees a line and seenLine is false, then:
			if(getFilteredData() < 60 && seenLine == false) {
				
				//Set seenLine to true.
				//Add 180 to the angle for the purposes of calculations below.
				seenLine = true;
				phi = odo.getAng() + 180;
				
				//If phi is greater than 360, bring it back into range by subtracting 360.
				if(phi > 360) {
					phi = phi-360;
				}
				
				//Case 1: If sensor approaches first black line, calculate x1 value,
				//where x1 is x distance of robot from line.
				if(Math.abs(phi - 270) < 50) {
					phi = phi - 270;
					x1 = -lightToMid*Math.sin((phi*2*Math.PI)/360);
					Sound.beep();
				}
				
				//Case 2: If sensor approaches second black line, calculate y1 value,
				//where y1 is y distance of robot from line.
				else if(Math.abs(phi - 0) < 50) {
					phi = phi - 0;
					y1 = -lightToMid*Math.sin((phi*2*Math.PI)/360);
					Sound.buzz();
				}
				
				//Case 3: If sensor approaches third black line, calculate x2 value,
				//where x2 is second x distance of robot from line.
				else if(Math.abs(phi - 90) < 50) {
					phi = 90 - phi;
					x2 = -lightToMid*Math.sin((phi*2*Math.PI)/360);
					Sound.beep();
				}
				
				//Case 4: If sensor approaches fourth black line, calculate y2 value,
				//where y2 is second y distance of robot from line.
				else if(Math.abs(phi - 180) < 50) {
					phi = 180 - phi;
					y2 = -lightToMid*Math.sin((phi*2*Math.PI)/360);
					Sound.buzz();
				}
				
			}
			
			//If it doesn't see a line, set seenLine to false
			else {
				seenLine = false;
			}
		}
		
		//Stop the motors.
		leftMotor.stop(true);
		rightMotor.stop(true);
		
		//Set final values of x and y (which are the averages of x and y 
		//calculated above, respectively).
		finalx = (x1+x2)/2;
		finaly = (y1+y2)/2;
		
		//Set robot's position and travel there. Then turn to 0.
		odo.setPosition(new double [] {finalx, finaly, odo.getAng()}, new boolean [] {true, true, true});
		nav.travelTo(0, 0);
		nav.turnTo(0, true);
		
	}
	
	//Get the color sensor data.
	private float getFilteredData() {
		//usdata = nothing
		colorSensor.fetchSample(colorData, 0);
		//usdata contains the distance data from the US sensor
		
		float distance = 100*colorData[0];
				
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
