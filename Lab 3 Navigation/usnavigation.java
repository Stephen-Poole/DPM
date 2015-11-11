package navigation;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.Sound;

//Get Navigation to extend Thread Class and implement UltrasonicController
public class usnavigation extends Thread implements UltrasonicController {
	//Declare variables that we will need.
	private UltrasonicPoller usPoller;
	private Odometer odometer;
	private TextLCD t;
	private double x;
	private double y;
	private double theta;
	private double desx;
	private double desy;
	private double destheta;
	private Object lock;
	private double deltax;
	private double deltay;
	private double deltatheta;
	private double USSensorReading;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int FORWARD_SPEED = 175;
	private static final int ROTATE_SPEED = 125;

	private static final int bandCenter = 29; // Offset from the wall (cm)
	// Set bandWidth to be 3 (margin on either side of our bandCenter)
	private static final int bandWidth = 3; // Width of dead band (cm)
	// Changed motorLow speed to 150, and motorHigh speed to 250.
	private static final int motorLow = 150; // Speed of slower rotating wheel
												// (deg/sec)
	private static final int motorHigh = 250; // Speed of the faster rotating
												// wheel (deg/seec)

	// constructor
	public usnavigation(Odometer odometer, TextLCD t, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, double desx, double desy) {
		//Set constructor inputs to be values of variables declared above.
		this.odometer = odometer;
		this.t = t;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.desx = desx;
		this.desy = desy;

	}
	
	//run() method tells it where to go.
	public void run() {
		travelTo(0, 60);
		travelTo(60, 0);

	}

	//travelTo(method)
	void travelTo(double desx, double desy) {
		
		//Get x an y from odometer.
		x = odometer.getX();
		y = odometer.getY();
		
		//Use the set desx and desy, along with the x and y values from the odometer to
		//get distance in x and distance in y that we need to travel.
		deltax = desx - x;
		deltay = desy - y;
		
		//set class variables of x and y to be the values passed into the method.
		this.desx = desx;
		this.desy = desy;

		//Calculate the destination theta based on our current position.
		destheta = Math.atan(deltax / deltay);

		//Conditions on how the robot needs to turn
		if (deltax < 0) {
			destheta = destheta - Math.PI;
		} else if (deltay < 0 && deltax > 0) {
			destheta += Math.PI;
		}

		//Call turnTo() method so that the robot adjusts to face the correct spot.
		turnTo(destheta);

		//Set motor speeds.
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		//Calculate how far to turn wheels based on deltax and deltay calculated above.
		leftMotor.rotate(convertDistance(Lab3.WHEEL_RADIUS, Math.sqrt(deltax * deltax + deltay * deltay)), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RADIUS, Math.sqrt(deltax * deltax + deltay * deltay)), false);

	}

	//isNavigating method.
	boolean isNavigating() {
		//If robot has arrived at its destination, return false, else return true.
		x = odometer.getX();
		y = odometer.getY();

		if (x > desx - 1 && y > desy - 1 && x < desx + 1 && y < desy + 1) {
			return false;
		} else {
			return true;
		}
	}

	//turnTo method.
	void turnTo(double destheta) {

		//Get theta reading from odometer.
		theta = odometer.getTheta();

		//Calculate change in theta that is required based on odometer theta and destination theta.
		deltatheta = destheta - theta;

		//Conditions to ensure that deltatheta actually gets us facing the right direction.
		if (deltatheta > Math.PI) {
			deltatheta = deltatheta - 2 * Math.PI;

		} else if (deltatheta < -Math.PI) {
			deltatheta = deltatheta + 2 * Math.PI;

		}

		// leftMotor.forward();
		// rightMotor.forward();
		leftMotor.rotate(convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, deltatheta * 360 / (2 * Math.PI)), true);
		rightMotor.rotate(-convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, deltatheta * 360 / (2 * Math.PI)), false);
		Sound.buzz();

	}

	//Use convertDistance and convertAngle methods to get correct distance and angle measurements for 
	//our motor.rotate methods, respectively.
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	//Methods implemented from Ultrasonic Controller
	@Override
	public void processUSData(int distance) {
		//Assign distance measured to USSensorReading
		USSensorReading = distance;
		
		//If robot is too close to wall, go around it.
		if (USSensorReading < 10) {

			//Turn right and move past block.
			leftMotor.rotate(convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, 90.0), true);
			rightMotor.rotate(-convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, 90.0), false);

			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(Lab3.WHEEL_RADIUS, 25.0), true);
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RADIUS, 25.0), false);

			//Turn left and move forward past block.
			leftMotor.rotate(-convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, 90.0), true);
			rightMotor.rotate(convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, 90.0), false);

			leftMotor.rotate(convertDistance(Lab3.WHEEL_RADIUS, 47.0), true);
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RADIUS, 47.0), false);
			
			//Turn left again to arrive at other side of block.
			leftMotor.rotate(-convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, 90.0), true);
			rightMotor.rotate(convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, 90.0), false);

			leftMotor.rotate(convertDistance(Lab3.WHEEL_RADIUS, 25.0), true);
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RADIUS, 25.0), false);

			//Call travelTo() method again in order to get robot to final destination.
			travelTo(desx, desy);

		}

	}

	@Override
	public int readUSDistance() {
		// TODO Auto-generated method stub
		return 0;
	}

}
