package navigation;

//Imports
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import navigation.Odometer;

//Get Navigation to extend Thread Class
public class Navigation extends Thread {
	//Declare variables that we will need.
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
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int FORWARD_SPEED = 175;
	private static final int ROTATE_SPEED = 125;
	

	//Constructor
	public Navigation(Odometer odometer, TextLCD t, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			double desx, double desy) {
		//Set constructor input to be values of variables declared above.
		this.odometer = odometer;
		this.t = t;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.desx = desx;
		this.desy = desy;
		
	}
	
	//run() method giving instructions on how the robot should travel.
	public void run(){

		//Call travelTo() method for each position on the grid that we want to go to.
		travelTo(60,30);
		travelTo(30,30);
		travelTo(30,60);
		travelTo(60,0);
		
	}
	
	
	//travelTo() method.
void travelTo(double desx, double desy){
		
		//Get x an y from odometer.
		x = odometer.getX();
		y = odometer.getY();
		
		//Use the set desx and desy, along with the x and y values from the odometer to
		//get distance in x and distance in y that we need to travel.
		deltax= desx-x;
		deltay= desy-y;
		
		//Calculate the destination theta based on our current position.
		destheta = Math.atan(deltax/deltay);
		
		//Conditions on how the robot needs to turn
		if (deltax < 0){
			destheta = destheta - Math.PI;
		}
		else if(deltay < 0 && deltax > 0){
			destheta += Math.PI;
		}
		
		//Call turnTo() method so that the robot adjusts to face the correct spot.
		turnTo(destheta);
		
		//Set motor speeds.
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		//Calculate how far to turn wheels based on deltax and deltay calculated above.
	    leftMotor.rotate(convertDistance(Lab3.WHEEL_RADIUS, Math.sqrt(deltax*deltax + deltay*deltay)), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RADIUS, Math.sqrt(deltax*deltax + deltay*deltay)), false);
		
	
}

//turnTo method.
void turnTo(double destheta){
		  
		 //Get theta reading from odometer.
		 theta = odometer.getTheta();	 
		 
		 //Calculate change in theta that is required based on odometer theta and destination theta.
		 deltatheta = destheta-theta;
		 
		 //Conditions to ensure that deltatheta actually gets us facing the right direction.
		 if(deltatheta > Math.PI ){
			 deltatheta = deltatheta - 2*Math.PI;	 
		 }
		 else if(deltatheta < -Math.PI){
			 deltatheta = deltatheta + 2*Math.PI;
			 
		 }
		 
		 //Set motor speeds.
		 leftMotor.setSpeed(ROTATE_SPEED);
		 rightMotor.setSpeed(ROTATE_SPEED);
		 
		 //Turn robot by delta theta.
		 leftMotor.rotate(convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, deltatheta*360/(2*Math.PI)), true);
		 rightMotor.rotate(-convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, deltatheta*360/(2*Math.PI)), false);
		 
	}
	
	//isNavigating method.
	boolean isNavigating(){
		//If robot has arrived at its destination, return false, else return true.
		x = odometer.getX();
		y = odometer.getY();
	
		if(x == desx && y == desy){
			return false;
		}
		else {
			return true;
		}
	}
	
	//Use convertDistance and convertAngle methods to get correct distance and angle measurements for 
	//our motor.rotate methods, respectively.
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}