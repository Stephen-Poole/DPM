/*
 * Odometer.java
 */

package odometry;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
	private double x, y, theta;

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	
	//Declare all variables we will need to use.
	private double wheelRadius;
	private double track;
	private int oldrotationsLeft;
	private int oldrotationsRight;
	
	// default constructor
	//Note: We added the wheelRadius and track variables to our constructor.
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double wheelRadius, double track) {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		lock = new Object();
		
		//Set the passed in values to be the values of the corresponding variables in this class.
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.wheelRadius = wheelRadius;
		this.track = track;
		
	}

	
	/*-----------------------------------
	OUR CODE STARTS HERE
	-----------------------------------*/
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;
		
		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here
			
			//NOTE: getTachoCount() returns the number of degrees, NOT the number of rotations.
			//Get rotations of both left and right motors.
			int rotationsLeft = leftMotor.getTachoCount();
			int rotationsRight = rightMotor.getTachoCount();
			
			//Get the difference in angle for each wheel, by looking at the difference between the
			//current number of rotations and the previous number of rotations. This gives us the wheel
			//rotations in radians.
			double deltaLambda = (2*Math.PI*(rotationsLeft-oldrotationsLeft))/(360.0);
			double deltaRow = (2*Math.PI*(rotationsRight-oldrotationsRight))/(360.0);
			
			//Get dsL (distance traveled by left wheel).
			double dsL = ((rotationsLeft-oldrotationsLeft)*wheelRadius*2*Math.PI)/360;
			//Get dsR (distance traveled by right wheel).
			double dsR = ((rotationsRight-oldrotationsRight)*wheelRadius*2*Math.PI)/360;
			//Get Avds (arclength) that center of robot has traveled by using dsL and dsR.
			double Avds = (dsL+dsR)/2;
			
			//Set oldrotationsLeft and oldrotationsRight to be the current rotationsLeft and 
			//rotationsRight, respectively.
			oldrotationsLeft = rotationsLeft;
			oldrotationsRight = rotationsRight;
			
			//Get the difference in Theta by looking at the difference between deltaLambda (left wheel) 
			//and deltaRow (right wheel).
			double deltaTheta = ((wheelRadius)*(deltaLambda-deltaRow))/((track));
					
			//Whatever is inside the synchronized (lock) can ONLY be updated/accessed by one thread at a time.
			//In this sense the variables are "protected" from different threads trying to use them at 
			//the same time.
			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				//Change the value of theta
				theta += deltaTheta;
				
				//Get change in y by looking at cos of angle.
				//Get change in x by looking at sin of angle.
				double deltay = Avds*Math.cos(theta);
				double deltax = Avds*Math.sin(theta);
				
				//Update x to be x + the change in x (deltax)
				//Update y to be y + the change in y (deltay)
				x = x + deltax;
				y = y + deltay;
				
				
			}
			
			
			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	//Getters for x, y, and theta.
	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}
	
	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
			
		}
	}

	//Setters for x, y, and theta.
	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}
}