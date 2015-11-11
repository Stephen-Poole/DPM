
import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController{
	private final int bandCenter, bandwidth;
	private final int motorLow, motorHigh;
	private int distance;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	//Added a filter so that the robot could avoid gaps.
	//NOTE: We used the same implementation as in the PController class, but used 
	//slighly different numbers to better fit the BangBang class.
	private final int FILTER_OUT = 25;
	private int filterControl = 0;
	
	public BangBangController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
							  int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		//Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		leftMotor.setSpeed(motorHigh);				// Start robot moving forward
		rightMotor.setSpeed(motorHigh);
		leftMotor.forward();
		rightMotor.forward();
	}
	
	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		
		//FILTER IMPLEMENTATION. 
		//NOTE: For comments on how this is working, please refer to the similar chunk of code in the 
		//PController class. This is used to help the robot avoid confusion when rolling past gaps.
		if (distance == 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the filter value
			filterControl ++;
		} else if (distance == 255){
			// true 255, therefore set distance to 255
			this.distance = distance;
		} else {
			// distance went below 255, therefore reset everything.
			filterControl = 0;
			this.distance = distance;
		}
		
		
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)
		
		//NOTE: The code that implements the conditional statements for BangBang are very straightforward as
		//compared to PController since we don't have need to implement the calcProp() method.
		
		//This conditional statement will execute if the robot is VERY close to the wall.
		//When the robot is less than 17 cm from the wall, this code will run.
		//NOTE: Because our robot is way too close to the  wall, we will want it to stop, and turn
		//right in place, so that it does not get any closer to the wall. Hence,we will get the 
		//right motor to spin backwards, and the left motor to spin forwards. This way the robot will 
		//stay in a fixed position while it turns right.
		if(this.distance < 17) {	
			
			//Set left and right motor speeds to both be "motorLow".
			leftMotor.setSpeed(motorLow);
			rightMotor.setSpeed(motorLow);
			
			//Turn left wheel forwards, and right wheel backwards so that the robot turns right while staying 
			//in place.
			leftMotor.forward();
			rightMotor.backward();					
			
		}
		
		//This conditional statement will execute if the robot is too close to the wall, though not 
		//as close as in the above case.
		//To get it away from the wall, we want it to turn right.To do this, we need to make the left
		//motor turn faster than the right one. Hence we set the leftMotor speed to be motorHigh, and 
		//rightMotor speed to be motorLow.
		else if(this.distance < bandCenter-bandwidth){	
			leftMotor.setSpeed(motorHigh);
			rightMotor.setSpeed(motorLow);
			
			//Turn both motors forward. In doing this, the robot will turn right.
			leftMotor.forward();
			rightMotor.forward();						
		}
		
		//This conditional statement will execute if the robot is too far from the wall.
		//To get it closer to the wall, we want it to turn left.To do this, we need to make the right
		//motor turn faster than the left one. Hence we set the rightMotor speed to be motorHigh, and 
		//leftMotor speed to be motorLow.
		else if(this.distance > bandCenter+bandwidth){		//When robot is too far from the wall.
			leftMotor.setSpeed(motorLow);
			rightMotor.setSpeed(motorHigh);
			
			//Turn both motors forward. In doing this, the robot will turn left.
			leftMotor.forward();
			rightMotor.forward();						
		}
		
		//This final conditional statement will execute if the robot is exactly the right distance from the wall.
		//i.e. It it is following the wall at its bandCenter distance.
		//NOTE: Because our robot is exactly the right distance from the wall, we don't want to turn left or right.
		//We simply want both motors to turn at the same speed. Hence, we set both motors to motorHigh speed value.
		else{
			leftMotor.setSpeed(motorHigh);				
			rightMotor.setSpeed(motorHigh);
			
			//Turn both motors forward. In doing this, the robot will move forward in a straight line.
			leftMotor.forward();
			rightMotor.forward();				
			
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
