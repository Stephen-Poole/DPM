
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {
	
	private final int bandCenter, bandwidth;
	//Increased the value of FILTER_OUT from 20 to 30 to make robot skip gaps more easily
	private final int motorStraight = 260, FILTER_OUT = 30;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int distance;
	private int filterControl;
	
	public PController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
					   int bandCenter, int bandwidth) {
		//Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		leftMotor.setSpeed(motorStraight);					// Initalize motor rolling forward
		rightMotor.setSpeed(motorStraight);
		leftMotor.forward();
		rightMotor.forward();
		filterControl = 0;
	}
	
	
	/*----------------------------------------
	CALC PROP METHOD 
	----------------------------------------*/
	//calcProp returns the amount that each motor needs to change speed, based on how 
	//far the robot is from the wall. The speed at which the robot will adjust is a function 
	//of its distance from the bandCenter. The distance between the robot's position and the
	//bandCenter is inputed as the first argument to the method, "diff". 
	//The second argument, "scale", is used to adjust the proportionality constant depending 
	//on the various possible situations. Namely, if the robot is much too far from the wall,
	//we will input a larger value of "scale" so that the robot can make its turn more slowly and
	//and smoothly.
	int calcProp (int diff, int scale) {
		//The proportionality constant, propConstant, will change based on the scale we provide it.
		//The higher the proporionality constant is, the more quickly the robot will turn.
		int propConstant=4/scale;
		//"correction" is the speed value we will return from the method that will adjust the speed 
		//of our robot.
		int correction;
		//We must also set a "maxCorrection" variable that will limit the max speed that can get returned
		//to our robot.
		int maxCorrection=80;
		
		//We want to return a positive speed, so if it is negative, we make it positive.
		if(diff<0) {
			diff = -diff;
		}
		//Get the value of correction to return based on the multiplication of "diff" with
		//our proportionality constant.
		correction = (int)(propConstant*(double)diff);
		//If our correction ends up being more than or equal to "maxCorrection", simply return maxCorrection.
		if(correction >= maxCorrection) {
			correction = maxCorrection;
		}
		
		//Return correction.
		return correction;
	}
	
	
	
	@Override
	public void processUSData(int distance) {
		
		// rudimentary filter - toss out invalid samples corresponding to null signal.
		// (n.b. this was not included in the Bang-bang controller, but easily could have).
		
		//Changed 255 to 90 since it isn't realistic that the robot will ever actually see
		//the full 255cm in the space that we have.
		if (distance > 90 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the filter value
			filterControl ++;
		} else if (distance > 90){
			// true 90, therefore set distance to 90
			this.distance = distance;
		} else {
			// distance went below 90, therefore reset everything.
			filterControl = 0;
			this.distance = distance;
		}
		
		// TODO: process a movement based on the us distance passed in (P style)
		//Change "distance" to "this.distance" to actually activate filter_out to skip gaps.
		
		//Changed "distance" to "this.distance" to activate FILTER_OUT for skipping gaps.
		//Set "distError" to be the difference between the distance from the wall and the bandCenter
		//(ideal distance from wall). "distError" is what we will use to see how far we are from wall,
		//and when to execute the different conditions below. If the robot is too far from the wall, 
		//"distError" will be positive. If it is too close, it will be negative.
		int distError=this.distance-bandCenter;
		
		//This conditional statement will execute if the robot is VERY far from the wall.
		//Our bandCenter is set to 29, so this will execute when the robot is more than 89 cm away.
		//NOTE: Because our robot is too far from wall, we will want it to turn left in this case. Hence,
		//the left motor will spin slower and the right motor will spin faster.
		if(distError>60) {		
			//Use the calcProp method to calculate how much to change speed based on our distance from 
			//bandCenter and set equal to "diff".
			//Here the second argument to calcProp will be 2 since for large distances we don't want the 
			//robot to turn too quickly.
			int diff=calcProp(distError,2);
			
			//Set speed for left motor to be the default speed (motorStraight) - the value returned from calcProp().
			//Want left motor to spin slower.
			int leftSpeed=motorStraight-diff;
			//Set speed for right motor to be the default speed (motorStraight) + the value returned from calcProp().
			//Want right motor to spin faster.
			int rightSpeed=motorStraight+diff;
			//Set speed of left motor to leftSpeed.
			leftMotor.setSpeed(leftSpeed);
			//Set speed of right motor to rightSpeed.
			rightMotor.setSpeed(rightSpeed);
			
			//Turn both motors forward.
			leftMotor.forward();
			rightMotor.forward();
			
			
			//This conditional statement will execute if the robot is too far from the wall, but not as far
			//away as in the case above.
			//NOTE: Because our robot is too far from wall, we will want it to turn left in this case. Hence,
			//the left motor will spin slower and the right motor will spin faster.
		}else if(distError>0 && distError<60) {		
			//Use the calcProp method to calculate how much to change speed based on our distance from 
			//bandCenter and set equal to "diff".
			//Here the second argument to calcProp will be 1 since we want the robot to turn more quickly
			//than in the previous case.
			int diff=calcProp(distError,1);
			
			//Set speed for left motor to be the default speed (motorStraight) - the value returned from calcProp().
			//Want left motor to spin slower.
			int leftSpeed=motorStraight-diff;
			//Set speed for right motor to be the default speed (motorStraight) + the value returned from calcProp().
			//Want right motor to spin faster.
			int rightSpeed=motorStraight+diff;
			//Set speed of left motor to leftSpeed.
			leftMotor.setSpeed(leftSpeed);
			//Set speed of right motor to rightSpeed.
			rightMotor.setSpeed(rightSpeed);
			
			//Turn both motors forward.
			leftMotor.forward();
			rightMotor.forward();	
		
			
			//This conditional statement will execute if the robot is VERY close to the wall.
			//NOTE: Because our robot is way too close to the  wall, we will want it to stop, and turn
			//right in place, so that it does not get any closer to the wall. Hence, we will use a 
			//similar approach to above, but we will get the right motor to spin backwards, and the left 
			//motor to spin forwards.
		}else if(distError<-8) {
			//Use the calcProp method to calculate how much to change speed based on our distance from 
			//bandCenter and set equal to "diff".
			//Here the second argument to calcProp will be 1 since we want the robot to turn quite quickly.
			int diff=calcProp(distError,1);
			
			//Set speed for right motor to be the default speed ((motorStraight) - the value returned from calcProp())/2.
			//We divide by 2 so that the robot doesn't jerk around as much (makes smoother).
			int rightSpeed=(motorStraight-diff)/2;
			//Set speed for left motor to be the default speed ((motorStraight) + the value returned from calcProp())/2.
			//We divide by 2 so that the robot doesn't jerk around as much (makes smoother).
			int leftSpeed=(motorStraight+diff)/2;
			//Set speed of left motor to leftSpeed.
			leftMotor.setSpeed(leftSpeed);
			//Set speed of right motor to rightSpeed.
			rightMotor.setSpeed(rightSpeed);
			
			//Turn left motor forward, and right motor backward so that the robot turns in place.
			leftMotor.forward();
			rightMotor.backward();					
				
			
			//This conditional statement will execute if the robot is close to the wall, but not as close as in the 
			//case above.
			//NOTE: Because our robot is too close to wall, we will want it to turn right in this case. Hence,
			//the left motor will spin faster and the right motor will spin slower.
		}else if(distError<0) {		
			//Use the calcProp method to calculate how much to change speed based on our distance from 
			//bandCenter and set equal to "diff".
			//Here the second argument to calcProp will be 1 since we want the robot to turn quite quickly.
			int diff=calcProp(distError,1);
			
			//Set speed for left motor to be the default speed (motorStraight) + the value returned from calcProp().
			//Want left motor to spin faster.
			int leftSpeed=motorStraight+diff;
			//Set speed for right motor to be the default speed (motorStraight) - the value returned from calcProp().
			//Want right motor to spin slower.
			int rightSpeed=motorStraight-diff;
			
			//Set speed of left motor to leftSpeed.
			leftMotor.setSpeed(leftSpeed);
			//Set speed of right motor to rightSpeed.
			rightMotor.setSpeed(rightSpeed);
			
			//Turn both motors forward.
			leftMotor.forward();
			rightMotor.forward();					
		
		
			//This final conditional statement will execute if the robot is exactly the right distance from the wall.
			//i.e. It it is following the wall at its bandCenter distance.
			//NOTE: Because our robot is exactly the right distance from the wall, we don't want to turn left or right.
			//We simply want both motors to turn at the same speed. This also means that we don't need to use the calcProp()
			//method.
		}else {		
			//Set speeds of left and right motors to be the default "motorStraight" speed.
			int leftSpeed=motorStraight;
			int rightSpeed=motorStraight;
			
			//Set speed of leftMotor to leftSpeed and rightMotor to rightSpeed.
			leftMotor.setSpeed(leftSpeed);
			rightMotor.setSpeed(rightSpeed);
			
			//Turn both motors forward.
			leftMotor.forward();
			rightMotor.forward();					
		}
		
		
		
	}

	
	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
