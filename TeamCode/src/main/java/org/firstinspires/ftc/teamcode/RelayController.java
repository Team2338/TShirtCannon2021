package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class wraps the servo class to allow for a motor-like interface to the spark PWM motor
 * controller through the 5V PWM servo ports on the rev expansion hub.
 */
public class RelayController {
	public final boolean ON = true;
	public final boolean OFF = false;

	private final double ON_FLOAT = 1.0;
	private final double OFF_FLOAT = 0.5;

	private HardwareMap map;
	private Servo servo;


	public RelayController(HardwareMap map) {
		this.map = map;
	}


	public RelayController(HardwareMap map, String name) {
		this.map = map;
		setServo(name);
	}


	public void setServo(String name) {
		servo = map.get(Servo.class, name);
//		servo.scaleRange(MIN, MAX);
	}


	public void setState(boolean state) {
		if (state) servo.setPosition(ON_FLOAT);
		else servo.setPosition(OFF_FLOAT);
	}

//	/**
//	 * takes a double value and maps it to the servo output on the revhub
//	 *
//	 * @param speed double value between -1 and 1;
//	 */
//	public void setSpeed(double speed) {
//		speed *= direction;
//		if (speed > 1) speed = 1.0;
//		if (speed < -1) speed = -1.0;
//		servo.setPosition((speed + 1.0) / 2.0);
//
//	}
//
//
//	public void setDirection(int direction) {
//		if(direction > 0) this.direction = FORWARD;
//		else this.direction = BACKWARD;
//	}
//
//
//	public void stop() {
//		setSpeed(0);
//	}


	public double getRealOutput() {
		return servo.getPosition();
	}


	//DEBUG for finding scaled range for spark
	double min = 0.0;
	double max = 1.0;
	public void incrementMinRange() {
		min += .01;
		applyRange();
	}


	public void incrementMaxRange() {
		max -= .01;
		applyRange();
	}


	public void resetRange() {
		min = 0.0;
		max = 1.0;
		applyRange();
	}


	private void applyRange() {
		try {
			servo.scaleRange(min, max);
		} catch (IllegalArgumentException iae) {
			min = 0.0;
			max = 1.0;
			servo.scaleRange(min, max);
		}
	}

}
