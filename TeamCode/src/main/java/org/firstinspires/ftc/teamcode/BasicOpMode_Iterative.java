/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new
 * name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Tshirt cannon tank", group = "Iterative Opmode")
//@Disabled
public class BasicOpMode_Iterative extends OpMode {
	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();

	private Victor88x rightDrive, rightDrive2;
	private Victor88x leftDrive;
	private Victor88x cannonAngle;
	private RelayController relay;

	private boolean triggerLatch = true;
	private final int LOOP_MAX = 75;
	private int loopCnt = LOOP_MAX;

	private long lastSystemTime = 0;
	private long lastControllerTime = 0;
	private double cnt = 50;



	/**
	 * Code to run ONCE when the driver hits INIT
	 */
	@Override
	public void init() {
		msStuckDetectLoop = 1000;
		rightDrive = new Victor88x(hardwareMap, "rightDrive");
		rightDrive2 =new Victor88x(hardwareMap, "rightDrive2");
		leftDrive = new Victor88x(hardwareMap, "leftDrive");
		cannonAngle = new Victor88x(hardwareMap, "cannonAngle");
		relay = new RelayController(hardwareMap, "relay");
		// Tell the driver that initialization is complete.
		gamepad1.refreshTimestamp();
		telemetry.addData("Status", "Initialized");
		telemetry.addData("ControllerTime", gamepad1.timestamp);

	}


	/**
	 * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
	 */
	@Override
	public void init_loop() {
		leftDrive.stop();
		rightDrive.stop();
		rightDrive2.stop();
		cannonAngle.stop();
		relay.setState(false);
	}


	/**
	 * Code to run ONCE when the driver hits PLAY
	 */
	@Override
	public void start() {
		runtime.reset();
	}


	/**
	 * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
	 */
	@Override
	public void loop() {
		double systemDelta = SystemClock.uptimeMillis() - lastSystemTime;
		double controllerDelta = gamepad1.timestamp - lastControllerTime;
		double specialRatio = systemDelta / controllerDelta;


//		if (controllerDelta == 0) {
//			cnt+= systemDelta;
//		} else {
//			cnt = 0;
//		}
//
//		if (cnt > 1500) {
//			leftDrive.stop();
//			rightDrive.stop();
//			rightDrive2.stop();
//			cannonAngle.stop();
//		} else {
			//Drive control
			double x = (gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x)) * .5;
			double y = (gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * .5;
			leftDrive.setSpeed(y - x);
			rightDrive.setSpeed(y + x);
			rightDrive2.setSpeed(y + x);
			leftDrive.setDirection(Victor88x.BACKWARD);

			//Angle control
			if (gamepad1.dpad_up) {
				cannonAngle.setSpeed(-.33);
			} else if (gamepad1.dpad_down) {
				cannonAngle.setSpeed(.2);
			} else {
				cannonAngle.stop();
			}


//		}

		lastSystemTime = SystemClock.uptimeMillis();
		lastControllerTime = gamepad1.timestamp;

		//Relay control
		if (gamepad1.right_trigger > .25 && !triggerLatch) {
			triggerLatch = true;
			loopCnt = 0;
		} else if (gamepad1.right_trigger < .25) {
			triggerLatch = false;
		}

		if (loopCnt < LOOP_MAX) {
			relay.setState(true);
			loopCnt+= systemDelta;
		} else {
			relay.setState(false);
		}

		telemetry.addData("SystemDelta", systemDelta);
		telemetry.addData("ControllerDelta", controllerDelta);
		telemetry.addData("SystemTime", SystemClock.uptimeMillis());
		telemetry.addData("ControllerTime", gamepad1.timestamp);
		telemetry.update();
	}


	/**
	 * Code to run ONCE after the driver hits STOP
	 */
	@Override
	public void stop() {
		leftDrive.stop();
		rightDrive.stop();
		cannonAngle.stop();
		relay.setState(false);
	}

}
