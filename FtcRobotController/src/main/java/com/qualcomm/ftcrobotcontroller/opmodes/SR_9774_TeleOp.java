/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode for 9774
 */
public class SR_9774_TeleOp extends OpMode {
    //define motors, servos and sensors.
	DcMotor motorBackRight;
	//Right rear motor
	//DcMotor motorFrontRight;
	//Right front motor
	DcMotor motorBackLeft;
	//Left rear motor
	DcMotor motorFrontLeft;
	//Left front motor
	DcMotor motorSweep;
	//Sweeper motor
	DcMotor motorArm;

	//DcMotor motorHang;
	// arm motor

	Servo climberDrop;
	Servo redZip;
	Servo blueZip;
	TouchSensor touch;

	/**
	 * Constructor
	 */
	public SR_9774_TeleOp() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name.
		 */
		//Hardware map naming

		motorBackRight = hardwareMap.dcMotor.get("motor_1");
		motorBackLeft = hardwareMap.dcMotor.get("motor_2");

		//motorFrontRight = hardwareMap.dcMotor.get("motor_7");
		motorFrontLeft = hardwareMap.dcMotor.get("motor_6");

		motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
		motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

		motorArm = hardwareMap.dcMotor.get("motor_3");
		//motorHang = hardwareMap.dcMotor.get("motor_5");
		motorSweep = hardwareMap.dcMotor.get("motor_4");

		climberDrop = hardwareMap.servo.get("servo_2");
		redZip = hardwareMap.servo.get("servo_1");
		blueZip = hardwareMap.servo.get("servo_3");
		touch = hardwareMap.touchSensor.get("touch_1");

		climberDrop.setDirection(Servo.Direction.REVERSE);
        redZip.setDirection(Servo.Direction.REVERSE);
		//Set position for the servos:

		climberDrop.setPosition(0.8);
		//When initialize goes all the way back
		//When initialize covers climber drop mechanism
		redZip.setPosition(0.);
		//When intialize goes back into robot
		blueZip.setPosition(0.45);
		//When intialize goes back into robot
	}

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		float right = -gamepad1.right_stick_y;
		float left =  -gamepad1.left_stick_y;

		//giving power to the right stick on gamepad1
		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		right = (float)scaleInput(right);
		left =  (float)scaleInput(left);

		//This allows the joystick value help control the speed
		//to move more precisely at slower speeds
      //  motorFrontRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackRight.setPower(right);
        motorBackLeft.setPower(left);



		if (gamepad1.a) {
			climberDrop.setPosition(0.9);
			// update the position of the arm.
			//By clicking gamepad 1 a the climber drop is released outward, to drop climbers
		}

		if (gamepad1.y) {
			// update the position of the climberdrop.
			//By clicking gamepad 1 y the climber drop moves forward to drop climbers
			climberDrop.setPosition(0.1);
		}

		// update the position of the claw
		//if (gamepad1.x) {

		//}

		//if (gamepad1.b) {

		//}

		if (gamepad2.y) {
			//climberPosition += climberDelta;
			//blueZip.setPosition(0.45);
			redZip.setPosition(0.0);
			//Release red zip climber and retract blue zip climber
		}

		if (gamepad2.a){
			redZip.setPosition(0.45);
			//retract red zip climber
		}

		// Start Second Climber
		if (gamepad2.b) {
			blueZip.setPosition(0.45);
			//retract blue zip climber
		}


		if (gamepad2.x){
			blueZip.setPosition(0.0);
			//redZip.setPosition(0.2);
			//release blue zip climber retract red zip climber
		}


		if(touch.isPressed()|gamepad2.dpad_down) {
			//This prevents the spinner from spinning unless the touch sensor is being pressed.
			if (gamepad2.right_bumper) {
				//backwards override
				//By pressing gamepad 2 dpad down and right bumper (and holding)
				//spinner can now move backward
				motorSweep.setPower(.6);
			} else if (gamepad2.left_bumper) {
				//forwards override
				//By pressing gamepad 2 dpad down and left bumper (and holding)
				//spinner can now move forward
				motorSweep.setPower(-0.5);
			} else {
				motorSweep.setPower(0);
				//if override was not pressed the spinner will not move
			}
		}


		//NN: Assigning motor gear to gamepad 2 right stick y
		float armPower = gamepad2.right_stick_y/2;
		//NN: Clip range of gear to +/- 1
		armPower = Range.clip(armPower, -1, 1);
		//NN: scale value of gear
		armPower = (float) scaleInput(armPower);
		//NN: Setting motor power to 1/4 of variable gear
		motorArm.setPower(armPower);

		float HangPower = gamepad2.left_stick_y/2;
		//NN: Clip range of gear to +/- 1
		HangPower = Range.clip(HangPower, -1, 1);
		//NN: scale value of gear
		HangPower = (float) scaleInput(HangPower);
		//NN: Setting motor power to 1/4 of variable gear
		//motorHang.setPower(HangPower);

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
		telemetry.addData("Text", "*** Robot Data***");
		telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
		telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {
	}

	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}
}
