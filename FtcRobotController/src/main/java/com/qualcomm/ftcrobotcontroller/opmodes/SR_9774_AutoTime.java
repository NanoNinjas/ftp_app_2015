package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public abstract class SR_9774_AutoTime extends LinearOpMode {
    //define motors, servos and sensors.
    protected DcMotor motorBackRight;
    //Right rear motor
    protected DcMotor motorFrontRight;
    //Right front motor
    protected DcMotor motorBackLeft;
    //Left rear motor
   // protected DcMotor motorFrontLeft;
    //Left front motor
    protected DcMotor motorSweep;
    //Sweeper motor
    protected DcMotor motorArm;

   // protected DcMotor motorHang;
    // arm motor

    protected Servo climberDrop;
    protected Servo redZip;
    protected Servo blueZip;
    protected TouchSensor touch;

    protected GyroSensor sensorGyro;

    private final double DRIVE_SPEED = .40f;
    //.3 previously

    public void initialize() {
        //Hardware map naming

        motorBackRight = hardwareMap.dcMotor.get("motor_1");
        motorBackLeft = hardwareMap.dcMotor.get("motor_2");

      //  motorFrontRight = hardwareMap.dcMotor.get("motor_7");
       // motorFrontLeft = hardwareMap.dcMotor.get("motor_6");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
      //  motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorArm = hardwareMap.dcMotor.get("motor_3");
        //motorHang = hardwareMap.dcMotor.get("motor_5");
        motorSweep = hardwareMap.dcMotor.get("motor_4");

        climberDrop = hardwareMap.servo.get("servo_2");
        climberDrop.setDirection(Servo.Direction.REVERSE);

        redZip = hardwareMap.servo.get("servo_1");
        blueZip = hardwareMap.servo.get("servo_3");
        touch = hardwareMap.touchSensor.get("touch_1");

        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        // calibrate the gyro.
        sensorGyro.calibrate();

        // make sure the gyro is calibrated.
        while (sensorGyro.isCalibrating()) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
            }
        }
        // Defining servos and motors

        //Set position for the servos:

        climberDrop.setPosition(0.8);
        redZip.setPosition(0.7);
        blueZip.setPosition(0.6);

    }
    /*
        Helper method to setting power for left and right motors
     */
    private void wheelRotation(double leftMotorSpeed,double rightMotorSpeed) {

        motorBackLeft.setPower(leftMotorSpeed);
       // motorFrontLeft.setPower(leftMotorSpeed);

        motorBackRight.setPower(rightMotorSpeed);
        motorFrontRight.setPower(rightMotorSpeed);
    }

    public void forward(float inches) throws InterruptedException {
        Forwards(inches, false);
    }

    public void reverse(float inches) throws InterruptedException {
        Forwards(inches, true);
    }

    private void Forwards(float inches, boolean isBackward) throws InterruptedException {
        if (!isBackward) {
            wheelRotation(-DRIVE_SPEED, -DRIVE_SPEED);

            sleep((long) (inches * 50.9554));

            wheelRotation(0f, 0f);
            sleep(100);
        } else {
            wheelRotation(DRIVE_SPEED, DRIVE_SPEED);

            sleep((long) (inches * 50.9554));

            wheelRotation(0f, 0f);
            sleep(100);
        }
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Forwards", (inches * 50.9554));
    }


    public void left(float degree) throws InterruptedException {
        turnOnCenter(degree, true);
    }

    public void right(float degree) throws InterruptedException {
        turnOnCenter(degree,false);
    }

    private void turnOnCenter(float degrees, boolean isLeft) throws InterruptedException {
        float inches = (float) ((degrees) * (Math.PI * 20) / 180);
        if (!isLeft) {
            wheelRotation(DRIVE_SPEED, -DRIVE_SPEED);
        } else {
            wheelRotation(-DRIVE_SPEED, DRIVE_SPEED);
        }
        sleep((long) (inches * 50.9554));

        wheelRotation(0f, 0f);

        sleep(100);
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("turnOnCenter", (inches * 50.9554));
    }

    //telemetry for turnoncenter
    public void dropClimber() throws InterruptedException {
        //Climber is released into shelter after a a small pause...
        climberDrop.setPosition(0.1);

        sleep(500);

        climberDrop.setPosition(0.9);
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("dropClimber", "Done");
        //telemetry for dropClimber
    }

    //Gyro turn specific
    public void turnGyro(double requiredHeading) throws InterruptedException {

        double MIN_ROTATE_POWER = 0.3;
        double MIN_TOLERANCE_ANGLE = 4.0;
        double power;
        // calibrate the gyro.
        sensorGyro.calibrate();
        // make sure the gyro is calibrated.
        while (sensorGyro.isCalibrating()) {
            Thread.sleep(50);
        }
        sensorGyro.resetZAxisIntegrator();

        double currentHeading = sensorGyro.getHeading();

        while (true) {
            if(currentHeading > requiredHeading+(2*MIN_TOLERANCE_ANGLE)){
                currentHeading = currentHeading-360;
            }

            if (currentHeading  > (requiredHeading-MIN_TOLERANCE_ANGLE)) {
                break;
            }

            // choose rotation direction
            if (requiredHeading > currentHeading) {
                power = MIN_ROTATE_POWER;
            } else {
                power = -MIN_ROTATE_POWER;
            }

            wheelRotation(-power, power);

            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Power", power);
            telemetry.addData("Required Heading", requiredHeading);
            Thread.sleep(50);
            stopMotors();
            currentHeading = sensorGyro.getHeading();
        }//exit loop
        stopMotors();
    }

    private void stopMotors() {
        wheelRotation(0f, 0f);
    }


}