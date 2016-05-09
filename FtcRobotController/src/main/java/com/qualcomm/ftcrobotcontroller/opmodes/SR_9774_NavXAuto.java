package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

public abstract class SR_9774_NavXAuto extends LinearOpMode {
    //define motors, servos and sensors.
    protected DcMotor motorBackRight;
    //Right rear motor
    protected DcMotor motorFrontRight;
    //Right front motor
    protected DcMotor motorBackLeft;
    //Left rear motor
    protected DcMotor motorFrontLeft;
    //Left front motor
    protected DcMotor motorSweep;
    //Sweeper motor
    protected DcMotor motorArm;

    protected DcMotor motorHang;
    // arm motor

    protected Servo climberDrop;
    protected Servo redZip;
    protected Servo blueZip;
    protected TouchSensor touch;

    /* This is the port on the Core Device Interface Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */

    protected AHRS navx_device;
    private navXPIDController yawPIDController=null;
    private boolean calibration_complete = false;
    private DecimalFormat df = new DecimalFormat("#.##");

    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int DEVICE_TIMEOUT_MS = 500;


    private double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }


    public void initialize() {
        //Hardware map naming

        motorBackRight = hardwareMap.dcMotor.get("motor_1");
        motorBackLeft = hardwareMap.dcMotor.get("motor_2");

        motorFrontRight = hardwareMap.dcMotor.get("motor_7");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_6");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorArm = hardwareMap.dcMotor.get("motor_3");
        motorHang = hardwareMap.dcMotor.get("motor_5");
        motorSweep = hardwareMap.dcMotor.get("motor_4");

        climberDrop = hardwareMap.servo.get("servo_1");
        redZip = hardwareMap.servo.get("servo_2");
        blueZip = hardwareMap.servo.get("servo_3");
        touch = hardwareMap.touchSensor.get("touch_1");

        //Color

        //Set position for the servos:

        climberDrop.setPosition(0.2);
        //When initialize goes all the way back
        //When initialize covers climber drop mechanism
        redZip.setPosition(0.3);
        //When intialize goes back into robot
        blueZip.setPosition(0.5);
        //When intialize goes back into robot

        final int NAVX_DIM_I2C_PORT = 1;
        final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        /* If possible, use encoders when driving, as it results in more */
        /* predictable drive system response.                           */
        //leftMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //Set position for the servos:

    }

    public void navXCalibrate() {
        double MAX_CALIBRATION_TIME_IN_SEC = 5.0;

        ElapsedTime timer = new ElapsedTime();

        while (!calibration_complete) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
            if (timer.time()>MAX_CALIBRATION_TIME_IN_SEC) {
                if (!navx_device.isConnected()) {
                    telemetry.addData("navX-Micro", "Exiting calibration as NavX not connected !");
                } else {
                    telemetry.addData("navX-Micro", "Calibration timedout !");
                }
                break;
            }
        }
    }

    /*
        Method to move straight forward
     */
    public void forward(double totalRunTime) {
        /* Drive straight forward at 1/2 of full drive speed */
        double drive_speed = 0.5;

        yawPIDController = getNavXPIDController(0.0f); // Set angle to zero to move straight

        //navx_device.zeroYaw();

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        ElapsedTime runtime = new ElapsedTime();
        try {
            while ((runtime.time() < totalRunTime) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        wheelRotation(drive_speed,drive_speed);

                        telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
                                df.format(drive_speed));
                    } else {
                        double output = limit(yawPIDResult.getOutput());

                        wheelRotation(output, output);
                        telemetry.addData("PIDOutput", df.format(output) + ", " +
                                df.format(-output));
                    }
                    telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                } else {
                    /* A timeout occurred */
                    Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        } catch (NullPointerException e) {
            telemetry.addData("Exception", e.getStackTrace()[0].toString());
            throw e;
        } finally {
            yawPIDController.enable(false);

            wheelRotation(0,0);
            telemetry.addData("LinearOp", "Complete Forward");
        }
    }

    /*
        Method to reverse
     */
    public void reverse(double totalRunTime) {
        /* Drive straight forward at 1/2 of full drive speed */
        double drive_speed = 0.5;

        yawPIDController = getNavXPIDController(0.0f); // Set angle to zero to move straight

        //navx_device.zeroYaw();

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        ElapsedTime runtime = new ElapsedTime();
        try {
            while ((runtime.time() < totalRunTime) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        wheelRotation(-drive_speed, -drive_speed);

                        telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
                                df.format(drive_speed));
                    } else {
                        double output = limit(yawPIDResult.getOutput());

                        wheelRotation(-output, -output);

                        telemetry.addData("PIDOutput", df.format(limit(output)) + ", " +
                                df.format(limit(-output)));
                    }
                    telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                } else {
                    /* A timeout occurred */
                    Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        } catch (NullPointerException e) {
            telemetry.addData("Exception", e.getStackTrace()[0].toString());
            throw e;
        } finally {
            yawPIDController.enable(false);

            wheelRotation(0,0);
            telemetry.addData("LinearOp", "Complete Reverse");
        }
    }


    private navXPIDController getNavXPIDController(double angle) {
        yawPIDController = new navXPIDController(navx_device, navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(angle); // Adding to angle from sensor yawPIDController.getSetpoint()+
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        return yawPIDController;
    }

 /*   public void navXTurnAndMoveTime(double angle, double totalRunTime) {
        *//* Drive straight forward at 1/2 of full drive speed *//*
        double drive_speed = 0.5;

        yawPIDController = getNavXPIDController(angle);

        navx_device.zeroYaw();

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        ElapsedTime runtime = new ElapsedTime();
        try {
            while ((runtime.time() < totalRunTime) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        wheelRotation(drive_speed,drive_speed);

                        telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
                                df.format(drive_speed));
                    } else {
                        double output = limit(yawPIDResult.getOutput());
                        wheelRotation(output, -output);

                        telemetry.addData("PIDOutput", df.format(limit(output)) + ", " +
                                df.format(limit(-output)));
                    }
                    telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                } else {
                    *//* A timeout occurred *//*
                    Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        } catch (NullPointerException e) {
            telemetry.addData("Exception", e.getStackTrace()[0].toString());
            throw e;
        } finally {
            wheelRotation(0,0);
            telemetry.addData("LinearOp", "Complete 45 band 10");
        }
    }
*/
    public void navXTurn(double angle,double timeInSeconds) {
        yawPIDController = getNavXPIDController(angle);
        double drive_speed = 0.5;


        try {
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            ElapsedTime runtime = new ElapsedTime();
            while ((runtime.time() < timeInSeconds) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        setAllWheelPowerFloat();

                        telemetry.addData("PIDOutput#", "At Target");
                        break; // Completed turn so exist
                    } else {
                        double output = limit(yawPIDResult.getOutput());
                        wheelRotation(drive_speed+output, drive_speed-output);

                        telemetry.addData("PIDOutput>", df.format(output));
                    }
                } else {
                    /* A timeout occurred */
                    Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
                telemetry.addData("Yaw A="+angle+" ", df.format(navx_device.getYaw()));
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        } catch (NullPointerException e) {
            telemetry.addData("Exception", e.getStackTrace()[0].toString());
            throw e;
        } finally {
            yawPIDController.enable(false);
            wheelRotation(0, 0);
            telemetry.addData("turnX", "Turn Complete.Angle="+angle);
        }
    }
    /*
        Helper method to setting power for left and right motors
     */
    private void wheelRotation(double leftMotorSpeed,double rightMotorSpeed) {
        motorBackLeft.setPower(leftMotorSpeed);
        motorFrontLeft.setPower(leftMotorSpeed);

        motorBackRight.setPower(rightMotorSpeed);
        motorFrontRight.setPower(rightMotorSpeed);
    }

    /*
        Helper method to setPowerFloat for all four motors
     */
    private void setAllWheelPowerFloat() {
        motorBackRight.setPowerFloat();
        motorFrontRight.setPowerFloat();

        motorBackLeft.setPowerFloat();
        motorFrontLeft.setPowerFloat();
    }

    /*
        Method to drop climber
     */
    public void dropClimber() throws InterruptedException {
        //Climber is released into shelter after a a small pause, so that its not thrown out
        climberDrop.setPosition(0.9);
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("dropClimber", "Done");
    }
    /*
        Method to drop climber
     */
    public void resetClimber() throws InterruptedException {
        //Climber is released into shelter after a a small pause, so that its not thrown out
        climberDrop.setPosition(0.2);
        telemetry.addData("resetClimber", "Done");
    }

}