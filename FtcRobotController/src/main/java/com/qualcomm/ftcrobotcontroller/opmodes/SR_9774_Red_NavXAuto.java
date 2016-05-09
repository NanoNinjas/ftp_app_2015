package com.qualcomm.ftcrobotcontroller.opmodes;



/**
 * This calls SR_9774_NavXAuto
 */
public class SR_9774_Red_NavXAuto extends SR_9774_NavXAuto {


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();
            waitForStart();

            navXCalibrate();
            sleep(10000);
            navXTurn(-103.0, 6); //navXTurn.

            //navXTurnAndMoveTime(-70, 2.3);

            dropClimber();
        } finally {
            navx_device.close();
            telemetry.addData("LinearOp", "Final step Complete");
        }
    }

}