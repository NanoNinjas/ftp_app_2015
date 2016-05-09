package com.qualcomm.ftcrobotcontroller.opmodes;


/**
 * This calls SR_9774_NavXAuto
 */
public class SR_9774_Blue_NavXAuto extends SR_9774_NavXAuto {


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();
            waitForStart();

            navXCalibrate();

            forward(1); // Go forward 0.5 sec

            navXTurn(-58, 2); // Turn right 110 degree and try that for maximum of 5 sec

            forward(2); // Go forward 0.5 sec

            navXTurn(-130, 2); // Turn right 110 degree and try that for maximum of 5 sec

            //forward(1); // Go forward 0.5 sec

//            reverse(1); // Reverse for 1 sec

/*            navXTurnAndMoveTime(0.0, .5); //navXTurn.
            navXTurnAndMoveTime(110, 7.3);*/

            //dropClimber();
        } catch (NullPointerException e) {
            telemetry.addData("Error",e.getStackTrace()[0].toString());
        } finally {
            //resetClimber();
            navx_device.close();
            telemetry.addData("LinearOp", "Final step Complete");
        }
    }

}