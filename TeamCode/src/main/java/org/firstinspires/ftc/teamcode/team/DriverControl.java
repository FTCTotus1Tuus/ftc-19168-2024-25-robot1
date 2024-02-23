package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class DriverControl extends DarienOpModeTeleop{


    public void runOpMode(){

        initControls(false);

        waitForStart();

        //Start
        while (this.opModeIsActive()) {
            runIntakeSystem();
            runFeederSystem();
            runWristSystem();
            runClawSystem();
            runArmSystem();
            runDriveSystem();
            runDroneSystem();

            runMacro("ReadyToPickup");
            runMacro("ReadyToDrop");
        }
    }
}