package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class DriverControl extends DarienOpModeTeleop{
    @Override
    public void runOpMode(){
        initControls(false);
        waitForStart();
        //Start
        while (this.opModeIsActive()) {

            runDriveSystem();
            runIntakeSystem();
            //runVerticalSlideSystem();


            //runMacro("ReadyToPickup");
            //runMacro("ReadyToDrop");
        }
    }
}