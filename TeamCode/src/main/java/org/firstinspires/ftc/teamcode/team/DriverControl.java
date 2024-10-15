package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class DriverControl extends DarienOpModeTeleop{
    public void runOpMode(){
        initControls(false);
        waitForStart();

        //Start
        while (this.opModeIsActive()) {


            runDriveSystem();


//            runServoTest();


            //runMacro("ReadyToPickup");
            //runMacro("ReadyToDrop");
        }
    }
}