package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class DriverControl extends DarienOpModeTeleop{
public CRServo wrist;
public Servo rightClaw;
public static double rightPos = 0.66;
public static double leftPos = 0.44;
    public void runOpMode(){
        initControls(false);
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        wrist = hardwareMap.get(CRServo.class, "Wrist");
        waitForStart();

        //Start
        while (this.opModeIsActive()) {
            /*
            runIntakeSystem();
            runFeederSystem();
            runWristSystem();
            runClawSystem();
            runArmSystem();
            */

            runDriveSystem();
            moveWrist();
            moveClaw();

//            runServoTest();

            //runDroneSystem();

            //runMacro("ReadyToPickup");
            //runMacro("ReadyToDrop");
        }
    }

    public void moveWrist(){
        // press A to move the wrist down, when nothing pressed the wrist goes up

        double wristpower = gamepad1.right_trigger - gamepad1.left_trigger;
        wrist.setPower(wristpower);
    }
    public void moveClaw(){
        // press right bumper to open claw, press left bumper to close claw
        if (gamepad1.right_bumper==true){
            rightClaw.setPosition(rightPos);
        }
        else if (gamepad1.left_bumper==true){
            rightClaw.setPosition(leftPos);
        }
    }
}