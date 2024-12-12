package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@TeleOp
@Config
@Disabled
public class MotorsDebug extends DarienOpModeAuto {

    public static double servoPosition = 0;

    public void runOpMode() {
        initControls();
        waitForStart();
        //setArmPosition(400, 0.1);
        //while (arm.isBusy()) {}
        //setArmPosition(0, 0.1);
        while (opModeIsActive()) {
            //print("arm Encoder", arm.getCurrentPosition());
            setBreakpoint();
            intakeWrist.setPosition(servoPosition);
        }
    }
}
