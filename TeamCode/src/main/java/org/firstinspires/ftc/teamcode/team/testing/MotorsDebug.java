package org.firstinspires.ftc.teamcode.team.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@TeleOp
public class MotorsDebug extends DarienOpModeAuto {

    public void runOpMode() {
        initControls(true);
        waitForStart();
        setArmPosition(400, 0.1);
        while (arm.isBusy()) {}
        setArmPosition(0, 0.1);
        while (opModeIsActive()) {
    print("arm Encoder", arm.getCurrentPosition());
}}}
