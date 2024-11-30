package org.firstinspires.ftc.teamcode.team.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class testResetEncoders extends DarienOpModeAuto {
    public void runOpMode() {
        initControls();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                resetEncoderNoStop();
            }
            telemetry.addData("wheel 0", omniMotor0.getCurrentPosition());
            telemetry.addData("wheel 1", omniMotor1.getCurrentPosition());
            telemetry.addData("wheel 2", omniMotor2.getCurrentPosition());
            telemetry.addData("wheel 3", omniMotor3.getCurrentPosition());
            telemetry.update();
        }
    }
}
