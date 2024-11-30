package org.firstinspires.ftc.teamcode.team.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class odometryTest extends DarienOpModeAuto {
    public void runOpMode() {
        initControls();
        waitForStart();

        moveToPosition(10, 0, normalPower);
        waitForMotors();
        moveToPosition(0, 10, normalPower);
        waitForMotors();
        autoRotate(90, normalPower);
        moveToPosition(0, 0, normalPower);
        waitForMotors();
        moveToPosition(5, -5, normalPower);
        waitForMotors();
    }

}
