package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
@Config
@Disabled
public class forwardBack extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        waitForStart();

        while (opModeIsActive()) {
            moveToPosition(25, 0, normalPower);
            waitForMotors();
            moveToPosition(0, 0, normalPower);
            waitForMotors();

        }
    }
}
