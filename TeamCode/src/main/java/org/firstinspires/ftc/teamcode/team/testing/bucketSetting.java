package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;
import org.firstinspires.ftc.teamcode.team.DarienOpModeTeleop;


@Autonomous
@Config
@Disabled
public class bucketSetting extends DarienOpModeAuto {
    public static double testingPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initControls();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                bucket.setPosition(testingPosition);
            }
        }
    }
}
