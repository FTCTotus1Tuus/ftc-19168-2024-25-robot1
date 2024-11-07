package org.firstinspires.ftc.teamcode.team.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;


@Config
@Autonomous
@Disabled
public class movementTestingAuto extends DarienOpModeAuto {
    public static double x = 0;
    public static double y = 0;

    public void runOpMode() {
        initControls();

        waitForStart();

        while (opModeIsActive()) {
            moveXY(x, y, normalPower);
            waitForMotors();
        }
    }
}
