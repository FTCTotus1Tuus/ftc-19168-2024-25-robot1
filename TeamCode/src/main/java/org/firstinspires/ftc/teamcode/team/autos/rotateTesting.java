package org.firstinspires.ftc.teamcode.team.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;


@Config
@Autonomous
@Disabled
public class rotateTesting extends DarienOpModeAuto {

    public static double targetPos;

    public void runOpMode() {
        initControls();

        waitForStart();

        while (opModeIsActive()) {
            setBreakpoint();

            autoRotate(targetPos, normalPower);
        }
    }
}
