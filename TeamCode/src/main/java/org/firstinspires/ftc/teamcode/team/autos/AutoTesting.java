package org.firstinspires.ftc.teamcode.team.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;


@Autonomous
public class AutoTesting extends DarienOpModeAuto {

    public void runOpMode() {
        initControls();

        waitForStart();

        moveXY(24, 0, 0.3);
        waitForMotors();

        moveXY(0, 24, 0.3);
        waitForMotors();

        moveXY(12, 12, 0.3);
        waitForMotors();

        moveXY(-12, -6, 0.3);
        waitForMotors();

//        autoRotate(PI, 0.3);
//
//        autoRotate(PI/2, 0.3);
//
//        autoRotate(-PI/2, 0.3);

    }

}
