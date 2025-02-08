package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
@Config
public class deadwheelTesting extends DarienOpModeAuto {
    public static double xPos;
    public static double yPos;
    public static double hPos;
    public static double powerR;

    @Override
    public void runOpMode() throws InterruptedException {

        initControls();

        waitForStart();

        while (opModeIsActive()) {

            updatePosition();

            telemetry.addData("X pos", getXPos());
            telemetry.addData("Y pos", getYPos());
            telemetry.addData("H pos", getRawHeading());

            telemetry.update();

            moveToPosition(xPos, yPos, hPos, powerR);
            waitForMotors();
            setBreakpoint();


        }

    }
}
