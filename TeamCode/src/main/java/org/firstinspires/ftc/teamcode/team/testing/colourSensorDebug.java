package org.firstinspires.ftc.teamcode.team.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Disabled
@Autonomous
public class colourSensorDebug extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initControls(true);

        waitForStart();
        while(opModeIsActive()) {
        telemetry.addData("Blue:" , isOnLine(true));
        telemetry.addData("Red:" , isOnLine(false));

        telemetry.addData("leftSensor blue", colourSensorLeft.blue());
        telemetry.addData("leftsensor red", colourSensorLeft.red());

        telemetry.addData("rightSensor blue", colourSensorRight.blue());
        telemetry.addData("rightsensor red", colourSensorRight.red());

        telemetry.update();
}}}
