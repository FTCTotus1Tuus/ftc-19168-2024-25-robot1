package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class DriverControl extends DarienOpModeTeleop {
    @Override
    public void runOpMode() {
        initControls();
        waitForStart();
        // Initialize the servo positions and record the current position.
        sampleYawSetPosition(POS_SAMPLE_YAW_CENTER);
        intakeWristSetPosition(intakeWristUpPosition);
        specimenWrist.setPosition(specimenWristPickup);
        bucket.setPosition(bucketUp);
        //Start
        while (this.opModeIsActive()) {

            //pollSensors();
            runDriveSystem();
            runIntakeSystem();
            runVerticalSlideSystem();
            runSpecimenSystem();
            runLift1System();

            telemetry.update();


            //runMacro("ReadyToPickup");
            //runMacro("ReadyToDrop");
        }
    }
}