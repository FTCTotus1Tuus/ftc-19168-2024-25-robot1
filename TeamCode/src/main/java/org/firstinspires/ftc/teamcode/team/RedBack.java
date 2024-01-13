package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedBack extends DarienOpModeAuto{
    @Override
    public void runOpMode() throws InterruptedException {
        initCamera(false);
        initControls(true);

        int propPosition;

        waitForStart();

        propPosition = getPropPosition();
        telemetry.addData("Prop", teamPropMaskPipeline.getLastResults());
        telemetry.update();
        autoRunMacro("ReadyToPickup");
        setClawPosition("leftClosed"); // makes sure that the purple pixel is picked up
        MoveY(24, 0.3); //centers on spike tile
            sleep(350);
            setArmPosition(250, 0.3); // extends the arm
            sleep(500);
            setWristPosition("dropGround"); // extends the wrist
        waitForMotors();
        switch (propPosition) {
            case 1:
                AutoRotate(90, 0.3,-1); // turns to spike mark
                autoRunMacro("dropPixel"); // places the purple pixel on the ground
                MoveY(-24, 0.3);  // moves 1 tile back to be facing the backdrop
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                AutoRotate(-90, 0.3, 1);
                MoveX(-5, 0.3);
                waitForMotors();
                break;
            case 2:
                autoRunMacro("dropPixel"); // places the pixel
                MoveX(24, 0.3); // goes to backdrop
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                AutoRotate(-90, 0.3 ,1); // turns to face the backdrop
                MoveX(-3.5, 0.3); // strafe left to be centered on position 2
                waitForMotors();
                MoveY(1.5, 0.3); // move up to the backdrop
                waitForMotors();
                break;
            case 3:
                AutoRotate(-90, 0.3,1); // turns to spike mark
                MoveY(3.25, 0.1);
                waitForMotors();
                autoRunMacro("dropPixel"); // places the pixel
                MoveY(-1, 0.1);
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                MoveX(20, 0.3); // strafe right: moves in line with top case
                waitForMotors();
                MoveY(28, 0.3);
                waitForMotors();
                AutoRotate(-90, 0.2, 0);
                MoveX(-24, 0.3); // in front of pixel spot - changing to center of pixel
                waitForMotors();
                break;
        }
//        AutoRotate(-90, 0.2, 0);
        // AT THIS POINT, THE ROBOT SHOULD BE FACING THE BACKDROP READY TO DROP IN THE RIGHT POSITION.
        autoRunMacro("ReadyToPickup");
        setArmPosition(-10,0.1);
        sleep(500);
            setClawPosition("closed"); // grabs yellow pixel
        sleep(250);
        setArmPosition(200, 0.1); // extends the arm a tiny bit
        while (leftArm.isBusy()) {}
        autoRunMacro("ReadyToDrop"); // extends the wrist
        setArmPosition(0, 0.3);

        alignBackPositions(true, propPosition);

        print("pls no crash","");
        switch (propPosition) {
            case 3:
                MoveX(23, 0.3);
                waitForMotors();
                break;
            case 2:
                MoveX(26, 0.3);
                waitForMotors();
                break;
            case 1:
                MoveX(31, 0.3);
                waitForMotors();
                break;
        }
        MoveY(10, 0.05);
        waitForMotors();
    }
}
