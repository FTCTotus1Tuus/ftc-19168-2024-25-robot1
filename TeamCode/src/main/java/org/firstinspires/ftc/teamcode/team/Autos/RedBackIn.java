package org.firstinspires.ftc.teamcode.team.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class RedBackIn extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initCamera(false);
        initControls(true);

        int propPosition;

        waitForStart();

        propPosition = getPropPosition();
        print("Prop", teamPropMaskPipeline.getLastResults());

        autoRunMacro("ReadyToPickup");
        setClawPosition("leftClosed"); // makes sure that the purple pixel is picked up
        MoveY(27, 0.4); //centers on spike tile
            setArmPosition(300, 0.3); // extends the arm
                 while(arm.isBusy()){print ("arm pos", arm.getCurrentPosition());}
            setWristPosition("dropGround"); // extends the wrist
        waitForMotors();
        switch (propPosition) {
            case 1:
                AutoRotate(90, 0.3,-1); // turns to spike mark
                MoveY(2,0.1);
                waitForMotors();
                autoRunMacro("dropPixel"); // places the purple pixel on the ground
                MoveY(-26, 0.3);  // moves 1 tile back to be facing the backdrop
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                    while(arm.isBusy()){}
                    setClawPosition("closed"); // grabs yellow pixel
                    waitForMotors();
                AutoRotate(-90, 0.3, 1);
                MoveX(-5, 0.3);
                waitForMotors();
                break;
            case 2:
                autoRunMacro("dropPixel"); // places the pixel
                MoveX(24, 0.3); // goes to backdrop
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                    while(arm.isBusy()){}
                    setClawPosition("closed"); // grabs yellow pixel
                waitForMotors();
                AutoRotate(-90, 0.3 ,1); // turns to face the backdrop
                MoveX(-3.5, 0.3); // strafe left to be centered on position 2
                waitForMotors();
                MoveY(1.5, 0.3); // move up to the backdrop
                waitForMotors();
                break;
            case 3:
                AutoRotate(-90, 0.3,1); // turns to spike mark
                MoveY(1.75, 0.1);
                waitForMotors();
                autoRunMacro("dropPixel"); // places the pixel
                MoveY(-1, 0.1);
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                MoveX(20, 0.3); // strafe right: moves in line with top case
                    while(arm.isBusy()){}
                    setClawPosition("closed"); // grabs yellow pixel
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
        setArmPosition(600, 0.3); // extends the arm a tiny bit
        while (arm.isBusy()) {}
        autoRunMacro("ReadyToDrop"); // extends the wrist
        setArmPosition(450, 0.3);
        alignBackPositions(true, propPosition);

        print("pls no crash","");

        park(false, true, propPosition);
    }
}
