package org.firstinspires.ftc.teamcode.team.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class BlueBackIn extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initCamera(true);
        initControls(true);

        int propPosition;

        waitForStart();

        propPosition = getPropPosition();
        MoveY(27, 0.6); //centers on spike tile
            print("Prop", teamPropMaskPipeline.getLastResults());
            autoRunMacro("ReadyToPickup");
            setClawPosition("leftClosed"); // makes sure that the purple pixel is picked up
            setArmPosition(500, 0.3); // extends the arm
                while(arm.isBusy()){print ("arm pos", arm.getCurrentPosition());}
            setWristPosition("dropGround"); // extends the wrist
        waitForMotors();

        switch (propPosition) {
            case 3:

                AutoRotate(-90, 0.4, 1); // turns to spike mark
                MoveY(3, 0.1); // move toward the spike mark
                waitForMotors();
                autoRunMacro("dropPixel"); // places the purple pixel on the ground
                MoveY(-23, 0.7);  // moves 1 tile back to be facing the backdrop
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                    while(arm.isBusy()){}
                    setClawPosition("closed"); // grabs yellow pixel
                waitForMotors();
                AutoRotate(90, 0.4, 1);
                MoveX(3.5, 0.3);
                waitForMotors();
                break;
            case 2:
                sleep(100);
                autoRunMacro("dropPixel"); // places the pixel
                AutoRotate(0,0.4, 0);

                MoveX(-24, 0.6); // goes 1 tile towards the pixel piles
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                    while(arm.isBusy()){}
                    setClawPosition("closed"); // grabs yellow pixel
                waitForMotors();
                AutoRotate(90, 0.4, -1);
                break;
            case 1:
                MoveY(2.5, 0.3);
                AutoRotate(90, 0.4, -1); // turns to spike mark
                MoveY(3, 0.1);
                waitForMotors();
                autoRunMacro("dropPixel"); // places the pixel
//                MoveY(-2.5, 0.1);
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                MoveX(-24, 0.6); // strafe left: moves in line with top case
                    while(arm.isBusy()){}
                    setClawPosition("closed"); // grabs yellow pixel
                waitForMotors();
                AutoRotate(90, 0.4, 1);
                MoveY(24, 0.6);
                waitForMotors();
                MoveX(16, 0.6);
                break;
        }

        // AT THIS POINT, THE ROBOT SHOULD BE FACING THE BACKDROP READY TO DROP IN THE RIGHT POSITION.
        setArmPosition(600, 0.75); // extends the arm a tiny bit
        while (arm.isBusy()) {}
        autoRunMacro("dropGround"); // extends the wrist
        setArmPosition(450, 0.5);

        alignBackPositions(true, propPosition);

        print("pls no crash", "");

        park(true, true, propPosition);
    }

}
