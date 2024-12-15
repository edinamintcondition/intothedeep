package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//autonomous for left side, put sample in zone
@Autonomous
public class LeftGrahutonomous extends LinearOpMode {

    public final static double TURN_SPEED = 0.5;
    public final static double TURN_TIME = 1.4;

    public final static double RUN_SPEED = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        CamaraOscura acamara = new CamaraOscura(hardwareMap, telemetry);
        MegaHamburgerDrive wheels = new MegaHamburgerDrive(hardwareMap, telemetry, gamepad1);
        Lagrandearmee hand = new Lagrandearmee(hardwareMap, telemetry, gamepad2, acamara);
        LplusRatio arm = new LplusRatio(hardwareMap, telemetry, gamepad2, hand);

        waitForStart();
//        hand.open();
        ElapsedTime runtime = new ElapsedTime();


        hand.close();
        while (opModeIsActive()) {
//            }]
            //initial venture to (BELOW IS WORK IN PROGRESS
            // 2 tiles: speed = 0.85, secs = 2
            wheels.forward(0.83, 2);
            // 90 deg: speed = turn_speed (0.5) , secs =1.3
            wheels.turnLeft(TURN_SPEED, 1.23);


            wheels.forward(0.35, 1);
            wheels.turnLeft(TURN_SPEED, 0.7);
            wheels.forward(0.60, 2);
            //turnright to zone
            wheels.turnRight(TURN_SPEED, 0.8);
            wheels.forward(0.4, 1);
            wheels.backwards(0.6, 1);

            //below is some experimental code to get a second block in
            wheels.turnLeft(0.35, 1);
            //drive backwards up to the sample
            wheels.backwards(0.6, 2);
            //turn to sample
            wheels.turnRight(TURN_SPEED, 0.85);
            //advance to sample
            wheels.forward(0.35, 1);
            //turn to sample in direction of the zone
            wheels.turnLeft(TURN_SPEED, 0.7);
            //push sample forth
            wheels.forward(0.75, 2);
            //turn with sample to the zone
            wheels.turnRight(TURN_SPEED, 0.45);
            //push sample in zone
            wheels.forward(0.5, 1);
            //go backwards to continue travel towards parking
            wheels.backwards(0.6, 1);

            //turn to parking and park!!!!!!!!!
            wheels.turnLeft(TURN_SPEED, 0.8);
            wheels.forward(0.38, 1);
            wheels.turnLeft(0.7, 0.7);
            //park
            wheels.forward(0.67, 2);
            wheels.turnRight(TURN_SPEED, 0.5);
            wheels.forward(0.37, 1);
            wheels.turnLeft(0.37, 1);
            // sample code :
//        wheels.turnRight(TURN_SPEED, TURN_TIME);
//        hand.open();
//
//        wheels.forward(RUN_SPEED, 4);
//        wheels.turnRight(TURN_SPEED, TURN_TIME);
//        // arm.extend();
//        hand.close();
//
//        wheels.forward(0.8, 2);
//        wheels.turnRight(TURN_SPEED, TURN_TIME * 2.8);
//        hand.open();
//        //arm.retract();
//
//        wheels.backwards(RUN_SPEED, 4);
//        wheels.turnRight(TURN_SPEED, TURN_TIME * 2.8);
//        hand.close();


            wheels.stop();
            break;
            // requestOpModeStop();
        }
    }

    public void parkOnly(MegaHamburgerDrive wheels) {
        //park
        wheels.forward(0.67, 3);
    }
}
