package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//this code isnt meant to be used yet
//the below will most likely get changed
@Autonomous
public class RightGrahutonomous extends LinearOpMode {

    public final static double TURN_SPEED = 0.5;
    public final static double TURN_TIME = 1.4;

    public final static double RUN_SPEED = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        MegaHamburgerDrive wheels = new MegaHamburgerDrive(hardwareMap, telemetry, gamepad1);
        Lagrandearmee hand = new Lagrandearmee(hardwareMap, telemetry, gamepad2);
        LplusRatio arm = new LplusRatio(hardwareMap, telemetry, gamepad2, hand);

        waitForStart();
//        hand.open();
        ElapsedTime runtime = new ElapsedTime();


        hand.close();
        while (opModeIsActive()) {
//            }]
            //initial venture to (BELOW IS WORK IN PROGRESS
            wheels.forward(0.6, 1);
            wheels.turnRight(TURN_SPEED, 1.3);
            wheels.forward(RUN_SPEED, 2);
            //reaches park, turns
            wheels.turnLeft(TURN_SPEED, 1.3);
            wheels.forward(RUN_SPEED, 2);
            //turns right away from sub
            wheels.turnRight(TURN_SPEED, 1.3);
            wheels.forward(0.5, 1);
            //turn to sample and push it to park
            wheels.turnRight(TURN_SPEED, 1.3);
            wheels.forward(RUN_SPEED, 2);
            //turn to drop off zone
            wheels.turnRight(TURN_SPEED, 1.3);
            //leave at zone
            wheels.forward(RUN_SPEED, 3);
            //turn around. go park
            wheels.turnRight(TURN_SPEED, 2.6);
            parkOnly(wheels);
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
        wheels.forward(0.8, 4);
    }
}
