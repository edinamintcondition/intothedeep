package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
            // 2 tiles: speed = 0.85, secs = 2
            wheels.forward(0.85, 2);
            // 45 deg: speed = turn_speed (0.5) , secs =1.3
            wheels.turnLeft(TURN_SPEED, 1.2);
            wheels.forward(0.5, 1);
            wheels.turnLeft(TURN_SPEED, 1.2);
            wheels.forward(1, 1);
            //turn right to zone (change this so push one sample to the zone
            wheels.turnRight(TURN_SPEED, 0.8);
            wheels.forward(RUN_SPEED, 1);
            wheels.backwards(0.8, 1);
            //turn to parking and park!!!!!!!!! change this part to fit the right
            wheels.turnLeft(TURN_SPEED, 0.6);
            wheels.forward(RUN_SPEED, 1);
            wheels.turnLeft(0.85, 0.78);
            //park
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
