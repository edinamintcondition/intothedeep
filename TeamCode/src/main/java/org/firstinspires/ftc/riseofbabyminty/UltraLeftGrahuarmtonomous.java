package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//ultra autonmous for left side, combining left armtonomous and left grahutonomous
@Disabled
@Autonomous
public class UltraLeftGrahuarmtonomous extends LinearOpMode {

    public final static double TURN_SPEED = 0.5;
    public final static double TURN_TIME = 1.4;

    public final static double RUN_SPEED = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        CamaraOscura acamara = new CamaraOscura(hardwareMap, telemetry);
        MegaHamburgerDrive wheels = new MegaHamburgerDrive(hardwareMap, telemetry, gamepad1);
        Lagrandearmee hand = new Lagrandearmee(hardwareMap, telemetry, gamepad2, acamara);
        LplusRatio arm = new LplusRatio(hardwareMap, telemetry, gamepad2, hand);
        hand.close();

        waitForStart();
//        hand.open();
        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive()) {
            //from here, code was copied from armtonomous
            arm.extend();
            wheels.forward(0.4, 1);
            hand.open();
            sleep(300);
            arm.retract();
            wheels.turnRight(0.5, 0.7);
            wheels.backwards(0.85, 4);
            wheels.turnRight(TURN_SPEED, 1.3);

            //below was copied from grauhotonomous

            //initial venture to (BELOW IS WORK IN PROGRESS
            // 2 tiles: speed = 0.85, secs = 2
            wheels.forward(0.83, 2);
            // 90 deg: speed = turn_speed (0.5) , secs =1.3
            wheels.turnLeft(TURN_SPEED, 1.23);


            wheels.forward(0.35, 1);
            wheels.turnLeft(TURN_SPEED, 0.55);
            wheels.forward(0.70, 2);
            //turnright to zone
            wheels.turnRight(TURN_SPEED, 0.8);
            wheels.forward(0.4, 1);
            wheels.backwards(0.6, 1);

            //below is some experimental code to get a second block in
            wheels.turnLeft(0.37, 1);
            //drive backwards up to the sample
            wheels.backwards(0.65, 2);
            //turn to sample
            wheels.turnRight(TURN_SPEED, 0.85);
            //advance to sample
            wheels.forward(0.6, 1);
            //turn to sample in direction of the zone
            wheels.turnLeft(TURN_SPEED, 0.7);
            //push sample forth
            wheels.forward(RUN_SPEED, 2);
            //turn with sample to the zone
            wheels.turnRight(TURN_SPEED, 0.45);
            //pzush sample in zone
            wheels.forward(0.2, 1);
            //go backwards to continue travel towards parking
            wheels.backwards(0.6, 1);

            //turn to parking and park!!!!!!!!!
            wheels.turnLeft(TURN_SPEED, 0.5);
            //going to observation
            wheels.backwards(0.6, 1);

            break;
            // requestOpModeStop();
        }
    }

    public void parkOnly(MegaHamburgerDrive wheels) {
        //park
        wheels.forward(0.67, 3);
    }
}


