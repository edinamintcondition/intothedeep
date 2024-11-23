package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DriveBlue extends LinearOpMode {

    public final static double TURN_SPEED = 0.5;
    public final static double TURN_TIME = 1.4;

    public final static double RUN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        MegaHamburgerDrive wheels = new MegaHamburgerDrive(hardwareMap, telemetry, gamepad1);
        Lagrandearmee hand = new Lagrandearmee(hardwareMap, telemetry, gamepad2);
        LplusRatio arm = new LplusRatio(hardwareMap, telemetry, gamepad2);

        waitForStart();
        hand.open();


        hand.close();

        wheels.forward(RUN_SPEED, 4);
        wheels.turnRight(TURN_SPEED, TURN_TIME);
        hand.open();

        wheels.forward(RUN_SPEED, 4);
        wheels.turnRight(TURN_SPEED, TURN_TIME);
        // arm.extend();
        hand.close();

        wheels.forward(0.8, 2);
        wheels.turnRight(TURN_SPEED, TURN_TIME * 2.8);
        hand.open();
        //arm.retract();

        wheels.backwards(RUN_SPEED, 4);
        wheels.turnRight(TURN_SPEED, TURN_TIME * 2.8);
        hand.close();


        wheels.stop();

        requestOpModeStop();
    }
}
