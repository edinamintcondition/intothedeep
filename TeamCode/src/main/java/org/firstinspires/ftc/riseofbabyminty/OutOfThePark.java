package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//below is solely the parking code, moves forward and parks
@Autonomous
public class OutOfThePark extends LinearOpMode {

    public final static double TURN_SPEED = 0.5;
    public final static double TURN_TIME = 1.25;

    public final static double RUN_SPEED = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        CamaraOscura acamara = new CamaraOscura(hardwareMap, telemetry);
        MegaHamburgerDrive wheels = new MegaHamburgerDrive(hardwareMap, telemetry, gamepad1);
        Lagrandearmee hand = new Lagrandearmee(hardwareMap, telemetry, gamepad2, acamara);
        LplusRatio arm = new LplusRatio(hardwareMap, telemetry, gamepad2, hand);

        waitForStart();
//        hand.open();


        hand.close();

        while (opModeIsActive()) {


            wheels.forward(RUN_SPEED, 3);
            //sample code:
//        wheels.turnLeft(TURN_SPEED, TURN_TIME);
//        hand.open();
//
//        wheels.forward(RUN_SPEED, 4);
//        wheels.turnLeft(TURN_SPEED, TURN_TIME);
//        //arm.extend();
//        hand.close();
//
//        wheels.forward(0.8, 2);
//        wheels.turnLeft(TURN_SPEED, TURN_TIME);
//        hand.open();
//
//        //arm.retract();
//        wheels.forward(RUN_SPEED, 4);
//        wheels.turnLeft(TURN_SPEED, TURN_TIME);
//        hand.close();
            wheels.stop();
            break;

        }
    }
}
