package org.firstinspires.ftc.riseofbabyminty;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//autonomous for left side, put sample in zone

@Autonomous
public class LeftArmtonomous extends LinearOpMode {

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
            wheels.forward(0.4, 1);
            arm.extend();
            hand.open();
            arm.retract();
            break;
            // requestOpModeStop();
        }
    }

    public void parkOnly(MegaHamburgerDrive wheels) {
        //park
        wheels.forward(0.67, 3);
    }
}

