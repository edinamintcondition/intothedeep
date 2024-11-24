package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SigmaAlphaScript", group = "B")
public class SigmaAlphaScript extends LinearOpMode {

    @Override
    public void runOpMode() {
        MegaHamburgerDrive wheels = new MegaHamburgerDrive(hardwareMap, telemetry, gamepad1);
        Lagrandearmee hand = new Lagrandearmee(hardwareMap, telemetry, gamepad2);
        LplusRatio arm = new LplusRatio(hardwareMap, telemetry, gamepad2, hand);
        //wait for??? Start OpMode
        waitForStart();

        while (opModeIsActive()) {
            //invoking the magic car deities
            wheels.drivecarvroom();
            hand.wingedhussars();
            arm.teutonicknight();

            telemetry.update();
        }

    }
}
//beta?