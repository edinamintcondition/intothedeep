package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SigmaAlphaScript", group = "B")
public class SigmaAlphaScript extends LinearOpMode {

    @Override
    public void runOpMode() {
        MegaHamburgerDrive wheels = new MegaHamburgerDrive(hardwareMap,telemetry, gamepad1);

        while (opModeIsActive()) {
            wheels.drivecarvroom();
            
            telemetry.update();
            //Start OpMode
            waitForStart();
        }

    }
}