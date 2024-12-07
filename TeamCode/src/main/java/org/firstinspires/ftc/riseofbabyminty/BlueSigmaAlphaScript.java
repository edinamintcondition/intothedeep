package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlueSigmaAlphaScript", group = "B")
public class BlueSigmaAlphaScript extends LinearOpMode {

    @Override
    public void runOpMode() {
        MegaHamburgerDrive wheels = new MegaHamburgerDrive(hardwareMap, telemetry, gamepad1);
        CamaraOscura camara = new CamaraOscura(hardwareMap, telemetry);
        Lagrandearmee hand = new Lagrandearmee(hardwareMap, telemetry, gamepad2, camara);
        LplusRatio arm = new LplusRatio(hardwareMap, telemetry, gamepad2, hand);
        //wait for??? Start OpMode
        waitForStart();

        while (opModeIsActive()) {
            //invoking the magic car deities
            wheels.drivecarvroom();
            hand.wingedhussars();
            arm.teutonicknight();
            //   camara.cameraOscuraQueMeQuiereComerMiCerebroMuyTriste();
            telemetry.update();
        }

    }
}
//beta?