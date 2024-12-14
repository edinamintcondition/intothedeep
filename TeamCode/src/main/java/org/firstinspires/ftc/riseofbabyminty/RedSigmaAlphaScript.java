package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//red and blue are different because depending on what team, the arm has a function where the claw will clos
//automatically when it detects a sample of its own color
@TeleOp(name = "RedSigmaAlphaScript", group = "B")
public class RedSigmaAlphaScript extends LinearOpMode {

    @Override
    public void runOpMode() {
        MegaHamburgerDrive wheels = new MegaHamburgerDrive(hardwareMap, telemetry, gamepad1);
        CamaraOscura camara = new CamaraOscura(hardwareMap, telemetry);
        Lagrandearmee hand = new Lagrandearmee(hardwareMap, telemetry, gamepad2, camara);
        LplusRatio arm = new LplusRatio(hardwareMap, telemetry, gamepad2, hand);
        hand.close();
        //wait for??? Start OpMode
        waitForStart();

        while (opModeIsActive()) {
            //invoking the magic car deities
            wheels.drivecarvroom();
            hand.polishhussars();
            arm.teutonicknight();
            //   camara.cameraOscuraQueMeQuiereComerMiCerebroMuyTriste();
            telemetry.update();
        }

    }
}
//beta?