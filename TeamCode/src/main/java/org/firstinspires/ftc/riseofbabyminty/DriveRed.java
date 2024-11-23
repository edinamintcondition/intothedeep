package org.firstinspires.ftc.riseofbabyminty;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DriveRed  extends LinearOpMode {
    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void runOpMode() {
        MegaHamburgerDrive wheels = new MegaHamburgerDrive(hardwareMap, telemetry, gamepad1);

        wheels.forward(0.5, 5);
        wheels.turnRight(2);
        wheels.forward(0.5, 2);
        wheels.turnLeft( 2);
        wheels.backwards(0.5, 2);
    }
}
