package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


//THIS PROGRAM IS OUT OF USE (NOT BEING USED)!!!!!!!!!!!!!!!!!!!!!!!!


@TeleOp
public class NowInColor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //starts (or waiting for start), identify the color sensor
        RevColorSensorV3 rainbowdashsensor = hardwareMap.get(RevColorSensorV3.class, "colorsensor1");
        double color = 0;
        //preparation for rumbling when color spotted (probably dont need all this code but not entirely sure
        Gamepad.RumbleEffect customRumbleEffect;
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 1000)  //  Rumble
                .build();


        waitForStart();
        //inside while shall be the repeating actions (in this case, color-sensing :P)
        while (opModeIsActive()) {
            //not sure what these do
            double red = rainbowdashsensor.red();
            double green = rainbowdashsensor.green();
            double blue = rainbowdashsensor.blue();

            NormalizedRGBA detectedColor = rainbowdashsensor.getNormalizedColors();
            detectedColor.toColor();
            //debuggin :D
            telemetry.addData("colordetected", detectedColor.toColor());
            telemetry.addData("red", red);
            telemetry.addData("green", green);
            telemetry.addData("blue", blue);
            //check the normalized rgba value later
            telemetry.addData("detectedcolor.red", detectedColor.red);
            telemetry.addData("detectedcolor.blue", detectedColor.blue);
            telemetry.addData("detectedcolor.green", detectedColor.green);
            //testing rumble function (after color test

            //rumble function
            //red rumble
            if (red == 1 && blue == 0) {
                gamepad1.runRumbleEffect(customRumbleEffect);
                telemetry.addData("redrumble!", "");
            }
            //blue rumble
            else if (green == 1 && red == 0) {
                gamepad1.runRumbleEffect(customRumbleEffect);
                telemetry.addData("bluerumble!", "");
            }
            //green rumble
            else if (blue == 1 && green == 0 && red == 0) {
                gamepad1.runRumbleEffect(customRumbleEffect);
                telemetry.addData("greenrumble!", "");
            }
            telemetry.addData(">", "rumble? %s\n", gamepad1.isRumbling() ? "YES!!" : "nein");

//print data
            telemetry.update();

        }
    }

}
