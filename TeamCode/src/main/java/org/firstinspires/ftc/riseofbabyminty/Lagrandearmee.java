package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
// This is the code for the arm(ee)
public class Lagrandearmee extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // step 1: all initial steps (hardwaremapwhee)
        Servo magicservo = hardwareMap.get(Servo.class, "servo1");
        magicservo.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            double movevalue = -gamepad1.left_stick_y;
            telemetry.addData("movevalue", movevalue);
            //calculate servo
            double magicservoPosition = (movevalue / 2) + 0.5;
            //step 2: desired robotic task to be executed below within these curly brackets
            // }: moustachio (the lorax?)
//            if (movevalue < -0.99) {
//                magicservo.setPosition(0);
//            }
//            if (movevalue == 0.5) {
//                magicservo.setPosition(0);
//            } else if (movevalue > 0.25 && movevalue < 0.99) {
//                magicservo.setPosition(0.5);
//            } else if (movevalue > 0.99 && movevalue > 0.25) {
//                magicservo.setPosition(1.0);
//            }

            magicservo.setPosition(magicservoPosition);

            telemetry.addData("armposition", magicservo.getPosition());
            telemetry.update();
        }


    }
}


// :{ i speak for the trees
