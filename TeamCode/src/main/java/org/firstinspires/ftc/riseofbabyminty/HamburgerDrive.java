package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class HamburgerDrive extends LinearOpMode {
    //to destroy the opps, we must cuisine them thickly in ratatouille seasoning
    // the opps must perish
    public void runOpMode() {
        //step:1 add hamburgers

        DcMotor leftFrontHamburger = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackHamburger = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontHamburger = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackHamburger = hardwareMap.get(DcMotor.class, "right_back_drive");
        //step:2 configurate hamburgers
        leftFrontHamburger.setDirection(DcMotor.Direction.FORWARD);
        leftBackHamburger.setDirection(DcMotor.Direction.FORWARD);
        rightFrontHamburger.setDirection(DcMotor.Direction.REVERSE);
        rightBackHamburger.setDirection(DcMotor.Direction.REVERSE);

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        //frontpowerlimit
        // double HamburgerderLimit = 0.60;
        //backpowerlimit
        //   double ChezburgermerLimit = -0.60;

        // step:3 tell hamburger where to go using control (values)


        waitForStart();
        //step: 4 hamburger begin spin with values
        while (opModeIsActive()) {
            //below: og hamburger drive
//            if (gamepad1.x) {
//                if (gamepad1.right_bumper) {
//                    leftFrontHamburger.setPower(0.75);
//                    leftBackHamburger.setPower(0.75);
//                    rightBackHamburger.setPower(0.75);
//                    rightFrontHamburger.setPower(0.75);
//                } else if (gamepad1.left_bumper) {
//                    leftFrontHamburger.setPower(0.15);
//                    leftBackHamburger.setPower(0.15);
//                    rightBackHamburger.setPower(0.15);
//                    rightFrontHamburger.setPower(0.15);
//                } else {
//                    leftFrontHamburger.setPower(0.5);
//                    leftBackHamburger.setPower(0.5);
//                    rightBackHamburger.setPower(0.53);
//                    rightFrontHamburger.setPower(0.53);
//                }
//
//            } else {
//                leftFrontHamburger.setPower(0);
//                leftBackHamburger.setPower(0);
//                rightBackHamburger.setPower(0);
//                rightFrontHamburger.setPower(0);
//
//            }
//            if (gamepad1.y) {
//                if (gamepad1.right_bumper) {
//                    leftFrontHamburger.setPower(-0.75);
//                    leftBackHamburger.setPower(-0.75);
//                    rightBackHamburger.setPower(-0.75);
//                    rightFrontHamburger.setPower(-0.75);
//                } else if (gamepad1.left_bumper) {
//                    leftFrontHamburger.setPower(-0.15);
//                    leftBackHamburger.setPower(-0.15);
//                    rightBackHamburger.setPower(-0.15);
//                    rightFrontHamburger.setPower(-0.15);
//                } else {
//                    leftFrontHamburger.setPower(-0.5);
//                    leftBackHamburger.setPower(-0.5);
//                    rightBackHamburger.setPower(-0.5);
//                    rightFrontHamburger.setPower(-0.5);
//                }
//
//            } else {
//                leftFrontHamburger.setPower(0);
//                leftBackHamburger.setPower(0);
//                rightBackHamburger.setPower(0);
//                rightFrontHamburger.setPower(0);
//
//            }
//            while (gamepad1.right_stick_x < 0) {
//                leftFrontHamburger.setPower(-0.25);
//                leftBackHamburger.setPower(-0.25);
//                rightBackHamburger.setPower(0.25);
//                rightFrontHamburger.setPower(0.25);
//            }
//            while (gamepad1.right_stick_x > 0) {
//                leftFrontHamburger.setPower(0.25);
//                leftBackHamburger.setPower(0.25);
//                rightBackHamburger.setPower(-0.25);
//                rightFrontHamburger.setPower(-0.25);
//            }
            double leftFrontHamburgerPower = axial + lateral + (yaw * 5);
            double leftBackHamburgerPower = axial - lateral + (yaw * 5);
            double rightFrontHamburgerPower = -axial + lateral + (yaw * 5);
            double rightBackHamburgerPower = axial + lateral - (yaw * 5);

            double leftFrontHamburgerPower = 0.5;
            double leftBackHamburgerPower = 0.5;
            double rightBackHamburgerPower = 0.53;
            double rightFrontHamburgerPower = 0.53;

            // move motor
            leftFrontHamburger.setPower(leftFrontHamburgerPower);
            leftBackHamburger.setPower(leftBackHamburgerPower);
            rightBackHamburger.setPower(rightBackHamburgerPower);
            rightFrontHamburger.setPower(rightFrontHamburgerPower);


        }
    }
}


