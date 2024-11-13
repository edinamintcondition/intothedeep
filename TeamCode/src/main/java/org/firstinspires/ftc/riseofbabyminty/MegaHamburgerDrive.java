package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



// This is the code for the wheels (hambibweurh)

public class MegaHamburgerDrive {
    public MegaHamburgerDrive() {}
    //am cooked
    public void drivecarvroom() {}

//    //to destroy the opps, we must cuisine them thickly in ratatouille seasoning
//    // the opps must perish
    public void runOpMode() {
//        //step:1 add hamburgers
//
//        DcMotor leftFrontHamburger = hardwareMap.get(DcMotor.class, "left_front_drive");
//        DcMotor leftBackHamburger = hardwareMap.get(DcMotor.class, "left_back_drive");
//        DcMotor rightFrontHamburger = hardwareMap.get(DcMotor.class, "right_front_drive");
//        DcMotor rightBackHamburger = hardwareMap.get(DcMotor.class, "right_back_drive");
//        //step:2 configurate hamburgers
//        leftFrontHamburger.setDirection(DcMotor.Direction.FORWARD);
//        leftBackHamburger.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontHamburger.setDirection(DcMotor.Direction.REVERSE);
//        rightBackHamburger.setDirection(DcMotor.Direction.REVERSE);

        // step one to  establishing a dictatorship:
        // sway the people to your will and promise fake promises so they "vote" for you
        // step 2:
        // take over the nation once you are "elected"
        // step 3:
        // lead your nation to victory regardless of the people's sentiments and crush everyone
        //step 4:
        // eat hamburger

        waitForStart();
        //step: 4 hamburger begin spin with values
        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

//debugging thingy

            telemetry.addData("axialvalue", axial);
            telemetry.addData("lateralvalue", lateral);
            telemetry.addData("yawvalue", yaw);

            double leftFrontHamburgerPower = axial + lateral + (yaw * 5);
            double leftBackHamburgerPower = axial - lateral + (yaw * 5);
            double rightFrontHamburgerPower = -axial + lateral + (yaw * 5);
            double rightBackHamburgerPower = axial + lateral - (yaw * 5);

//            double rightFrontHamburgerPower = 0.53;

            // move motor
            leftFrontHamburger.setPower(leftFrontHamburgerPower);
            leftBackHamburger.setPower(leftBackHamburgerPower);
            rightBackHamburger.setPower(rightBackHamburgerPower);
            rightFrontHamburger.setPower(rightFrontHamburgerPower);

            telemetry.addData("LFrontHamburgerPower: ", leftFrontHamburgerPower);
            telemetry.addData("RFrontHamburgerPower: ", rightFrontHamburgerPower);
            telemetry.addData("LBackHamburgerPower: ", leftBackHamburgerPower);
            telemetry.addData("RBackHamburgerPower: ", rightBackHamburgerPower);
            telemetry.update();
//
//        }
//    }
}
//
//
