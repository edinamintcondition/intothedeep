package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


// This is the code for the wheels (hambibweurh)

public class MegaHamburgerDrive {

    Gamepad gamepad;
    Telemetry telemetry;

    DcMotor leftFrontHamburger;
    DcMotor leftBackHamburger;
    DcMotor rightFrontHamburger;
    DcMotor rightBackHamburger;

    public MegaHamburgerDrive(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        //step:1 add hamburgers
        leftFrontHamburger = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackHamburger = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontHamburger = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackHamburger = hardwareMap.get(DcMotor.class, "right_back_drive");


        //step:2 configurate hamburgers
        leftFrontHamburger.setDirection(DcMotor.Direction.FORWARD);
        leftBackHamburger.setDirection(DcMotor.Direction.FORWARD);
        rightFrontHamburger.setDirection(DcMotor.Direction.REVERSE);
        rightBackHamburger.setDirection(DcMotor.Direction.FORWARD);

    }

    //am cooked fr fr
    public void drivecarvroom() {
        // drive wheel function
        //to destroy the opps, we must cuisine them thickly in ratatouille seasoning
        // the opps must perish


        // step one to  establishing a dictatorship:
        // sway the people to your will and promise fake promises so they "vote" for you
        // step 2:
        // take over the nation once you are "elected"
        // step 3:
        // lead your nation to victory regardless of the people's sentiments and crush everyone
        //step 4:
        // eat hamburger

//        waitForStart();
        //step: 4 hamburger begin spin with values
//        while (opModeIsActive()) {

        double axial = -gamepad.left_stick_y;
        double lateral = gamepad.left_stick_x;
        double yaw = gamepad.right_stick_x;

//debugging thingy

        telemetry.addData("axialvalue", axial);
        telemetry.addData("lateralvalue", lateral);
        telemetry.addData("yawvalue", yaw);

        double leftFrontHamburgerPower = axial + lateral + (yaw * 5);
        double leftBackHamburgerPower = axial - lateral + (yaw * 5);
        double rightFrontHamburgerPower = -axial + lateral + (yaw * 5);
        double rightBackHamburgerPower = axial + lateral - (yaw * 5);

//          double rightFrontHamburgerPower = 0.53;

        // move motor
        leftFrontHamburger.setPower(leftFrontHamburgerPower);
        leftBackHamburger.setPower(leftBackHamburgerPower);
        rightBackHamburger.setPower(rightBackHamburgerPower);
        rightFrontHamburger.setPower(rightFrontHamburgerPower);

        telemetry.addData("LFrontHamburgerPower: ", leftFrontHamburgerPower);
        telemetry.addData("RFrontHamburgerPower: ", rightFrontHamburgerPower);
        telemetry.addData("LBackHamburgerPower: ", leftBackHamburgerPower);
        telemetry.addData("RBackHamburgerPower: ", rightBackHamburgerPower);
//
//        }
    }
}

