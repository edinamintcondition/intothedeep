package org.firstinspires.ftc.riseofbabyminty;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.time.LocalTime;
import java.time.ZoneId;
import java.time.temporal.ChronoUnit;
import java.time.temporal.TemporalField;
import java.time.temporal.TemporalUnit;
import java.util.Calendar;
import java.util.Date;


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


        //step:2 configurate hamburgers, right front is configurated differently because robot motor is goofy
        leftFrontHamburger.setDirection(DcMotor.Direction.REVERSE);
        leftBackHamburger.setDirection(DcMotor.Direction.REVERSE);
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

        double leftFrontHamburgerPower = (axial * 0.75) + (lateral * 0.75) + (yaw * 2);
        double leftBackHamburgerPower = (axial * 0.75) - (lateral * 0.75) + (yaw * 2);
        //reconfigure below if not working
        double rightFrontHamburgerPower = -(axial * 0.75) + (lateral * 0.75) + (yaw * 2);
        //
        double rightBackHamburgerPower = (axial * 0.75) + (lateral * 0.75) - (yaw * 2);

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

    public void forward(double speed, int seconds) {
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData("Going forward for %0.2f seconds ", seconds);
        telemetry.update();
        while (runtime.seconds() <= seconds) {
            leftFrontHamburger.setPower(speed);
            leftBackHamburger.setPower(speed);
            rightFrontHamburger.setPower(-speed);
            rightBackHamburger.setPower(speed);
        }
    }

    public void backwards(double speed, int seconds) {
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData("Going backwards for %0.2f seconds ", seconds);
        telemetry.update();
        while (runtime.seconds() <= seconds) {
            leftFrontHamburger.setPower(-speed);
            leftBackHamburger.setPower(-speed);
            rightFrontHamburger.setPower(speed);
            rightBackHamburger.setPower(-speed);
        }
    }

    public void turnRight(double speed, double seconds) {
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData("Turning right for %0.2f seconds ", seconds);
        telemetry.update();
        while (runtime.seconds() <= seconds) {
            leftFrontHamburger.setPower(speed);
            leftBackHamburger.setPower(speed);
            rightFrontHamburger.setPower(speed);
            rightBackHamburger.setPower(-speed);
        }
    }

    public void turnLeft(double speed, double seconds) {
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData("Turning left for %0.2f seconds ", seconds);
        telemetry.update();

        while (runtime.seconds() <= seconds) {
            leftFrontHamburger.setPower(-speed);
            leftBackHamburger.setPower(-speed);
            rightFrontHamburger.setPower(-speed);
            rightBackHamburger.setPower(speed);
        }
    }

    public void stop() {
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData("Stopping", "");
        telemetry.update();

        leftFrontHamburger.setPower(0);
        leftBackHamburger.setPower(0);
        rightFrontHamburger.setPower(0);
        rightBackHamburger.setPower(0);
    }
}

