package org.firstinspires.ftc.riseofbabyminty;

//arm motor code

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LplusRatio {
    Gamepad gamepad;
    Telemetry telemetry;
    DcMotor lagarra;

    double MAX_SPEED = 0.30;

    public LplusRatio(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.lagarra = hardwareMap.get(DcMotor.class, "armmotor");
        lagarra.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //add actual arm code
    public void teutonicknight() {
        double armposition = -gamepad.left_stick_y;
        if (Math.abs(armposition) > MAX_SPEED) {
            if (armposition < 0) {
                armposition = -MAX_SPEED;
            } else {
                armposition = MAX_SPEED;
            }
        }
        telemetry.addData("armposition", armposition);
        lagarra.setPower(armposition);
        telemetry.addData("armpower", lagarra.getPower());
    }

    public void extend() throws InterruptedException {
        telemetry.addData("Extending arm", lagarra.getPower());
        telemetry.update();

        int newTarget = lagarra.getCurrentPosition() + 10;
        lagarra.setTargetPosition(newTarget);
        lagarra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lagarra.setPower(abs(0.10));

        while (lagarra.isBusy()) {
            telemetry.addData("Currently at", " at %7d", lagarra.getCurrentPosition());
            telemetry.addData("Running to", " %7d", newTarget);
            telemetry.update();
        }

        lagarra.setPower(0);
        telemetry.addData("Extending arm", lagarra.getCurrentPosition());
        telemetry.update();
        sleep(250);   // optional pause after each move.
    }

    public void retract() throws InterruptedException {
        telemetry.addData("Retracting arm", lagarra.getPower());

        lagarra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lagarra.setTargetPosition(500);
        lagarra.setPower(-0.20);
        lagarra.setMode(RUN_TO_POSITION);
        sleep(1);
    }
}
