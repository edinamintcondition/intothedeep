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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LplusRatio {
    Gamepad gamepad;
    Telemetry telemetry;
    DcMotor lagarra;
    Lagrandearmee hand;

    double MAX_SPEED = 0.30;

    public LplusRatio(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, Lagrandearmee hand) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.lagarra = hardwareMap.get(DcMotor.class, "armmotor");
        this.hand = hand;
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
//        if (armposition > 0) {
//            hand.close();
//        }
        lagarra.setPower(armposition);
        telemetry.addData("armpower", lagarra.getPower());
    }

    //take note: extend is not actually extending the arm, it's more of a... swinging it around its axis to
    // reach the destination... if it were extending it, the arm would be a linear slide, but it's not, so...
    // same goes for the retract method, just keep it in mind!!!!z
    public void extend() throws InterruptedException {
        ElapsedTime walktime = new ElapsedTime();
        telemetry.addData("Extending arm", lagarra.getPower());
        telemetry.update();
        while (walktime.seconds() <= 1.0) {
//        int newTarget = lagarra.getCurrentPosition() + 10;
//        lagarra.setTargetPosition(newTarget);
//        lagarra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lagarra.setPower(0.5);
        }
        lagarra.setPower(0);
        telemetry.addData("Extending arm", lagarra.getCurrentPosition());
        telemetry.update();
    } // optional pause after each move.

    public void retract() throws InterruptedException {
        ElapsedTime walktime = new ElapsedTime();
        telemetry.addData("Extending arm", lagarra.getPower());
        telemetry.update();
        while (walktime.seconds() <= 0.5) {
            lagarra.setPower(-0.5);
        }

        lagarra.setPower(0);
        telemetry.addData("Retracting arm", lagarra.getCurrentPosition());
        telemetry.update();
    }
    //we are done
}
