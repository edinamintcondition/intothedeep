package org.firstinspires.ftc.riseofbabyminty;

//arm motor code

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

    public LplusRatio(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.lagarra = hardwareMap.get(DcMotor.class, "armmotor");
        lagarra.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //add actual arm code
    public void teutonicknight() {
        double armposition = -gamepad.left_stick_y;
        telemetry.addData("armposition", armposition);
        lagarra.setPower(armposition);
        telemetry.addData("armpower", lagarra.getPower());
    }
}
