package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


// This is the code for the arm(ee)
public class Lagrandearmee {
    Gamepad gamepad;
    Telemetry telemetry;
    Servo magicservo;

    public Lagrandearmee(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        // step 1: all initial steps (hardwaremapwhee)
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.magicservo = hardwareMap.get(Servo.class, "clawservo");
        magicservo.setDirection(Servo.Direction.FORWARD);
    }


    //ADJUST INPUTS SO IT'S BUMPERS INSTEAD OF JOYSTICK and camera detects color
    public void wingedhussars() {
        double movevalue = gamepad.right_stick_y;
        ;
        telemetry.addData("movevalue", movevalue);
        //calculate servo
        double magicservoPosition = (movevalue / 2) + 0.5;

        //lorax -> }:

        magicservo.setPosition(magicservoPosition);

        telemetry.addData("armposition", magicservo.getPosition());
    }
    // sticking out your gamepad left stick Y for the rizzler
    // armbutton = gamepad1.left_stick_y
}


// :{ i speak for the trees
