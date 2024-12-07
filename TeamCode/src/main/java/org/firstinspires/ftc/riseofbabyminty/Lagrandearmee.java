package org.firstinspires.ftc.riseofbabyminty;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;


// This is the code for the arm(ee) (specifically, the wrist and claw, l+ratio is the actual arm motor
public class Lagrandearmee {
    Gamepad gamepad;
    Telemetry telemetry;
    Servo magicservo;
    CamaraOscura thecamara;
    //    Servo leftheppeservo;
//    Servo rightheppeservo;
    boolean isClosed = false;

    public Lagrandearmee(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, CamaraOscura acamara) {
        // step 1: all initial steps (hardwaremapwhee)
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.magicservo = hardwareMap.get(Servo.class, "clawservo");
        this.thecamara = acamara;
//        this.leftheppeservo = hardwareMap.get(Servo.class, "leftservo");
//        this.rightheppeservo = hardwareMap.get(Servo.class, "rightservo");
        magicservo.setDirection(Servo.Direction.FORWARD);
//        leftheppeservo.setDirection(Servo.Direction.FORWARD);
//        rightheppeservo.setDirection(Servo.Direction.FORWARD);
    }


    //ADJUST INPUTS SO camera detects color
    public void wingedhussars() {
        double magicservoPosition;
        if (gamepad.left_bumper && thecamara.IsBlue()) {
            magicservoPosition = 1;

        } else if (gamepad.right_bumper) {
            magicservoPosition = 0.9;
        } else {
            //below: the servo for the hand
            double movevalue = -gamepad.right_stick_y;
            telemetry.addData("movevalue", movevalue);
            //calculate servo
            magicservoPosition = (movevalue / 2) + 0.5;

            magicservo.setPosition(magicservoPosition);
        }
        //lorax -> }:

//        //below is experimentalcode for claws, the HEPPESERVOS are purely for experimental, comment them when actual use
//        if (gamepad.right_bumper) {
//            rightheppeservo.setPosition(0.75);
//        } else {
//            rightheppeservo.setPosition(0);
//        }
//        if (gamepad.left_bumper) {
//            leftheppeservo.setPosition(0.75);
//        } else {
//            leftheppeservo.setPosition(0);
//        }
//
        magicservo.setPosition(magicservoPosition);
//        telemetry.addData("leftposition", leftheppeservo.getPosition());
//        telemetry.addData("rightclawposition", rightheppeservo.getPosition());
        telemetry.addData("clawposition", magicservo.getPosition());
    }
    // sticking out your gamepad left stick Y for the rizzler
    // armbutton = gamepad1.left_stick_y

    public void open() {
        magicservo.setPosition(0);

        telemetry.addData("Opening", magicservo.getPosition());
    }

    public void close() {
        magicservo.setPosition(1);
        telemetry.addData("Closing", magicservo.getPosition());
    }

    public void polishhussars() {
        double magicservoPosition;
        if (gamepad.left_bumper && thecamara.IsRed()) {
            magicservoPosition = 1;
        } else if (gamepad.right_bumper) {
            magicservoPosition = 0.9;
        } else {
            //below: the servo for the hand
            double movevalue = -gamepad.right_stick_y;
            telemetry.addData("movevalue", movevalue);
            //calculate servo
            magicservoPosition = (movevalue / 2) + 0.5;
        }
        //lorax -> }:

//        //below is experimentalcode for claws, the HEPPESERVOS are purely for experimental, comment them when actual use
//        if (gamepad.right_bumper) {
//            rightheppeservo.setPosition(0.75);
//        } else {
//            rightheppeservo.setPosition(0);
//        }
//        if (gamepad.left_bumper) {
//            leftheppeservo.setPosition(0.75);
//        } else {
//            leftheppeservo.setPosition(0);
//        }
//
        magicservo.setPosition(magicservoPosition);
//        telemetry.addData("leftposition", leftheppeservo.getPosition());
//        telemetry.addData("rightclawposition", rightheppeservo.getPosition());
        telemetry.addData("clawposition", magicservo.getPosition());
    }
    // sticking out your gamepad left stick Y for the rizzler
    // armbutton = gamepad1.left_stick_y
    //what
    //what
}


// :{ i speak for the trees
