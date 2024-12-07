package org.firstinspires.ftc.riseofbabyminty;

import android.util.Size;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

//widewy crashing out
public class CamaraOscura {
    String cameraName = "CoolCamera";
    Telemetry telemetry;
    VisionPortal visionPortal;
    PredominantColorProcessor colorSensor;
    boolean yellow;
    boolean blue;
    boolean red;

    public CamaraOscura(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    public void cameraOscuraQueMeQuiereComerMiCerebroMuyTriste() {
        //trolololololololo
        //add function where x button is pressed and closing stops

    }

    public boolean IsRed() {
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        red = false;
        telemetry.addData("Best Match:", result.closestSwatch);
        if (result.closestSwatch == PredominantColorProcessor.Swatch.RED || result.closestSwatch == PredominantColorProcessor.Swatch.YELLOW) {
            red = true;
        }

        return red;
    }

    public boolean IsBlue() {
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        blue = false;
        telemetry.addData("Best Match:", result.closestSwatch);
        if (result.closestSwatch == PredominantColorProcessor.Swatch.BLUE || result.closestSwatch == PredominantColorProcessor.Swatch.YELLOW) {
            blue = true;

        }
        return blue;
    }
}
//crashing out because of a camara D:
