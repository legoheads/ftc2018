package org.firstinspires.ftc.teamcode.subsystems.samplingNew;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public interface tensorFlow {

    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia();


    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod();
}




