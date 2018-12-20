//Run from the package
package org.firstinspires.ftc.teamcode.legoheads;

//Import necessary items

//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.filters.LeviColorFilter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.subsystems.tensorFlow.sampling;
//import org.firstinspires.ftc.teamcode.subsystems.sampling.GoldMineralDetector;
//import org.opencv.core.Point;

@Autonomous(name="Gang") //Name the program
public class autoTensor extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor mineralSpool;
    DcMotor spinner;
    DcMotor mineralFlipper;
    DcMotor hanger;

    CRServo pin;
    Servo markerDropper;
    Servo mineralFlipInit;

    //Define drive powers to avoid magic numbers
    float drivePower = (float) 0.3;
    float shiftPower = (float) 0.3;
    float turnPower = (float) 0.3;



    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private sampling genericDetector = null;

//***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
//


        genericDetector = new sampling(telemetry, hardwareMap, vuforia, tfod);


//***************************************************************************************************************************

            genericDetector.initVuforia();

            genericDetector.initTfod();

            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();
            waitForStart();

            if (opModeIsActive()) {
                /** Activate Tensor Flow Object Detection. */
                if (tfod != null) {
                    tfod.activate();
                }

                while (opModeIsActive()) {

                    genericDetector.getMineral(5000);

                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }

//            break;
        }//Close while opModeIsActive loop
//    } //Close "run Opmode" loop
} //Close class and end program