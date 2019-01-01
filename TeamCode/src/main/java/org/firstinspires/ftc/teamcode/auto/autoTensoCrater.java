//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.DriveFunctions;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.TeamMarker;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.claiming;
import org.firstinspires.ftc.teamcode.subsystems.tensorFlow.TensorFlow;
import org.firstinspires.ftc.teamcode.subsystems.tensorFlow.twoSampling;

@Autonomous(name="AutoTensorCrater") //Name the program
public class autoTensoCrater extends LinearOpMode {
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor mineralSpool;
    DcMotor spinner;
    DcMotor hanger;

    Servo mineralFlipper;
    Servo dunker;
    Servo markerDropper;

    TeamMarker teamMarker;

    //Define drive powers to avoid magic numbers
    float drivePower = (float) 0.6;
    float shiftPower = (float) 0.6;
    float turnPower = (float) 0.6;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private TensorFlow tensor;

    private TensorFlow.goldMineral goldMineral;

    //***************************************************************************************************************************
    //MAIN BELOW

    @Override
    public void runOpMode() throws InterruptedException {
        //Hardware Map
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        mineralSpool = hardwareMap.dcMotor.get("mineralSpool");
        spinner = hardwareMap.dcMotor.get("spinner");
        hanger = hardwareMap.dcMotor.get("hanger");
        mineralFlipper = hardwareMap.servo.get("mineralFlipper");
        dunker = hardwareMap.servo.get("dunker");
        markerDropper = hardwareMap.servo.get("markerDropper");

        //Construct Subsystems
        teamMarker = new claiming(markerDropper);
        tensor = new twoSampling(telemetry, hardwareMap, vuforia, tfod);
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, hanger);

        //Initializations
        functions.initializeMotorsAndSensors();

        //Find Gold Mineral after Initialization but before game starts
        goldMineral = tensor.getMineral();

        waitForStart();

        //Code to run once play is pressed
        while(opModeIsActive())
        {
            teamMarker.hold();

            //Drop down
            functions.hang((float) -1.0, -2500);

            hanger.setPower(-1.0);
            Thread.sleep(1500);
            hanger.setPower(0.0);

            functions.leftShiftAutonomous(shiftPower, 250);

            functions.driveAutonomous(drivePower, 400);

            if (goldMineral == TensorFlow.goldMineral.LEFT){
                functions.leftShiftAutonomous(shiftPower, 100);
            }
            if (goldMineral == TensorFlow.goldMineral.CENTER){
                functions.rightShiftAutonomous(shiftPower, 100);
            }
            if (goldMineral == TensorFlow.goldMineral.RIGHT){
                functions.rightShiftAutonomous(shiftPower, 300);

            }

            functions.driveAutonomous(drivePower, 550);


            functions.leftTurnAutonomous(turnPower, 1000);

            functions.driveAutonomous(drivePower, 1000);

            functions.leftTurnAutonomous(turnPower, 450);

            functions.driveAutonomous(drivePower, 3100);

            teamMarker.drop();

            Thread.sleep(1000);

            functions.driveAutonomous(-drivePower, -200);

            functions.leftTurnAutonomous(turnPower, 1950);

            //Go to Our crater
            functions.driveAutonomous(drivePower, 3800);

            idle();
            break;
        }
    }
}