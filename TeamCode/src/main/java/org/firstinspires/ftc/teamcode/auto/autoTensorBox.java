//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.DriveFunctions;

import org.firstinspires.ftc.teamcode.subsystems.dunk.Dunk;
import org.firstinspires.ftc.teamcode.subsystems.dunk.servoArmDunk;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.hang.linearActuator;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.intakeMinerals;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.Flip;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.mineralFlip;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.*;
import org.firstinspires.ftc.teamcode.subsystems.tensorFlow.*;

import static org.firstinspires.ftc.teamcode.DriveFunctions.oneMotorEncoder;

@Autonomous(name="AutoTensorBox") //Name the program
public class autoTensorBox extends LinearOpMode {
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

    ColorSensor colorSensor;
    DistanceSensor distanceSensorLeft;
    DistanceSensor distanceSensorRight;

    //Define drive powers to avoid magic numbers
    float drivePower = (float) 0.6;
    float shiftPower = (float) 0.6;
    float turnPower = (float) 0.6;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private TensorFlow tensor;

    private TensorFlow.goldMineral goldMineral;

    //Subsystems
    Flip flip;
    Dunk dunk;
    Intake intake;
    Hang hang;
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
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");

        //Construct Subsystems
        teamMarker = new claiming(markerDropper);
        tensor = new twoSampling(telemetry, hardwareMap, vuforia, tfod);
        DriveFunctions functions = new DriveFunctions(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, hanger);

        //Intialize Subsystems
        flip = new mineralFlip(mineralFlipper);
        dunk = new servoArmDunk(hanger, dunker);
        intake = new intakeMinerals(spinner, mineralSpool);
        hang = new linearActuator(hanger);

        teamMarker.hold();

        waitForStart();

        //Code to run once play is pressed
        while(opModeIsActive())
        {
            hang.down();

            goldMineral = tensor.getMineral();

            telemetry.addData("GoldPosition", goldMineral);

            functions.leftShiftAutonomous(shiftPower, 200);

            functions.driveAutonomous(drivePower, 300);

            dunk.dunkNoPause();

            oneMotorEncoder(mineralSpool, (float) 1.0, 2000);

            flip.down();

            dunk.down();

            intake.start();

            functions.rightShiftAutonomous(shiftPower, 200);

            if (goldMineral == TensorFlow.goldMineral.UNKNOWN)
            {
                goldMineral = TensorFlow.goldMineral.RIGHT;
            }
            if (goldMineral == TensorFlow.goldMineral.LEFT)
            {
                functions.driveAutonomous(drivePower, 300);
                functions.leftShiftAutonomous(shiftPower, 700);
                functions.driveAutonomous(drivePower, 1250);
                functions.rightShiftAutonomous(shiftPower, 700);
                functions.driveAutonomous(drivePower, 400);
            }
            if (goldMineral == TensorFlow.goldMineral.CENTER)
            {
                functions.driveAutonomous(drivePower, 1950);

            }
            if (goldMineral == TensorFlow.goldMineral.RIGHT)
            {
                functions.driveAutonomous(drivePower, 300);
                functions.rightShiftAutonomous(shiftPower, 700);
                functions.driveAutonomous(drivePower, 1250);
                functions.leftShiftAutonomous(shiftPower, 700);
                functions.driveAutonomous(drivePower, 400);
            }

            functions.leftTurnAutonomous(turnPower, 500);

            functions.leftShiftAutonomous(shiftPower, 300);

            while (distanceSensorRight.getDistance(DistanceUnit.INCH) > 8)
            {
                functions.shiftTeleop(shiftPower/2);
            }

            functions.leftTurnAutonomous(turnPower, 400);

            teamMarker.drop();

            Thread.sleep(500);
            functions.leftTurnAutonomous(turnPower, 600);

            functions.leftShiftAutonomous(shiftPower, 400);

            while (distanceSensorRight.getDistance(DistanceUnit.INCH) > 8)
            {
                functions.shiftTeleop(shiftPower/2);
            }

            Thread.sleep(500);

            functions.driveAutonomous(drivePower, 3000);

            dunker.setPosition(0.2);
            mineralSpool.setPower(0.5);
            Thread.sleep(1500);
            mineralSpool.setPower(0.0);
            dunk.down();


            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        }
    }
}