//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.subsystems.DriveFunctions;

import org.firstinspires.ftc.teamcode.subsystems.dunk.Dunk;
import org.firstinspires.ftc.teamcode.subsystems.dunk.dunkMinerals;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.hang.linearActuator;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.intakeMinerals;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.Flip;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.mineralFlip;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.*;
import org.firstinspires.ftc.teamcode.subsystems.tensorFlow.*;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;

@Autonomous(name="AutoTensorBox") //Name the program
public class autoTensorBox extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor mineralSpool;
    DcMotor spinner;
    DcMotor lifter;
    DcMotor hanger;

    Servo mineralFlipper;
    Servo dunker;
    Servo markerDropper;

    TeamMarker teamMarker;

    ColorSensor colorSensor;

    BNO055IMU boschIMU;

    //Define drive powers to avoid magic numbers
    float drivePower = (float) 0.7;
    float shiftPower = (float) 0.7;
    float turnPower = (float) 0.7;

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
        lifter = hardwareMap.dcMotor.get("lifter");
        hanger = hardwareMap.dcMotor.get("hanger");

        mineralFlipper = hardwareMap.servo.get("mineralFlipper");
        dunker = hardwareMap.servo.get("dunker");
        markerDropper = hardwareMap.servo.get("markerDropper");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        boschIMU = hardwareMap.get(BNO055IMU.class, "boschIMU");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions chassis = new DriveFunctions(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, boschIMU);

        //Construct Subsystems
        teamMarker = new claiming(markerDropper);
        tensor = new twoSampling(telemetry, hardwareMap, vuforia, tfod);

        //Intialize Subsystems
        flip = new mineralFlip(mineralFlipper);
        dunk = new dunkMinerals(hanger, dunker);
        intake = new intakeMinerals(spinner, mineralSpool);
        hang = new linearActuator(hanger);

        //Initializations
        chassis.initializeRobotBrake();

//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start");
//        telemetry.update();

        //Find Gold Mineral after Initialization but before game starts

        teamMarker.hold();

        waitForStart();

        //Code to run once play is pressed
        while(opModeIsActive())
        {

            //Drop down
            hang.down();

            //Set goldMineral object to output of getMineralTime function
            goldMineral = tensor.getMineralTime();

            //Display gold mineral position
            telemetry.addData("GoldPosition", goldMineral);

            //Shift off lander
            chassis.rightShiftAutonomous(shiftPower, 200);

            //Move forward
            chassis.driveAutonomous(drivePower, 250);

            //Put intake on ground
            flip.down();

            //Start intake
            intake.start();

            //Center the robot
            chassis.leftShiftAutonomous(shiftPower, 200);

            //Default is RIGHT
            if (goldMineral == TensorFlow.goldMineral.UNKNOWN)
            {
                goldMineral = TensorFlow.goldMineral.RIGHT;
            }

            if (goldMineral == TensorFlow.goldMineral.RIGHT){
                //Turn to right mineral
                chassis.rightTurnIMU(0.6, 45);
                //Move to mineral and intake
                chassis.driveAutonomous( 0.5, 250);
                //Move back to starting spot
                chassis.driveAutonomous( -0.5, -250);
                //Become parallel with lander
                chassis.leftTurnIMU(0.6, 135);
            }

            if (goldMineral == TensorFlow.goldMineral.LEFT){
                //Turn to left mineral
                chassis.leftTurnIMU(0.6, 45);

                //Move forward and intake
                chassis.driveAutonomous( 0.5, 250);

                //Move back to starting spot
                chassis.driveAutonomous( -0.5, -250);

                //Become parallel with lander
                chassis.leftTurnIMU(0.6, 45);
            }
            if (goldMineral == TensorFlow.goldMineral.CENTER){
                //Move forward and intake gold
                chassis.driveAutonomous( 0.5, 250);
                //Move back to starting spot
                chassis.driveAutonomous( -0.5, -250);
                //Become parallel with lander
                chassis.leftTurnIMU(0.6, 90);
            }

            //Move forward towards the wall
            chassis.driveAutonomous(drivePower, 1300);

            //Become parallel with wall facing crater
            chassis.leftTurnIMU(0.6, 45);

            //Back into depot
            chassis.driveAutonomous(-drivePower, -1000);

            //Drop team marker
            teamMarker.drop();

            //Drive to crater
            chassis.driveAutonomous(drivePower, 2700);

            //Spool out into crater
            oneMotorEncoder(mineralSpool, (float) 1.0, 1000);

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        }
    }
}