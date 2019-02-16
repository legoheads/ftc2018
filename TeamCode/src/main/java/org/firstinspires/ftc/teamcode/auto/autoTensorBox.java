//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    DcMotor lifter;
    DcMotor hanger;

    Servo mineralFlipper;
    Servo dunker;
    Servo markerDropper;
    CRServo spinner;

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
        spinner = hardwareMap.crservo.get("spinner");
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

            hang.down();

            goldMineral = tensor.getMineral();

            telemetry.addData("GoldPosition", goldMineral);

            chassis.rightShiftAutonomous(shiftPower, 200);

            chassis.driveAutonomous(drivePower, 250);

            oneMotorEncoder(mineralSpool, (float) 1.0, 1000);

            flip.down();

            intake.start();

            chassis.leftShiftAutonomous(shiftPower, 200);

            if (goldMineral == TensorFlow.goldMineral.UNKNOWN)
            {
                goldMineral = TensorFlow.goldMineral.RIGHT;
            }
            if (goldMineral == TensorFlow.goldMineral.LEFT)
            {
                chassis.driveAutonomous(drivePower, 300);
                chassis.leftShiftAutonomous(shiftPower, 850);
                chassis.driveAutonomous(drivePower, 1350);
                chassis.rightShiftAutonomous(shiftPower, 850);
                chassis.driveAutonomous(drivePower, 300);
            }
            if (goldMineral == TensorFlow.goldMineral.CENTER)
            {
                chassis.driveAutonomous(drivePower, 1950);

            }
            if (goldMineral == TensorFlow.goldMineral.RIGHT)
            {
                chassis.driveAutonomous(drivePower, 300);
                chassis.rightShiftAutonomous(shiftPower, 700);
                chassis.driveAutonomous(drivePower, 1250);
                chassis.leftShiftAutonomous(shiftPower, 700);
                chassis.driveAutonomous(drivePower, 400);
            }

            intake.stop();

            oneMotorEncoder(mineralSpool, -(float) 1.0, -800);

            teamMarker.drop();

            chassis.leftTurnIMU(turnPower, 115);

            chassis.driveAutonomous(drivePower, 300);

            chassis.rightShiftAutonomous(shiftPower, 800);

            chassis.driveAutonomous(drivePower, 2700);

            oneMotorEncoder(mineralSpool, (float) 1.0, 1000);

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        }
    }
}