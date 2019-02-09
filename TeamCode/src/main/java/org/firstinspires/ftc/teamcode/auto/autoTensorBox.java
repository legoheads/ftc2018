//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.subsystems.DriveFunctions;

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

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;

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
        hanger = hardwareMap.dcMotor.get("hanger");
        mineralFlipper = hardwareMap.servo.get("mineralFlipper");
        dunker = hardwareMap.servo.get("dunker");
        markerDropper = hardwareMap.servo.get("markerDropper");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        boschIMU = hardwareMap.get(BNO055IMU.class, "boschIMU");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions chasis = new DriveFunctions(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, boschIMU);

        //Construct Subsystems
        teamMarker = new claiming(markerDropper);
        tensor = new twoSampling(telemetry, hardwareMap, vuforia, tfod);

        //Intialize Subsystems
        flip = new mineralFlip(mineralFlipper);
        dunk = new servoArmDunk(hanger, dunker);
        intake = new intakeMinerals(spinner, mineralSpool);
        hang = new linearActuator(hanger);

        //Initializations
        chasis.initializeRobotBrake();

//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start");
//        telemetry.update();

        //Find Gold Mineral after Initialization but before game starts
        mineralSpool.setDirection(DcMotorSimple.Direction.REVERSE);


        teamMarker.hold();
        waitForStart();

        //Code to run once play is pressed
        while(opModeIsActive())
        {
            teamMarker.hold();

//            hang.down();

            goldMineral = tensor.getMineral();

            telemetry.addData("GoldPosition", goldMineral);

            chasis.leftShiftAutonomous(shiftPower, 200);

            chasis.driveAutonomous(drivePower, 250);

            oneMotorEncoder(mineralSpool, (float) 1.0, 1000);

            flip.down();

            intake.start();

            chasis.rightShiftAutonomous(shiftPower, 200);

            if (goldMineral == TensorFlow.goldMineral.UNKNOWN)
            {
                goldMineral = TensorFlow.goldMineral.RIGHT;
            }
            if (goldMineral == TensorFlow.goldMineral.LEFT)
            {
                chasis.driveAutonomous(drivePower, 300);
                chasis.leftShiftAutonomous(shiftPower, 850);
                chasis.driveAutonomous(drivePower, 1350);
                chasis.rightShiftAutonomous(shiftPower, 850);
                chasis.driveAutonomous(drivePower, 300);
            }
            if (goldMineral == TensorFlow.goldMineral.CENTER)
            {
                chasis.driveAutonomous(drivePower, 1950);

            }
            if (goldMineral == TensorFlow.goldMineral.RIGHT)
            {
                chasis.driveAutonomous(drivePower, 300);
                chasis.rightShiftAutonomous(shiftPower, 700);
                chasis.driveAutonomous(drivePower, 1250);
                chasis.leftShiftAutonomous(shiftPower, 700);
                chasis.driveAutonomous(drivePower, 400);
            }

            intake.stop();

            oneMotorEncoder(mineralSpool, -(float) 1.0, -800);

            chasis.leftTurnIMU(turnPower, 90);

            teamMarker.drop();

            chasis.leftTurnIMU(turnPower, 15);

            chasis.driveAutonomous(drivePower, 300);

            chasis.rightShiftAutonomous(shiftPower, 800);

            chasis.driveAutonomous(drivePower, 2700);

            oneMotorEncoder(mineralSpool, (float) 1.0, 1000);

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        }
    }
}