//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.subsystems.DriveFunctions;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.hang.linearActuator;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.intakeMinerals;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.Flip;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.mineralFlip;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.*;
import org.firstinspires.ftc.teamcode.subsystems.tensorFlow.*;
import org.firstinspires.ftc.teamcode.subsystems.dunk.*;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;

@Autonomous(name="AutoTensorCrater") //Name the program
public class autoTensorCrater extends LinearOpMode
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
    float drivePower = (float) 0.8;
    float shiftPower = (float) 0.6;
    float turnPower = (float) 0.8;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    TensorFlow tensor;

    TensorFlow.goldMineral goldMineral;

    //Subsystems
    Flip flip;
    Dunk dunk;
    Intake intake;
    Hang hang;
//***************************************************************************************************************************
    //MAIN BELOW

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Hardware Map
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        mineralSpool = hardwareMap.dcMotor.get("mineralSpool");
        lifter = hardwareMap.dcMotor.get("lifter");
        hanger = hardwareMap.dcMotor.get("hanger");

        mineralFlipper = hardwareMap.servo.get("mineralFlipper");
        dunker = hardwareMap.servo.get("dunker");
        markerDropper = hardwareMap.servo.get("markerDropper");
        spinner = hardwareMap.crservo.get("spinner");

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

        mineralSpool.setDirection(DcMotorSimple.Direction.REVERSE);

        teamMarker.hold();

        waitForStart();

        //Code to run once play is pressed
        while(opModeIsActive())
        {
            hang.down();

            goldMineral = tensor.getMineral();

            chassis.rightShiftAutonomous(shiftPower, 200);

            chassis.driveAutonomous(drivePower, 500);

            if (goldMineral == TensorFlow.goldMineral.UNKNOWN)
            {
                goldMineral = TensorFlow.goldMineral.RIGHT;
            }
            if (goldMineral == TensorFlow.goldMineral.LEFT)
            {
                chassis.leftShiftAutonomous(shiftPower, 1000);
                oneMotorEncoder(mineralSpool, (float) 1.0, 800);
                flip.down();
                intake.start();
                chassis.driveAutonomous(drivePower, 450);
                chassis.driveAutonomous(-drivePower, -400);
                intake.stop();
                oneMotorEncoder(mineralSpool, (float) -1.0, -600);
                chassis.leftTurnIMU(turnPower, 90);
                chassis.driveAutonomous(drivePower, 1050);
            }
            if (goldMineral == TensorFlow.goldMineral.CENTER)
            {
                chassis.leftShiftAutonomous(shiftPower, 200);
                oneMotorEncoder(mineralSpool, (float) 1.0, 800);
                flip.down();
                intake.start();
                chassis.driveAutonomous(drivePower, 450);
                chassis.driveAutonomous(-drivePower, -400);
                intake.stop();
                oneMotorEncoder(mineralSpool, (float) -1.0, -600);
                chassis.leftTurnIMU(turnPower, 90);
                chassis.driveAutonomous(drivePower, 1550);
            }
            if (goldMineral == TensorFlow.goldMineral.RIGHT)
            {
                chassis.rightShiftAutonomous(shiftPower, 600);
                oneMotorEncoder(mineralSpool, (float) 1.0, 800);
                flip.down();
                intake.start();
                chassis.driveAutonomous(drivePower, 450);
                chassis.driveAutonomous(-drivePower, -400);
                intake.stop();
                oneMotorEncoder(mineralSpool, (float) -1.0, -600);
                chassis.leftTurnIMU(turnPower, 90);
                chassis.driveAutonomous(drivePower, 2050);
            }

            chassis.leftTurnIMU(turnPower, 45);

            chassis.rightShiftAutonomous(shiftPower, 1000);

            chassis.leftShiftAutonomous(shiftPower, 200);

            chassis.driveAutonomous(drivePower, 2300);

            teamMarker.drop();

            chassis.driveAutonomous(-drivePower, -200);

            chassis.leftTurnIMU(turnPower, 180);

            //Go to Our crater
            chassis.driveAutonomous(drivePower, 500);

            chassis.leftShiftAutonomous(shiftPower, 200);

            chassis.driveAutonomous(drivePower, 2600);

            oneMotorEncoder(mineralSpool, (float) 1.0, 1000);

            idle();
            break;
        }
    }
}