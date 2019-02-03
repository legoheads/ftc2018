//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.DriveFunctions;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.hang.linearActuator;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.intakeMinerals;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.Flip;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.mineralFlip;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.*;
import org.firstinspires.ftc.teamcode.subsystems.tensorFlow.*;
import org.firstinspires.ftc.teamcode.subsystems.dunk.*;

import static org.firstinspires.ftc.teamcode.DriveFunctions.oneMotorEncoder;

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
    DcMotor spinner;
    DcMotor hanger;

    Servo mineralFlipper;
    Servo dunker;
    Servo markerDropper;

    TeamMarker teamMarker;

    ColorSensor colorSensor;

    //Define drive powers to avoid magic numbers
    float drivePower = (float) 0.7;
    float shiftPower = (float) 0.7;
    float turnPower = (float) 0.7;

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
        spinner = hardwareMap.dcMotor.get("spinner");
        hanger = hardwareMap.dcMotor.get("hanger");
        mineralFlipper = hardwareMap.servo.get("mineralFlipper");
        dunker = hardwareMap.servo.get("dunker");
        markerDropper = hardwareMap.servo.get("markerDropper");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //Construct Subsystems
        teamMarker = new claiming(markerDropper);
        tensor = new twoSampling(telemetry, hardwareMap, vuforia, tfod);
        DriveFunctions functions = new DriveFunctions(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack);
        //Intialize Subsystems
        flip = new mineralFlip(mineralFlipper);
        dunk = new servoArmDunk(hanger, dunker);
        intake = new intakeMinerals(spinner, mineralSpool);
        hang = new linearActuator(hanger);

        mineralSpool.setDirection(DcMotorSimple.Direction.REVERSE);


        teamMarker.hold();

        waitForStart();

        //Code to run once play is pressed
        while(opModeIsActive())
        {
            teamMarker.hold();

            hang.down();

            //Find Gold Mineral after Initialization but before game starts
            goldMineral = tensor.getMineral();

            functions.leftShiftAutonomous(shiftPower, 200);

            functions.driveAutonomous(drivePower, 400);

            dunk.dunkNoPause();

            oneMotorEncoder(mineralSpool, (float) 1.0, 1000);

            flip.down();

            dunk.down();

            intake.start();

            if (goldMineral == TensorFlow.goldMineral.UNKNOWN)
            {
                goldMineral = TensorFlow.goldMineral.RIGHT;
            }
            if (goldMineral == TensorFlow.goldMineral.LEFT)
            {
                functions.leftShiftAutonomous(shiftPower, 600);
            }
            if (goldMineral == TensorFlow.goldMineral.CENTER)
            {
                functions.rightShiftAutonomous(shiftPower, 500);
            }
            if (goldMineral == TensorFlow.goldMineral.RIGHT)
            {
                functions.rightShiftAutonomous(shiftPower, 1300);
            }

            functions.driveAutonomous(drivePower, 750);

            functions.driveAutonomous(-drivePower, -550);

            functions.leftTurnAutonomous(turnPower, 1000);

            functions.driveAutonomous(drivePower, 1550);

            functions.leftTurnAutonomous(turnPower, 500);

            functions.rightShiftAutonomous(shiftPower, 800);

            functions.driveAutonomous(drivePower, 2500);

            teamMarker.drop();

            Thread.sleep(1000);

            functions.driveAutonomous(-drivePower, -200);

            functions.leftTurnAutonomous(turnPower, 1950);

            //Go to Our crater
            functions.driveAutonomous(drivePower, 2800);

            mineralSpool.setPower(1.0);
            Thread.sleep(1500);
            mineralSpool.setPower(0.0);

            dunk.down();

            hang.setDunk();

            idle();
            break;
        }
    }
}