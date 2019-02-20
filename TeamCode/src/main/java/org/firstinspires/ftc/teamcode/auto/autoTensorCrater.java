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
    float drivePower = (float) 0.4;
    float shiftPower = (float) 0.6;
    float turnPower = (float) 0.6;

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

        mineralSpool.setDirection(DcMotorSimple.Direction.REVERSE);

        teamMarker.hold();
//        tensor.initTfod();
//        tensor.initVuforia();
//        tensor.tFodActivate();

        waitForStart();

        //Code to run once play is pressed
        while(opModeIsActive())
        {
//            //Drop down
//            hang.down();

            //Set goldMineral to gold position found from getMineral()
            goldMineral = tensor.getMineral();

            //Shift off lander
            chassis.rightShiftAutonomous(shiftPower, 200);

            //Move forward
            chassis.driveAutonomous(drivePower, 500);

            //Center robot
            chassis.rightShiftAutonomous(-shiftPower, -200);

            //Default to right
            if (goldMineral == TensorFlow.goldMineral.UNKNOWN)
            {
                goldMineral = TensorFlow.goldMineral.RIGHT;
            }

            if (goldMineral == TensorFlow.goldMineral.LEFT)
            {
                //Turn to right mineral
                chassis.leftTurnIMU(turnPower, 45);
                //Move to mineral and intake
                oneMotorEncoder(mineralSpool, (float) 1.0, 1000);
                flip.down();
                intake.start();
                chassis.driveAutonomous( 0.5, 800);
                //Move back to starting spot
                chassis.driveAutonomous( -0.5, -800);
                chassis.rightTurnIMU(turnPower, -90);
            }
            if (goldMineral == TensorFlow.goldMineral.CENTER)
            {
                oneMotorEncoder(mineralSpool, (float) 1.0, 1000);
                flip.down();
                intake.start();
                //Move to mineral and intake
                chassis.driveAutonomous( drivePower, 800);
                //Move back to starting spot
                chassis.driveAutonomous( -drivePower, -800);
                chassis.rightTurnIMU(turnPower, -90);

            }
            if (goldMineral == TensorFlow.goldMineral.RIGHT)
            {

                //Turn to right mineral
                chassis.rightTurnIMU(turnPower, -50);

                oneMotorEncoder(mineralSpool, (float) 1.0, 1000);
                flip.down();
                intake.start();
                //Move to mineral and intake
                chassis.driveAutonomous( drivePower, 800);
                //Move back to starting spot
                chassis.driveAutonomous( -drivePower, -550);
                chassis.rightTurnIMU(turnPower, -90);
            }

            intake.stop();

            //Back up towards wall
            chassis.driveAutonomous(-drivePower, -2500);

            //Become parallel with wall facing crater
            chassis.leftTurnIMU(turnPower, -45);

            //Back into depot
            chassis.driveAutonomous(-drivePower, -1200);



            chassis.rightTurnIMU(turnPower/2, -65);

            Thread.sleep(1000);

            //Drop marker in depot
            teamMarker.drop();

            Thread.sleep(1000);

            chassis.rightTurnIMU(turnPower/2, -45);

            //Drive into crater
            chassis.driveAutonomous(drivePower, 2800);

            //Spool into crater

            idle();
            break;
        }
    }
}