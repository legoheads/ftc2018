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
    DcMotor hanger;

    Servo mineralFlipper;
    Servo dunker;
    Servo markerDropper;

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

        mineralSpool.setDirection(DcMotorSimple.Direction.REVERSE);


        teamMarker.hold();

        waitForStart();

        //Code to run once play is pressed
        while(opModeIsActive())
        {
            teamMarker.hold();

//            hang.down();

            //Find Gold Mineral after Initialization but before game starts
            goldMineral = tensor.getMineral();

            chasis.leftShiftAutonomous(shiftPower, 200);

            chasis.driveAutonomous(drivePower, 500);

            if (goldMineral == TensorFlow.goldMineral.UNKNOWN)
            {
                goldMineral = TensorFlow.goldMineral.RIGHT;
            }
            if (goldMineral == TensorFlow.goldMineral.LEFT)
            {
                chasis.leftShiftAutonomous(shiftPower, 600);
                oneMotorEncoder(mineralSpool, (float) 1.0, 800);
                flip.down();
                intake.start();
                chasis.driveAutonomous(drivePower, 450);
                chasis.driveAutonomous(-drivePower, -400);
                intake.stop();
                oneMotorEncoder(mineralSpool, (float) -1.0, -600);
                chasis.leftTurnIMU(turnPower, 90);
                chasis.driveAutonomous(drivePower, 1050);
            }
            if (goldMineral == TensorFlow.goldMineral.CENTER)
            {
                chasis.rightShiftAutonomous(shiftPower, 200);
                oneMotorEncoder(mineralSpool, (float) 1.0, 800);
                flip.down();
                intake.start();
                chasis.driveAutonomous(drivePower, 450);
                chasis.driveAutonomous(-drivePower, -400);
                intake.stop();
                oneMotorEncoder(mineralSpool, (float) -1.0, -600);
                chasis.leftTurnIMU(turnPower, 90);
                chasis.driveAutonomous(drivePower, 1550);
            }
            if (goldMineral == TensorFlow.goldMineral.RIGHT)
            {
                chasis.rightShiftAutonomous(shiftPower, 1000);
                oneMotorEncoder(mineralSpool, (float) 1.0, 800);
                flip.down();
                intake.start();
                chasis.driveAutonomous(drivePower, 450);
                chasis.driveAutonomous(-drivePower, -400);
                intake.stop();
                oneMotorEncoder(mineralSpool, (float) -1.0, -600);
                chasis.leftTurnIMU(turnPower, 90);
                chasis.driveAutonomous(drivePower, 2050);
            }

            chasis.leftTurnIMU(turnPower, 45);

            chasis.rightShiftAutonomous(shiftPower, 1000);

            chasis.leftShiftAutonomous(shiftPower, 200);

            chasis.driveAutonomous(drivePower, 2300);

            teamMarker.drop();

            chasis.driveAutonomous(-drivePower, -200);

            chasis.leftTurnIMU(turnPower, 180);

            //Go to Our crater
            chasis.driveAutonomous(drivePower, 500);

            chasis.leftShiftAutonomous(shiftPower, 200);

            chasis.driveAutonomous(drivePower, 2600);

            oneMotorEncoder(mineralSpool, (float) 1.0, 1000);

            idle();
            break;
        }
    }
}