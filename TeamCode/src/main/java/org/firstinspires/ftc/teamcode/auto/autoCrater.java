//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.subsystems.dunk.Dunk;
import org.firstinspires.ftc.teamcode.subsystems.dunk.servoArmDunk;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.hang.linearActuator;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.intakeMinerals;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.Flip;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.mineralFlip;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.*;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.DriveFunctions;

@Autonomous(name="Auto Crater") //Name the program
@Disabled
public class autoCrater extends LinearOpMode
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

    //Define drive powers to avoid magic numbers
    float drivePower = (float) 0.3;
    float shiftPower = (float) 0.3;
    float turnPower = (float) 0.3;

    //Subsystems
    TeamMarker teamMarker;
    Flip flip;
    Dunk dunk;
    Intake intake;
    Hang hang;
    //***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
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

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions functions = new DriveFunctions(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack);

        teamMarker = new claiming(markerDropper);
        flip = new mineralFlip(mineralFlipper);
        dunk = new servoArmDunk(hanger, dunker);
        intake = new intakeMinerals(spinner, mineralSpool);
        hang = new linearActuator(hanger);

        //Set the sensor to active mode
        //Set the directions and modes of the motors.
//        functions.initializeRobotBrake();

        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            teamMarker.hold();
            hang.down();

            functions.leftShiftAutonomous(shiftPower, 150);

            functions.driveAutonomous(drivePower, 950);

            functions.leftTurnAutonomous(turnPower, 950);

            functions.driveAutonomous(drivePower, 1350);

            functions.leftTurnAutonomous(turnPower, 470);

            functions.driveAutonomous(drivePower, 2650);

            functions.leftTurnAutonomous(turnPower, 500);

            teamMarker.drop();

            Thread.sleep(500);

            functions.leftTurnAutonomous(turnPower, 1600);

            //Go to Our crater
            functions.driveAutonomous(drivePower, 1500);

            functions.rightTurnAutonomous(turnPower, 100);

            functions.driveAutonomous(drivePower, 1700);

            dunk.dunkNoPause();
            mineralSpool.setPower(0.5);
            Thread.sleep(1000);
            mineralSpool.setPower(0.0);
            flip.up();
            mineralSpool.setPower(0.5);
            Thread.sleep(2000);
            mineralSpool.setPower(0.0);
            dunk.down();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
            //Break the loop after one run
            break;
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program