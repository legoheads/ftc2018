//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.subsystems.team_marker.*;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.DriveFunctions;

@Autonomous(name="Auto Crater") //Name the program
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

    TeamMarker teamMarker;



//    private GoldMineralDetector genericDetector = null;

    //Define possible mineral locations in enum
    enum location {
        LEFT, CENTER, RIGHT, UNKNOWN
    };

    //Create location object to store the mineral location data
    autoBox.location mineralLocation;

    //Define drive powers to avoid magic numbers
    float drivePower = (float) 0.6;
    float shiftPower = (float) 0.6;
    float turnPower = (float) 0.6;


    //claiming teamMarker = new claiming(markerDropper);

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
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, hanger);

        teamMarker = new claiming(markerDropper);

        //Set the sensor to active mode
        //Set the directions and modes of the motors.
        functions.initializeRobotBrake();

        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            teamMarker.hold();

            //Drop down
            functions.hang((float) -1.0, -2500);

            hanger.setPower(-1.0);
            Thread.sleep(1500);
            hanger.setPower(0.0);

            functions.leftShiftAutonomous(shiftPower, 250);

            functions.driveAutonomous(drivePower, 950);


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

//            mineralSpool.setPower(-0.5);
//            Thread.sleep(1500);
//            mineralSpool.setPower(0.0);

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
            //Break the loop after one run
            break;
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program