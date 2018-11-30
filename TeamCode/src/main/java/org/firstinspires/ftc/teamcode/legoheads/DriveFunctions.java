//Run from the package
package org.firstinspires.ftc.teamcode.legoheads;

//Import necessary items

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.filters.LeviColorFilter;

import org.opencv.core.Point;

@Disabled
public class DriveFunctions extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor mineralSpool;
    DcMotor spinner;
    DcMotor mineralFlipper;
    DcMotor hanger;

    CRServo pin;
    Servo markerDropper;
    Servo mineralFlipInit;

    //Define possible mineral locations in enum
    enum location {
        LEFT, CENTER, RIGHT, UNKNOWN
    };

    /**
     * Initialize all the hardware
     * This creates a data type DriveFunctions to store all the hardware devices
     */
    public DriveFunctions(DcMotor leftMotorFront, DcMotor rightMotorFront, DcMotor leftMotorBack, DcMotor rightMotorBack, DcMotor mineralSpool, DcMotor spinner, DcMotor mineralFlipper, DcMotor hanger, CRServo pin, Servo markerDropper, Servo mineralFlipInit)
    {
        //These lines enable us to store the motors, sensors and CDI without having to write them over and over again
        //Initialize DC motors
        this.leftMotorFront = leftMotorFront;
        this.leftMotorBack = leftMotorBack;
        this.rightMotorFront = rightMotorFront;
        this.rightMotorBack = rightMotorBack;

        this.mineralSpool = mineralSpool;
        this.spinner = spinner;
        this.mineralFlipper = mineralFlipper;
        this.hanger = hanger;

        this.pin = pin;
        this.markerDropper = markerDropper;
        this.mineralFlipInit = mineralFlipInit;
    }

    /**
     * Set sensor addresses, modes and DC motor directions, modes
     */
    public void initializeMotorsAndSensors()
    {
//        //Set the sensor to the mode that we want
//        colorSensorCenter.enableLed(true);
//        colorSensorRight.enableLed(true);

        //Reverse some motors and keep others forward
        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set the drive motors to brake mode to prevent rolling due to chain
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Takes in motor powers for 4 drive motors
     */
    public void setDriveMotorPowers(float leftFrontPower, float leftBackPower, float rightFrontPower, float rightBackPower)
    {
        //Use the entered powers and feed them to the motors
        leftMotorFront.setPower(leftFrontPower);
        leftMotorBack.setPower(leftBackPower);
        rightMotorFront.setPower(rightFrontPower);
        rightMotorBack.setPower(rightBackPower);
    }

    /**
     * If this function is called, stop the drive motors
     */
    public void stopDriving()
    {
        //Set all drive motor powers as zero
        setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
    }


    /**
     * If this function is called, turn on the drive motors at the given powers to make it drive forward or backwards
     */
    public void driveTeleop(float power)
    {
        //Send all the motors in the same direction
        setDriveMotorPowers(power, power, power, power);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn left
     */
    public void leftTurnTeleop(float power)
    {
        //Turn the left motors backwards and the right motors forward so that it turns left
        setDriveMotorPowers(power, power, -power, -power);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn right
     */
    public void rightTurnTeleop(float power)
    {
        //Turn the right motors backwards and the left motors forward so that it turns right
        setDriveMotorPowers(-power, -power, power, power);
    }

    /**
     * If this function is called, turn on the drive motors at the
     * given powers, to make it shift in the desired direction
     */
    public void shiftTeleop(float power)
    {
        //This sequence of backwards, forwards, forwards, backwards makes the robot shift
        setDriveMotorPowers(-power, power, power, -power);
    }

    public void resetEncoders()
    {
        //Reset the encoders
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Use the encoders
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * Takes in powers for 4 drive motors, as well as 4 encoder distances
     * Allows us to run at the entered power, for the entered distance
     */
    public void moveDriveMotorsWithEncoders(int leftFrontDegrees, int leftBackDegrees, int rightFrontDegrees, int rightBackDegrees, float leftFrontPower, float leftBackPower, float rightFrontPower, float rightBackPower)
    {
        //Reset the encoders
        resetEncoders();

        //Set up the motors to run to the given position
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the target position as the corresponding values entered
        leftMotorFront.setTargetPosition(leftFrontDegrees);
        leftMotorBack.setTargetPosition(leftBackDegrees);
        rightMotorFront.setTargetPosition(rightFrontDegrees);
        rightMotorBack.setTargetPosition(rightBackDegrees);

        //Turn on the motors at the corresponding powers
        setDriveMotorPowers(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

        //Empty while loop while the motors are moving
        while ((leftMotorFront.isBusy()) && (rightMotorFront.isBusy()) && (leftMotorBack.isBusy()) && (rightMotorBack.isBusy()))
        { }

        //Stop driving
        stopDriving();

        //Use the encoders in the future
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * If this function is called, it enables us to run one DC motor to a specific distance
     */
    public static void oneMotorEncoder(DcMotor motor, float power, int degrees)
    {
        //Reset the encoder
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Use the encoder
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set up the motor to run to the given position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the target position as the value entered
        motor.setTargetPosition(degrees);

        //Turn the motor on at the corresponding power
        motor.setPower(power);

        //Empty while loop while the motor is moving
        while ((motor.isBusy()))
        { }

        //Stop the motor
        motor.setPower(0.0);

        //Use the encoder in the future
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * If this function is called, it enables us to run one continuous rotation servo motor for a specific time
     * without affecting the rest of the robot
     */
    public void crServoTime(CRServo motor, float power, int time)
    {
        //Define an elapsed time variable to count time in milliseconds
        ElapsedTime count = new ElapsedTime();

        //Reset the time counter
        count.reset();

        //Turn on the continuous rotation servo motor at the given power
        motor.setPower(power);

        //If the time counter passes the time given, shut off the robot
        if (count.time() > time)
        {
            //Turn off the motor
            motor.setPower(0.0);
        }
    }

    /**
     * If this function is called, it enables us to run one servo motor for a specific time
     * without affecting the rest of the robot
     */
    public void servoTime(Servo motor, double position1, double position2, int time)
    {
        //Define an elapsed time variable to count time in milliseconds
        ElapsedTime counter = new ElapsedTime();

        //Set the servo motor to the first position entered
        motor.setPosition(position1);

        //Reset the time counter
        counter.reset();

        //While the timecounter is less than the entered time, delay
        if (counter.time() <= time)
        {

        }

        //After the entered time has elapsed, move the servo to the second position
        motor.setPosition(position2);
    }

    /**
     * Drive for the given distance at the given power
     * @param degrees distance
     */
    public void driveAutonomous(float power, int degrees) throws InterruptedException
    {
        //Everything in the same direction creates linear driving
        moveDriveMotorsWithEncoders(-degrees, -degrees, -degrees, -degrees, -power, -power, -power, -power);
    }

    /**
     * Turn left for the given distance at the given power
     * @param degrees distance
     */
    public void leftTurnAutonomous(float power, int degrees) throws InterruptedException
    {
        //Left motors backwards and right motors forwards gives us a left turn
        moveDriveMotorsWithEncoders(degrees, degrees, -degrees, -degrees, power, power, -power, -power);
    }

    /**
     * Turn right for the given distance at the given power
     * @param degrees distance
     */
    public void rightTurnAutonomous(float power, int degrees) throws InterruptedException
    {
        //Right motors backwards and left motors forwards gives us a right turn
        moveDriveMotorsWithEncoders(-degrees, -degrees, degrees, degrees, -power, -power, power, power);
    }

    /**
     * Shift left for the given distance at the given power
     * @param degrees distance
     */
    public void leftShiftAutonomous(float power, int degrees) throws InterruptedException
    {
        //This sequence of backwards, forwards, forwards, backwards makes the robot shift left
        moveDriveMotorsWithEncoders(degrees, -degrees, -degrees, degrees, power, -power, -power, power);
    }

    /**
     * Shift right for the given distance at the given power
     * @param degrees distance
     */
    public void rightShiftAutonomous(float power, int degrees) throws InterruptedException
    {
        //This sequence of forwards, backwards, backwards, forwards makes the robot shift right
        moveDriveMotorsWithEncoders(-degrees, degrees, degrees, -degrees, -power, power, power, -power);
    }

    /**
     * @param colorSensor take in the correct color sensor
     * @return returns true if the supplied ColorSensor either red or blue.  False otherwise
     */
    public boolean iSeeAColor(ColorSensor colorSensor)
    {
        //This is an array that stores the hue[0], saturation[1], and value[2], values
        float[] hsvValues = {0F, 0F, 0F};

        //Convert from RGB to HSV (red-green-blue to hue-saturation-value)
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        //If no value, return false
        if (hsvValues[2] == 0)
        {
            return false;
        }

        //Otherwise return true
        return true;
    }

    /**
     * Determines what color the color sensor is seeing
     * @param colorSensor take in the correct color sensor
     * @return The string "Blue" if we see the color blue, "Red" if we see the color red
     */
    public String yellowOrWhite(ColorSensor colorSensor)
    {
        //Define float for hue
        float hue;

        //This is an array that stores the hue[0], saturation[1], and value[2], values
        float[] hsvValues = {0F, 0F, 0F};

        //Convert from RGB to HSV (red-green-blue to hue-saturation-value)
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        //store the first value from the array into hue
        hue = hsvValues[0]; //NEED TO FIND

        //If hue is greater than 120, we are looking at blue, so return blue
        if (hue < 70) //SOMETHING
        {
            return "yellow";
        }

        //Otherwise return red
        return "white";
    }

    public void hang(float power, int degrees)
    {
        oneMotorEncoder(hanger, power, degrees);
    }

//    public void dropDown() throws InterruptedException
//    {
//        hang((float) -0.6, -2925);
//
//        pin.setPower(-0.75) ;
//        Thread.sleep(7200);
//        pin.setPower(0.0);
//    }

    Point blockLocation = null;

    public void goldMineralDetectionCV(Point blockLocation) throws InterruptedException {
        float power = (float) 0.3;
        int startDistance = 300;
        int distanceToBlock = 500;
        int turnDistance = 1400;

        double blockDistance = 0.0;
        int count = 0;
        location goldLocation;
        while (((blockLocation.y < 30.0) || (blockLocation.y > 500.0)) && (count < 300))
        {
            sleep(10);
            count++;
        }

        blockDistance = blockLocation.y;

        if (blockDistance < 120)
        {
            goldLocation = location.RIGHT;
        }
        else if (blockDistance < 320)
        {
            goldLocation = location.CENTER;
        }
        else if (blockDistance < 520)
        {
            goldLocation = location.LEFT;
        }
        else
        {
            goldLocation = location.CENTER;
        }


        if (goldLocation == location.RIGHT)
        {
            driveAutonomous(-power, -startDistance);
        }
        if (goldLocation == location.LEFT)
        {
            driveAutonomous(power, startDistance);
        }

        rightTurnAutonomous(power, turnDistance);

        driveAutonomous(power, distanceToBlock);
    }

//    public void knockCube(ColorSensor colorSensor) throws InterruptedException
//    {
//        //Define constants to avoid magic numbers
//        float power = (float) 0.3;
//        int shiftDistance = 800;
//        int clearDistance = 1100;
//        int startDistance = 800;
//
//        //Drop the arm
//
//        //Wait for 0.3 second
//        Thread.sleep(300);
//
//        //Delay until a color is seen
//        while (!iSeeAColor(colorSensor))
//        { }
//
//        if (yellowOrWhite(colorSensor).equals("yellow"))
//        {
//            driveAutonomous(-power, -clearDistance);
//        }
//        else
//        {
//            leftShiftAutonomous(power, shiftDistance);
//            if (yellowOrWhite(colorSensor).equals("yellow"))
//            {
//                driveAutonomous(-power, -clearDistance);
//            }
//            else
//            {
//                rightShiftAutonomous(power, 2*shiftDistance);
//                driveAutonomous(-power, -clearDistance);
//            }
//        }
//    }

    //Empty main
    @Override
    public void runOpMode() throws InterruptedException
    {

    }
} //Close class and end program

