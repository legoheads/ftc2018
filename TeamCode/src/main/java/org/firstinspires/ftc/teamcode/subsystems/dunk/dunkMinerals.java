package org.firstinspires.ftc.teamcode.subsystems.dunk;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.hang.linearActuator;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;


public class dunkMinerals implements Dunk {

    private DcMotor lifter;
    private Servo dunker;

    //Dunk Servo Variables
    private final double DUNK_POSITION = 0.32;
    private final double DOWN_POSTITION = 0.75;
    private final double HOLD_POSITION = 0.7;

    final float LIFT_POWER = (float) 1.0;
    final int UP_DISTANCE = 5000;
    final int DOWN_DISTANCE = 4000;

    public dunkMinerals(DcMotor lifter, Servo dunker)
    {
        this.lifter = lifter;
        this.dunker = dunker;
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Dunk
    @Override
    public void dunk() throws InterruptedException
    {
        dunker.setPosition(HOLD_POSITION);
        oneMotorEncoder(lifter, LIFT_POWER, UP_DISTANCE);
        dunker.setPosition(DUNK_POSITION);
    }

    //Dunk return
    @Override
    public void down() throws InterruptedException
    {
        dunker.setPosition(DOWN_POSTITION);
        oneMotorEncoder(lifter, -LIFT_POWER/2, -DOWN_DISTANCE);
    }
}




