package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;

public class intakeMinerals implements Intake
{
    DcMotor spinner;
    DcMotor spool;
    final double SPIN_POWER = (float) 1.0;
    final float SPOOL_POWER = (float) 1.0;

    public intakeMinerals(DcMotor spinner, DcMotor spool)
    {
        this.spinner = spinner;
        this.spool = spool;
        spool.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start(){ spinner.setPower(SPIN_POWER); }

    @Override
    public void stop(){ spinner.setPower(0.0); }

    public void reverse(){ spinner.setPower(-SPIN_POWER); }


    @Override
    public void spoolIn(int degrees) throws InterruptedException
    {
        oneMotorEncoder(spool, SPOOL_POWER, degrees);
    }

    @Override
    public void spoolOut(int degrees) throws InterruptedException
    {
        oneMotorEncoder(spool, -SPOOL_POWER, -degrees);
    }

    @Override
    public void returnSpool()
    {
        //fill in
    }
}

