package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.DriveFunctions;

public class intakeMinerals implements Intake{

    DcMotor spinner;
    DcMotor spool;
    final double SPIN_POWER = (float) 1.0;
    final float SPOOL_POWER = (float) 0.5;

    public intakeMinerals(DcMotor intakeMotor, DcMotor spoolMotor){spinner = intakeMotor; spool = spoolMotor;}

    @Override
    public void start(){ spinner.setPower(SPIN_POWER); }

    @Override
    public void stop(){ spinner.setPower(0.0); }

    public void reverse(){ spinner.setPower(-SPIN_POWER); }


    @Override
    public void spoolIn() {
//        DriveFunctions.oneMotorEncoder(spool, SPOOL_POWER, degrees);
    }

    @Override
    public void spoolOut() {

    }

    @Override
    public void returnSpool() {

    }
}

