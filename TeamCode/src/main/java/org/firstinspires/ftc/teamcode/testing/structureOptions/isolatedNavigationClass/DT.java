package org.firstinspires.ftc.teamcode.testing.structureOptions.isolatedNavigationClass;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DT {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;


    void setMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br){
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    void setDirections(DcMotor.Direction fl, DcMotor.Direction fr, DcMotor.Direction bl, DcMotor.Direction br){
        this.fl.setDirection(fl);
        this.fr.setDirection(fr);
        this.bl.setDirection(bl);
        this.br.setDirection(br);
    }

    void motorPow(double fl, double fr, double bl, double br){
        double max = Math.max(1, Math.max(
                Math.max(fl, fr),
                Math.max(bl, br)
        ));
        this.fl.setPower(fl/max);
        this.fr.setPower(fr/max);
        this.bl.setPower(bl/max);
        this.br.setPower(br/max);
    }
}
