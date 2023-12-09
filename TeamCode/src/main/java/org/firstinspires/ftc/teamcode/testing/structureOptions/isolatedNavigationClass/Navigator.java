package org.firstinspires.ftc.teamcode.testing.structureOptions.isolatedNavigationClass;

public class Navigator {
    private Movement pos = new Movement();
    private DTEncoderState prevDTEncoderState;
    private Hardware hardware;

    private double targetDistance = 0;



    private void sleep(long milliseconds){
        try{
            Thread.sleep(milliseconds);
        }catch(InterruptedException e){
            // I have no clue what this does.
            Thread.currentThread().interrupt();
        }
    }

    void moveTo(Movement target){
        while(pos.distance(target) <= targetDistance){

        }
        MecDT dt = hardware.getDT();
//        dt.mecPow(power, 0, 0);
//        sleep(ms);
        dt.stop();
    }

    void updatePos(){
        DTEncoderState current = hardware.getDT().generateEncoderState();
        DTEncoderState encoderDifference = prevDTEncoderState.difference(current);
        double RADIANS_PER_TICK = 0;
        Movement delta = new Movement(
                (encoderDifference.getFl() - encoderDifference.getFr() - encoderDifference.getBl() + encoderDifference.getBr())/4,
                (encoderDifference.getFl() + encoderDifference.getFr() + encoderDifference.getBl() + encoderDifference.getBr())/4,
                (-encoderDifference.getFl() + encoderDifference.getFr() - encoderDifference.getBl() + encoderDifference.getBr())/4*RADIANS_PER_TICK
        );
        pos.add(delta);
        pos.setRot(hardware.getDirection());
        prevDTEncoderState = current;
    }

    void PID(Movement target){

    }

    void setHardware(Hardware hardware){
        this.hardware = hardware;
        prevDTEncoderState = hardware.getDT().generateEncoderState();
    }
    void setPos(Movement pos){
        this.pos = pos;
    }

    Hardware getHardware(){
        return hardware;
    }
    Movement getPos(){
        return pos;
    }
}
