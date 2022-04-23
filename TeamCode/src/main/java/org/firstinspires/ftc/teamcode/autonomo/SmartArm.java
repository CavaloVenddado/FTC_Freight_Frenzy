package org.firstinspires.ftc.teamcode.autonomo;

import org.firstinspires.ftc.teamcode.Cinematica_5;

public class SmartArm {
    private Cinematica_5 Arm = new Cinematica_5();
    public double speed = 0;
    public double Te1;
    public double Te2;
    public double Te3;
    private double currentPx = 0;
    private double currentPy = 0;
    private double targetPx = 0;
    private double targetPy = 0;
    private long lastTime = System.nanoTime();
    private double deltaT = 0;
    public void update(){

        double deltax = targetPx - currentPx;
        double deltay = targetPy - currentPy;
        /*
        double time = (deltax/speed) * deltaT;
        */
    }
    private void calculateDT(){
        long time = System.nanoTime();
        deltaT = (time-lastTime)/100000.0;
        lastTime = time;
    }
}
