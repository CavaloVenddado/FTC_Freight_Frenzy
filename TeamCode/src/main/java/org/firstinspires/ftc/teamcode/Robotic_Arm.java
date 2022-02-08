package org.firstinspires.ftc.teamcode;

public class Robotic_Arm {
    /**
     * codigo escrito pela equipe cavalo vendado 16786 temporada freght frenzy dia 2/2/2022
     * este codigo realiza a cinemática de um  braço mecanico de 2 eixos utilizando trigonometria
     *
     */

    double a = 0.3; //distancia entre o cotovelo e o pulso
    double b = 0.304; //distancia entre o ombro e o cotovelo
    private double Ma;
    private double angC;
    private double c;
    public void setPos(double x2, double y2) { //inicio da funcao cinematica
        c = Math.sqrt((x2 * x2) + (y2 * y2)); //calculo da hipotenusa entre o ombro e pulso
        double angA = Math.acos((b * b) - (c * c) - (a * a) / (-2 * b * c)); //calculo angulo ombro com hipotenusa
        angC = Math.acos((a * a) - (b * b) - (c * c) / (-2 * a * b)); //calculo angulo cotovelo
        double alfa = Math.atan(y2 / x2); //calculo do angulo hipotenusa com o chao
        Ma = Math.toRadians(180) - (angA + alfa); //angulo do ombro com o chao
    }
    public double getMa(){
        return Ma;
    }
    public double getC(){
        return angC;
    }
    public double getc() {return c;}

}
