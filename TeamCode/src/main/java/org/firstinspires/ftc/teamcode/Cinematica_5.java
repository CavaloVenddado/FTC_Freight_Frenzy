package org.firstinspires.ftc.teamcode;

public class Cinematica_5 {
    /**
     * codigo escrito pela equipe cavalo vendado 16786 temporada freght frenzy dia 2/2/2022
     * este codigo realiza a cinemática de um  braço mecanico de 2 eixos utilizando trigonometria
     */

    double a = 0.30; //distancia entre o ombro e o cotovelo
    double b = 0.33; //distancia entre o cotovelo e o pulso ANTIGA ERA 0.18
    double a3 = 0.12; //tamanho da garra
    private double Te2;
    private double Te1;
    private double Te3;
    private double delta;//Quadrado da Hipotenusa entre ombro e pulso
    private double c2;
    private double s2;
    private double s1;
    private double c1;
    private double wx; //posição do pulso em x
    private double wy; //posição do pulso e y
    /*
        px: posição da ponta do atuador em x
        py: posição da ponta do atuador em y
        phi: ângulo desejado do atudaor
        as posiçoes do braço são calculadas a partir da posição da ponta da garra e
        ângulo do pulso desejado.
     */
    public void setPos(double px, double py, double phi) { //inicio da funcao cinematica

        wx = px - a3*Math.cos(phi);
        wy = py - a3*Math.sin(phi);

        delta = (wx*wx) + (wy*wy);
        c2 = (delta - (a*a) - (b*b))/(2*a*b);
        s2 = Math.sqrt(1-(c2*c2));
        Te2 = Math.atan2(s2, c2);

        s1 = ((a+b*c2)*wy - b*s2*wx)/delta;
        c1 = ((a+b*c2)*wx + b*s2*wy)/delta;
        Te1 = Math.atan2(s1, c1);

        Te3 = phi - Te1 - Te2;
    }
    // parte do código responsavel por retornar o robo, ao código principal
    public double getTe1() {return Te1;}//ombro
    public double getTe2() {
        return (Math.toRadians(180) - Te2);
    }//cotovelo
    public double getTe3() {return Te3;}//pulso
}
