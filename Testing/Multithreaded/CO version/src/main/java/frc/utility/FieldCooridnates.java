
package frc.utility;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class FieldCooridnates{

public static Translation2d a1 = new Translation2d(1.524,0.762);
public static Translation2d a2 = new Translation2d(1.524,1.524);
public static Translation2d a3 = new Translation2d(1.524,2.286);
public static Translation2d a4 = new Translation2d(1.524,3.048);
public static Translation2d a5 = new Translation2d(1.524,3.81);
public static Translation2d a6 = new Translation2d(1.524,4.572);
public static Translation2d a7 = new Translation2d(1.524,5.3340000000000005);
public static Translation2d a8 = new Translation2d(1.524,6.096);
public static Translation2d a9 = new Translation2d(1.524,6.8580000000000005);
public static Translation2d a10 = new Translation2d(1.524,7.62);
public static Translation2d a11= new Translation2d(1.524,8.382);
public static Translation2d a12 = new Translation2d(1.524,9.144);
public static Translation2d b1 = new Translation2d(0.762,0.762);
public static Translation2d b2 = new Translation2d(0.762,1.524);
public static Translation2d b3 = new Translation2d(0.762,2.286);
public static Translation2d b4 = new Translation2d(0.762,3.048);
public static Translation2d b5 = new Translation2d(0.762,3.048);
public static Translation2d b6 = new Translation2d(0.762,4.572);
public static Translation2d b7 = new Translation2d(0.762,5.3340000000000005);
public static Translation2d b8 = new Translation2d(0.762,6.096);
public static Translation2d b9 = new Translation2d(0.762,6.8580000000000005);
public static Translation2d b10 = new Translation2d(0.762,7.62);
public static Translation2d b11 = new Translation2d(0.762,8.382);
public static Translation2d b12 = new Translation2d(0.762,9.144);
public static Translation2d c1 = new Translation2d(0.0,0.762);
public static Translation2d c2 = new Translation2d(0.0,1.524);
public static Translation2d c3 = new Translation2d(0.0,2.286);
public static Translation2d c4 = new Translation2d(0.0,3.048);
public static  Translation2d c5 = new Translation2d(0.0,2.286);
public static Translation2d c6 = new Translation2d(0.0,4.572);
public static Translation2d c7 = new Translation2d(0.0,5.3340000000000005);
public static Translation2d c8 = new Translation2d(0.0,6.096);
public static Translation2d c9 = new Translation2d(0.0,6.8580000000000005);
public static Translation2d c10 = new Translation2d(0.0,7.62);
public static Translation2d c11 = new Translation2d(0.0,8.382);
public static Translation2d c12 = new Translation2d(0.0,9.144);
public static Translation2d d1 = new Translation2d(-0.762,0.762);
public static Translation2d d2 = new Translation2d(-0.762,1.524);
public static Translation2d d3 = new Translation2d(-0.762,2.286);
public static Translation2d d4 = new Translation2d(-0.762, 3.048);
public static Translation2d d5 = new Translation2d(-0.762,1.524);
public static Translation2d d6 = new Translation2d(-0.762,4.572);
public static Translation2d d7 = new Translation2d(-0.762,5.3340000000000005);
public static Translation2d d8 = new Translation2d(-0.762,6.096);
public static Translation2d d9 = new Translation2d(-0.762,6.8580000000000005);
public static Translation2d d10 = new Translation2d(-0.762,7.62);
public static Translation2d d11 = new Translation2d(-0.762,8.382);
public static Translation2d d12 = new Translation2d(-0.762,9.144);
public static Translation2d e1 = new Translation2d(-1.524,0.762);
public static Translation2d e2 = new Translation2d(-1.524,1.524);
public static Translation2d e3 = new Translation2d(-1.524,2.286);
public static Translation2d e4 = new Translation2d(-1.524,3.048);
public static Translation2d e5 = new Translation2d(-1.524,0.762);
public static Translation2d e6 = new Translation2d(-1.524,4.572);
public static Translation2d e7 = new Translation2d(-1.524,5.3340000000000005);
public static Translation2d e8 = new Translation2d(-1.524,6.096);
public static Translation2d e9 = new Translation2d(-1.524,6.8580000000000005);
public static Translation2d e10 = new Translation2d(-1.524,7.62);
public static Translation2d e11 =new Translation2d(-1.524,8.382);
public static Translation2d e12 = new Translation2d(-1.524,9.144);
    
public double getXDist(Translation2d from , Translation2d to){
    return to.getX()-from.getX();
    
}


public double getYDist(Translation2d from , Translation2d to){
    return to.getY()-from.getY();
}

public FieldCooridnates getInstance(){
    return new FieldCooridnates();
}

    
}