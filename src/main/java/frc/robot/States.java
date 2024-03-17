package frc.robot;

public class States 
{
//Drive states
    public static enum DriveStates 
    {
        standard, d0, d90, d180, d270
    }

//Drive states
    public static enum ArmStates 
    {
        standard, Source, Amp, Speaker, Floor
    }
    public static DriveStates driveState = DriveStates.standard;
    public static ArmStates armState = ArmStates.standard;


}
