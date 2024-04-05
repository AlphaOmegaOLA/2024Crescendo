package frc.robot;

public class States 
{
    //Drive states
    public static enum DriveStates 
    {
        standard, d0, d90, d180, d270
    }

    //Arm states
    public static enum ArmStates 
    {
        standard, Source, Amp, Speaker, Floor, Longshot
    }
    public static DriveStates driveState = DriveStates.standard;
    public static ArmStates armState = ArmStates.standard;
}
