package config.core;


import com.acmerobotics.dashboard.config.Config;

// nia fdsf
@Config
public class robotConstants {

/// This class stores all values such as positions for bucket, transfer, and claws.
/// allowing the dashboard to confiyg it allows us to modify it
/// All these variables need to be public and static

    public static String outmotorName = "out";
    public static double ep = 0.005, ei = 0, ed = 0.00005, f = 0.05; // Extend pid constants

    public static int zero = 1;

    public static int max = 2300;

    /// follower
    ///
    public static String inmotorname = "in";

    public static double speed = 0.6;
    public static double turnSpeed = 0.5;
    public static boolean robotCentric = false;




}
