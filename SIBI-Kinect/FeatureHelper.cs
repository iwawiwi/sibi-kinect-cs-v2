using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Kinect.Toolbox;
using Microsoft.Kinect;
using System.Windows.Media.Imaging;
using System.Windows.Input;
using System.IO;
using SIBI_Kinect.MathStat;

namespace SIBI_Kinect
{
    public class SphericalAngle
    {
        public double degreeY;
        public double degreeZ;

        public void get_rad_polar(SkeletonPoint lower, SkeletonPoint upper)
        {
            double diffX = lower.X - upper.X;
            double diffY = lower.Y - upper.Y;
            double diffZ = lower.Z - upper.Z;

            degreeY = RadianToDegree(Math.Atan2(diffY, diffX));
            degreeZ = RadianToDegree(Math.Atan2(diffZ, diffX));

            double hypLength = Math.Sqrt(Math.Pow(lower.X - upper.X, 2) + Math.Pow(lower.Y - upper.Y, 2) + Math.Pow(lower.Z - upper.Z, 2));
        }
        
        public double DegreeToRadian(double angle) { return Math.PI * angle / 180.0; }
        
        public double RadianToDegree(double angle) { return angle * (180.0 / Math.PI); }
    }

    public class SomeFeatureMaker
    {

    }
}
