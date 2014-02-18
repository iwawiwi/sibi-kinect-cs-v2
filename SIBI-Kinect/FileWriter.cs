using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Kinect.Toolbox;
using Microsoft.Kinect;
using System.Windows.Media.Imaging;
using System.Windows.Input;
using System.IO;
using SIBI_Kinect;
using System.Windows.Media;

namespace SIBI_Kinect
{
    public class BarycenterHelper
    {
        private Dictionary<JointType, List<Vector3>> positions = new Dictionary<JointType, List<Vector3>>();
        readonly int windowSize;

        public float Threshold { get; set; }

        public BarycenterHelper(int windowSize = 20, float threshold = 0.05f)
        {
            this.windowSize = windowSize;
            Threshold = threshold;
        }

        public bool IsStable(JointType jointType)
        {
            List<Vector3> currentPositions = positions[jointType];
            if (currentPositions.Count != windowSize)
                return false;

            Vector3 current = currentPositions[currentPositions.Count - 1];

            for (int index = 0; index < currentPositions.Count - 2; index++)
            {
                if ((currentPositions[index] - current).Length > Threshold)
                    return false;
            }

            return true;
        }

        public void Add(Vector3 position, JointType jointType)
        {
            if (!positions.ContainsKey(jointType))
                positions.Add(jointType, new List<Vector3>());

            positions[jointType].Add(position);

            if (positions[jointType].Count > windowSize)
                positions[jointType].RemoveAt(0);
        }
    }

    public class HandMoveContainer
    {
        private Dictionary<JointType, List<Vector3>> savedPositions = new Dictionary<JointType, List<Vector3>>();
        public bool recording { get; set; }

        public HandMoveContainer()
        {
            recording = false;
        }

        public void record(Vector3 vectorInput, JointType jointType)
        {
            if (!savedPositions.ContainsKey(jointType))
                savedPositions.Add(jointType, new List<Vector3>());

            savedPositions[jointType].Add(vectorInput);
        }

        public void clear()
        {
            savedPositions.Clear();
        }

        public void printToFile()
        {

        }
    }

    public class WordDataContainer
    {
        public Skeleton skeletonData { get; set; }
        public WriteableBitmap colorLeftBitmapData { get; set; }
        public WriteableBitmap colorRightBitmapData { get; set; }
        public WriteableBitmap depthLeftBitmapData { get; set; }
        public WriteableBitmap depthRightBitmapData { get; set; }

        public WordDataContainer(Skeleton s,
            WriteableBitmap colorLeft, WriteableBitmap colorRight,
            WriteableBitmap depthLeft, WriteableBitmap DepthRight)
        {
            skeletonData = s;
            colorLeftBitmapData = colorLeft.Clone();
            colorRightBitmapData = colorRight.Clone();
            depthLeftBitmapData = depthLeft.Clone();
            depthRightBitmapData = DepthRight.Clone();
        }

        //public double getPolarAngleFrom2Joints(JointType upperType, JointType lowerType)
        //{
        //    SkeletonPoint upperJoint = skeletonData.Joints[upperType].Position;
        //    SkeletonPoint lowerJoint = skeletonData.Joints[lowerType].Position;
        //}
    }

    public class FileWriterV2
    {
        public static string SIBIPATH = "D:\\Dropbox\\Research Assistant\\SIBI Data Feature\\";
        public static string DELIMITER = ",";
        public WriteableBitmap blackTemplate = new WriteableBitmap(120, 120, 96.0, 96.0, PixelFormats.Bgr32, null);

        List<WordDataContainer> wordDataList = new List<WordDataContainer>();
        SphericalAngle SA = new SphericalAngle();        

        public void addWordData(WordDataContainer container)
        {
            wordDataList.Add(container);
        }

        public void addWordData(Skeleton s,
            WriteableBitmap colorLeft, bool isColorLeftMove,
            WriteableBitmap colorRight, bool isColorRightMove,
            WriteableBitmap depthLeft, bool isDepthLeftMove,
            WriteableBitmap depthRight, bool isDepthRightMove)
        {
            if (!isColorLeftMove)
            {
                if (wordDataList.Count == 0)
                    colorLeft = blackTemplate.Clone();
                else
                    colorLeft = wordDataList.Last().colorLeftBitmapData.Clone();
            }
            if (!isColorRightMove)
            {
                if (wordDataList.Count == 0)
                    colorRight = blackTemplate.Clone();
                else
                    colorRight = wordDataList.Last().colorRightBitmapData.Clone();
            }
            if (!isDepthLeftMove)
            {
                if (wordDataList.Count == 0)
                    depthLeft = blackTemplate.Clone();
                else
                    depthLeft = wordDataList.Last().depthLeftBitmapData.Clone();
            }
            if (!isDepthRightMove)
            {
                if (wordDataList.Count == 0)
                    depthRight = blackTemplate.Clone();
                else
                    depthRight = wordDataList.Last().depthRightBitmapData.Clone();
            }

            wordDataList.Add(new WordDataContainer(s,colorLeft,colorRight,depthLeft,depthRight));
        }

        public void clearWordData()
        {
            wordDataList.Clear();
        }

        public void writeEverything(string folderName, double frameLength)
        {
            if (folderName == "")
                return;

            string filePath = SIBIPATH + folderName;
            Directory.CreateDirectory(filePath);
            String upperPointFilePath = Path.Combine(filePath, "Data Polar Upper Point.csv");
            String headPointFilePath = Path.Combine(filePath, "Data Polar Head Point.csv");

            int bitmapNumber = 1;

            StringBuilder upperStringBuilder = new StringBuilder();
            StringBuilder headStringBuilder = new StringBuilder();

            List<string> shoulderElbowLeftYXs = new List<string>();
            List<string> shoulderElbowLeftZXs = new List<string>();
            List<string> elbowHandLeftYXs = new List<string>();
            List<string> elbowHandLeftZXs = new List<string>();
            List<string> shoulderElbowRightYXs = new List<string>();
            List<string> shoulderElbowRightZXs = new List<string>();
            List<string> elbowHandRightYXs = new List<string>();
            List<string> elbowHandRightZXs = new List<string>();

            List<string> headElbowLeftYXs = new List<string>();
            List<string> headElbowLeftZXs = new List<string>();
            List<string> headHandLeftYXs = new List<string>();
            List<string> headHandLeftZXs = new List<string>();
            List<string> headElbowRightYXs = new List<string>();
            List<string> headElbowRightZXs = new List<string>();
            List<string> headHandRightYXs = new List<string>();
            List<string> headHandRightZXs = new List<string>();

            double dataCount = (double) wordDataList.Count;
            double skip = dataCount / frameLength;
            double dataIndex = 0;
            //skip = len / len_sample;
            //idx = 1;
            //for ii=1:skip:len,
            //res(idx) = input_wave((round(ii)));
            //idx = idx+1;
            //end

            while (Math.Round(dataIndex) < wordDataList.Count)
            {
                WordDataContainer data = wordDataList[Convert.ToInt32(Math.Floor(dataIndex))];
                dataIndex += skip;                

                Joint handLeft = data.skeletonData.Joints[JointType.HandLeft];
                Joint handRight = data.skeletonData.Joints[JointType.HandRight];
                Joint elbowLeft = data.skeletonData.Joints[JointType.ElbowLeft];
                Joint elbowRight = data.skeletonData.Joints[JointType.ElbowRight];
                Joint shoulderLeft = data.skeletonData.Joints[JointType.ShoulderLeft];
                Joint shoulderRight = data.skeletonData.Joints[JointType.ShoulderRight];
                Joint shoulderCenter = data.skeletonData.Joints[JointType.ShoulderCenter];
                Joint head = data.skeletonData.Joints[JointType.Head];

                #region UpperPoint
                SA.get_rad_polar(elbowLeft.Position, shoulderLeft.Position);
                shoulderElbowLeftYXs.Add(SA.degreeY.ToString());
                shoulderElbowLeftZXs.Add(SA.degreeZ.ToString());

                SA.get_rad_polar(handLeft.Position, elbowLeft.Position);
                elbowHandLeftYXs.Add(SA.degreeY.ToString());
                elbowHandLeftZXs.Add(SA.degreeZ.ToString());

                SA.get_rad_polar(elbowRight.Position, shoulderRight.Position);
                shoulderElbowRightYXs.Add(SA.degreeY.ToString());
                shoulderElbowRightZXs.Add(SA.degreeZ.ToString());

                SA.get_rad_polar(handRight.Position, elbowRight.Position);
                elbowHandRightYXs.Add(SA.degreeY.ToString());
                elbowHandRightZXs.Add(SA.degreeZ.ToString());

                #endregion

                #region HeadPoint
                SA.get_rad_polar(elbowLeft.Position, shoulderCenter.Position);
                headElbowLeftYXs.Add(SA.degreeY.ToString());
                headElbowLeftZXs.Add(SA.degreeZ.ToString());

                SA.get_rad_polar(handLeft.Position, shoulderCenter.Position);
                headHandLeftYXs.Add(SA.degreeY.ToString());
                headHandLeftZXs.Add(SA.degreeZ.ToString());

                SA.get_rad_polar(elbowRight.Position, shoulderCenter.Position);
                headElbowRightYXs.Add(SA.degreeY.ToString());
                headElbowRightZXs.Add(SA.degreeZ.ToString());

                SA.get_rad_polar(handRight.Position, shoulderCenter.Position);
                headHandRightYXs.Add(SA.degreeY.ToString());
                headHandRightZXs.Add(SA.degreeZ.ToString());       
                #endregion

                #region Bitmap
                try
                {
                    BitmapEncoder bitmapEncoder = new PngBitmapEncoder();
                    string path;

                    bitmapEncoder.Frames.Clear();
                    bitmapEncoder.Frames.Add(BitmapFrame.Create(data.colorLeftBitmapData.SIBIBitmapResize(120, 120)));
                    path = Path.Combine(filePath, folderName + "-LeftColor-" + bitmapNumber + ".png");
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        bitmapEncoder.Save(fs);
                    }

                    bitmapEncoder = new PngBitmapEncoder();
                    bitmapEncoder.Frames.Clear();
                    bitmapEncoder.Frames.Add(BitmapFrame.Create(data.colorRightBitmapData.SIBIBitmapResize(120,120)));
                    path = Path.Combine(filePath, folderName + "-RightColor-" + bitmapNumber + ".png");
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        bitmapEncoder.Save(fs);
                    }

                    bitmapEncoder = new PngBitmapEncoder();
                    bitmapEncoder.Frames.Clear(); ;
                    bitmapEncoder.Frames.Add(BitmapFrame.Create(data.depthLeftBitmapData.SIBIBitmapResize(120, 120)));
                    path = Path.Combine(filePath, folderName + "-LeftDepth-" + bitmapNumber + ".png");
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        bitmapEncoder.Save(fs);
                    }

                    bitmapEncoder = new PngBitmapEncoder();
                    bitmapEncoder.Frames.Clear();
                    bitmapEncoder.Frames.Add(BitmapFrame.Create(data.depthRightBitmapData.SIBIBitmapResize(120, 120)));
                    path = Path.Combine(filePath, folderName + "-RightDepth-" + bitmapNumber + ".png");
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        bitmapEncoder.Save(fs);
                    }
                }
                catch (IOException e) { Console.WriteLine(e.Message); }
                #endregion

                bitmapNumber++;
            }

            headStringBuilder.AppendLine(String.Join(DELIMITER, headElbowLeftYXs.ToArray()));
            headStringBuilder.AppendLine(String.Join(DELIMITER, headElbowLeftZXs.ToArray()));
            headStringBuilder.AppendLine(String.Join(DELIMITER, headHandLeftYXs.ToArray()));
            headStringBuilder.AppendLine(String.Join(DELIMITER, headHandLeftZXs.ToArray()));
            headStringBuilder.AppendLine(String.Join(DELIMITER, headElbowRightYXs.ToArray()));
            headStringBuilder.AppendLine(String.Join(DELIMITER, headElbowRightZXs.ToArray()));
            headStringBuilder.AppendLine(String.Join(DELIMITER, headHandRightYXs.ToArray()));
            headStringBuilder.AppendLine(String.Join(DELIMITER, headHandRightZXs.ToArray()));

            upperStringBuilder.AppendLine(String.Join(DELIMITER, shoulderElbowLeftYXs.ToArray()));
            upperStringBuilder.AppendLine(String.Join(DELIMITER, shoulderElbowLeftZXs.ToArray()));
            upperStringBuilder.AppendLine(String.Join(DELIMITER, elbowHandLeftYXs.ToArray()));
            upperStringBuilder.AppendLine(String.Join(DELIMITER, elbowHandLeftZXs.ToArray()));
            upperStringBuilder.AppendLine(String.Join(DELIMITER, shoulderElbowRightYXs.ToArray()));
            upperStringBuilder.AppendLine(String.Join(DELIMITER, shoulderElbowRightZXs.ToArray()));
            upperStringBuilder.AppendLine(String.Join(DELIMITER, elbowHandRightYXs.ToArray()));
            upperStringBuilder.AppendLine(String.Join(DELIMITER, elbowHandRightZXs.ToArray()));

            File.AppendAllText(upperPointFilePath, upperStringBuilder.ToString());
            File.AppendAllText(headPointFilePath, headStringBuilder.ToString());
        }

        public void saveOneImageToFile(string folderName, string fileName, WriteableBitmap wbitmap)
        {
            string myPhotos = SIBIPATH + folderName;
            Directory.CreateDirectory(myPhotos);

            BitmapEncoder bitmapEncoder = new PngBitmapEncoder();
            bitmapEncoder.Frames.Clear();
            bitmapEncoder.Frames.Add(BitmapFrame.Create(wbitmap.Clone()));
            string path = Path.Combine(myPhotos, fileName + ".png");
            using (FileStream fs = new FileStream(path, FileMode.Create))
            {
                bitmapEncoder.Save(fs);
            }
        }
    }

}