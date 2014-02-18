using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using Kinect.Toolbox;
using Kinect.Toolbox.Record;
using System.IO;

namespace SIBI_Kinect
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;
        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
        private WriteableBitmap depthWriteableBitmap;
        private WriteableBitmap colorWriteableBitmap;
        private WriteableBitmap rightHandWriteableDepthBitmap;
        private WriteableBitmap leftHandWriteableDepthBitmap;
        private WriteableBitmap rightHandWriteableColorBitmap;
        private WriteableBitmap leftHandWriteableColorBitmap;

        private WriteableBitmap newRightHandDepthBitmap;
        private WriteableBitmap newLeftHandDepthBitmap;

        private WriteableBitmap newRightHandColorBitmap;
        private WriteableBitmap newLeftHandColorBitmap;

        DepthImagePoint rightHandDepthPoint;
        DepthImagePoint leftHandDepthPoint;

        ColorImagePoint rightHandColorPoint;
        ColorImagePoint leftHandColorPoint;

        /// <summary>
        /// Intermediate storage for the depth data received from the camera
        /// </summary>
        private DepthImagePixel[] depthPixels;
        private byte[] colorPixels;
        private byte[] depthColorPixelsRight;
        private byte[] depthColorPixelsLeft;

        #region skeleton variables

        /// <summary>
        /// primary skeleton for hand tracking
        /// </summary>
        private Skeleton skelHand = null;

        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;
        #endregion

        private ContextTracker contextTracker;        
        private FileWriterV2 fileWriterV2;

        private bool RecordingState = false;
        private bool OldRecordingState = false;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug,
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit

            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the depth stream to receive depth frames
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                
                this.sensor.SkeletonStream.Enable();
                this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;

                // Allocate space to put the pixels we'll receive
                this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];
                this.colorPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];                

                this.depthColorPixelsRight = new byte[this.sensor.DepthStream.FramePixelDataLength * sizeof(int)];
                this.depthColorPixelsLeft = new byte[this.sensor.DepthStream.FramePixelDataLength * sizeof(int)];

                // This is the bitmap we'll display on-screen
                this.depthWriteableBitmap = new WriteableBitmap(this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.colorWriteableBitmap = new WriteableBitmap(this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                this.rightHandWriteableDepthBitmap = new WriteableBitmap(120, 120, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.leftHandWriteableDepthBitmap = new WriteableBitmap(120, 120, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.rightHandWriteableColorBitmap = new WriteableBitmap(120, 120, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.leftHandWriteableColorBitmap = new WriteableBitmap(120, 120, 96.0, 96.0, PixelFormats.Bgr32, null);

                this.newRightHandDepthBitmap = new WriteableBitmap(120, 120, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.newLeftHandDepthBitmap = new WriteableBitmap(120, 120, 96.0, 96.0, PixelFormats.Bgr32, null);

                this.newRightHandColorBitmap = new WriteableBitmap(120, 120, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.newLeftHandColorBitmap = new WriteableBitmap(120, 120, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set the image we display to point to the bitmap where we'll put the image data
                //Depth Image
                this.imageRightHand.Source = this.rightHandWriteableColorBitmap;
                this.imageLeftHand.Source = this.leftHandWriteableColorBitmap;
                //Color Image
                //this.imageRightHand.Source = this.rightHandWriteableColorBitmap;
                //this.imageLeftHand.Source = this.leftHandWriteableColorBitmap;

                this.drawingGroup = new DrawingGroup();
                this.imageSource = new DrawingImage(this.drawingGroup);
                image2.Source = this.imageSource;

                // Add an event handler to be called whenever there is new depth frame data
                this.sensor.AllFramesReady += this.AllFramesReady;
                //this.sensor.DepthFrameReady += this.SensorDepthFrameReady;
                //this.sensor.ColorFrameReady += this.SensorColorFrameReady;
                //this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                TransformSmoothParameters smoothingParam = new TransformSmoothParameters();
                {
                    //Level2
                    //smoothingParam.Smoothing = 0.5f;
                    //smoothingParam.Correction = 0.1f;
                    //smoothingParam.Prediction = 0.5f;
                    //smoothingParam.JitterRadius = 0.1f;
                    //smoothingParam.MaxDeviationRadius = 0.1f;
                    //Level3
                    //smoothingParam.Smoothing = 0.7f;
                    //smoothingParam.Correction = 0.3f;
                    //smoothingParam.Prediction = 1.0f;
                    //smoothingParam.JitterRadius = 1.0f;
                    //smoothingParam.MaxDeviationRadius = 1.0f;
                    //Level2.5
                    smoothingParam.Smoothing = 0.5f;
                    smoothingParam.Correction = 0.2f;
                    smoothingParam.Prediction = 0.5f;
                    smoothingParam.JitterRadius = 0.4f;
                    smoothingParam.MaxDeviationRadius = 0.4f;
                };
                this.sensor.SkeletonStream.Enable(smoothingParam);

                this.contextTracker = new ContextTracker();                                
                this.fileWriterV2 = new FileWriterV2();

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        private void AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            bool depthReceived = false;
            bool colorReceived = false;
            short depthMinTreshold = 0;
            short depthMaxTreshold = 0;
            int fpsCaptureValue = 0;
            int fN = 0;
            int constantaCrop = 60;

            bool drawRightColorHand = true;
            bool drawLeftColorHand = true;
            bool drawRightColorHand2 = true;
            bool drawLeftColorHand2 = true;

            bool drawDepthRightHand = true;
            bool drawDepthLeftHand = true;
            bool drawDepthRightHand2 = true;
            bool drawDepthLeftHand2 = true;

            Int16.TryParse(MinTresholdtextBox.Text, out depthMinTreshold);
            Int16.TryParse(MaxTresholdtextBox.Text, out depthMaxTreshold);

            if (RecordingState)
            {
                Int32.TryParse(jumlahFrameTextBox.Text, out fpsCaptureValue);
            }

            Skeleton[] skeletons = new Skeleton[0];
            Skeleton primarySkeleton = null;

            #region FrameUsing Area
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);
                    depthReceived = true;
                }
            }

            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    colorFrame.CopyPixelDataTo(this.colorPixels);
                    colorReceived = true;
                }
            }

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);

                    fN = skeletonFrame.FrameNumber;
                }
            }
            #endregion

            #region Skeleton Area
            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw background of the skeleton
                //dc.DrawImage(depthWriteableBitmap, new Rect(0.0, 0.0, depthWriteableBitmap.PixelWidth, depthWriteableBitmap.PixelHeight));
                dc.DrawImage(colorWriteableBitmap, new Rect(0.0, 0.0, colorWriteableBitmap.PixelWidth, colorWriteableBitmap.PixelHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);

                            if (primarySkeleton == null || (primarySkeleton != null && primarySkeleton.Position.Z > skel.Position.Z))
                            {
                                    primarySkeleton = skel;
                            }
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
            #endregion

            #region Color Area
            if (colorReceived)
            {
                // Write the pixel data into our bitmap
                this.colorWriteableBitmap.WritePixels(
                    new Int32Rect(0, 0, this.colorWriteableBitmap.PixelWidth, this.colorWriteableBitmap.PixelHeight),
                    this.colorPixels,
                    this.colorWriteableBitmap.PixelWidth * sizeof(int),
                    0);

                /// Start take hand bitmap
                if (primarySkeleton != null)
                {
                    int width = rightHandWriteableDepthBitmap.PixelWidth;
                    int height = rightHandWriteableDepthBitmap.PixelHeight;
                    int stride = width * (rightHandWriteableDepthBitmap.Format.BitsPerPixel / 8);
                    byte[] croppedPixels = new byte[height * stride];

                    rightHandColorPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(primarySkeleton.Joints[JointType.HandRight].Position, ColorImageFormat.RgbResolution640x480Fps30);
                    leftHandColorPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(primarySkeleton.Joints[JointType.HandLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);

                    Int32Rect rightHandRect = new Int32Rect(
                        rightHandColorPoint.X - 60,
                        rightHandColorPoint.Y - 60,
                        120,
                        120);
                    Int32Rect leftHandRect = new Int32Rect(
                        leftHandColorPoint.X - 60,
                        leftHandColorPoint.Y - 60,
                        120,
                        120);

                    if (rightHandRect.X < 0 ||
                        rightHandRect.Y < 0 ||
                        rightHandRect.X + rightHandRect.Width > colorWriteableBitmap.PixelWidth ||
                        rightHandRect.Y + rightHandRect.Height > colorWriteableBitmap.PixelHeight)
                        drawRightColorHand = false;

                    if (leftHandRect.X < 0 ||
                        leftHandRect.Y < 0 ||
                        leftHandRect.X + leftHandRect.Width > colorWriteableBitmap.PixelWidth ||
                        leftHandRect.Y + leftHandRect.Height > colorWriteableBitmap.PixelHeight)
                        drawLeftColorHand = false;

                    /////

                    float realRightHand = primarySkeleton.Joints[JointType.HandRight].Position.Z;
                    float realLeftHand = primarySkeleton.Joints[JointType.HandLeft].Position.Z;
                    
                    int rightRatioCrop = Convert.ToInt32(Math.Floor((1.4 / realRightHand) * constantaCrop));
                    int leftRatioCrop = Convert.ToInt32(Math.Floor((1.4 / realLeftHand) * constantaCrop));

                    this.newRightHandColorBitmap = new WriteableBitmap(rightRatioCrop * 2, rightRatioCrop * 2, 96.0, 96.0, PixelFormats.Bgr32, null);
                    this.newLeftHandColorBitmap = new WriteableBitmap(leftRatioCrop * 2, leftRatioCrop * 2, 96.0, 96.0, PixelFormats.Bgr32, null);

                    int widthRight = newRightHandColorBitmap.PixelWidth;
                    int heightRight = newRightHandColorBitmap.PixelHeight;
                    int strideRight = widthRight * (newRightHandColorBitmap.Format.BitsPerPixel / 8);
                    byte[] croppedPixelsRight = new byte[heightRight * strideRight];

                    int widthLeft = newLeftHandColorBitmap.PixelWidth;
                    int heightLeft = newLeftHandColorBitmap.PixelHeight;
                    int strideLeft = widthLeft * (newLeftHandColorBitmap.Format.BitsPerPixel / 8);
                    byte[] croppedPixelsLeft = new byte[heightLeft * strideLeft];


                    Int32Rect rightHandRect2 = new Int32Rect(
                        rightHandColorPoint.X - rightRatioCrop,
                        rightHandColorPoint.Y - rightRatioCrop,
                        widthRight,
                        heightRight);
                    Int32Rect leftHandRect2 = new Int32Rect(
                        leftHandColorPoint.X - leftRatioCrop,
                        leftHandColorPoint.Y - leftRatioCrop,
                        widthLeft,
                        heightLeft);

                    if (rightHandRect2.X < 0 ||
                        rightHandRect2.Y < 0 ||
                        rightHandRect2.X + rightHandRect2.Width > colorWriteableBitmap.PixelWidth ||
                        rightHandRect2.Y + rightHandRect2.Height > colorWriteableBitmap.PixelHeight)
                        drawRightColorHand2 = false;

                    if (leftHandRect2.X < 0 ||
                        leftHandRect2.Y < 0 ||
                        leftHandRect2.X + leftHandRect2.Width > colorWriteableBitmap.PixelWidth ||
                        leftHandRect2.Y + leftHandRect2.Height > colorWriteableBitmap.PixelHeight)
                        drawLeftColorHand2 = false;
                    /////

                    if (drawRightColorHand)
                    {
                        //this.rightHandWriteableColorBitmap = colorWriteableBitmap.Crop(rightHandRect);

                        this.colorWriteableBitmap.CopyPixels(
                            rightHandRect,
                            croppedPixels,
                            stride,
                            0);

                        this.rightHandWriteableColorBitmap.WritePixels(
                            new Int32Rect(0, 0, width, height),
                            croppedPixels,
                                stride,
                            0);

                        if (drawRightColorHand2)
                        {
                            this.colorWriteableBitmap.CopyPixels(
                           rightHandRect2,
                           croppedPixelsRight,
                           strideRight,
                           0);

                            this.newRightHandColorBitmap.WritePixels(
                                new Int32Rect(0, 0, widthRight, heightRight),
                                croppedPixelsRight,
                                    strideRight,
                                0);
                        }
                    }

                    if (drawLeftColorHand)
                    {                        
                        this.colorWriteableBitmap.CopyPixels(
                            leftHandRect,
                            croppedPixels,
                            stride,
                            0);

                        this.leftHandWriteableColorBitmap.WritePixels(
                            new Int32Rect(0, 0, width, height),
                            croppedPixels,
                                stride,
                            0);

                        if (drawLeftColorHand2)
                        {
                            this.colorWriteableBitmap.CopyPixels(
                           leftHandRect2,
                           croppedPixelsLeft,
                           strideLeft,
                           0);

                            this.newLeftHandColorBitmap.WritePixels(
                                new Int32Rect(0, 0, widthLeft, heightLeft),
                                croppedPixelsLeft,
                                    strideLeft,
                                0);
                        }
                    }
                }
            }
            #endregion

            #region Depth Area
            if (depthReceived && primarySkeleton != null)
            {
                rightHandDepthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(primarySkeleton.Joints[JointType.HandRight].Position, DepthImageFormat.Resolution640x480Fps30);
                leftHandDepthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(primarySkeleton.Joints[JointType.HandLeft].Position, DepthImageFormat.Resolution640x480Fps30);

                contextTracker.Add(primarySkeleton, JointType.HandRight);
                stableTextBox.Text = contextTracker.IsStable(primarySkeleton.TrackingId).ToString();

                int minDepthRight = rightHandDepthPoint.Depth - depthMinTreshold;
                int maxDepthRight = rightHandDepthPoint.Depth + depthMaxTreshold;
                int minDepthLeft = leftHandDepthPoint.Depth - depthMinTreshold;
                int maxDepthLeft = leftHandDepthPoint.Depth + depthMaxTreshold;

                RightDepthLabel.Content = rightHandDepthPoint.Depth;
                LeftDepthLabel.Content = leftHandDepthPoint.Depth;

                int depthColorPixelIndexRight = 0;
                int depthColorPixelIndexLeft = 0;

                List<short> depthRightHandList = new List<short>();
                List<short> depthLeftHandList = new List<short>();

                int width = rightHandWriteableDepthBitmap.PixelWidth;
                int height = rightHandWriteableDepthBitmap.PixelHeight;
                int stride = width * (rightHandWriteableDepthBitmap.Format.BitsPerPixel / 8);
                byte[] croppedPixels = new byte[height * stride];

                #region otsu test
                //Right Otsu Threshold
                /*
                if (rightHandDepthPoint.X - leftHandWriteableDepthBitmap.PixelWidth/2 > 0 &&
                   rightHandDepthPoint.Y - leftHandWriteableDepthBitmap.PixelHeight/2 > 0 &&
                   rightHandDepthPoint.X + leftHandWriteableDepthBitmap.PixelWidth/2 < depthWriteableBitmap.PixelWidth &&
                   rightHandDepthPoint.Y + leftHandWriteableDepthBitmap.PixelHeight/2 < depthWriteableBitmap.PixelHeight)
                {
                    int xMin = rightHandDepthPoint.X - 60;
                    int xMax = rightHandDepthPoint.X + 60;
                    int yMin = rightHandDepthPoint.Y - 60;
                    int yMax = rightHandDepthPoint.Y + 60;

                    //-----------
                    System.Diagnostics.Debug.WriteLine("Pos : " + rightHandDepthPoint.X + "-" + rightHandDepthPoint.Y);
                    StringBuilder sb = new StringBuilder();
                    List<String> line = new List<String>();
                    //-----------

                    for (int y = yMin; y < yMax; y++) 
                    {
                        for (int x = xMin; x < xMax; x++)
                        {
                            depthRightHandList.Add(depthPixels[(y * 640) + x].Depth);
                            //-------------------
                            line.Add(depthPixels[(y * 640) + x].Depth.ToString()); 
                            //-------------------
                        }
                        //-------------------
                        sb.AppendLine(String.Join(",", line.ToArray()));
                        line.Clear();
                        //-------------------
                    }

                    //---------
                    string filePath = FileWriter.SIBIPATH + gerakanTextbox.Text;
                    Directory.CreateDirectory(filePath);
                    string fileName = "righthand" + fN + ".csv";
                    string path = System.IO.Path.Combine(filePath, fileName);
                    File.AppendAllText(path, sb.ToString());
                    //--------

                    maxDepthLeft = otsuThreshold.doOtsu(depthRightHandList, croppedPixels.Length, gerakanTextbox.Text);

                    for (int i = 0; i < depthRightHandList.Count; ++i)
                    {
                        short depthLeft = depthRightHandList[i];

                        if (depthLeft >= minDepthLeft && depthLeft <= maxDepthLeft)
                        {
                            croppedPixels[depthColorPixelIndexLeft++] = 0xFF;
                            croppedPixels[depthColorPixelIndexLeft++] = 0xFF;
                            croppedPixels[depthColorPixelIndexLeft++] = 0xFF;
                        }
                        else
                        {
                            croppedPixels[depthColorPixelIndexLeft++] = 0x00;
                            croppedPixels[depthColorPixelIndexLeft++] = 0x00;
                            croppedPixels[depthColorPixelIndexLeft++] = 0x00;
                        }
                        ++depthColorPixelIndexLeft;
                    }

                    this.leftHandWriteableDepthBitmap.WritePixels(
                        new Int32Rect(0, 0, width, height),
                        croppedPixels,
                            stride,
                        0);
                }
                */
                #endregion

                //Right hand normal threshold method
                for (int i = 0; i < this.depthPixels.Length; ++i)
                {
                    short depth = depthPixels[i].Depth;

                    if (depth >= minDepthRight && depth <= maxDepthRight)
                    {
                        this.depthColorPixelsRight[depthColorPixelIndexRight++] = 0xFF;
                        this.depthColorPixelsRight[depthColorPixelIndexRight++] = 0xFF;
                        this.depthColorPixelsRight[depthColorPixelIndexRight++] = 0xFF;
                    }
                    else
                    {
                        this.depthColorPixelsRight[depthColorPixelIndexRight++] = 0x00;
                        this.depthColorPixelsRight[depthColorPixelIndexRight++] = 0x00;
                        this.depthColorPixelsRight[depthColorPixelIndexRight++] = 0x00;
                    }

                    if (depth >= minDepthLeft && depth <= maxDepthLeft)
                    {
                        this.depthColorPixelsLeft[depthColorPixelIndexLeft++] = 0xFF;
                        this.depthColorPixelsLeft[depthColorPixelIndexLeft++] = 0xFF;
                        this.depthColorPixelsLeft[depthColorPixelIndexLeft++] = 0xFF;
                    }
                    else
                    {
                        this.depthColorPixelsLeft[depthColorPixelIndexLeft++] = 0x00;
                        this.depthColorPixelsLeft[depthColorPixelIndexLeft++] = 0x00;
                        this.depthColorPixelsLeft[depthColorPixelIndexLeft++] = 0x00;
                    }

                    // We're outputting BGR, the last byte in the 32 bits is unused so skip it
                    // If we were outputting BGRA, we would write alpha here.
                    ++depthColorPixelIndexRight;
                    ++depthColorPixelIndexLeft;
                }

                /// Start take hand only

                #region depth Normal
                Int32Rect rightHandRect = new Int32Rect(
                    rightHandDepthPoint.X - 60,
                    rightHandDepthPoint.Y - 60,
                    width,
                    height);
                Int32Rect leftHandRect = new Int32Rect(
                    leftHandDepthPoint.X - 60,
                    leftHandDepthPoint.Y - 60,
                    width,
                    height);

                if (rightHandRect.X < 0 ||
                    rightHandRect.Y < 0 ||
                    rightHandRect.X + rightHandRect.Width > colorWriteableBitmap.PixelWidth ||
                    rightHandRect.Y + rightHandRect.Height > colorWriteableBitmap.PixelHeight)
                    drawDepthRightHand = false;

                if (leftHandRect.X < 0 ||
                    leftHandRect.Y < 0 ||
                    leftHandRect.X + leftHandRect.Width > colorWriteableBitmap.PixelWidth ||
                    leftHandRect.Y + leftHandRect.Height > colorWriteableBitmap.PixelHeight)
                    drawDepthLeftHand = false;

                if (drawDepthRightHand)
                {
                    // Write the pixel data into our bitmap
                    this.depthWriteableBitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthWriteableBitmap.PixelWidth, this.depthWriteableBitmap.PixelHeight),
                        this.depthColorPixelsRight,
                        this.depthWriteableBitmap.PixelWidth * sizeof(int),
                        0);

                    //cut the hand area pixels
                    this.depthWriteableBitmap.CopyPixels(
                        rightHandRect,
                        croppedPixels,
                        stride,
                        0);

                    //write the hand area bitmap
                    rightHandWriteableDepthBitmap.WritePixels(
                        new Int32Rect(0, 0, width, height),
                        croppedPixels,
                            stride,
                        0);
                }

                if (drawDepthLeftHand)
                {
                    // Write the pixel data into our bitmap
                    this.depthWriteableBitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthWriteableBitmap.PixelWidth, this.depthWriteableBitmap.PixelHeight),
                        this.depthColorPixelsLeft,
                        this.depthWriteableBitmap.PixelWidth * sizeof(int),
                        0);

                    //cut the hand area pixels
                    this.depthWriteableBitmap.CopyPixels(
                        leftHandRect,
                        croppedPixels,
                        stride,
                        0);

                    //write the hand area bitmap
                    leftHandWriteableDepthBitmap.WritePixels(
                        new Int32Rect(0, 0, width, height),
                        croppedPixels,
                            stride,
                        0);
                }
                #endregion

                #region depth zoomed
                float realRightHand = primarySkeleton.Joints[JointType.HandRight].Position.Z;
                float realLeftHand = primarySkeleton.Joints[JointType.HandLeft].Position.Z;

                int rightRatioCrop = Convert.ToInt32(Math.Floor((1.4 / realRightHand) * constantaCrop));
                int leftRatioCrop = Convert.ToInt32(Math.Floor((1.4 / realLeftHand) * constantaCrop));

                this.newRightHandDepthBitmap = new WriteableBitmap(rightRatioCrop*2, rightRatioCrop*2, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.newLeftHandDepthBitmap = new WriteableBitmap(leftRatioCrop*2, leftRatioCrop * 2, 96.0, 96.0, PixelFormats.Bgr32, null);

                int widthRight = newRightHandDepthBitmap.PixelWidth;
                int heightRight = newRightHandDepthBitmap.PixelHeight;
                int strideRight = widthRight * (newRightHandDepthBitmap.Format.BitsPerPixel / 8);
                byte[] croppedPixelsRight = new byte[heightRight * strideRight];

                int widthLeft = newLeftHandDepthBitmap.PixelWidth;
                int heightLeft = newLeftHandDepthBitmap.PixelHeight;
                int strideLeft = widthLeft * (newLeftHandDepthBitmap.Format.BitsPerPixel / 8);
                byte[] croppedPixelsLeft = new byte[heightLeft * strideLeft];

                Int32Rect rightHandRect2 = new Int32Rect(
                    rightHandDepthPoint.X - rightRatioCrop,
                    rightHandDepthPoint.Y - rightRatioCrop,
                    widthRight,
                    heightRight);
                Int32Rect leftHandRect2 = new Int32Rect(
                    leftHandDepthPoint.X - leftRatioCrop,
                    leftHandDepthPoint.Y - leftRatioCrop,
                    widthLeft,
                    heightLeft);

                if (rightHandRect2.X < 0 ||
                    rightHandRect2.Y < 0 ||
                    rightHandRect2.X + rightHandRect2.Width > colorWriteableBitmap.PixelWidth ||
                    rightHandRect2.Y + rightHandRect2.Height > colorWriteableBitmap.PixelHeight)
                    drawDepthRightHand2 = false;

                if (leftHandRect2.X < 0 ||
                    leftHandRect2.Y < 0 ||
                    leftHandRect2.X + leftHandRect2.Width > colorWriteableBitmap.PixelWidth ||
                    leftHandRect2.Y + leftHandRect2.Height > colorWriteableBitmap.PixelHeight)
                    drawDepthLeftHand2 = false;

                if (drawDepthRightHand2)
                {
                    // Write the pixel data into our bitmap
                    this.depthWriteableBitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthWriteableBitmap.PixelWidth, this.depthWriteableBitmap.PixelHeight),
                        this.depthColorPixelsRight,
                        this.depthWriteableBitmap.PixelWidth * sizeof(int),
                        0);

                        //cut the hand area pixels
                        this.depthWriteableBitmap.CopyPixels(
                            rightHandRect2,
                            croppedPixelsRight,
                            strideRight,
                            0);

                        //write the hand area bitmap
                        newRightHandDepthBitmap.WritePixels(
                            new Int32Rect(0, 0, widthRight, heightRight),
                            croppedPixelsRight,
                                strideRight,
                            0);
                }

                if (drawDepthLeftHand2)
                {
                    // Write the pixel data into our bitmap
                    this.depthWriteableBitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthWriteableBitmap.PixelWidth, this.depthWriteableBitmap.PixelHeight),
                        this.depthColorPixelsLeft,
                        this.depthWriteableBitmap.PixelWidth * sizeof(int),
                        0);

                    //cut the hand area pixels
                    this.depthWriteableBitmap.CopyPixels(
                        leftHandRect2,
                        croppedPixelsLeft,
                        strideLeft,
                        0);

                    //write the hand area bitmap
                    newLeftHandDepthBitmap.WritePixels(
                        new Int32Rect(0, 0, widthLeft, heightLeft),
                        croppedPixelsLeft,
                            strideLeft,
                        0);
                }
                #endregion
            }
            #endregion

            #region masking test area
            /////
            //float depthHandRight = primarySkeleton.Joints[JointType.HandRight].Position.Z;
            //float depthShoulder = primarySkeleton.Joints[JointType.ShoulderCenter].Position.Z;
            //ColorImagePoint shoulderColorPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(primarySkeleton.Joints[JointType.ShoulderCenter].Position, ColorImageFormat.RgbResolution640x480Fps30);

            //HandXLabel.Content = rightHandColorPoint.X;
            //HandYLabel.Content = rightHandColorPoint.Y;
            //HandZLabel.Content = Math.Floor(depthHandRight * 1000);

            //ShoulderXLabel.Content = shoulderColorPoint.X;
            //ShoulderYLabel.Content = shoulderColorPoint.Y;
            //ShoulderZLabel.Content = Math.Floor(depthShoulder * 1000);
            //////
            #endregion

            #region File Saving area
            //simpan gambar tangan selama gerakan
            if (primarySkeleton != null && RecordingState)
            {
                    fileWriterV2.addWordData(primarySkeleton,
                        newLeftHandColorBitmap, drawLeftColorHand2,
                        newRightHandColorBitmap, drawRightColorHand2,
                        newLeftHandDepthBitmap, drawDepthLeftHand2,
                        newRightHandDepthBitmap, drawDepthRightHand2);
                    //fileWriterV2.addWordData(primarySkeleton,
                    //    leftHandWriteableColorBitmap, rightHandWriteableColorBitmap,
                    //    leftHandWriteableDepthBitmap, rightHandWriteableDepthBitmap);
            }

            //akhir dari proses perekaman simpan gerakan ke file
            if (OldRecordingState && !RecordingState)
            {
                OldRecordingState = false;
            }
            #endregion
        }

        private void recordingButton_Click(object sender, RoutedEventArgs e)
        {
            OldRecordingState = RecordingState;
            RecordingState = !RecordingState;

            if (RecordingState)
            {
                recordingButton.Content = "Recording";
                recordingButton.Background = Brushes.Red;
            }
            else
            {
                fileWriterV2.writeEverything(gerakanTextbox.Text, Double.Parse(jumlahFrameTextBox.Text));
                fileWriterV2.clearWordData();

                recordingButton.Content = "Not Recording";
                recordingButton.Background = Brushes.Gray;
            }
        }

        private void jumlahFrameTextBox_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void simpanGambarAlphabetButton_Click(object sender, RoutedEventArgs e)
        {
            string filename = AlphabetNamaTextBox.Text + "_" + AlphabetNomorTextBox.Text;
            string foldername = "Alphabet";

            fileWriterV2.saveOneImageToFile(foldername, filename + "_color", rightHandWriteableColorBitmap);
            fileWriterV2.saveOneImageToFile(foldername, filename + "_depth", rightHandWriteableDepthBitmap);

            int no = int.Parse(AlphabetNomorTextBox.Text);
            no++;
            AlphabetNomorTextBox.Text = no.ToString();
        }

        private void AlphabetButton_Click(object sender, RoutedEventArgs e)
        {
            
        }

        #region UNUSED

        /// <summary>
        /// Event handler for Kinect sensor's DepthFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            bool depthReceived = false;
            int minDepthRight = 0;
            int maxDepthRight = 0;
            int minDepthLeft = 0;
            int maxDepthLeft = 0;
            short depthMinTreshold = 175;
            short depthMaxTreshold = 50;
            DepthImagePoint rightHandPoint;
            DepthImagePoint leftHandPoint;

            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);
                    depthReceived = true;
                }
            }

            if (depthReceived && skelHand != null)
            {

                rightHandPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelHand.Joints[JointType.HandRight].Position, DepthImageFormat.Resolution640x480Fps30);
                leftHandPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelHand.Joints[JointType.HandLeft].Position, DepthImageFormat.Resolution640x480Fps30);

                contextTracker.Add(skelHand, JointType.ElbowRight);

                stableTextBox.Text = skelHand.TrackingId.ToString();
                //textBox1.Text = barycenterHelper.IsStable(JointType.ElbowRight).ToString();

                minDepthRight = rightHandPoint.Depth - depthMinTreshold;
                maxDepthRight = rightHandPoint.Depth + depthMaxTreshold;
                minDepthLeft = leftHandPoint.Depth - depthMinTreshold;
                maxDepthLeft = leftHandPoint.Depth + depthMaxTreshold;

                int colorPixelIndexRight = 0;
                int colorPixelIndexLeft = 0;

                //ColorImagePoint[] tempColorPoint = new ColorImagePoint[depthColorPixelsRight.Length];
                //this.sensor.CoordinateMapper.MapDepthFrameToColorFrame(DepthImageFormat.Resolution640x480Fps30, depthColorPixelsRight, ColorImageFormat.RgbResolution640x480Fps30, tempColorPoint);

                for (int i = 0; i < this.depthPixels.Length; ++i)
                {
                    // Get the depth for this pixel
                    short depth = depthPixels[i].Depth;

                    if (/*depthPixels[i].PlayerIndex > 0 && */depth >= minDepthRight && depth <= maxDepthRight)
                    {
                        this.depthColorPixelsRight[colorPixelIndexRight++] = 0x00;
                        this.depthColorPixelsRight[colorPixelIndexRight++] = 0x00;
                        this.depthColorPixelsRight[colorPixelIndexRight++] = 0x00;
                    }
                    else
                    {
                        this.depthColorPixelsRight[colorPixelIndexRight++] = 0xFF;
                        this.depthColorPixelsRight[colorPixelIndexRight++] = 0xFF;
                        this.depthColorPixelsRight[colorPixelIndexRight++] = 0xFF;
                    }


                    if (/*depthPixels[i].PlayerIndex == 0 && */depth >= minDepthLeft && depth <= maxDepthLeft)
                    {
                        this.depthColorPixelsLeft[colorPixelIndexLeft++] = 0xFF;
                        this.depthColorPixelsLeft[colorPixelIndexLeft++] = 0xFF;
                        this.depthColorPixelsLeft[colorPixelIndexLeft++] = 0xFF;
                    }
                    else
                    {
                        this.depthColorPixelsLeft[colorPixelIndexLeft++] = 0x00;
                        this.depthColorPixelsLeft[colorPixelIndexLeft++] = 0x00;
                        this.depthColorPixelsLeft[colorPixelIndexLeft++] = 0x00;
                    }

                    // We're outputting BGR, the last byte in the 32 bits is unused so skip it
                    // If we were outputting BGRA, we would write alpha here.
                    ++colorPixelIndexRight;
                    ++colorPixelIndexLeft;
                }

                /// Start take hand only
                int width = rightHandWriteableDepthBitmap.PixelWidth;
                int height = rightHandWriteableDepthBitmap.PixelHeight;
                int stride = width * (rightHandWriteableDepthBitmap.Format.BitsPerPixel / 8);
                byte[] croppedPixels = new byte[height * stride];
                bool drawRightHand = true;
                bool drawLeftHand = true;

                //get this from palm area skeleton
                Int32Rect rightHandRect = new Int32Rect(0, 0, 125, 125);
                Int32Rect leftHandRect = new Int32Rect(0, 0, 125, 125);

                rightHandRect.X = rightHandPoint.X - 60;
                rightHandRect.Y = rightHandPoint.Y - 60;
                leftHandRect.X = leftHandPoint.X - 60;
                leftHandRect.Y = leftHandPoint.Y - 60;
                if (rightHandRect.X < 0 ||
                    rightHandRect.Y < 0 ||
                    rightHandRect.X + rightHandRect.Width > colorWriteableBitmap.PixelWidth ||
                    rightHandRect.Y + rightHandRect.Height > colorWriteableBitmap.PixelHeight)
                    drawRightHand = false;

                if (leftHandRect.X < 0 ||
                    leftHandRect.Y < 0 ||
                    leftHandRect.X + leftHandRect.Width > colorWriteableBitmap.PixelWidth ||
                    leftHandRect.Y + leftHandRect.Height > colorWriteableBitmap.PixelHeight)
                    drawLeftHand = false;

                if (drawRightHand)
                {
                    // Write the pixel data into our bitmap
                    this.depthWriteableBitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthWriteableBitmap.PixelWidth, this.depthWriteableBitmap.PixelHeight),
                        this.depthColorPixelsRight,
                        this.depthWriteableBitmap.PixelWidth * sizeof(int),
                        0);

                    this.rightHandWriteableDepthBitmap = new WriteableBitmap(new CroppedBitmap(depthWriteableBitmap, rightHandRect));
                }

                if (drawLeftHand)
                {
                    this.depthWriteableBitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthWriteableBitmap.PixelWidth, this.depthWriteableBitmap.PixelHeight),
                        this.depthColorPixelsLeft,
                        this.depthWriteableBitmap.PixelWidth * sizeof(int),
                        0);

                    this.leftHandWriteableDepthBitmap = new WriteableBitmap(new CroppedBitmap(depthWriteableBitmap, leftHandRect));
                }
            }
        }

        private void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            bool colorReceived = false;

            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    colorFrame.CopyPixelDataTo(this.colorPixels);
                    colorReceived = true;
                }
            }
            if (colorReceived && skelHand != null)
            {
                // Write the pixel data into our bitmap
                this.colorWriteableBitmap.WritePixels(
                    new Int32Rect(0, 0, this.colorWriteableBitmap.PixelWidth, this.colorWriteableBitmap.PixelHeight),
                    this.colorPixels,
                    this.colorWriteableBitmap.PixelWidth * sizeof(int),
                    0);

                #region unused
                //ColorImagePoint rightHandColorPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skelHand.Joints[JointType.HandRight].Position, ColorImageFormat.RgbResolution640x480Fps30);
                //ColorImagePoint leftHandColorPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skelHand.Joints[JointType.HandLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);
                //int stride = width * (rightHandWriteableBitmap.Format.BitsPerPixel / 8);
                //byte[] croppedPixels = new byte[rightHandWriteableBitmap.PixelHeight * stride];

                //int width = rightHandWriteableBitmap.PixelWidth;
                //int height = rightHandWriteableBitmap.PixelHeight;
                //int stride = width * (rightHandWriteableBitmap.Format.BitsPerPixel / 8);
                //byte[] croppedPixels = new byte[height * stride];
                //bool drawRightHand = true;
                //bool drawLeftHand = true;

                ////get this from palm area skeleton
                //Int32Rect rightHandRect = new Int32Rect(0, 0, 100, 100);
                //Int32Rect leftHandRect = new Int32Rect(0, 0, 100, 100);
                //if (skelHand != null)
                //{
                //    ColorImagePoint rightHandPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skelHand.Joints[JointType.HandRight].Position, ColorImageFormat.RgbResolution640x480Fps30);
                //    ColorImagePoint leftHandPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skelHand.Joints[JointType.HandLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);
                //    rightHandRect.X = rightHandPoint.X - 50;
                //    rightHandRect.Y = rightHandPoint.Y - 50;
                //    leftHandRect.X = leftHandPoint.X - 30;
                //    leftHandRect.Y = leftHandPoint.Y - 50;
                //    if (rightHandRect.X < 0 || rightHandRect.Y < 0 || rightHandRect.X + 100 > colorWriteableBitmap.PixelWidth || rightHandRect.Y + 100 > colorWriteableBitmap.PixelHeight)
                //        drawRightHand = false;

                //    if (leftHandRect.X < 0 || leftHandRect.Y < 0 || leftHandRect.X + 90 > colorWriteableBitmap.PixelWidth || leftHandRect.Y + 100 > colorWriteableBitmap.PixelHeight)
                //        drawLeftHand = false;
                //}

                //if (drawRightHand)
                //{
                //    this.colorWriteableBitmap.CopyPixels(
                //        rightHandRect,
                //        croppedPixels,
                //        stride,
                //        0);

                //    this.rightHandWriteableBitmap.WritePixels(
                //        new Int32Rect(0, 0, width, height),
                //        croppedPixels,
                //         stride,
                //        0);
                //}

                //if (drawLeftHand)
                //{
                //    this.colorWriteableBitmap.CopyPixels(
                //        leftHandRect,
                //        croppedPixels,
                //        stride,
                //        0);

                //    this.leftHandWriteableBitmap.WritePixels(
                //        new Int32Rect(0, 0, width, height),
                //        croppedPixels,
                //         stride,
                //        0);
                //}
                #endregion
            }
        }

        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];
            Skeleton primarySkeleton = null;

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw background of the skeleton
                //dc.DrawImage(depthWriteableBitmap, new Rect(0.0, 0.0, depthWriteableBitmap.PixelWidth, depthWriteableBitmap.PixelHeight));
                dc.DrawImage(colorWriteableBitmap, new Rect(0.0, 0.0, colorWriteableBitmap.PixelWidth, colorWriteableBitmap.PixelHeight));

                if (skeletons.Length != 0)
                {

                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);

                            if (primarySkeleton == null || (primarySkeleton != null && primarySkeleton.Position.Z > skel.Position.Z))
                            {
                                primarySkeleton = skel;
                            }
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                if (primarySkeleton != null)
                {
                    skelHand = primarySkeleton;
                }
                else
                {
                    skelHand = null;
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            //DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            //return new Point(depthPoint.X, depthPoint.Y);

            ColorImagePoint colorPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skelpoint, ColorImageFormat.RgbResolution640x480Fps30);
            return new Point(colorPoint.X, colorPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        #endregion

        private void button1_Click(object sender, RoutedEventArgs e)
        {
            MatlabConnection.testOutput();
        }

      

        



    }
}
