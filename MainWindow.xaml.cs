//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.DepthBasics
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using Emgu.CV;
    using Emgu.CV.Structure;
    using System.Runtime.InteropServices;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {

        private int frameIndex = 0;

        private const int MapDepthToByte = 8000 / 256;

        private string colorPathFormat = "D:/ZY/Pmomo/Data/Cloth/Color_{0}.png";
        private string depthPathFormat = "D:/ZY/Pmomo/Data/Cloth/Depth_{0}.png";

        int nearground = 300;

        int farground = 3000;

        Matrix<ushort> background = new Matrix<ushort>(424,512);

        bool isRecording = false;
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for color frames
        /// </summary>
        private MultiSourceFrameReader multiSourceFrameReader = null;

        FrameDescription colorFrameDescription = null;
        FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;
        private WriteableBitmap depthBitmap = null;


        private ushort[] depthPixels = null;
        private ushort[] rawDepthPixels = null;
        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {

           
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the color frames and depth frames
            this.multiSourceFrameReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color);

            // wire handler for frame arrival
            this.multiSourceFrameReader.MultiSourceFrameArrived += MultiSourceFrameReader_MultiSourceFrameArrived;

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            this.colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            //
            this.depthPixels = new ushort[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.rawDepthPixels =  new ushort[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(this.colorFrameDescription.Width, this.colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();

            colorPathFormat = tbImagePath.Text + "Color_{0}.png";
            depthPathFormat = tbImagePath.Text + "Depth_{0}.png";
        }

        private int framePerSave = 3;

        private void MultiSourceFrameReader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame frame = e.FrameReference.AcquireFrame();

            this.frameIndex++;

            if (this.frameIndex % this.framePerSave == 0)
            {
                StoreDepthFrame(frame, frameIndex / framePerSave);
                DisplayAndStoreColorFrame(frame, frameIndex / framePerSave);
            }
        }

        private void StoreDepthFrame(MultiSourceFrame frame, int frameIndex)
        {
            using (DepthFrame depthFrame = frame.DepthFrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    using (KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.depthBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.depthBitmap.PixelHeight))
                        {
                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, frameIndex);
                        }
                    }
                }
            }
        }

        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, int frameIndex)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            //for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            for(int i=0;i<depthFrameDescription.Height;i++)
                for(int j=0;j<depthFrameDescription.Width;j++)
                {
                    int background_depth = background.Data[i,j];
                    ushort depth = frameData[i*depthFrameDescription.Width+j];
                    if ((Math.Abs(depth - background_depth) > 50 ) && (depth>nearground) &&(depth<farground))
                        this.depthPixels[i * depthFrameDescription.Width + j] = depth;
                    else
                        this.depthPixels[i * depthFrameDescription.Width + j] = 0;

                    this.rawDepthPixels[i * depthFrameDescription.Width + j] = depth;
                }

            Mat m = new Mat(depthFrameDescription.Height, depthFrameDescription.Width, Emgu.CV.CvEnum.DepthType.Cv16U, 1);
            m.SetTo(this.depthPixels);
            if (isRecording == true)
            {
                m.Save(String.Format(this.depthPathFormat, frameIndex));
            }
        }

        private void DisplayAndStoreColorFrame(MultiSourceFrame frame, int frameIndex)
        {
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = frame.ColorFrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();

                        // verify data and write the new color frame data to the display bitmap
                        if ((this.colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (this.colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.colorBitmap.BackBuffer,
                                (uint)(this.colorFrameDescription.Width * this.colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                        }

                        this.colorBitmap.Unlock();
                    }
                    if (isRecording == true)
                    {
                        SaveBitmap(this.colorBitmap, String.Format(this.colorPathFormat, frameIndex));
                    }
                }
            }
        }

        private void SaveBitmap(WriteableBitmap bitmap, String path)
        {
            if (bitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();// PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(bitmap));

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }
                }
                catch (IOException)
                {
                }
            }
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.colorBitmap;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.multiSourceFrameReader != null)
            {
                // ColorFrameReder is IDisposable
                this.multiSourceFrameReader.Dispose();
                this.multiSourceFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private void btnStartRecording_Click(object sender, RoutedEventArgs e)
        {
            isRecording = true;
        }

        private void btnStopRecording_Click(object sender, RoutedEventArgs e)
        {
            isRecording = false;
        }

        private void tbImagePath_DataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            colorPathFormat = tbImagePath.Text + "Color_{0}.png";
            depthPathFormat = tbImagePath.Text + "Depth_{0}.png";
        }

        private void btnLoadBackground_Click(object sender, RoutedEventArgs e)
        {
            Mat m= new Mat(tbBackgroundPath.Text, Emgu.CV.CvEnum.LoadImageType.Unchanged);
            m.CopyTo(background);
        }

        private void btnSaveBackground_Click(object sender, RoutedEventArgs e)
        {
            Mat m = new Mat(depthFrameDescription.Height, depthFrameDescription.Width, Emgu.CV.CvEnum.DepthType.Cv16U, 1);
            m.SetTo(this.rawDepthPixels);
            m.Save(tbBackgroundPath.Text);
        }
    }
}
