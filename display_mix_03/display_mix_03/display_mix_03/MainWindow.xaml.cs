using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Emgu.CV;
using Emgu.CV.Structure;
using Microsoft.Kinect;
using Microsoft.Win32;
using System.Windows.Threading;
using System.Runtime.InteropServices;
using System.Windows.Media.Media3D;

namespace display_mix_03
{
    /// <summary>
    /// MainWindow.xaml 的互動邏輯
    /// </summary>
    public partial class MainWindow : Window
    {
        //
        private KinectSensor kinectSensor = null;
        private CoordinateMapper coordinateMapper = null;//坐标映射器
        private MultiSourceFrameReader multiFrameSourceReader = null;//多源阅读器
        private BodyFrameReader bodyFrameReader = null;

        private WriteableBitmap bitmap_color = null;
        private uint bitmapBackBufferSize = 0;
        private ColorSpacePoint[] depthMappedToCameraPoints = null;
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        private bool dataReceived = false;
        private const float InferredZPositionClamp = 0.1f;
        private Body[] bodies = null;
        private List<Tuple<JointType, JointType>> bones;

        private bool body_ready = false;
        private bool readyT = false;

        DispatcherTimer timer_Path;
        int t = 1;
        int timer2 = 0;


        //learner 軌跡位置校正
        private float learner_center_X;
        private float learner_center_Y;
        private float learner_center_Z;

        private float color_DX = 0;
        private float color_DY = 0;
        private float color_DZ = 0;



        //3D coach
        private Model3DGroup MGroup_coach;
        private GeometryModel3D GeometryMode_set_coach;
        private MeshGeometry3D Mesh_coach;
        private DirectionalLight DirLight_coach;
        private PerspectiveCamera Camera_coach;

        private Model3DGroup MGroup_coach_path;

        //3D learner
        private Model3DGroup MGroup_learner;
        private GeometryModel3D GeometryMode_set_learner;
        private MeshGeometry3D Mesh_learner;
        private DirectionalLight DirLight_learner;
        private PerspectiveCamera Camera_learner;


        private Model3DGroup MGroup_learner_path;

        private int frameCnt;
        private int frameCnt2;
        private int DogCnt=0;

        private List<int> TempIndex = new List<int>(); //1
        private bool dog_first = false;
        private double Dog_TH =1;

        private Path_Record PathColor = new Path_Record();
        private Path_Record PathDepth = new Path_Record();
        private Path_Record Path3D = new Path_Record();
        private Path_Record PathDog = new Path_Record();

        private List<int> SpineBase_D = new List<int>(); //1
        private List<int> SpineMid_D = new List<int>(); //2
        private List<int> Neck_D = new List<int>(); //3
        private List<int> Head_D = new List<int>(); //4
        private List<int> ShoulderLeft_D = new List<int>(); //5
        private List<int> ElbowLeft_D = new List<int>(); //6
        private List<int> WristLeft_D = new List<int>(); //7
        private List<int> HandLeft_D = new List<int>(); //8
        private List<int> ShoulderRight_D = new List<int>(); //9
        private List<int> ElbowRight_D = new List<int>(); //10
        private List<int> WristRight_D = new List<int>(); //11
        private List<int> HandRight_D = new List<int>(); //12
        private List<int> HipLeft_D = new List<int>(); //13
        private List<int> KneeLeft_D = new List<int>(); //14
        private List<int> AnkleLeft_D = new List<int>(); //15
        private List<int> FootLeft_D = new List<int>(); //16
        private List<int> HipRight_D = new List<int>(); //17
        private List<int> KneeRight_D = new List<int>(); //18
        private List<int> AnkleRight_D = new List<int>(); //19
        private List<int> FootRight_D = new List<int>(); //20
        private List<int> SpineShoulder_D = new List<int>(); //21
        private List<int> HandTipLeft_D = new List<int>(); //22
        private List<int> ThumbLeft_D = new List<int>(); //23
        private List<int> HandTipRight_D = new List<int>(); //24
        private List<int> ThumbRight_D = new List<int>(); //25

        //private List<Point3D> SpineBase = new List<Point3D>(); //1
        //private List<Point3D> SpineMid = new List<Point3D>(); //2
        //private List<Point3D> Neck = new List<Point3D>(); //3
        //private List<Point3D> Head = new List<Point3D>(); //4
        //private List<Point3D> ShoulderLeft = new List<Point3D>(); //5
        //private List<Point3D> ElbowLeft = new List<Point3D>(); //6
        //private List<Point3D> WristLeft = new List<Point3D>(); //7
        //private List<Point3D> HandLeft = new List<Point3D>(); //8
        //private List<Point3D> ShoulderRight = new List<Point3D>(); //9
        //private List<Point3D> ElbowRight = new List<Point3D>(); //10
        //private List<Point3D> WristRight = new List<Point3D>(); //11
        //private List<Point3D> HandRight = new List<Point3D>(); //12
        //private List<Point3D> HipLeft = new List<Point3D>(); //13
        //private List<Point3D> KneeLeft = new List<Point3D>(); //14
        //private List<Point3D> AnkleLeft = new List<Point3D>(); //15
        //private List<Point3D> FootLeft = new List<Point3D>(); //16
        //private List<Point3D> HipRight = new List<Point3D>(); //17
        //private List<Point3D> KneeRight = new List<Point3D>(); //18
        //private List<Point3D> AnkleRight = new List<Point3D>(); //19
        //private List<Point3D> FootRight = new List<Point3D>(); //20
        //private List<Point3D> SpineShoulder = new List<Point3D>(); //21
        //private List<Point3D> HandTipLeft = new List<Point3D>(); //22
        //private List<Point3D> ThumbLeft = new List<Point3D>(); //23
        //private List<Point3D> HandTipRight = new List<Point3D>(); //24
        //private List<Point3D> ThumbRight = new List<Point3D>(); //25

        //private List<Point3D> SpineBase1 = new List<Point3D>(); //1
        //private List<Point3D> SpineMid1 = new List<Point3D>(); //2
        //private List<Point3D> Neck1 = new List<Point3D>(); //3
        //private List<Point3D> Head1 = new List<Point3D>(); //4
        //private List<Point3D> ShoulderLeft1 = new List<Point3D>(); //5
        //private List<Point3D> ElbowLeft1 = new List<Point3D>(); //6
        //private List<Point3D> WristLeft1 = new List<Point3D>(); //7
        //private List<Point3D> HandLeft1 = new List<Point3D>(); //8
        //private List<Point3D> ShoulderRight1 = new List<Point3D>(); //9
        //private List<Point3D> ElbowRight1 = new List<Point3D>(); //10
        //private List<Point3D> WristRight1 = new List<Point3D>(); //11
        //private List<Point3D> HandRight1 = new List<Point3D>(); //12
        //private List<Point3D> HipLeft1 = new List<Point3D>(); //13
        //private List<Point3D> KneeLeft1 = new List<Point3D>(); //14
        //private List<Point3D> AnkleLeft1 = new List<Point3D>(); //15
        //private List<Point3D> FootLeft1 = new List<Point3D>(); //16
        //private List<Point3D> HipRight1 = new List<Point3D>(); //17
        //private List<Point3D> KneeRight1 = new List<Point3D>(); //18
        //private List<Point3D> AnkleRight1 = new List<Point3D>(); //19
        //private List<Point3D> FootRight1 = new List<Point3D>(); //20
        //private List<Point3D> SpineShoulder1 = new List<Point3D>(); //21
        //private List<Point3D> HandTipLeft1 = new List<Point3D>(); //22
        //private List<Point3D> ThumbLeft1 = new List<Point3D>(); //23
        //private List<Point3D> HandTipRight1 = new List<Point3D>(); //24
        //private List<Point3D> ThumbRight1 = new List<Point3D>(); //25

        //private List<Point3D> SpineBase2 = new List<Point3D>(); //1
        //private List<Point3D> SpineMid2 = new List<Point3D>(); //2
        //private List<Point3D> Neck2 = new List<Point3D>(); //3
        //private List<Point3D> Head2 = new List<Point3D>(); //4
        //private List<Point3D> ShoulderLeft2 = new List<Point3D>(); //5
        //private List<Point3D> ElbowLeft2 = new List<Point3D>(); //6
        //private List<Point3D> WristLeft2 = new List<Point3D>(); //7
        //private List<Point3D> HandLeft2 = new List<Point3D>(); //8
        //private List<Point3D> ShoulderRight2 = new List<Point3D>(); //9
        //private List<Point3D> ElbowRight2 = new List<Point3D>(); //10
        //private List<Point3D> WristRight2 = new List<Point3D>(); //11
        //private List<Point3D> HandRight2 = new List<Point3D>(); //12
        //private List<Point3D> HipLeft2 = new List<Point3D>(); //13
        //private List<Point3D> KneeLeft2 = new List<Point3D>(); //14
        //private List<Point3D> AnkleLeft2 = new List<Point3D>(); //15
        //private List<Point3D> FootLeft2 = new List<Point3D>(); //16
        //private List<Point3D> HipRight2 = new List<Point3D>(); //17
        //private List<Point3D> KneeRight2 = new List<Point3D>(); //18
        //private List<Point3D> AnkleRight2 = new List<Point3D>(); //19
        //private List<Point3D> FootRight2 = new List<Point3D>(); //20
        //private List<Point3D> SpineShoulder2 = new List<Point3D>(); //21
        //private List<Point3D> HandTipLeft2 = new List<Point3D>(); //22
        //private List<Point3D> ThumbLeft2 = new List<Point3D>(); //23
        //private List<Point3D> HandTipRight2 = new List<Point3D>(); //24
        //private List<Point3D> ThumbRight2 = new List<Point3D>(); //25


        //影片
        private OpenFileDialog openFileDialog_video;
        string file_name_video;
        private Capture cap;
        DispatcherTimer timer_fps;

        public MainWindow()
        {

            //
            this.kinectSensor = KinectSensor.GetDefault();
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.Body);
            this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;

            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;
            int colorWidth = colorFrameDescription.Width;
            int colorHeight = colorFrameDescription.Height;
            this.bitmap_color = new WriteableBitmap(colorWidth, colorHeight, 96.0, 96.0, PixelFormats.Bgra32, null);//新图
            this.bitmapBackBufferSize = (uint)((this.bitmap_color.BackBufferStride * (this.bitmap_color.PixelHeight - 1)) + (this.bitmap_color.PixelWidth * this.bytesPerPixel));


            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            this.kinectSensor.Open();
            this.DataContext = this;

            //
            timer_Path = new DispatcherTimer();
            timer_Path.Interval = new TimeSpan(0, 0, 0, 0, 100);
            timer_Path.Tick += new EventHandler(timer_Tick_Path);

            this.frameCnt = 0;
            this.frameCnt2 = 0;
            
            //3D coach
            this.MGroup_coach = new Model3DGroup();
            this.GeometryMode_set_coach = new GeometryModel3D();
            this.Mesh_coach = new MeshGeometry3D();
            this.DirLight_coach = new DirectionalLight();
            this.Camera_coach = new PerspectiveCamera();

            this.MGroup_coach_path = new Model3DGroup();

            //3D learner
            this.MGroup_learner = new Model3DGroup();
            this.GeometryMode_set_learner = new GeometryModel3D();
            this.Mesh_learner = new MeshGeometry3D();
            this.DirLight_learner = new DirectionalLight();
            this.Camera_learner = new PerspectiveCamera();

            this.MGroup_learner_path = new Model3DGroup();

            InitializeComponent();
        }


        public class Path_Record
        {
            public List<Point3D> SpineBase = new List<Point3D>(); //1
            public List<Point3D> SpineMid = new List<Point3D>(); //2
            public List<Point3D> Neck = new List<Point3D>(); //3
            public List<Point3D> Head = new List<Point3D>(); //4
            public List<Point3D> ShoulderLeft = new List<Point3D>(); //5
            public List<Point3D> ElbowLeft = new List<Point3D>(); //6
            public List<Point3D> WristLeft = new List<Point3D>(); //7
            public List<Point3D> HandLeft = new List<Point3D>(); //8
            public List<Point3D> ShoulderRight = new List<Point3D>(); //9
            public List<Point3D> ElbowRight = new List<Point3D>(); //10
            public List<Point3D> WristRight = new List<Point3D>(); //11
            public List<Point3D> HandRight = new List<Point3D>(); //12
            public List<Point3D> HipLeft = new List<Point3D>(); //13
            public List<Point3D> KneeLeft = new List<Point3D>(); //14
            public List<Point3D> AnkleLeft = new List<Point3D>(); //15
            public List<Point3D> FootLeft = new List<Point3D>(); //16
            public List<Point3D> HipRight = new List<Point3D>(); //17
            public List<Point3D> KneeRight = new List<Point3D>(); //18
            public List<Point3D> AnkleRight = new List<Point3D>(); //19
            public List<Point3D> FootRight = new List<Point3D>(); //20
            public List<Point3D> SpineShoulder = new List<Point3D>(); //21
            public List<Point3D> HandTipLeft = new List<Point3D>(); //22
            public List<Point3D> ThumbLeft = new List<Point3D>(); //23
            public List<Point3D> HandTipRight = new List<Point3D>(); //24
            public List<Point3D> ThumbRight = new List<Point3D>(); //25
        }

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            if(readyT ==true)
            {
                SB7.Content = "計時(秒):" + timer2 / 30;
                timer2++;
                if(timer2/30 == 5)
                {
                    play_all();
                    readyT = false;
                }
            }



            ColorFrame colorFrame = null;
            bool isBitmapLocked = false;

            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();
            if (multiSourceFrame == null)
            {
                return;
            }
            try
            {
                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();
                if (colorFrame == null)
                {
                    return;
                }
                this.bitmap_color.Lock();
                colorFrame.CopyConvertedFrameDataToIntPtr(this.bitmap_color.BackBuffer, this.bitmapBackBufferSize, ColorImageFormat.Bgra);
                this.bitmap_color.AddDirtyRect(new Int32Rect(0, 0, this.bitmap_color.PixelWidth, this.bitmap_color.PixelHeight));
                this.bitmap_color.Unlock();

                //ImageBrush imgBrush = new ImageBrush(bitmap_color);
                //DiffuseMaterial side1Material = new DiffuseMaterial(imgBrush);
                //GeometryMode_set_learner.Material = side1Material;

            }
            finally
            {
                if (isBitmapLocked)
                {
                    this.bitmap_color.Unlock();
                }
                if (colorFrame != null) colorFrame.Dispose();
            }

            using (BodyFrame bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (bodies == null)
                    {
                        bodies = new Body[bodyFrame.BodyCount];
                    }
                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                foreach (Body body in bodies)
                {
                    if (body.IsTracked)
                    {
                        body_ready = true;
                        IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                        // convert the joint points to depth (display) space
                        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                        foreach (JointType jointType in joints.Keys)
                        {
                            // sometimes the depth(Z) of an inferred joint may show as negative
                            // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                            CameraSpacePoint position = joints[jointType].Position;
                            if (position.Z < 0)
                            {
                                position.Z = InferredZPositionClamp;
                            }

                            ColorSpacePoint ColorSpacePoint = coordinateMapper.MapCameraPointToColorSpace(position);
                            DepthSpacePoint depthSpacePoint = coordinateMapper.MapCameraPointToDepthSpace(position);
                            //jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            jointPoints[jointType] = new Point((int)(ColorSpacePoint.X + 0.5), (int)(ColorSpacePoint.Y + 0.5));

                            learner_center_X = ColorSpacePoint.X;
                            learner_center_Y = ColorSpacePoint.Y;
                        }
                    }
                }
            }
        }

        private void timer_Tick_Path(object sender, EventArgs e)
        {
            // MGroup.Children.Clear();
            // MGroup.Children.Add(myDirLight);

            int i = 0;
            while ((frameCnt2 < frameCnt) && (i < 1))
            {
                //Path3D(HandLeft[frameCnt2], HandLeft[frameCnt2+1], Colors.Gold);
                //Path3D(HandRight[frameCnt2], HandRight[frameCnt2+1], Colors.Yellow);
                //Path3D(FootLeft[frameCnt2], FootLeft[frameCnt2+1], Colors.Red);
                //Path3D(FootRight[frameCnt2], FootRight[frameCnt2+1], Colors.Pink);
                //Path3D(Head[frameCnt2], Head[frameCnt2+1], Colors.Green);



                Pt3D_coach(PathDepth.HandLeft[frameCnt2], Colors.Pink);
                Pt3D_coach(PathDepth.HandRight[frameCnt2], Colors.Yellow);
                //Pt3D(FootLeft[frameCnt2], Colors.Red);
                //Pt3D(FootRight[frameCnt2], Colors.Pink);
                //Pt3D(Head[frameCnt2], Colors.Green);
                //Path3D(HandLeft[i], HandLeft[i + 1], Colors.Red);
                MyModel_coach.Content = MGroup_coach;

                frameCnt2++;
                i++;
            }
            if (frameCnt2 > frameCnt)
            { timer_Path.Stop(); }
            //label1.Content = t.ToString();
            t++;
        }

        private void timer_Tick_fps(object sender, EventArgs e)
        {
            Image<Bgr, Byte> img = cap.QueryFrame();
            if (img == null)
            {
                return;
            }
            MIplImage mImg = (MIplImage)Marshal.PtrToStructure(img.Ptr, typeof(MIplImage));
            int hei = img.Height;
            int wid = img.Width;
            Image<Bgra, byte> imga = new Image<Bgra, byte>(wid, hei);
            MIplImage mImga = (MIplImage)Marshal.PtrToStructure(imga.Ptr, typeof(MIplImage));
            unsafe
            {
                byte* ptra = (byte*)mImga.imageData.ToPointer();
                byte* ptr = (byte*)mImg.imageData.ToPointer();
                for (int y = 0; y < hei; y++)
                    for (int x = 0; x < wid; x++)
                    {
                        int index = y * wid + x;
                        index = index * 4;

                        int i = y * wid + x;
                        i = i * 3;

                        if (ptr[i] > 220 && ptr[i + 1] > 220 && ptr[i + 2] > 220)
                        {
                            ptra[index + 3] = 0;
                        }
                        else
                        {
                            ptra[index] = ptr[i];
                            ptra[index + 1] = ptr[i + 1];
                            ptra[index + 2] = ptr[i + 2];
                            ptra[index + 3] = 0xff;
                        }
                    }
            }

            ////刷新set3D的畫布
            //BitmapSource BSource = ToBitmapSource(imga);
            //ImageBrush imgBrush = new ImageBrush(BSource);
            //DiffuseMaterial side1Material = new DiffuseMaterial(imgBrush);
            //GeometryMode_set_coach.Material = side1Material;


            //////
            if (frameCnt2 % 30 == 0 && frameCnt2 > 0)
            {
                MGroup_learner_path.Children.Clear();
                //MGroup_coach_path.Children.Clear();


                for (int dd = 0; dd < HandLeft_D.Count; dd++)
                {
                    if ((HandLeft_D[dd] < frameCnt2) && (HandLeft_D[dd] > (frameCnt2 - 30)) && dd < HandLeft_D.Count )
                    {
                        Path3D_learner(PathColor.HandLeft[HandLeft_D[dd - 1]], PathColor.HandLeft[HandLeft_D[dd]], Colors.Pink);
                        //Pt3D_learner(PathColor.HandLeft[HandLeft_D[dd]], Colors.IndianRed);

                        Path3D_coach(PathDepth.HandLeft[HandLeft_D[dd - 1]], PathDepth.HandLeft[HandLeft_D[dd]], Colors.Pink);
                        //Pt3D_coach(PathDepth.HandLeft[HandLeft_D[dd]], Colors.IndianRed);
                    }
                }

                for (int ff = 0; ff < HandRight_D.Count; ff++)
                {
                    if ((HandRight_D[ff] < frameCnt2) && (HandRight_D[ff] > (frameCnt2 - 30)) && ff < HandRight_D.Count )
                    {
                        Path3D_learner(PathColor.HandRight[HandRight_D[ff - 1]], PathColor.HandRight[HandRight_D[ff]], Colors.SkyBlue);
                        //Pt3D_learner(PathColor.HandRight[HandRight_D[ff]], Colors.AliceBlue);

                        Path3D_coach(PathDepth.HandRight[HandRight_D[ff - 1]], PathDepth.HandRight[HandRight_D[ff]], Colors.SkyBlue);
                        //Pt3D_coach(PathDepth.HandRight[HandRight_D[ff]], Colors.AliceBlue);
                    }
                }



                //for (int ff = 0; ff < ShoulderLeft_D.Count; ff++)
                //{
                //    if ((ShoulderLeft_D[ff] < frameCnt2) && (ShoulderLeft_D[ff] > (frameCnt2 - 20)) && ff < ShoulderLeft_D.Count - 1)
                //    {
                //        Path3D_learner(PathColor.ShoulderLeft[ShoulderLeft_D[ff - 1]], PathColor.ShoulderLeft[ShoulderLeft_D[ff]], Colors.Chocolate);
                //        //Pt3D_learner(PathColor.HandRight[HandRight_D[ff]], Colors.AliceBlue);

                //        Path3D_coach(PathDepth.HandRight[HandRight_D[ff - 1]], PathDepth.HandRight[HandRight_D[ff]], Colors.Blue);
                //        //Pt3D_coach(PathDepth.HandRight[HandRight_D[ff]], Colors.AliceBlue);
                //    }
                //}

                //for (int ff = 0; ff < ShoulderRight_D.Count; ff++)
                //{
                //    if ((ShoulderRight_D[ff] < frameCnt2) && (ShoulderRight_D[ff] > (frameCnt2 - 20)) && ff < ShoulderRight_D.Count - 1)
                //    {
                //        Path3D_learner(PathColor.ShoulderRight[ShoulderRight_D[ff - 1]], PathColor.ShoulderRight[ShoulderRight_D[ff]], Colors.Orange);
                //        //Pt3D_learner(PathColor.HandRight[HandRight_D[ff]], Colors.AliceBlue);

                //        Path3D_coach(PathDepth.HandRight[HandRight_D[ff - 1]], PathDepth.HandRight[HandRight_D[ff]], Colors.Blue);
                //        //Pt3D_coach(PathDepth.HandRight[HandRight_D[ff]], Colors.AliceBlue);
                //    }
                //}



                for (int ff = 0; ff < FootLeft_D.Count; ff++)
                {
                    if ((FootLeft_D[ff] < frameCnt2) && (FootLeft_D[ff] > (frameCnt2 - 30)) && ff < FootLeft_D.Count )
                    {
                        Path3D_learner(PathColor.FootLeft[FootLeft_D[ff - 1]], PathColor.FootLeft[FootLeft_D[ff]], Colors.LightGreen);
                        //Pt3D_learner(PathColor.FootLeft[FootLeft_D[ff]], Colors.GreenYellow);

                        Path3D_coach(PathDepth.FootLeft[FootLeft_D[ff - 1]], PathDepth.FootLeft[FootLeft_D[ff]], Colors.LightGreen);
                        //Pt3D_coach(PathDepth.FootLeft[FootLeft_D[ff]], Colors.GreenYellow);
                    }
                }

                for (int ff = 0; ff < FootRight_D.Count; ff++)
                {
                    if ((FootRight_D[ff] < frameCnt2) && (FootRight_D[ff] > (frameCnt2 - 30)) && ff < FootRight_D.Count)
                    {
                        Path3D_learner(PathColor.FootRight[FootRight_D[ff - 1]], PathColor.FootRight[FootRight_D[ff]], Colors.Brown);
                        //Pt3D_learner(PathColor.FootRight[FootRight_D[ff]], Colors.BurlyWood);

                        Path3D_coach(PathDepth.FootRight[FootRight_D[ff - 1]], PathDepth.FootRight[FootRight_D[ff]], Colors.Brown);
                        //Pt3D_coach(PathDepth.FootRight[FootRight_D[ff]], Colors.BurlyWood);
                    }
                }

                for (int ff = 0; ff < SpineBase_D.Count; ff++)
                {
                    if ((SpineBase_D[ff] < frameCnt2) && (SpineBase_D[ff] > (frameCnt2 - 30)) && ff < SpineBase_D.Count)
                    {
                        Path3D_learner(PathColor.SpineBase[SpineBase_D[ff - 1]], PathColor.SpineBase[SpineBase_D[ff]], Colors.LightYellow);
                        //Pt3D_learner(PathColor.SpineBase[frameCnt2], Colors.Yellow);

                        Path3D_coach(PathDepth.SpineBase[SpineBase_D[ff - 1]], PathDepth.SpineBase[SpineBase_D[ff]], Colors.LightYellow);
                        //Pt3D_coach(PathDepth.SpineBase[frameCnt2], Colors.Yellow);
                    }
                }
            }


            //軌跡FPS產生長度
            //if (frameCnt2 % 20 == 0)
            //{
            //    MGroup_coach_path.Children.Clear();
            //    //MGroup_learner_path.Children.Clear();
            //}

            //Path3D_coach(HandLeft[frameCnt2], HandLeft[frameCnt2 + 1], Colors.Blue);
            //Path3D_coach(HandRight[frameCnt2], HandRight[frameCnt2 + 1], Colors.Red);

            //Path3D_learner(PathColor.HandLeft[frameCnt2], PathColor.HandLeft[frameCnt2 + 1], Colors.Pink);
            //Path3D_learner(PathColor.HandRight[frameCnt2], PathColor.HandRight[frameCnt2 + 1], Colors.Silver);

            //Path3D_learner(PathColor.ShoulderRight[frameCnt2], PathColor.ShoulderRight[frameCnt2 + 1], Colors.Green);
            //Path3D_learner(PathColor.ShoulderLeft[frameCnt2], PathColor.ShoulderLeft[frameCnt2 + 1], Colors.Red);

            //Path3D_learner(PathColor.FootLeft[frameCnt2], PathColor.FootLeft[frameCnt2 + 1], Colors.LightGreen);
            //Path3D_learner(PathColor.FootRight[frameCnt2], PathColor.FootRight[frameCnt2 + 1], Colors.Brown);

            //////Path3D(FootLeft[frameCnt2], FootLeft[frameCnt2+1], Colors.Red);


            //Pt3D_learner(PathColor.HandLeft[frameCnt2], Colors.IndianRed);
            ////Pt3D_coach(HandLeft[frameCnt2], Colors.Green);

            //Pt3D_learner(PathColor.HandRight[frameCnt2], Colors.AliceBlue);
            ////Pt3D_coach(HandRight[frameCnt2], Colors.Pink);

            //Pt3D_learner(PathColor.FootLeft[frameCnt2], Colors.GreenYellow);

            //Pt3D_learner(PathColor.FootRight[frameCnt2], Colors.BurlyWood);

            //Pt3D_learner(PathColor.SpineBase[frameCnt2], Colors.Yellow);
            ////Pt3D_coach(SpineBase[frameCnt2], Colors.Yellow);

            //MyModel_coach.Content = MGroup_coach;
            //MyModel_coach_path.Content = MGroup_coach_path;
            MyModel_learner_path.Content = MGroup_learner_path;

            frameCnt2++;
            if (frameCnt2 == frameCnt-10)
            {
                timer_Path.Stop();
                timer_fps.Stop();
            }
            SB2.Content ="影片Frame："+frameCnt.ToString();
            SB4.Content = "當前影片Frame" + t.ToString();
            SB5.Content = t/30+"秒";
            t++;
            //image1.Source = BSource;
        }

        private void Pt3D_coach(Point3D p, Color cl)
        {
            double x = 512-p.X;
            double y = 424-p.Y;
            double z = p.Z;

            MeshGeometry3D sidePlane = new MeshGeometry3D();
            GeometryModel3D GM = new GeometryModel3D();

            //sidePlane.Positions.Add(new Point3D(0, 0, 0));
            //sidePlane.Positions.Add(new Point3D(0, 0, 5));
            //sidePlane.Positions.Add(new Point3D(5, 0, 0));
            //sidePlane.Positions.Add(new Point3D(5, 0, 0));
            //sidePlane.Positions.Add(new Point3D(0, 5, 0));
            //sidePlane.Positions.Add(new Point3D(5, 5, 0));
            //sidePlane.Positions.Add(new Point3D(0, 5, 5));
            //sidePlane.Positions.Add(new Point3D(5, 5, 5));

            sidePlane.Positions.Add(new Point3D(x, y, z));
            sidePlane.Positions.Add(new Point3D(x, y, z + 5));
            sidePlane.Positions.Add(new Point3D(x + 5, y, z));
            sidePlane.Positions.Add(new Point3D(x + 5, y, z + 5));
            sidePlane.Positions.Add(new Point3D(x, y + 5, z));
            sidePlane.Positions.Add(new Point3D(x + 5, y + 5, z));
            sidePlane.Positions.Add(new Point3D(x, y + 5, z + 5));
            sidePlane.Positions.Add(new Point3D(x + 5, y + 5, z + 5));

            sidePlane.TriangleIndices.Add(2);
            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(0);

            sidePlane.TriangleIndices.Add(3);
            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(2);

            sidePlane.TriangleIndices.Add(3);
            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(1);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(1);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(3);
            sidePlane.TriangleIndices.Add(2);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(2);
            sidePlane.TriangleIndices.Add(5);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(5);
            sidePlane.TriangleIndices.Add(6);

            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(5);
            sidePlane.TriangleIndices.Add(4);

            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(4);
            sidePlane.TriangleIndices.Add(1);

            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(4);
            sidePlane.TriangleIndices.Add(0);

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(4);
            sidePlane.TriangleIndices.Add(5);

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(5);
            sidePlane.TriangleIndices.Add(2);

            SolidColorBrush brush = new SolidColorBrush(cl);
            DiffuseMaterial sideMaterial = new DiffuseMaterial(brush);

            //TranslateTransform3D TT = new TranslateTransform3D(new Vector3D(x, y, z));
            //Transform3DGroup TG = new Transform3DGroup();
            //TG.Children.Add(TT);

            //GM.Transform = TT;
            GM.Material = sideMaterial;
            GM.Geometry = sidePlane;
            MGroup_coach_path.Children.Add(GM);
        }

        private void Pt3D_learner(Point3D p, Color cl)
        {
            double x = 1920 - p.X+100;
            double y = 1080 - p.Y;
            double z = p.Z;

            MeshGeometry3D sidePlane = new MeshGeometry3D();
            GeometryModel3D GM = new GeometryModel3D();

            sidePlane.Positions.Add(new Point3D(x+10, y+10, z-10));
            sidePlane.Positions.Add(new Point3D(x+10, y+10, z+10));
            sidePlane.Positions.Add(new Point3D(x+10, y-10, z-10));
            sidePlane.Positions.Add(new Point3D(x+10, y-10, z+10));
            sidePlane.Positions.Add(new Point3D(x-10, y+10, z-10));
            sidePlane.Positions.Add(new Point3D(x-10, y+10, z+10));
            sidePlane.Positions.Add(new Point3D(x-10, y-10, z-10));
            sidePlane.Positions.Add(new Point3D(x-10, y-10, z+10));

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(2);

            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(3);
            sidePlane.TriangleIndices.Add(2);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(3);
            sidePlane.TriangleIndices.Add(1);

            sidePlane.TriangleIndices.Add(5);
            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(1);

            sidePlane.TriangleIndices.Add(5);
            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(4);

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(4);
            sidePlane.TriangleIndices.Add(1);

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(2);
            sidePlane.TriangleIndices.Add(6);

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(4);

            sidePlane.TriangleIndices.Add(2);
            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(3);

            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(3);

            sidePlane.TriangleIndices.Add(3);
            sidePlane.TriangleIndices.Add(4);
            sidePlane.TriangleIndices.Add(7);

            sidePlane.TriangleIndices.Add(4);
            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(7);

            SolidColorBrush brush = new SolidColorBrush(cl);
            DiffuseMaterial sideMaterial = new DiffuseMaterial(brush);



            GM.Material = sideMaterial;
            GM.Geometry = sidePlane;
            MGroup_learner_path.Children.Add(GM);
        }



        private void Path3D_coach(Point3D p1, Point3D p2, Color cl)
        {
            double x1 = 512 - p1.X;
            double y1 = 424 - p1.Y;
            double z1 = p1.Z*2;
            double x2 = 512 - p2.X;
            double y2 = 424 - p2.Y;
            double z2 = p2.Z*2;
            double dis = 0;

            dis = Math.Sqrt(Math.Pow((x2 - x1), 2) + Math.Pow((y2 - y1), 2) + Math.Pow((z2 - z1), 2));

            MeshGeometry3D sidePlane = new MeshGeometry3D();
            GeometryModel3D GM = new GeometryModel3D();

            MeshGeometry3D sidePlane2 = new MeshGeometry3D();
            GeometryModel3D GM2 = new GeometryModel3D();

            sidePlane2.Positions.Add(new Point3D(dis - 7, 2, -2));
            sidePlane2.Positions.Add(new Point3D(dis - 7, 2, 2));
            sidePlane2.Positions.Add(new Point3D(dis - 7, -2, -2));
            sidePlane2.Positions.Add(new Point3D(dis - 7, -2, 2));

            sidePlane2.Positions.Add(new Point3D(dis, 0, 0));

            sidePlane2.TriangleIndices.Add(2);
            sidePlane2.TriangleIndices.Add(1);
            sidePlane2.TriangleIndices.Add(0);

            sidePlane2.TriangleIndices.Add(2);
            sidePlane2.TriangleIndices.Add(3);
            sidePlane2.TriangleIndices.Add(1);

            sidePlane2.TriangleIndices.Add(0);
            sidePlane2.TriangleIndices.Add(4);
            sidePlane2.TriangleIndices.Add(2);

            sidePlane2.TriangleIndices.Add(0);
            sidePlane2.TriangleIndices.Add(1);
            sidePlane2.TriangleIndices.Add(4);

            sidePlane2.TriangleIndices.Add(1);
            sidePlane2.TriangleIndices.Add(3);
            sidePlane2.TriangleIndices.Add(4);

            sidePlane2.TriangleIndices.Add(2);
            sidePlane2.TriangleIndices.Add(4);
            sidePlane2.TriangleIndices.Add(3);


            sidePlane.Positions.Add(new Point3D(5, 0.5, 0.5));
            sidePlane.Positions.Add(new Point3D(5, 0.5, -0.5));
            sidePlane.Positions.Add(new Point3D(5, -0.5, 0.5));
            sidePlane.Positions.Add(new Point3D(5, -0.5, -0.5));
            sidePlane.Positions.Add(new Point3D(dis - 7, 0.5, 0.5));
            sidePlane.Positions.Add(new Point3D(dis - 7, 0.5, -0.5));
            sidePlane.Positions.Add(new Point3D(dis - 7, -0.5, 0.5));
            sidePlane.Positions.Add(new Point3D(dis - 7, -0.5, -0.5));

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(2);

            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(3);
            sidePlane.TriangleIndices.Add(2);

            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(5);
            sidePlane.TriangleIndices.Add(4);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(5);
            sidePlane.TriangleIndices.Add(6);

            sidePlane.TriangleIndices.Add(5);
            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(0);

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(4);
            sidePlane.TriangleIndices.Add(5);

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(4);

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(2);
            sidePlane.TriangleIndices.Add(6);

            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(2);
            sidePlane.TriangleIndices.Add(7);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(2);
            sidePlane.TriangleIndices.Add(3);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(5);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(3);
            sidePlane.TriangleIndices.Add(1);

            SolidColorBrush brush = new SolidColorBrush(cl);

            DiffuseMaterial sideMaterial = new DiffuseMaterial(brush);

            double angle;
            double radians;

            Transform3DGroup TG = new Transform3DGroup();

            radians = Math.Atan2(z2 - z1, x2 - x1);
            angle = radians * (180 / Math.PI);
            RotateTransform3D RT3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, -1, 0), angle));
            TG.Children.Add(RT3D);

            radians = Math.Atan2(y2 - y1, Math.Sqrt((x2 - x1) * (x2 - x1) + (z2 - z1) * (z2 - z1)));
            angle = radians * (180 / Math.PI);
            RT3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(-(z2 - z1), 0, x2 - x1), angle));
            TG.Children.Add(RT3D);

            //radians = Math.Atan2(-(z2 - z1), y2 - y1);
            //angle = radians * (180 / Math.PI);
            //RT3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), angle));
            //TG.Children.Add(RT3D);

            TranslateTransform3D TT = new TranslateTransform3D(new Vector3D(x1, y1, z1));
            TG.Children.Add(TT);

            GM.Transform = TG;

            GM.Material = sideMaterial;

            //GM.BackMaterial = sideMaterial;

            GM.Geometry = sidePlane;

            GM2.Material = sideMaterial;
            GM2.Geometry = sidePlane2;
            GM2.Transform = TG;


            MGroup_coach_path.Children.Add(GM);
            MGroup_coach_path.Children.Add(GM2);

        }

        //private void Path3D_learner(Point3D p1, Point3D p2, Color cl)
        //{
        //    double x1 = 1920 - p1.X;
        //    double y1 = 1080 - p1.Y;
        //    double z1 = p1.Z * 2;
        //    double x2 = 1920 - p2.X;
        //    double y2 = 1080 - p2.Y;
        //    double z2 = p2.Z * 2;

        //    MeshGeometry3D sidePlane = new MeshGeometry3D();
        //    GeometryModel3D GM = new GeometryModel3D();

        //    sidePlane.Positions.Add(new Point3D(x1, y1 + 5, z1 + 5));
        //    sidePlane.Positions.Add(new Point3D(x1, y1 + 5, z1 - 5));
        //    sidePlane.Positions.Add(new Point3D(x1, y1 - 5, z1 + 5));
        //    sidePlane.Positions.Add(new Point3D(x1, y1 - 5, z1 - 5));
        //    sidePlane.Positions.Add(new Point3D(x2, y2 + 5, z2 + 5));
        //    sidePlane.Positions.Add(new Point3D(x2, y2 + 5, z2 - 5));
        //    sidePlane.Positions.Add(new Point3D(x2, y2 - 5, z2 + 5));
        //    sidePlane.Positions.Add(new Point3D(x2, y2 - 5, z2 - 5));

        //    sidePlane.TriangleIndices.Add(0);
        //    sidePlane.TriangleIndices.Add(1);
        //    sidePlane.TriangleIndices.Add(2);

        //    sidePlane.TriangleIndices.Add(1);
        //    sidePlane.TriangleIndices.Add(3);
        //    sidePlane.TriangleIndices.Add(2);

        //    sidePlane.TriangleIndices.Add(6);
        //    sidePlane.TriangleIndices.Add(5);
        //    sidePlane.TriangleIndices.Add(4);

        //    sidePlane.TriangleIndices.Add(7);
        //    sidePlane.TriangleIndices.Add(5);
        //    sidePlane.TriangleIndices.Add(6);

        //    sidePlane.TriangleIndices.Add(5);
        //    sidePlane.TriangleIndices.Add(1);
        //    sidePlane.TriangleIndices.Add(0);

        //    sidePlane.TriangleIndices.Add(0);
        //    sidePlane.TriangleIndices.Add(4);
        //    sidePlane.TriangleIndices.Add(5);

        //    sidePlane.TriangleIndices.Add(0);
        //    sidePlane.TriangleIndices.Add(6);
        //    sidePlane.TriangleIndices.Add(4);

        //    sidePlane.TriangleIndices.Add(0);
        //    sidePlane.TriangleIndices.Add(2);
        //    sidePlane.TriangleIndices.Add(6);

        //    sidePlane.TriangleIndices.Add(6);
        //    sidePlane.TriangleIndices.Add(2);
        //    sidePlane.TriangleIndices.Add(7);

        //    sidePlane.TriangleIndices.Add(7);
        //    sidePlane.TriangleIndices.Add(2);
        //    sidePlane.TriangleIndices.Add(3);

        //    sidePlane.TriangleIndices.Add(7);
        //    sidePlane.TriangleIndices.Add(1);
        //    sidePlane.TriangleIndices.Add(5);

        //    sidePlane.TriangleIndices.Add(7);
        //    sidePlane.TriangleIndices.Add(3);
        //    sidePlane.TriangleIndices.Add(1);

        //    //sidePlane.Positions.Add(new Point3D(x1, y1 + 10, z1 + 10));
        //    //sidePlane.Positions.Add(new Point3D(x1, y1 + 10, z1 - 10));
        //    //sidePlane.Positions.Add(new Point3D(x1, y1 - 10, z1 + 10));
        //    //sidePlane.Positions.Add(new Point3D(x1, y1 - 10, z1 - 10));
        //    //sidePlane.Positions.Add(new Point3D(x2, y2 + 10, z2 + 10));
        //    //sidePlane.Positions.Add(new Point3D(x2, y2 + 10, z2 - 10));
        //    //sidePlane.Positions.Add(new Point3D(x2, y2 - 10, z2 + 10));
        //    //sidePlane.Positions.Add(new Point3D(x2, y2 - 10, z2 - 10));


        //    //if(x2>x1)
        //    //{
        //    //    sidePlane.TriangleIndices.Add(0);
        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(2);

        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(3);
        //    //    sidePlane.TriangleIndices.Add(2);

        //    //    sidePlane.TriangleIndices.Add(5);
        //    //    sidePlane.TriangleIndices.Add(6);
        //    //    sidePlane.TriangleIndices.Add(7);

        //    //    sidePlane.TriangleIndices.Add(4);
        //    //    sidePlane.TriangleIndices.Add(6);
        //    //    sidePlane.TriangleIndices.Add(5);

        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(5);
        //    //    sidePlane.TriangleIndices.Add(3);

        //    //    sidePlane.TriangleIndices.Add(5);
        //    //    sidePlane.TriangleIndices.Add(7);
        //    //    sidePlane.TriangleIndices.Add(3);

        //    //    sidePlane.TriangleIndices.Add(4);
        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(0);

        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(4);
        //    //    sidePlane.TriangleIndices.Add(5);

        //    //    sidePlane.TriangleIndices.Add(0);
        //    //    sidePlane.TriangleIndices.Add(2);
        //    //    sidePlane.TriangleIndices.Add(4);

        //    //    sidePlane.TriangleIndices.Add(2);
        //    //    sidePlane.TriangleIndices.Add(4);
        //    //    sidePlane.TriangleIndices.Add(6);

        //    //    sidePlane.TriangleIndices.Add(3);
        //    //    sidePlane.TriangleIndices.Add(6);
        //    //    sidePlane.TriangleIndices.Add(2);

        //    //    sidePlane.TriangleIndices.Add(3);
        //    //    sidePlane.TriangleIndices.Add(7);
        //    //    sidePlane.TriangleIndices.Add(6);
        //    //}
        //    //else
        //    //{
        //    //    sidePlane.TriangleIndices.Add(0);
        //    //    sidePlane.TriangleIndices.Add(2);
        //    //    sidePlane.TriangleIndices.Add(1);

        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(2);
        //    //    sidePlane.TriangleIndices.Add(3);

        //    //    sidePlane.TriangleIndices.Add(5);
        //    //    sidePlane.TriangleIndices.Add(7);
        //    //    sidePlane.TriangleIndices.Add(6);

        //    //    sidePlane.TriangleIndices.Add(5);
        //    //    sidePlane.TriangleIndices.Add(6);
        //    //    sidePlane.TriangleIndices.Add(4);

        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(3);
        //    //    sidePlane.TriangleIndices.Add(5);

        //    //    sidePlane.TriangleIndices.Add(3);
        //    //    sidePlane.TriangleIndices.Add(7);
        //    //    sidePlane.TriangleIndices.Add(5);

        //    //    sidePlane.TriangleIndices.Add(4);
        //    //    sidePlane.TriangleIndices.Add(0);
        //    //    sidePlane.TriangleIndices.Add(1);

        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(5);
        //    //    sidePlane.TriangleIndices.Add(4);

        //    //    sidePlane.TriangleIndices.Add(0);
        //    //    sidePlane.TriangleIndices.Add(4);
        //    //    sidePlane.TriangleIndices.Add(2);

        //    //    sidePlane.TriangleIndices.Add(2);
        //    //    sidePlane.TriangleIndices.Add(6);
        //    //    sidePlane.TriangleIndices.Add(4);

        //    //    sidePlane.TriangleIndices.Add(3);
        //    //    sidePlane.TriangleIndices.Add(2);
        //    //    sidePlane.TriangleIndices.Add(6);

        //    //    sidePlane.TriangleIndices.Add(3);
        //    //    sidePlane.TriangleIndices.Add(6);
        //    //    sidePlane.TriangleIndices.Add(7);
        //    //}


        //    //sidePlane.Positions.Add(new Point3D(x1, y1 + 20, z1));
        //    //sidePlane.Positions.Add(new Point3D(x1, y1, z1 + 34));
        //    //sidePlane.Positions.Add(new Point3D(x1, y1 - 20, z1));

        //    //sidePlane.Positions.Add(new Point3D(x2, y2, z2));

        //    //if (x2 > x1)
        //    //{
        //    //    sidePlane.TriangleIndices.Add(2);
        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(0);

        //    //    sidePlane.TriangleIndices.Add(0);
        //    //    sidePlane.TriangleIndices.Add(3);
        //    //    sidePlane.TriangleIndices.Add(2);

        //    //    sidePlane.TriangleIndices.Add(0);
        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(3);

        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(2);
        //    //    sidePlane.TriangleIndices.Add(3);
        //    //}
        //    //else
        //    //{
        //    //    sidePlane.TriangleIndices.Add(0);
        //    //    sidePlane.TriangleIndices.Add(1);
        //    //    sidePlane.TriangleIndices.Add(2);

        //    //    sidePlane.TriangleIndices.Add(0);
        //    //    sidePlane.TriangleIndices.Add(2);
        //    //    sidePlane.TriangleIndices.Add(3);

        //    //    sidePlane.TriangleIndices.Add(0);
        //    //    sidePlane.TriangleIndices.Add(3);
        //    //    sidePlane.TriangleIndices.Add(1);

        //    //    sidePlane.TriangleIndices.Add(3);
        //    //    sidePlane.TriangleIndices.Add(2);
        //    //    sidePlane.TriangleIndices.Add(1);
        //    //}



        //    //sidePlane.Positions.Add(new Point3D(x1, y1 + 10, z1 +10));
        //    //sidePlane.Positions.Add(new Point3D(x1, y1 + 10, z1 - 10));
        //    //sidePlane.Positions.Add(new Point3D(x1, y1 - 10, z1 + 10));
        //    //sidePlane.Positions.Add(new Point3D(x1, y1 -10, z1 - 10));

        //    //sidePlane.Positions.Add(new Point3D(x2, y2, z2));

        //    //sidePlane.TriangleIndices.Add(2);
        //    //sidePlane.TriangleIndices.Add(1);
        //    //sidePlane.TriangleIndices.Add(0);

        //    //sidePlane.TriangleIndices.Add(3);
        //    //sidePlane.TriangleIndices.Add(1);
        //    //sidePlane.TriangleIndices.Add(2);

        //    //sidePlane.TriangleIndices.Add(1);
        //    //sidePlane.TriangleIndices.Add(4);
        //    //sidePlane.TriangleIndices.Add(3);

        //    //sidePlane.TriangleIndices.Add(2);
        //    //sidePlane.TriangleIndices.Add(3);
        //    //sidePlane.TriangleIndices.Add(4);

        //    //sidePlane.TriangleIndices.Add(0);
        //    //sidePlane.TriangleIndices.Add(2);
        //    //sidePlane.TriangleIndices.Add(4);

        //    //sidePlane.TriangleIndices.Add(0);
        //    //sidePlane.TriangleIndices.Add(1);
        //    //sidePlane.TriangleIndices.Add(4);

        //    SolidColorBrush brush = new SolidColorBrush(cl);

        //    DiffuseMaterial sideMaterial = new DiffuseMaterial(brush);


        //    GM.Material = sideMaterial;

        //    //GM.BackMaterial = sideMaterial;

        //    GM.Geometry = sidePlane;

        //    //double angle;
        //    //double radians;
        //    //double result;
        //    //radians = Math.Atan2(1, 1);
        //    //angle = radians * (180 / Math.PI);

        //    //RotateTransform3D RT3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), angle));

        //    //Transform3DGroup TG = new Transform3DGroup();
        //    //TG.Children.Add(RT3D);

        //    //GM.Transform = TG;


        //    MGroup_learner_path.Children.Add(GM);
        //}

        private void Path3D_learner(Point3D p1, Point3D p2, Color cl)
        {
            double x1 = 1920 - p1.X+300;
            double y1 = 1080 - p1.Y;
            double z1 = p1.Z*2;
            double x2 = 1920 - p2.X+300;
            double y2 = 1080 - p2.Y;
            double z2 = p2.Z*2;

            double dis = 0;

            dis = Math.Sqrt(Math.Pow((x2 - x1),2) + Math.Pow((y2 - y1),2) + Math.Pow((z2 - z1),2));

            MeshGeometry3D sidePlane = new MeshGeometry3D();
            GeometryModel3D GM = new GeometryModel3D();


            MeshGeometry3D sidePlane2 = new MeshGeometry3D();
            GeometryModel3D GM2 = new GeometryModel3D();

            sidePlane2.Positions.Add(new Point3D(dis - 30, 6, -6));
            sidePlane2.Positions.Add(new Point3D(dis - 30, 6, 6));
            sidePlane2.Positions.Add(new Point3D(dis - 30, -6, -6));
            sidePlane2.Positions.Add(new Point3D(dis - 30, -6, 6));

            sidePlane2.Positions.Add(new Point3D(dis - 5, 0, 0));

            sidePlane2.TriangleIndices.Add(2);
            sidePlane2.TriangleIndices.Add(1);
            sidePlane2.TriangleIndices.Add(0);

            sidePlane2.TriangleIndices.Add(2);
            sidePlane2.TriangleIndices.Add(3);
            sidePlane2.TriangleIndices.Add(1);

            sidePlane2.TriangleIndices.Add(0);
            sidePlane2.TriangleIndices.Add(4);
            sidePlane2.TriangleIndices.Add(2);

            sidePlane2.TriangleIndices.Add(0);
            sidePlane2.TriangleIndices.Add(1);
            sidePlane2.TriangleIndices.Add(4);

            sidePlane2.TriangleIndices.Add(1);
            sidePlane2.TriangleIndices.Add(3);
            sidePlane2.TriangleIndices.Add(4);

            sidePlane2.TriangleIndices.Add(2);
            sidePlane2.TriangleIndices.Add(4);
            sidePlane2.TriangleIndices.Add(3);


            sidePlane.Positions.Add(new Point3D(20, 1.5, 1.5));
            sidePlane.Positions.Add(new Point3D(20, 1.5, -1.5));
            sidePlane.Positions.Add(new Point3D(20, -1.5, 1.5));
            sidePlane.Positions.Add(new Point3D(20, -1.5, -1.5));
            sidePlane.Positions.Add(new Point3D(dis - 30, 1.5, 1.5));
            sidePlane.Positions.Add(new Point3D(dis - 30, 1.5, -1.5));
            sidePlane.Positions.Add(new Point3D(dis - 30, -1.5, 1.5));
            sidePlane.Positions.Add(new Point3D(dis - 30, -1.5, -1.5));

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(2);

            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(3);
            sidePlane.TriangleIndices.Add(2);

            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(5);
            sidePlane.TriangleIndices.Add(4);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(5);
            sidePlane.TriangleIndices.Add(6);

            sidePlane.TriangleIndices.Add(5);
            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(0);

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(4);
            sidePlane.TriangleIndices.Add(5);

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(4);

            sidePlane.TriangleIndices.Add(0);
            sidePlane.TriangleIndices.Add(2);
            sidePlane.TriangleIndices.Add(6);

            sidePlane.TriangleIndices.Add(6);
            sidePlane.TriangleIndices.Add(2);
            sidePlane.TriangleIndices.Add(7);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(2);
            sidePlane.TriangleIndices.Add(3);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(1);
            sidePlane.TriangleIndices.Add(5);

            sidePlane.TriangleIndices.Add(7);
            sidePlane.TriangleIndices.Add(3);
            sidePlane.TriangleIndices.Add(1);

            SolidColorBrush brush = new SolidColorBrush(cl);

            DiffuseMaterial sideMaterial = new DiffuseMaterial(brush);




            double angle;
            double radians;

            Transform3DGroup TG = new Transform3DGroup();

            radians = Math.Atan2(z2-z1, x2 - x1);
            angle = radians * (180 / Math.PI);
            RotateTransform3D RT3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, -1, 0), angle));
            TG.Children.Add(RT3D);

            radians = Math.Atan2(y2-y1, Math.Sqrt((x2 - x1)*(x2 - x1) + (z2 - z1)*(z2 - z1)));
            angle = radians * (180 / Math.PI);
            RT3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(-(z2 - z1), 0, x2 - x1), angle));
            TG.Children.Add(RT3D);

            //radians = Math.Atan2(-(z2 - z1), y2 - y1);
            //angle = radians * (180 / Math.PI);
            //RT3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), angle));
            //TG.Children.Add(RT3D);

            TranslateTransform3D TT = new TranslateTransform3D(new Vector3D(x1, y1, z1));
            TG.Children.Add(TT);

            GM.Transform = TG;

            GM.Material = sideMaterial;

            //GM.BackMaterial = sideMaterial;

            GM.Geometry = sidePlane;

            GM2.Material = sideMaterial;
            GM2.Geometry = sidePlane2;
            GM2.Transform = TG;


            MGroup_learner_path.Children.Add(GM);
            MGroup_learner_path.Children.Add(GM2);
        }


        private void set3D_coach()
        {
            // Specify where in the 3D scene the camera is.
            Camera_coach.Position = new Point3D(256, 212, -150);
            // Specify the direction that the camera is pointing.
            Camera_coach.LookDirection = new Vector3D(0, 0, 1);
            // Define camera's horizontal field of view in degrees.
            Camera_coach.FieldOfView = 110;

            V3D_coach.Camera = Camera_coach;

            DirLight_coach.Color = Colors.White;
            DirLight_coach.Direction = new Vector3D(0, 0, 50);

            Mesh_coach.Positions.Add(new Point3D(0, 0, 6));
            Mesh_coach.Positions.Add(new Point3D(0, 424, 6));
            Mesh_coach.Positions.Add(new Point3D(512, 0, 6));
            Mesh_coach.Positions.Add(new Point3D(512, 424, 6));

            //Mesh_coach.Positions.Add(new Point3D(-256, -212, 50));
            //Mesh_coach.Positions.Add(new Point3D(-256, 212, 50));
            //Mesh_coach.Positions.Add(new Point3D(256, -212, 50));
            //Mesh_coach.Positions.Add(new Point3D(256, 212, 50));

            Mesh_coach.TextureCoordinates.Add(new Point(1, 1));
            Mesh_coach.TextureCoordinates.Add(new Point(1, 0));
            Mesh_coach.TextureCoordinates.Add(new Point(0, 1));
            Mesh_coach.TextureCoordinates.Add(new Point(0, 0));

            Mesh_coach.TriangleIndices.Add(0);
            Mesh_coach.TriangleIndices.Add(1);
            Mesh_coach.TriangleIndices.Add(2);

            Mesh_coach.TriangleIndices.Add(2);
            Mesh_coach.TriangleIndices.Add(1);
            Mesh_coach.TriangleIndices.Add(3);

            #region

            //Mesh_coach.Positions.Add(new Point3D(0, 0, 0));
            //Mesh_coach.Positions.Add(new Point3D(0, 0, 20));
            //Mesh_coach.Positions.Add(new Point3D(20, 0, 0));
            //Mesh_coach.Positions.Add(new Point3D(20, 0, 20));
            //Mesh_coach.Positions.Add(new Point3D(0, 20, 0));
            //Mesh_coach.Positions.Add(new Point3D(20, 20, 0));
            //Mesh_coach.Positions.Add(new Point3D(0, 20, 20));
            //Mesh_coach.Positions.Add(new Point3D(20, 20, 20));

            //Mesh_coach.TriangleIndices.Add(2);
            //Mesh_coach.TriangleIndices.Add(1);
            //Mesh_coach.TriangleIndices.Add(0);

            //Mesh_coach.TriangleIndices.Add(3);
            //Mesh_coach.TriangleIndices.Add(1);
            //Mesh_coach.TriangleIndices.Add(2);

            //Mesh_coach.TriangleIndices.Add(3);
            //Mesh_coach.TriangleIndices.Add(7);
            //Mesh_coach.TriangleIndices.Add(1);

            //Mesh_coach.TriangleIndices.Add(7);
            //Mesh_coach.TriangleIndices.Add(6);
            //Mesh_coach.TriangleIndices.Add(1);

            //Mesh_coach.TriangleIndices.Add(7);
            //Mesh_coach.TriangleIndices.Add(3);
            //Mesh_coach.TriangleIndices.Add(2);

            //Mesh_coach.TriangleIndices.Add(7);
            //Mesh_coach.TriangleIndices.Add(2);
            //Mesh_coach.TriangleIndices.Add(5);

            //Mesh_coach.TriangleIndices.Add(7);
            //Mesh_coach.TriangleIndices.Add(5);
            //Mesh_coach.TriangleIndices.Add(6);

            //Mesh_coach.TriangleIndices.Add(6);
            //Mesh_coach.TriangleIndices.Add(5);
            //Mesh_coach.TriangleIndices.Add(4);

            //Mesh_coach.TriangleIndices.Add(6);
            //Mesh_coach.TriangleIndices.Add(4);
            //Mesh_coach.TriangleIndices.Add(1);

            //Mesh_coach.TriangleIndices.Add(1);
            //Mesh_coach.TriangleIndices.Add(4);
            //Mesh_coach.TriangleIndices.Add(0);

            //Mesh_coach.TriangleIndices.Add(0);
            //Mesh_coach.TriangleIndices.Add(4);
            //Mesh_coach.TriangleIndices.Add(5);

            //Mesh_coach.TriangleIndices.Add(0);
            //Mesh_coach.TriangleIndices.Add(5);
            //Mesh_coach.TriangleIndices.Add(2);


            #endregion

            //Color cl = new Color();
            //cl.R = (byte)0;
            //cl.G = (byte)0;
            //cl.B = (byte)0;
            //cl.A = (byte)255;
            //SolidColorBrush brush = new SolidColorBrush(cl);
            //DiffuseMaterial side1Material = new DiffuseMaterial(brush);

            //GeometryMode_set_coach.Material = side1Material;
            GeometryMode_set_coach.Geometry = Mesh_coach;
            MGroup_coach.Children.Add(GeometryMode_set_coach);
            MGroup_coach.Children.Add(DirLight_coach);
            MyModel_coach.Content = MGroup_coach;
        }

        private void set3D_learner()
        {
            // Specify where in the 3D scene the camera is.
            Camera_learner.Position = new Point3D(960, 540, -350);
            // Specify the direction that the camera is pointing.
            Camera_learner.LookDirection = new Vector3D(0, 0, 1);
            // Define camera's horizontal field of view in degrees.
            Camera_learner.FieldOfView = 500;

            V3D_learner.Camera = Camera_learner;

            DirLight_learner.Color = Colors.White;
            DirLight_learner.Direction = new Vector3D(-20, -10, 50);

            DirectionalLight Dr = new DirectionalLight();
            Dr.Color = Colors.White;
            Dr.Direction = new Vector3D(0, -10, 20);


            Mesh_learner.Positions.Add(new Point3D(0, 0, 7));
            Mesh_learner.Positions.Add(new Point3D(0, 1080, 7));
            Mesh_learner.Positions.Add(new Point3D(1920, 0, 7));
            Mesh_learner.Positions.Add(new Point3D(1920, 1080, 7));

            Mesh_learner.TextureCoordinates.Add(new Point(1, 1));
            Mesh_learner.TextureCoordinates.Add(new Point(1, 0));
            Mesh_learner.TextureCoordinates.Add(new Point(0, 1));
            Mesh_learner.TextureCoordinates.Add(new Point(0, 0));

            Mesh_learner.TriangleIndices.Add(0);
            Mesh_learner.TriangleIndices.Add(1);
            Mesh_learner.TriangleIndices.Add(2);

            Mesh_learner.TriangleIndices.Add(2);
            Mesh_learner.TriangleIndices.Add(1);
            Mesh_learner.TriangleIndices.Add(3);

            #region

            //side1Plane.Positions.Add(new Point3D(0, 0, 0));
            //side1Plane.Positions.Add(new Point3D(0, 0, 20));
            //side1Plane.Positions.Add(new Point3D(20, 0, 0));
            //side1Plane.Positions.Add(new Point3D(20, 0, 20));
            //side1Plane.Positions.Add(new Point3D(0, 20, 0));
            //side1Plane.Positions.Add(new Point3D(20, 20, 0));
            //side1Plane.Positions.Add(new Point3D(0, 20, 20));
            //side1Plane.Positions.Add(new Point3D(20, 20, 20));

            //side1Plane.TriangleIndices.Add(2);
            //side1Plane.TriangleIndices.Add(1);
            //side1Plane.TriangleIndices.Add(0);

            //side1Plane.TriangleIndices.Add(3);
            //side1Plane.TriangleIndices.Add(1);
            //side1Plane.TriangleIndices.Add(2);

            //side1Plane.TriangleIndices.Add(3);
            //side1Plane.TriangleIndices.Add(7);
            //side1Plane.TriangleIndices.Add(1);

            //side1Plane.TriangleIndices.Add(7);
            //side1Plane.TriangleIndices.Add(6);
            //side1Plane.TriangleIndices.Add(1);

            //side1Plane.TriangleIndices.Add(7);
            //side1Plane.TriangleIndices.Add(3);
            //side1Plane.TriangleIndices.Add(2);

            //side1Plane.TriangleIndices.Add(7);
            //side1Plane.TriangleIndices.Add(2);
            //side1Plane.TriangleIndices.Add(5);

            //side1Plane.TriangleIndices.Add(7);
            //side1Plane.TriangleIndices.Add(5);
            //side1Plane.TriangleIndices.Add(6);

            //side1Plane.TriangleIndices.Add(6);
            //side1Plane.TriangleIndices.Add(5);
            //side1Plane.TriangleIndices.Add(4);

            //side1Plane.TriangleIndices.Add(6);
            //side1Plane.TriangleIndices.Add(4);
            //side1Plane.TriangleIndices.Add(1);

            //side1Plane.TriangleIndices.Add(1);
            //side1Plane.TriangleIndices.Add(4);
            //side1Plane.TriangleIndices.Add(0);

            //side1Plane.TriangleIndices.Add(0);
            //side1Plane.TriangleIndices.Add(4);
            //side1Plane.TriangleIndices.Add(5);

            //side1Plane.TriangleIndices.Add(0);
            //side1Plane.TriangleIndices.Add(5);
            //side1Plane.TriangleIndices.Add(2);


            #endregion

            Color cl = new Color();
            cl.R = (byte)0;
            cl.G = (byte)0;
            cl.B = (byte)0;
            cl.A = (byte)255;
            SolidColorBrush brush = new SolidColorBrush(cl);
            DiffuseMaterial side1Material = new DiffuseMaterial(brush);



            GeometryMode_set_learner.Material = side1Material;
            GeometryMode_set_learner.Geometry = Mesh_learner;
            MGroup_learner.Children.Add(GeometryMode_set_learner);
            MGroup_learner.Children.Add(DirLight_learner);
            MGroup_learner.Children.Add(Dr);
            MyModel_learner.Content = MGroup_learner;
        }

        private void set3D_learner2()
        {
            // Specify where in the 3D scene the camera is.
            Camera_learner.Position = new Point3D(50, 30, -50);
            // Specify the direction that the camera is pointing.
            Camera_learner.LookDirection = new Vector3D(0, 0, 1);
            // Define camera's horizontal field of view in degrees.
            Camera_learner.FieldOfView = 500;

            V3D_learner.Camera = Camera_learner;

            DirLight_learner.Color = Colors.White;
            DirLight_learner.Direction = new Vector3D(-5, -5, 10);

            DirectionalLight Dr = new DirectionalLight();
            Dr.Color = Colors.White;
            Dr.Direction = new Vector3D(0, 0, 50);

            Color cl = new Color();
            cl.R = (byte)0;
            cl.G = (byte)0;
            cl.B = (byte)255;
            cl.A = (byte)255;
            SolidColorBrush brush = new SolidColorBrush(cl);
            DiffuseMaterial side1Material = new DiffuseMaterial(brush);

            double x1 = 10, y1 = 0, z1 = 0;
            double x2 = 15, y2 = 0, z2 = 0;

            Mesh_learner.Positions.Add(new Point3D(x1, y1 + 0.5, z1 + 0.5));
            Mesh_learner.Positions.Add(new Point3D(x1, y1 + 0.5, z1 - 0.5));
            Mesh_learner.Positions.Add(new Point3D(x1, y1 - 0.5, z1 + 0.5));
            Mesh_learner.Positions.Add(new Point3D(x1, y1 - 0.5, z1 - 0.5));
            Mesh_learner.Positions.Add(new Point3D(x2, y2 + 0.5, z2 + 0.5));
            Mesh_learner.Positions.Add(new Point3D(x2, y2 + 0.5, z2 - 0.5));
            Mesh_learner.Positions.Add(new Point3D(x2, y2 - 0.5, z2 + 0.5));
            Mesh_learner.Positions.Add(new Point3D(x2, y2 - 0.5, z2 - 0.5));

            Mesh_learner.TriangleIndices.Add(0);
            Mesh_learner.TriangleIndices.Add(1);
            Mesh_learner.TriangleIndices.Add(2);

            Mesh_learner.TriangleIndices.Add(1);
            Mesh_learner.TriangleIndices.Add(3);
            Mesh_learner.TriangleIndices.Add(2);

            Mesh_learner.TriangleIndices.Add(6);
            Mesh_learner.TriangleIndices.Add(5);
            Mesh_learner.TriangleIndices.Add(4);

            Mesh_learner.TriangleIndices.Add(7);
            Mesh_learner.TriangleIndices.Add(5);
            Mesh_learner.TriangleIndices.Add(6);

            Mesh_learner.TriangleIndices.Add(5);
            Mesh_learner.TriangleIndices.Add(1);
            Mesh_learner.TriangleIndices.Add(0);

            Mesh_learner.TriangleIndices.Add(0);
            Mesh_learner.TriangleIndices.Add(4);
            Mesh_learner.TriangleIndices.Add(5);

            Mesh_learner.TriangleIndices.Add(0);
            Mesh_learner.TriangleIndices.Add(6);
            Mesh_learner.TriangleIndices.Add(4);

            Mesh_learner.TriangleIndices.Add(0);
            Mesh_learner.TriangleIndices.Add(2);
            Mesh_learner.TriangleIndices.Add(6);

            Mesh_learner.TriangleIndices.Add(6);
            Mesh_learner.TriangleIndices.Add(2);
            Mesh_learner.TriangleIndices.Add(7);

            Mesh_learner.TriangleIndices.Add(7);
            Mesh_learner.TriangleIndices.Add(2);
            Mesh_learner.TriangleIndices.Add(3);

            Mesh_learner.TriangleIndices.Add(7);
            Mesh_learner.TriangleIndices.Add(1);
            Mesh_learner.TriangleIndices.Add(5);

            Mesh_learner.TriangleIndices.Add(7);
            Mesh_learner.TriangleIndices.Add(3);
            Mesh_learner.TriangleIndices.Add(1);

            MeshGeometry3D sidePlaneX = new MeshGeometry3D();
            GeometryModel3D GMX = new GeometryModel3D();

            sidePlaneX.Positions.Add(new Point3D(-20, 0.5,  0.5));
            sidePlaneX.Positions.Add(new Point3D(-20, 0.5, -0.5));
            sidePlaneX.Positions.Add(new Point3D(-20, -0.5, 0.5));
            sidePlaneX.Positions.Add(new Point3D(-20, -0.5, -0.5));
            sidePlaneX.Positions.Add(new Point3D(20,0.5,  0.5));
            sidePlaneX.Positions.Add(new Point3D(20, 0.5, -0.5));
            sidePlaneX.Positions.Add(new Point3D(20, -0.5, 0.5));
            sidePlaneX.Positions.Add(new Point3D(20, -0.5, -0.5));

            sidePlaneX.TriangleIndices.Add(0);
            sidePlaneX.TriangleIndices.Add(1);
            sidePlaneX.TriangleIndices.Add(2);

            sidePlaneX.TriangleIndices.Add(1);
            sidePlaneX.TriangleIndices.Add(3);
            sidePlaneX.TriangleIndices.Add(2);

            sidePlaneX.TriangleIndices.Add(6);
            sidePlaneX.TriangleIndices.Add(5);
            sidePlaneX.TriangleIndices.Add(4);

            sidePlaneX.TriangleIndices.Add(7);
            sidePlaneX.TriangleIndices.Add(5);
            sidePlaneX.TriangleIndices.Add(6);

            sidePlaneX.TriangleIndices.Add(5);
            sidePlaneX.TriangleIndices.Add(1);
            sidePlaneX.TriangleIndices.Add(0);

            sidePlaneX.TriangleIndices.Add(0);
            sidePlaneX.TriangleIndices.Add(4);
            sidePlaneX.TriangleIndices.Add(5);

            sidePlaneX.TriangleIndices.Add(0);
            sidePlaneX.TriangleIndices.Add(6);
            sidePlaneX.TriangleIndices.Add(4);

            sidePlaneX.TriangleIndices.Add(0);
            sidePlaneX.TriangleIndices.Add(2);
            sidePlaneX.TriangleIndices.Add(6);

            sidePlaneX.TriangleIndices.Add(6);
            sidePlaneX.TriangleIndices.Add(2);
            sidePlaneX.TriangleIndices.Add(7);

            sidePlaneX.TriangleIndices.Add(7);
            sidePlaneX.TriangleIndices.Add(2);
            sidePlaneX.TriangleIndices.Add(3);

            sidePlaneX.TriangleIndices.Add(7);
            sidePlaneX.TriangleIndices.Add(1);
            sidePlaneX.TriangleIndices.Add(5);

            sidePlaneX.TriangleIndices.Add(7);
            sidePlaneX.TriangleIndices.Add(3);
            sidePlaneX.TriangleIndices.Add(1);

            MeshGeometry3D sidePlaneY = new MeshGeometry3D();
            GeometryModel3D GMY = new GeometryModel3D();

            sidePlaneY.Positions.Add(new Point3D(0.5, -20, 0.5));
            sidePlaneY.Positions.Add(new Point3D(0.5, -20, -0.5));
            sidePlaneY.Positions.Add(new Point3D(-0.5, -20, 0.5));
            sidePlaneY.Positions.Add(new Point3D(-0.5, -20, -0.5));
            sidePlaneY.Positions.Add(new Point3D(0.5, 20, 0.5));
            sidePlaneY.Positions.Add(new Point3D(0.5, 20, -0.5));
            sidePlaneY.Positions.Add(new Point3D(-0.5, 20, 0.5));
            sidePlaneY.Positions.Add(new Point3D(-0.5, 20, -0.5));

            sidePlaneY.TriangleIndices.Add(0);
            sidePlaneY.TriangleIndices.Add(1);
            sidePlaneY.TriangleIndices.Add(2);

            sidePlaneY.TriangleIndices.Add(1);
            sidePlaneY.TriangleIndices.Add(3);
            sidePlaneY.TriangleIndices.Add(2);

            sidePlaneY.TriangleIndices.Add(6);
            sidePlaneY.TriangleIndices.Add(5);
            sidePlaneY.TriangleIndices.Add(4);

            sidePlaneY.TriangleIndices.Add(7);
            sidePlaneY.TriangleIndices.Add(5);
            sidePlaneY.TriangleIndices.Add(6);

            sidePlaneY.TriangleIndices.Add(5);
            sidePlaneY.TriangleIndices.Add(1);
            sidePlaneY.TriangleIndices.Add(0);

            sidePlaneY.TriangleIndices.Add(0);
            sidePlaneY.TriangleIndices.Add(4);
            sidePlaneY.TriangleIndices.Add(5);

            sidePlaneY.TriangleIndices.Add(0);
            sidePlaneY.TriangleIndices.Add(6);
            sidePlaneY.TriangleIndices.Add(4);

            sidePlaneY.TriangleIndices.Add(0);
            sidePlaneY.TriangleIndices.Add(2);
            sidePlaneY.TriangleIndices.Add(6);

            sidePlaneY.TriangleIndices.Add(6);
            sidePlaneY.TriangleIndices.Add(2);
            sidePlaneY.TriangleIndices.Add(7);

            sidePlaneY.TriangleIndices.Add(7);
            sidePlaneY.TriangleIndices.Add(2);
            sidePlaneY.TriangleIndices.Add(3);

            sidePlaneY.TriangleIndices.Add(7);
            sidePlaneY.TriangleIndices.Add(1);
            sidePlaneY.TriangleIndices.Add(5);

            sidePlaneY.TriangleIndices.Add(7);
            sidePlaneY.TriangleIndices.Add(3);
            sidePlaneY.TriangleIndices.Add(1);

            MeshGeometry3D sidePlaneZ = new MeshGeometry3D();
            GeometryModel3D GMZ = new GeometryModel3D();

            sidePlaneZ.Positions.Add(new Point3D(0.5, 0.5, -20));
            sidePlaneZ.Positions.Add(new Point3D(0.5, -0.5, -20));
            sidePlaneZ.Positions.Add(new Point3D(-0.5, 0.5, -20));
            sidePlaneZ.Positions.Add(new Point3D(-0.5, -0.5,-20));
            sidePlaneZ.Positions.Add(new Point3D(0.5, 0.5 , 20));
            sidePlaneZ.Positions.Add(new Point3D(0.5, -0.5, 20));
            sidePlaneZ.Positions.Add(new Point3D(-0.5, 0.5, 20));
            sidePlaneZ.Positions.Add(new Point3D(-0.5, -0.5, 20));

            sidePlaneZ.TriangleIndices.Add(0);
            sidePlaneZ.TriangleIndices.Add(1);
            sidePlaneZ.TriangleIndices.Add(2);

            sidePlaneZ.TriangleIndices.Add(1);
            sidePlaneZ.TriangleIndices.Add(3);
            sidePlaneZ.TriangleIndices.Add(2);

            sidePlaneZ.TriangleIndices.Add(6);
            sidePlaneZ.TriangleIndices.Add(5);
            sidePlaneZ.TriangleIndices.Add(4);

            sidePlaneZ.TriangleIndices.Add(7);
            sidePlaneZ.TriangleIndices.Add(5);
            sidePlaneZ.TriangleIndices.Add(6);

            sidePlaneZ.TriangleIndices.Add(5);
            sidePlaneZ.TriangleIndices.Add(1);
            sidePlaneZ.TriangleIndices.Add(0);

            sidePlaneZ.TriangleIndices.Add(0);
            sidePlaneZ.TriangleIndices.Add(4);
            sidePlaneZ.TriangleIndices.Add(5);

            sidePlaneZ.TriangleIndices.Add(0);
            sidePlaneZ.TriangleIndices.Add(6);
            sidePlaneZ.TriangleIndices.Add(4);

            sidePlaneZ.TriangleIndices.Add(0);
            sidePlaneZ.TriangleIndices.Add(2);
            sidePlaneZ.TriangleIndices.Add(6);

            sidePlaneZ.TriangleIndices.Add(6);
            sidePlaneZ.TriangleIndices.Add(2);
            sidePlaneZ.TriangleIndices.Add(7);

            sidePlaneZ.TriangleIndices.Add(7);
            sidePlaneZ.TriangleIndices.Add(2);
            sidePlaneZ.TriangleIndices.Add(3);

            sidePlaneZ.TriangleIndices.Add(7);
            sidePlaneZ.TriangleIndices.Add(1);
            sidePlaneZ.TriangleIndices.Add(5);

            sidePlaneZ.TriangleIndices.Add(7);
            sidePlaneZ.TriangleIndices.Add(3);
            sidePlaneZ.TriangleIndices.Add(1);

            MeshGeometry3D sidePlaneB = new MeshGeometry3D();
            GeometryModel3D GMB= new GeometryModel3D();

            double xb = -7, yb = -8, zb = 4;

            sidePlaneB.Positions.Add(new Point3D(xb+1, yb+1, zb-1));
            sidePlaneB.Positions.Add(new Point3D(xb+1, yb-1, zb-1));
            sidePlaneB.Positions.Add(new Point3D(xb+1, yb+1, zb+1));
            sidePlaneB.Positions.Add(new Point3D(xb+1, yb-1, zb+1));
            sidePlaneB.Positions.Add(new Point3D(xb-1, yb+1, zb-1));
            sidePlaneB.Positions.Add(new Point3D(xb-1, yb-1, zb-1));
            sidePlaneB.Positions.Add(new Point3D(xb-1, yb+1, zb+1));
            sidePlaneB.Positions.Add(new Point3D(xb-1, yb-1, zb+1));

            sidePlaneB.TriangleIndices.Add(0);
            sidePlaneB.TriangleIndices.Add(2);
            sidePlaneB.TriangleIndices.Add(1);

            sidePlaneB.TriangleIndices.Add(1);
            sidePlaneB.TriangleIndices.Add(2);
            sidePlaneB.TriangleIndices.Add(3);

            sidePlaneB.TriangleIndices.Add(6);
            sidePlaneB.TriangleIndices.Add(4);
            sidePlaneB.TriangleIndices.Add(5);

            sidePlaneB.TriangleIndices.Add(7);
            sidePlaneB.TriangleIndices.Add(6);
            sidePlaneB.TriangleIndices.Add(5);

            sidePlaneB.TriangleIndices.Add(5);
            sidePlaneB.TriangleIndices.Add(0);
            sidePlaneB.TriangleIndices.Add(1);

            sidePlaneB.TriangleIndices.Add(5);
            sidePlaneB.TriangleIndices.Add(4);
            sidePlaneB.TriangleIndices.Add(0);

            sidePlaneB.TriangleIndices.Add(0);
            sidePlaneB.TriangleIndices.Add(4);
            sidePlaneB.TriangleIndices.Add(6);

            sidePlaneB.TriangleIndices.Add(0);
            sidePlaneB.TriangleIndices.Add(6);
            sidePlaneB.TriangleIndices.Add(2);

            sidePlaneB.TriangleIndices.Add(6);
            sidePlaneB.TriangleIndices.Add(7);
            sidePlaneB.TriangleIndices.Add(2);

            sidePlaneB.TriangleIndices.Add(7);
            sidePlaneB.TriangleIndices.Add(3);
            sidePlaneB.TriangleIndices.Add(2);

            sidePlaneB.TriangleIndices.Add(7);
            sidePlaneB.TriangleIndices.Add(5);
            sidePlaneB.TriangleIndices.Add(1);

            sidePlaneB.TriangleIndices.Add(7);
            sidePlaneB.TriangleIndices.Add(1);
            sidePlaneB.TriangleIndices.Add(3);

            double angle;
            double radians;

            Transform3DGroup TG = new Transform3DGroup();

            double x = -7, y =-8, z =4;

            double VtX =-z, VtZ = x;

            //if (y > 0)
            //{
            //    VtX = -z;
            //    VtZ = x;
            //}
            //else if (y<0)
            //{
            //    VtX = z;
            //    VtZ = -x;
            //}


            //radians = Math.Atan2(y, x);
            //angle = radians * (180 / Math.PI);
            //RotateTransform3D RT3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 4), angle));
            //TG.Children.Add(RT3D);

            radians = Math.Atan2(z, x);
            angle = radians * (180 / Math.PI);
            RotateTransform3D RT3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, -1, 0), angle));
            TG.Children.Add(RT3D);

            radians = Math.Atan2(y,Math.Sqrt(x*x+z*z));
            angle = radians * (180 / Math.PI);
            RT3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(VtX, 0, VtZ), angle));
            TG.Children.Add(RT3D);

            //radians = Math.Atan2(y, z);
            //angle = radians * (180 / Math.PI);
            //RT3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(-1, 0, 0), angle));
            //TG.Children.Add(RT3D);

            TranslateTransform3D TT = new TranslateTransform3D(new Vector3D(x, y, z));
            TG.Children.Add(TT);

            GeometryMode_set_learner.Transform = TG;


            GMX.Geometry = sidePlaneX;
            GMX.Material = new DiffuseMaterial(new SolidColorBrush(Colors.Red));

            GMY.Geometry = sidePlaneY;
            GMY.Material = new DiffuseMaterial(new SolidColorBrush(Colors.Black));

            GMZ.Geometry = sidePlaneZ;
            GMZ.Material = new DiffuseMaterial(new SolidColorBrush(Colors.Green));

            GMB.Geometry = sidePlaneB;
            GMB.Material = new DiffuseMaterial(new SolidColorBrush(Colors.Yellow));

            GeometryMode_set_learner.Material = side1Material;
            GeometryMode_set_learner.Geometry = Mesh_learner;
            MGroup_learner.Children.Add(GeometryMode_set_learner);
            MGroup_learner.Children.Add(GMX);
            MGroup_learner.Children.Add(GMY);
            MGroup_learner.Children.Add(GMZ);
            MGroup_learner.Children.Add(GMB);
            MGroup_learner.Children.Add(DirLight_learner);
            //MGroup_learner.Children.Add(Dr);
            MyModel_learner.Content = MGroup_learner;
        }

        [System.Runtime.InteropServices.DllImport("gdi32")]
        private static extern int DeleteObject(IntPtr o);
        public static BitmapSource ToBitmapSource(IImage image)
        {
            using (System.Drawing.Bitmap source = image.Bitmap)
            {
                IntPtr ptr = source.GetHbitmap(); //obtain the Hbitmap
                BitmapSource bs = System.Windows.Interop
                  .Imaging.CreateBitmapSourceFromHBitmap(
                  ptr,
                  IntPtr.Zero,
                  Int32Rect.Empty,
                  System.Windows.Media.Imaging.BitmapSizeOptions.FromEmptyOptions());

                DeleteObject(ptr); //release the HBitmap
                return bs;
            }
        }


        private void Douglas(int a, int b, List<Point3D> PointRecoTr)
        {
            int Tindex = 0;
            double x1 = PointRecoTr[a].X * 1000, y1 = PointRecoTr[a].Y * 1000, z1 = PointRecoTr[a].Z * 1000;
            double x2 = PointRecoTr[b].X * 1000, y2 = PointRecoTr[b].Y * 1000, z2 = PointRecoTr[b].Z * 1000;
            double x3 = 0, y3 = 0, z3 = 0;
            double ACAB, AC, AB, DIS;
            double MaxDis = 0;

            double Diff=0;
            bool Diff_bool=false;


            do
            {
                if (a == 0 && b == PointRecoTr.Count - 1)
                {
                    TempIndex.Clear();
                    TempIndex.Add(a);
                    TempIndex.Add(b);
                    //Dog_TH = 10;
                }

                if (b - a == 1)
                {
                    return;
                }
                else
                {
                    for (int i = a; i <= b; i++)
                    {
                        x3 = PointRecoTr[i].X * 1000;
                        y3 = PointRecoTr[i].Y * 1000;
                        z3 = PointRecoTr[i].Z * 1000;

                        AB = Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));

                        ACAB = (x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1) + (z2 - z1) * (z3 - z1);

                        AC = Math.Sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1) + (z3 - z1) * (z3 - z1));

                        DIS = AC * Math.Sqrt(1 - Math.Pow((ACAB / (AC * AB)), 2));

                        if (DIS >= 1 && DIS > MaxDis)
                        {
                            MaxDis = DIS;
                            Tindex = i;
                        }
                    }
                    if (MaxDis < Dog_TH)
                    {
                        return;
                    }
                    else
                    {
                        TempIndex.Add(Tindex);
                        Douglas(a, Tindex, PointRecoTr);
                        Douglas(Tindex, b, PointRecoTr);
                    }
                }

                if (a == 0 && b == PointRecoTr.Count - 1)
                {
                    sort();
                    Diff = check_diff(TempIndex, PointRecoTr);
                    if (Diff > 0.24)
                    {
                        if (Diff > 0.26)
                        {
                            Dog_TH = Dog_TH - 0.1;
                        }else
                        {
                            Diff_bool = true; SB8.Content = Diff;
                        }
                    }
                    Dog_TH = Dog_TH + 1;
                }
            } while ((Diff_bool == false)&& (a == 0 && b == PointRecoTr.Count - 1));


            if (a == 0 && b == PointRecoTr.Count - 1)
            {
                Dog_TH = 1;
            }

            return;
        }

        private void sort()
        {
            int Temp = 0;
            for (int j = 0; j < TempIndex.Count; j++)
            {
                for (int i = j; i < TempIndex.Count; i++)
                {
                    if (TempIndex[i] < TempIndex[j])
                    {
                        Temp = TempIndex[j];
                        TempIndex[j] = TempIndex[i];
                        TempIndex[i] = Temp;
                    }
                }
            }
        }

        private double check_diff(List<int> Dog, List<Point3D> PointReco)
        {
            int a = 0; 
            int b = 0;
            int i = 0;

            double x1 = 0, y1 = 0, z1 = 0;
            double x2 = 0, y2 = 0, z2 = 0;
            double x3 = 0, y3 = 0, z3 = 0;
            double ACAB, AC, AB, DIS;

            
            double Diff = 0;
            double Dmin;


            for (int cntf= 0; cntf < PointReco.Count; cntf++)
            {
                if((Dog[i] <= cntf) && (Dog[i+1] > cntf))
                {
                    x1 = PointReco[Dog[i]].X * 1000;
                    y1 = PointReco[Dog[i]].Y * 1000;
                    z1 = PointReco[Dog[i]].Z * 1000;

                    x2 = PointReco[Dog[i + 1]].X * 1000;
                    y2 = PointReco[Dog[i + 1]].Y * 1000;
                    z2 = PointReco[Dog[i + 1]].Z * 1000;

                    x3 = PointReco[cntf].X*1000;
                    y3 = PointReco[cntf].Y*1000;
                    z3 = PointReco[cntf].Z*1000;

                    AB = Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));

                    ACAB = (x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1) + (z2 - z1) * (z3 - z1);

                    AC = Math.Sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1) + (z3 - z1) * (z3 - z1));

                    DIS = AC * Math.Sqrt(1 - Math.Pow((ACAB / (AC * AB)), 2));

                    DIS = Math.Round(DIS, 5);

                    Dmin = Math.Sqrt(Math.Pow(DIS / Dog_TH, 2));

                    //Dmin = Math.Pow(DIS / Dog_TH, 2);

                    if (Dmin > 0)
                    {
                        Diff = Dmin + Diff;
                    }

                }
                else 
                {
                    i++;
                    if (i == Dog.Count)
                    {
                        break;
                    }
                }
            }

            Diff = Diff / PointReco.Count;

            //Diff = Math.Sqrt(Diff)/PointReco.Count;

            return Diff;


        }

        private void check_diff(object sender, RoutedEventArgs e)
        {
            check_diff(HandLeft_D, Path3D.HandLeft);
        }

        private void set_3D(object sender, RoutedEventArgs e)
        {
            set3D_coach();
            set3D_learner();
        }

        private void load_path_demo(object sender, RoutedEventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.ShowDialog();
            String st = openFileDialog1.FileName;
            String str = null;

            if (st != "")
            {
                System.IO.StreamReader file = new System.IO.StreamReader(st);
                // str = file.ReadLine();
                while ((str = file.ReadLine()) != null)
                {
                    if (str == "F")
                    {
                        //frameCnt++;
                        frameCnt = (int)Convert.ToDouble(file.ReadLine());
                        SB3.Content ="軌跡總frame數："+frameCnt.ToString();
                        PathDepth.SpineBase.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.SpineMid.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.Neck.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.Head.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.ShoulderLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.ElbowLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.WristLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.HandLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.ShoulderRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.ElbowRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.WristRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.HandRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.HipLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.KneeLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.AnkleLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.FootLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.HipRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.KneeRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.AnkleRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.FootRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.SpineShoulder.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.HandTipLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.ThumbLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.HandTipRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathDepth.ThumbRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                    }
                }

                file.Dispose();
            }
        }

        private void load_path_guild(object sender, RoutedEventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.ShowDialog();
            String st = openFileDialog1.FileName;
            String str = null;

            if (st != "")
            {
                System.IO.StreamReader file = new System.IO.StreamReader(st);
                // str = file.ReadLine();
                while ((str = file.ReadLine()) != null)
                {
                    if (str == "F")
                    {
                        //frameCnt++;
                        frameCnt = (int)Convert.ToDouble(file.ReadLine());
                        SB3.Content = "軌跡總frame數：" + frameCnt.ToString();
                        PathColor.SpineBase.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.SpineMid.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.Neck.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.Head.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.ShoulderLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.ElbowLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.WristLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.HandLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.ShoulderRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.ElbowRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.WristRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.HandRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.HipLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.KneeLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.AnkleLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.FootLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.HipRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.KneeRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.AnkleRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.FootRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.SpineShoulder.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.HandTipLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.ThumbLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.HandTipRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        PathColor.ThumbRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                    }
                }
                file.Dispose();
            }
        }

        private void load_path_compare(object sender, RoutedEventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.ShowDialog();
            String st = openFileDialog1.FileName;
            String str = null;

            if (st != "")
            {
                System.IO.StreamReader file = new System.IO.StreamReader(st);
                // str = file.ReadLine();
                while ((str = file.ReadLine()) != null)
                {
                    if (str == "F")
                    {
                        //frameCnt++;
                        frameCnt = (int)Convert.ToDouble(file.ReadLine());
                        SB3.Content = "軌跡總frame數：" + frameCnt.ToString();
                        Path3D.SpineBase.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.SpineMid.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.Neck.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.Head.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.ShoulderLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.ElbowLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.WristLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.HandLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.ShoulderRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.ElbowRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.WristRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.HandRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.HipLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.KneeLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.AnkleLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.FootLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.HipRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.KneeRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.AnkleRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.FootRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.SpineShoulder.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.HandTipLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.ThumbLeft.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.HandTipRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                        Path3D.ThumbRight.Add(new Point3D(Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine()), Convert.ToDouble(file.ReadLine())));
                    }
                }
                file.Dispose();
            }
        }


        private void load_video(object sender, RoutedEventArgs e)
        {
            openFileDialog_video = new OpenFileDialog();
            openFileDialog_video.ShowDialog();
            file_name_video = openFileDialog_video.FileName;
        }
        private void play_path(object sender, RoutedEventArgs e)
        {
            timer_Path.Start();
        }

        private void play_all(object sender, RoutedEventArgs e)
        {
            //if (body_ready == true)
            //{
                if (file_name_video != "")
                {
                    cap = new Capture(file_name_video);
                    double fps = cap.GetCaptureProperty(Emgu.CV.CvEnum.CAP_PROP.CV_CAP_PROP_FPS);
                    timer_fps = new DispatcherTimer();
                    timer_fps.Tick += new EventHandler(timer_Tick_fps);

                    int delay = (int)(1000 / fps);
                    SB1.Content = "fps:"+fps;
                    timer_fps.Interval = new TimeSpan(0, 0, 0, 0, delay);
                    timer_fps.Start();
                    //timer_Path.Start();
                }
            //}
            //else
            //{
            //    MessageBox.Show("NO BODY");
            //}
        }

        private void play_all()
        {
            //if (body_ready == true)
            //{
            if (file_name_video != "")
            {
                cap = new Capture(file_name_video);
                double fps = cap.GetCaptureProperty(Emgu.CV.CvEnum.CAP_PROP.CV_CAP_PROP_FPS);
                timer_fps = new DispatcherTimer();
                timer_fps.Tick += new EventHandler(timer_Tick_fps);

                int delay = (int)(1000 / fps);
                SB1.Content = "fps:" + fps;
                timer_fps.Interval = new TimeSpan(0, 0, 0, 0, delay);
                timer_fps.Start();
                //timer_Path.Start();
            }
            //}
            //else
            //{
            //    MessageBox.Show("NO BODY");
            //}
        }

        private void simpfy_path(object sender, RoutedEventArgs e)
        {
            Douglas(0, Path3D.SpineBase.Count - 1, Path3D.SpineBase);           SpineBase_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.SpineMid.Count - 1, Path3D.SpineMid);             SpineMid_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.Neck.Count - 1, Path3D.Neck);                     Neck_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.Head.Count - 1, Path3D.Head);                     Head_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.ShoulderLeft.Count - 1, Path3D.ShoulderLeft);     ShoulderLeft_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.ElbowLeft.Count - 1, Path3D.ElbowLeft);           ElbowLeft_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.WristLeft.Count - 1, Path3D.WristLeft);           WristLeft_D = new List<int>(TempIndex);// TempIndex.Clear();
            Douglas(0, Path3D.HandLeft.Count - 1, Path3D.HandLeft);             HandLeft_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.ShoulderRight.Count - 1, Path3D.ShoulderRight);   ShoulderRight_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.ElbowRight.Count - 1, Path3D.ElbowRight);         ElbowRight_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.WristRight.Count - 1, Path3D.WristRight);         WristRight_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.HandRight.Count - 1, Path3D.HandRight);           HandRight_D = new List<int>(TempIndex);// TempIndex.Clear();
            Douglas(0, Path3D.HipLeft.Count - 1, Path3D.HipLeft);               HipLeft_D = new List<int>(TempIndex);// TempIndex.Clear();
            Douglas(0, Path3D.KneeLeft.Count - 1, Path3D.KneeLeft);             KneeLeft_D = new List<int>(TempIndex);// TempIndex.Clear();
            Douglas(0, Path3D.AnkleLeft.Count - 1, Path3D.AnkleLeft);           AnkleLeft_D = new List<int>(TempIndex);// TempIndex.Clear();
            Douglas(0, Path3D.FootLeft.Count - 1, Path3D.FootLeft);             FootLeft_D = new List<int>(TempIndex);// TempIndex.Clear();
            Douglas(0, Path3D.HipRight.Count - 1, Path3D.HipRight);             HipRight_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.KneeRight.Count - 1, Path3D.KneeRight);           KneeRight_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.AnkleRight.Count - 1, Path3D.AnkleRight);         AnkleRight_D = new List<int>(TempIndex);// TempIndex.Clear();
            Douglas(0, Path3D.FootRight.Count - 1, Path3D.FootRight);           FootRight_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.SpineShoulder.Count - 1, Path3D.SpineShoulder);   SpineShoulder_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.HandTipLeft.Count - 1, Path3D.HandTipLeft);       HandTipLeft_D = new List<int>(TempIndex);// TempIndex.Clear();
            Douglas(0, Path3D.ThumbLeft.Count - 1, Path3D.ThumbLeft);           sort(); ThumbLeft_D = new List<int>(TempIndex);// TempIndex.Clear();
            Douglas(0, Path3D.HandTipRight.Count - 1, Path3D.HandTipRight);     sort(); HandTipRight_D = new List<int>(TempIndex); //TempIndex.Clear();
            Douglas(0, Path3D.ThumbRight.Count - 1, Path3D.ThumbRight);         sort(); ThumbRight_D = new List<int>(TempIndex); TempIndex.Clear();
        }

        private void MenuItem_Click(object sender, RoutedEventArgs e)
        {
            readyT = true;
        }


    }
}
