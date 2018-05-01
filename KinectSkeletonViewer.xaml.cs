//------------------------------------------------------------------------------
// <copyright file="KinectSkeletonViewer.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.WpfViewers
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Media;
    using System.Windows.Shapes;
    using Microsoft.Kinect;
    using System.IO;
    using System.Linq;
    using System.Text;
    using System.Diagnostics;
    public enum ImageType
    {
        Color,
        Depth,
    }

    internal enum TrackingMode
    {
        DefaultSystemTracking,
        Closest1Player,
        Closest2Player,
        Sticky1Player,
        Sticky2Player,
        MostActive1Player,
        MostActive2Player
    }

    /// <summary>
    /// Interaction logic for KinectSkeletonViewer.xaml
    /// </summary>
    public partial class KinectSkeletonViewer : ImageViewer, INotifyPropertyChanged
    {
        private const float ActivityFalloff = 0.98f;
        private readonly List<ActivityWatcher> recentActivity = new List<ActivityWatcher>();
        private readonly List<int> activeList = new List<int>();
        private List<KinectSkeleton> skeletonCanvases;
        private List<Dictionary<JointType, JointMapping>> jointMappings = new List<Dictionary<JointType, JointMapping>>();
        private Skeleton[] skeletonData;
        int useless = 0;
        public KinectSkeletonViewer()
        {
            InitializeComponent();
            this.ShowJoints = true;
            this.ShowBones = true;
            this.ShowCenter = true;
        }
        
        public bool ShowBones { get; set; }

        public bool ShowJoints { get; set; }

        public bool ShowCenter { get; set; }

        public ImageType ImageType { get; set; }

        internal TrackingMode TrackingMode { get; set; }

        public void HideAllSkeletons()
        {
            if (this.skeletonCanvases != null)
            {
                foreach (KinectSkeleton skeletonCanvas in this.skeletonCanvases)
                {
                    skeletonCanvas.Reset();
                }
            }
        }

        protected override void OnKinectChanged(KinectSensor oldKinectSensor, KinectSensor newKinectSensor)
        {
            if (oldKinectSensor != null)
            {
                oldKinectSensor.AllFramesReady -= this.KinectAllFramesReady;
                this.HideAllSkeletons();
            }

            if (newKinectSensor != null && newKinectSensor.Status == KinectStatus.Connected)
            {
                newKinectSensor.AllFramesReady += this.KinectAllFramesReady;
            }
        }

        private void KinectAllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            // Have we already been "shut down" by the user of this viewer, 
            // or has the SkeletonStream been disabled since this event was posted?
            if ((this.Kinect == null) || !((KinectSensor)sender).SkeletonStream.IsEnabled)
            {
                return;
            }
            // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "  0  ");
            bool haveSkeletonData = false;

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
             //   File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "  1  ");
                if (skeletonFrame != null)
                {
                    if (this.skeletonCanvases == null)
                    {
                       // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "  2  ");
                        this.CreateListOfSkeletonCanvases();
                    }

                    if ((this.skeletonData == null) || (this.skeletonData.Length != skeletonFrame.SkeletonArrayLength))
                    {
                        this.skeletonData = new Skeleton[skeletonFrame.SkeletonArrayLength];
                      //  File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "  3  ");
                    }

                    skeletonFrame.CopySkeletonDataTo(this.skeletonData);

                    haveSkeletonData = true;
                }
            }


            if (haveSkeletonData)
            {  
                using (DepthImageFrame depthImageFrame = e.OpenDepthImageFrame())
                {
                    if (depthImageFrame != null)
                    {
                     
                       // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "  4  ");
                        int trackedSkeletons = 0;
                        foreach (Skeleton skeleton in this.skeletonData)
                        {
                   
                            if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                            {
                             //   File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt","Tracked"); 
                            }
                            else if (skeleton.TrackingState == SkeletonTrackingState.PositionOnly)
                            {
                               // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", trackedSkeletons.ToString() + "  10  ");
                           
                            }
                            else if (skeleton.TrackingState == SkeletonTrackingState.NotTracked)
                            {
                               //File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "Nottrack");
                          
                            }
                            Dictionary<JointType, JointMapping> jointMapping = this.jointMappings[trackedSkeletons];
                            jointMapping.Clear();

                            KinectSkeleton skeletonCanvas = this.skeletonCanvases[trackedSkeletons++];
                            skeletonCanvas.ShowBones = this.ShowBones;
                            skeletonCanvas.ShowJoints = this.ShowJoints;
                            skeletonCanvas.ShowCenter = this.ShowCenter;
                            int temp = 0;
                            int count1 = 0;
                            float [] data_joint=new float [42];
                            // Transform the data into the correct space
                            // For each joint, we determine the exact X/Y coordinates for the target view
                            foreach (Joint joint in skeleton.Joints)
                            {

                               // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "  6  ");
                                Point mappedPoint = this.GetPosition2DLocation(depthImageFrame, joint.Position);
                                jointMapping[joint.JointType] = new JointMapping
                                    {
                                        Joint = joint, 
                                        MappedPoint = mappedPoint
                                    };

                                Joint asd = joint;
                                if (asd.TrackingState == JointTrackingState.Tracked && asd != skeleton.Joints[JointType.FootRight] && asd != skeleton.Joints[JointType.FootLeft] && asd != skeleton.Joints[JointType.KneeLeft] && asd != skeleton.Joints[JointType.AnkleLeft] && asd != skeleton.Joints[JointType.KneeRight] && asd != skeleton.Joints[JointType.AnkleRight])
                                {
                                   // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", " j "+ j.ToString());
                                    temp++;
                                   //  File.AppendAllText(path,p.ToString()+" "+q.ToString()+" " +" ");
                                    data_joint[count1] = asd.Position.X;
                                    count1++;
                                    data_joint[count1] = asd.Position.Y;
                                    count1++;
                                    data_joint[count1] = asd.Position.Z;
                                    count1++;
                                   //string text;
                                    // text =(asd.Position.Z).ToString()+" ";
                                        // WriteAllText creates a file, writes the specified string to the file,
                                    // and then closes the file.    You do NOT need to call Flush() or Close().
                                  //  System.IO.File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", text);
                                   // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt",asd.Position.X.ToString() + "  " + asd.Position.Y.ToString() + "  " + asd.Position.Z.ToString());
                                 //  Console.WriteLine(asd.Position.X.ToString() + asd.Position.Y.ToString() + asd.Position.Z.ToString());

                                }
                               /* else if(j!=1 )
                                {
                                   // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "NONE track"+ count.ToString() +  Environment.NewLine);

                                    //File.AppendAllText(@"C:\Users\pk1601cs33\helll.csv", " -1 -1 -1 ");
                                    j = 0;
                                 }*/
                            }

                             if(temp==14)
                            {
                                useless++;
                               /*string path=@"C:\Users\pk1601cs33\exer.csv";
                                    StringBuilder sbOutput = new StringBuilder();
                                for(int j =0 ; j< 42 ; j++)
                                {
                                    if(j==0)
                                    File.AppendAllText(path,data_joint[j].ToString());
                                    else
                                        File.AppendAllText(path," "+ data_joint[j].ToString());

                                }
                                File.AppendAllText(path, Environment.NewLine);
                             */
                               if(useless>=10)
                                 {
                                       useless=0;
                                      Insert(data_joint);
                                 }
                             
                               temp = 0; 
                            }
                                
                            //File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", Environment.NewLine + count.ToString() + "  " +Environment.NewLine);
                       /*     if (i == 1)
                            {
                               File.AppendAllText(@"C:\Users\pk1601cs33\not_valid.csv",Environment.NewLine);
                                i = 0;
                            }
                                */
                                
                         //  button1_Click(null, null);
                            // Look up the center point
                            Point centerPoint = this.GetPosition2DLocation(depthImageFrame, skeleton.Position);

                            // Scale the skeleton thickness
                            // 1.0 is the desired size at 640 width
                            double scale = this.RenderSize.Width / 640;

                            skeletonCanvas.RefreshSkeleton(skeleton, jointMapping, centerPoint, scale);
                        }

                        if (ImageType == ImageType.Depth)
                        {
                            this.ChooseTrackedSkeletons(this.skeletonData);
                        }
                    }
                }
            }
        }
        private void  Insert(float [] values)
        {
            // full path of python interpreter
            string python = @"C:\Users\pk1601cs33\AppData\Local\Programs\Python\Python36-32\python.exe";
            // python app to call
            string myPythonApp = @"C:\Users\pk1601cs33\code.py";
            // Create new process start info
            ProcessStartInfo myProcessStartInfo = new ProcessStartInfo(python);

            // make sure we can read the output from stdout
            myProcessStartInfo.UseShellExecute = false;
            myProcessStartInfo.RedirectStandardOutput = true;
            myProcessStartInfo.CreateNoWindow = true;
            myProcessStartInfo.WindowStyle = ProcessWindowStyle.Minimized;

            // start python app with 3 arguments 
            // 1st arguments is pointer to itself, 2nd and 3rd are actual arguments we want to send
            myProcessStartInfo.Arguments = myPythonApp + " " + values[0] + " " + values[1] + " " + values[2] + " " + values[3] + " " + values[4] + " " + values[5]
                                                       + " " + values[6] + " " + values[7] + " " + values[8] + " " + values[9] + " " + values[10] + " " + values[11]
                                                       + " " + values[12] + " " + values[13] + " " + values[14] + " " + values[15] + " " + values[16] + " " + values[17]
                                                       + " " + values[18] + " " + values[19] + " " + values[20] + " " + values[21] + " " + values[22] + " " + values[23]
                                                       + " " + values[24] + " " + values[25] + " " + values[26] + " " + values[27] + " " + values[28] + " " + values[29]
                                                       + " " + values[30] + " " + values[31] + " " + values[32] + " " + values[33] + " " + values[34] + " " + values[35]
                                                       + " " + values[36] + " " + values[37] + " " + values[38] + " " + values[39] + " " + values[40] + " " + values[41];


            Process myProcess = new Process();
            // assign start information to the process
            myProcess.StartInfo = myProcessStartInfo;


            myProcess.Start();

            // Read the standard output of the app we called. 
            // in order to avoid deadlock we will read output first and then wait for process terminate:
            StreamReader myStreamReader = myProcess.StandardOutput;
            string myString = myStreamReader.ReadLine();

            /*if you need to read multiple lines, you might use:
                string myString = myStreamReader.ReadToEnd() */

            // wait exit signal from the app we called and then close it.
            myProcess.WaitForExit();

            myProcess.Close();

            // write the output we got from python app
            Console.WriteLine("Value received from script: " + myString);
            //Console.WriteLine("Value received from script: " + myString);
        }
    
       /* private void run_cmd()
        {

          string fileName = @"C:\Users\pk1601cs33\codesub.py";

            Process p = new Process();
            p.StartInfo = new ProcessStartInfo(@"C:\Users\pk1601cs33\AppData\Local\Programs\Python\Python36-32\python.exe", fileName)
            {
                RedirectStandardOutput = true,
                UseShellExecute = false,
                CreateNoWindow = true
            };
            p.Start();

            string output = p.StandardOutput.ReadToEnd();
            p.WaitForExit();
            
            Console.WriteLine(output);
            Console.ReadLine();

         //   Console.WriteLine("gdhflgkf");
        }*/

        private Point GetPosition2DLocation(DepthImageFrame depthFrame, SkeletonPoint skeletonPoint)
        {
            DepthImagePoint depthPoint = depthFrame.MapFromSkeletonPoint(skeletonPoint);

            switch (ImageType)
            {
                case ImageType.Color:
                    ColorImagePoint colorPoint = depthFrame.MapToColorImagePoint(depthPoint.X, depthPoint.Y, this.Kinect.ColorStream.Format);

                    // map back to skeleton.Width & skeleton.Height
                    return new Point(
                        (int)(this.RenderSize.Width * colorPoint.X / this.Kinect.ColorStream.FrameWidth),
                        (int)(this.RenderSize.Height * colorPoint.Y / this.Kinect.ColorStream.FrameHeight));
                case ImageType.Depth:
                    return new Point(
                        (int)(this.RenderSize.Width * depthPoint.X / depthFrame.Width),
                        (int)(this.RenderSize.Height * depthPoint.Y / depthFrame.Height));
                default:
                    throw new ArgumentOutOfRangeException("ImageType was a not expected value: " + ImageType.ToString());
            }
        }

        private void CreateListOfSkeletonCanvases()
        {
            this.skeletonCanvases = new List<KinectSkeleton>
                {
                    this.skeletonCanvas1,
                    this.skeletonCanvas2,
                    this.skeletonCanvas3,
                    this.skeletonCanvas4,
                    this.skeletonCanvas5,
                    this.skeletonCanvas6
                };

            this.skeletonCanvases.ForEach(s => this.jointMappings.Add(new Dictionary<JointType, JointMapping>()));
        }

        // NOTE: The ChooseTrackedSkeletons part of the KinectSkeletonViewer would be useful
        // separate from the SkeletonViewer.
        private void ChooseTrackedSkeletons(IEnumerable<Skeleton> skeletonDataValue)
        {
            switch (TrackingMode)
            {
                case TrackingMode.Closest1Player:
                    this.ChooseClosestSkeletons(skeletonDataValue, 1);
                    break;
                case TrackingMode.Closest2Player:
                    this.ChooseClosestSkeletons(skeletonDataValue, 2);
                    break;
                case TrackingMode.Sticky1Player:
                    this.ChooseOldestSkeletons(skeletonDataValue, 1);
                    break;
                case TrackingMode.Sticky2Player:
                    this.ChooseOldestSkeletons(skeletonDataValue, 2);
                    break;
                case TrackingMode.MostActive1Player:
                    this.ChooseMostActiveSkeletons(skeletonDataValue, 1);
                    break;
                case TrackingMode.MostActive2Player:
                    this.ChooseMostActiveSkeletons(skeletonDataValue, 2);
                    break;
            }
        }

        private void ChooseClosestSkeletons(IEnumerable<Skeleton> skeletonDataValue, int count)
        {
            SortedList<float, int> depthSorted = new SortedList<float, int>();

            foreach (Skeleton s in skeletonDataValue)
            {
                if (s.TrackingState != SkeletonTrackingState.NotTracked)
                {
                    float valueZ = s.Position.Z;
                    while (depthSorted.ContainsKey(valueZ))
                    {
                        valueZ += 0.0001f;
                    }

                    depthSorted.Add(valueZ, s.TrackingId);
                }
            }

            this.ChooseSkeletonsFromList(depthSorted.Values, count);
        }

        private void ChooseOldestSkeletons(IEnumerable<Skeleton> skeletonDataValue, int count)
        {
            List<int> newList = new List<int>();
            
            foreach (Skeleton s in skeletonDataValue)
            {
                if (s.TrackingState != SkeletonTrackingState.NotTracked)
                {
                    newList.Add(s.TrackingId);
                }
            }

            // Remove all elements from the active list that are not currently present
            this.activeList.RemoveAll(k => !newList.Contains(k));

            // Add all elements that aren't already in the activeList
            this.activeList.AddRange(newList.FindAll(k => !this.activeList.Contains(k)));

            this.ChooseSkeletonsFromList(this.activeList, count);
        }

        private void ChooseMostActiveSkeletons(IEnumerable<Skeleton> skeletonDataValue, int count)
        {
            foreach (ActivityWatcher watcher in this.recentActivity)
            {
                watcher.NewPass();
            }

            foreach (Skeleton s in skeletonDataValue)
            {
                if (s.TrackingState != SkeletonTrackingState.NotTracked)
                {
                    ActivityWatcher watcher = this.recentActivity.Find(w => w.TrackingId == s.TrackingId);
                    if (watcher != null)
                    {
                        watcher.Update(s);
                    }
                    else
                    {
                        this.recentActivity.Add(new ActivityWatcher(s));
                    }
                }
            }

            // Remove any skeletons that are gone
            this.recentActivity.RemoveAll(aw => !aw.Updated);

            this.recentActivity.Sort();
            this.ChooseSkeletonsFromList(this.recentActivity.ConvertAll(f => f.TrackingId), count);
        }

        private void ChooseSkeletonsFromList(IList<int> list, int max)
        {
            if (this.Kinect.SkeletonStream.IsEnabled)
            {
                int argCount = Math.Min(list.Count, max);

                if (argCount == 0)
                {
                    this.Kinect.SkeletonStream.ChooseSkeletons();
                }

                if (argCount == 1)
                {
                    this.Kinect.SkeletonStream.ChooseSkeletons(list[0]);
                }

                if (argCount >= 2)
                {
                    this.Kinect.SkeletonStream.ChooseSkeletons(list[0], list[1]);
                }
            }
        }

        private class ActivityWatcher : IComparable<ActivityWatcher>
        {
            private float activityLevel;
            private SkeletonPoint previousPosition;
            private SkeletonPoint previousDelta;

            internal ActivityWatcher(Skeleton s)
            {
                this.activityLevel = 0.0f;
                this.TrackingId = s.TrackingId;
                this.Updated = true;
                this.previousPosition = s.Position;
                this.previousDelta = new SkeletonPoint();
            }

            internal int TrackingId { get; private set; }

            internal bool Updated { get; private set; }

            public int CompareTo(ActivityWatcher other)
            {
                // Use the existing CompareTo on float, but reverse the arguments,
                // since we wish to have larger activityLevels sort ahead of smaller values.
                return other.activityLevel.CompareTo(this.activityLevel);
            }

            internal void NewPass()
            {
                this.Updated = false;
            }

            internal void Update(Skeleton s)
            {
                SkeletonPoint newPosition = s.Position;
                SkeletonPoint newDelta = new SkeletonPoint
                    {
                        X = newPosition.X - this.previousPosition.X,
                        Y = newPosition.Y - this.previousPosition.Y,
                        Z = newPosition.Z - this.previousPosition.Z
                    };

                SkeletonPoint deltaV = new SkeletonPoint
                    {
                        X = newDelta.X - this.previousDelta.X,
                        Y = newDelta.Y - this.previousDelta.Y,
                        Z = newDelta.Z - this.previousDelta.Z
                    };

                this.previousPosition = newPosition;
                this.previousDelta = newDelta;

                float deltaVLengthSquared = (deltaV.X * deltaV.X) + (deltaV.Y * deltaV.Y) + (deltaV.Z * deltaV.Z);
                float deltaVLength = (float)Math.Sqrt(deltaVLengthSquared);

                this.activityLevel = this.activityLevel * ActivityFalloff;
                this.activityLevel += deltaVLength;

                this.Updated = true;
            }
        }
    }
}
