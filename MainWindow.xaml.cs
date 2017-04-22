/*************************************************************************************************************************************************************************
Ersteller: Schorkops Joé
Datum: 21.03.2017

E-Mail: jschorkops@yahoo.com
Handy: 0032471393542

© 2017 by Schorkops Joé
************************************************************************************************************************************************************************/
//Zuerst initialisiere ich, welche Funktionen ich benötige, damit alles funktioniert.
namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    //Grundfunktionen
    using System.IO;
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
    //Diese Funktion muss ich einfügen, damit ich die Kinect Objekte nutzen kann.
    using Microsoft.Kinect;
    //Mit dieser Funktion kann ich den Serial Port ansprechen.
    using System.IO.Ports;
    //Die beiden letzten Funktionen nutze ich für die Serial Kommunikation mit dem Arduino Due
    using CommandMessenger.Transport.Serial;
    using CommandMessenger;

    // Interaction Logik für MainWindow.xaml
    public partial class MainWindow : Window
    {
        //Prüfen, ob der Kinect Sensor angeschlossen ist.
        KinectSensor sensor2 = KinectSensor.KinectSensors[0];
        //Serial Port starten
        SerialPort sp = new SerialPort();
        //CmdMessenger starten
        CmdMessenger cmdMessemger = null;
        //Serial Kommunikation starten
        SerialTransport serialTransport = null;

        //CmdMessenger Commands initialisieren, also in welcher Reihenfolge die Daten gesendet werden sollen.
        enum Commands
        {
            cmdSendRot1,
            cmdSendRot2,
            cmdSendRot4,
            cmdwahlschalter,
            cmdSendRot45
        };
        //Bild initialisieren, also Farbe/Größe, usw...
        private const float RenderWidth = 640.0f;
        private const float RenderHeight = 480.0f;
        private const double JointThickness = 3;
        private const double BodyCenterThickness = 10;
        private const double ClipBoundsThickness = 10;
        private readonly Brush centerPointBrush = Brushes.Blue;
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));   
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6); 
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private KinectSensor sensor;
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        private Int16 a = 0;
        private string s2;
        public int Gradzahl { get; private set; }

        //Wenn MainWindow aktiviert wurde
        public MainWindow()
        {
            InitializeComponent();

            this.Loaded += new RoutedEventHandler(MainWindow_Loaded);
            this.Unloaded += new RoutedEventHandler(MainWindow_Unloaded);
        }

        //MainWindow fehler beim starten
        void MainWindow_Unloaded(object sender, RoutedEventArgs e)
        {
            sensor2.Stop();
        }

        //MainWindow erfolgreich geladen
        void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            sensor2.Start();
            Gradzahl = 6;
            sensor.ElevationAngle = Gradzahl;
        }

        //Funktion zum rendern
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

        //Wenn Window geladen wurde
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            //erstelle die drawing Gruppe
            this.drawingGroup = new DrawingGroup();

            //erstelle die image, es wird benutzt für die image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            //Display zeichnet das image control
            Image.Source = this.imageSource;

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
                this.sensor.SkeletonStream.Enable();
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;
                // Starte den Sensor
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }
        }
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

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
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
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
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            //Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            //Linker Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            //Rechter Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            //Hier lese ich alle Daten aus, die ich für meine Endarbeit benötige.
            //Base 1
            //Zuerst lege ich fest, welche Punkte ich nutze will, in meinem Fall ist das hier die Rechte Hand und die Rechte Hüfte.
            Joint handRight = skeleton.Joints[JointType.HandRight];
            Joint hipRight = skeleton.Joints[JointType.HipRight];
            //Wenn der Abstand von der Rechten Hand und der Rechten Hüfte mehr als 0 in der X-Achse ist, wird folgende Funktion ausgeführt.
            if (handRight.Position.X > hipRight.Position.X)
            {
                //Hier setze ich die Variable Speed, sie gibt den Abstand zwischen der Rechten Hand und der Rechten Hüfte.
                var speed = (handRight.Position.X - hipRight.Position.X) * 300;
                //Danach mache ich aus der Variable einen Int16 Wert, da ich einen Int16 Wert mit Cmd Messenger übertragen möchte.
                Int16 i = (Int16)Math.Round(speed);
                //Wenn der Wert zwischen 0 und 180 liegt, wird der Wert an den Arduino Due gesendet.
                if (i <= 180 && i >= 0)
                {
                    //Wenn der Wahlschalter auf "Kinect" steht, werden die Daten via Cmd Messenger übertragen.
                    if (a == 1)
                    {
                        //Gemessene Werte an den Arduino Due senden.
                        SendCommand(Commands.cmdSendRot1, i);
                    }
                }
            }
            //Shoulder 2
            //Wenn der Abstand von der Rechten Hand und der Rechten Hüfte mehr als 0 in der X-Achse ist, wird folgende Funktion ausgeführt.
            if (handRight.Position.Y > hipRight.Position.Y)
            {
                //Hier setze ich die Variable Speed2, sie gibt den Abstand zwischen der Rechten Hand und der Rechten Hüfte.
                var speed2 = (handRight.Position.Y - hipRight.Position.Y) * 300;
                //Danach mache ich aus der Variable einen Int16 Wert, da ich einen Int16 Wert mit Cmd Messenger übertragen möchte.
                Int16 i2 = (Int16)Math.Round(speed2);
                //Wenn der Wert zwischen 0 und 150 liegt, wird der Wert an den Arduino Due gesendet.
                if (i2 < 150 && i2 > 0)
                {
                    //Wenn der Wahlschalter auf "Kinect" steht, werden die Daten via Cmd Messenger übertragen.
                    if (a == 1)
                    {
                        //Gemessene Werte an den Arduino Due senden.
                        SendCommand(Commands.cmdSendRot2, (Int16)i2);
                    }
                }
            }
            //Zuerst lege ich fest, welche Punkte ich nutze will, in meinem Fall ist das hier die Linke Hand und die Linke Hüfte.
            Joint handLeft = skeleton.Joints[JointType.HandLeft];
            Joint hipLeft = skeleton.Joints[JointType.HipLeft];
            //Gripper 4
            //Wenn der Abstand von der Linken Hand und der Linken Hüfte mehr als 0 in der X-Achse ist, wird folgende Funktion ausgeführt.
            if (handLeft.Position.X < hipLeft.Position.X)
            {
                //Hier setze ich die Variable Speed4, sie gibt den Abstand zwischen der Linken Hand und der Linken Hüfte.
                var speed4 = (hipLeft.Position.X - handLeft.Position.X) * 200;
                //Danach mache ich aus der Variable einen Int16 Wert, da ich einen Int16 Wert mit Cmd Messenger übertragen möchte.
                Int16 i4 = (Int16)Math.Round(speed4);
                //Wenn der Wert zwischen 10 und 73 liegt, wird der Wert an den Arduino Due gesendet.
                if (i4 <= 73 && i4 >= 10)
                {
                    //Wenn der Wahlschalter auf "Kinect" steht, werden die Daten via Cmd Messenger übertragen.
                    if (a == 1)
                    {
                        //Gemessene Werte an den Arduino Due senden.
                        SendCommand(Commands.cmdSendRot4, (Int16)i4);
                    }
                }
            }
            //Elbow und Ver-Wrist 45
            //Wenn der Abstand von der Linken Hand und der Linken Hüfte mehr als 0 in der Y-Achse ist, wird folgende Funktion ausgeführt.
            if (handLeft.Position.Y > hipLeft.Position.Y)
            {
                //Hier setze ich die Variable Speed5, sie gibt den Abstand zwischen der Rechten Hand und der Rechten Hüfte.
                var speed5 = (handLeft.Position.Y - hipLeft.Position.Y) * 150;
                //Danach mache ich aus der Variable einen Int16 Wert, da ich einen Int16 Wert mit Cmd Messenger übertragen möchte.
                Int16 i5 = (Int16)Math.Round(speed5);
                //Wenn der Wert zwischen 0 und 180 liegt, wird der Wert an den Arduino Due gesendet.
                if (i5 < 180 && i5 > 0)
                {
                    //Wenn der Wahlschalter auf "Kinect" steht, werden die Daten via Cmd Messenger übertragen.
                    if (a == 1)
                    {
                        //Gemessene Werte an den Arduino Due senden.
                        SendCommand(Commands.cmdSendRot45, (Int16)i5);
                    }
                }
            }

            //Linkes Bein
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            //Rechtes Bein
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            //Render Joints
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

        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        //Wenn der "Connect" Button gedrückt wird, dann wird folgende Funktion ausgeführt,
        //sie dient dazu, die Verbindung zwischen dem PC und dem Arduino Due herzustellen.
        private void button_connect(object sender, RoutedEventArgs e)
        {
            //Wenn alles richtig funktioniert ohne "Errors" wird folgendes ausgeführt.
            try
            {
                //Die Serial Leitung wird Initialisiert
                serialTransport = new SerialTransport
                {
                    //Hier muss man den Port angeben, an dem der Arduino Due angeschlossen ist und seine Baud Rate für die Übertragung.
                    CurrentSerialSettings = { PortName = "COM3", BaudRate = 250000 } 
                };

                //Cmd Messenger wird gestartet udn es wird festgelegt, was für ein Prozessor angeschlossen wurde.
                //Die meisten Arduino Modell haben als BoardType ein 16Bit System.
                //der Arduino Due hingegen hat ein 32Bit System, deswegen schreibe ich hier :"BoardType.Bit32"
                cmdMessemger = new CmdMessenger(serialTransport, BoardType.Bit32);
                //Cmd Messenger Verbindung zum Arduino Due starten
                cmdMessemger.Connect();

                //Status Bar Schreibt: "Verbunden mit dem Arduino".
                statusBarText2.Text = "Verbunden mit dem Arduino";
            }
            //Wenn ein Error passiert, wird folgende Funktion ausgeführt:
            catch (Exception)
            {
                //Status Bar schreibt:" Verbindungsfehler mit dem Arduino"
                statusBarText2.Text = "Verbindungsfehler mit der Arduino";
            }
        }

        //Diese Funktion wird aufgerufen, wenn man den "Disconnect" Butten drückt, 
        //sie dint dafür, um die Verbindung zwischen dem PC und dem Arduino Due zu unterbrechen.
        private void button_disconnect(object sender, RoutedEventArgs e)
        {
            //Wenn alles richtig funktioniert ohne "Errors" wird folgendes ausgeführt.
            try
            {
                //Verbundung zum Ardiuno Due unterbrechen.
                cmdMessemger.Disconnect();

                if (cmdMessemger != null)
                    cmdMessemger.Dispose();

                if (serialTransport != null)
                    serialTransport.Dispose();

                //den Text von der Statusbar wird verändert zu :" Verbindung zum Arduino unterbrochen"
                statusBarText2.Text = "Verbindung zum Arduino unterbrochen";
            }
            //Bei einem Fehler wird folgendes ausgeführt.
            catch (Exception)
            {
                //Wenn ein Fehler auftritt wird folgendes in der Statusbar geschrieben: "Trennen der Verbindung Fehlgeschlagen!"
                statusBarText2.Text = "Trennen der Verbindung Fehlgeschagen!";
            }
        }

        //Diese Funktion nutze ich, um das Senden von Daten zum Arduino Due zu verkürzen.
        private void SendCommand(Commands cmd, Int16 data)
        {
            //Variablen festlegen, was gesendet werden soll.
            var command = new SendCommand((int)cmd);
            //Festlegen, welcher Datenwert gesendet werden soll.
            command.AddBinArgument(data);
            //Senden
            cmdMessemger.SendCommand(command);
        }

        //Hier ist die Funktion von dem Wechselschalter, sie wird ausgeführt, wenn der Taster gedrückt wurde.
        private void Wechselbutton(object sender, RoutedEventArgs e)
        {
            //Int a um 1 erhöhen
            a++;
            //Wenn a gleich an 0 ist, dann...
            if (a == 0)
            {
                //... wird der Text vom Taster auf "Internet" gesetzt und...
                Button1.Content = "Internet";
                //... CmdMessenger sendet der Wert 0 zum Arduino Due.
                SendCommand(Commands.cmdwahlschalter, 0);
            }
            //Ansonsten wird folgende Funktion ausgeführt:
            else
            {
                //Wenn a gleich an 1 ist, dann...
                if (a == 1)
                {
                    //... wird der Text vom Taster auf "Kinect" gesetzt und...
                    Button1.Content = "Kinect";
                    //... CmdMessenger sendet den Wert 1 zum Arduino Due.
                    SendCommand(Commands.cmdwahlschalter, 1);
                }
                //Ansonsten wird folgende Funktion ausgeführt:
                else
                {
                    //Wenn a gleich an 2 ist, dann...
                    if (a == 2)
                    {
                        //... wird der Int a wieder auf 0 zurück gesetzt...
                        a = 0;
                        //, der Butten Text wird auch wieder auf "Internet" gesetzt und...
                        Button1.Content = "Internet";
                        //... CmdMessenger sendet den Wert 0 zum Arduino Due.
                        SendCommand(Commands.cmdwahlschalter, 0);
                    }
                }
            }
        }

        private void Sliderbewegen(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            double test = 45;
            Gradzahl = Convert.ToInt32(test);
            sensor.ElevationAngle = Gradzahl;
        }
    }
}