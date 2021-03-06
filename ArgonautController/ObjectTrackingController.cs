﻿using ArgonautController.Actuators;
using ArgonautController.Sensors;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using Windows.System.Threading;

namespace ArgonautController
{
    public enum ControllerStateID
    {
        Scan,
        Track,
        Follow
    }

    public interface ControllerState
    {
        void Enter();
        void Leave();
        ControllerStateID Update();
    }

    public class ObjectBlocksEventArgs : EventArgs
    {
        public ObjectBlocksEventArgs(List<ObjectBlock> blocks)
        {
            Blocks = blocks;
        }

        public List<ObjectBlock> Blocks;
    }

    public class ObjectTrackingController : IDisposable
    {
        const long X_CENTER = 160;
        const long Y_CENTER = 100;
        const long RCS_MIN_POS = 0;
        const long RCS_MAX_POS = 1000;
        const long RCS_CENTER_POS = ((RCS_MAX_POS - RCS_MIN_POS) / 2);

        const int PIXY_X_MAX = 320;
        const int PIXY_Y_MAX = 200;

        class ServoLoop
        {
            public long Position;
            public long PrevError;
            public long ProportionalGain;
            public long DerivativeGain;

            public ServoLoop(long proportionalGain, long derivativeGain)
            {
                Position = RCS_CENTER_POS;
                ProportionalGain = proportionalGain;
                DerivativeGain = derivativeGain;
                PrevError = 0x8000000L;
            }

            public void Update(long error)
            {
                long velocity;

                if (PrevError != 0x80000000L)
                {
                    velocity = (error * ProportionalGain + (error - PrevError) * DerivativeGain) >> 10;
                    Position += velocity;

                    if (Position > RCS_MAX_POS)
                    {
                        Position = RCS_MAX_POS;
                    }
                    else if (Position < RCS_MIN_POS)
                    {
                        Position = RCS_MIN_POS;
                    }
                }

                PrevError = error;
            }
        }

        public ObjectTrackingController()
        {
            ZumoMotorShieldConfig config;
            config = new ZumoMotorShieldConfig();
            config.LeftMotorDirPin = 5;
            config.RightMotorDirPin = 4;
            config.LeftPwmChannel = 1;
            config.RightPwmChannel = 0;
            config.BuzzerPwmChannel = 2;
            config.PwmDriverSlaveAddress = 0x40;

            watch = new Stopwatch();

            motorDriver = new ZumoMotorShield(config);
            pixyCam = new PixyCam();
            panLoop = new ServoLoop(200, 200);
            tiltLoop = new ServoLoop(150, 200);

            stateMachine = new Dictionary<ControllerStateID, ControllerState>();
            stateMachine.Add(ControllerStateID.Scan, new ScanState(this));
            stateMachine.Add(ControllerStateID.Track, new TrackState(this));
            stateMachine.Add(ControllerStateID.Follow, new FollowState(this));
            currentState = ControllerStateID.Scan;
        }

        public async Task Init()
        {
            Debug.WriteLine("Initializing ObjectTrackingController");

            await motorDriver.Init();
            await pixyCam.Init();

            pixyCam.SetServos(0, 0);
        }

        public Task RunAsync(float fps)
        {
            // Start reading frames from camera
            return ThreadPool.RunAsync((s) =>
            {
                Debug.WriteLine("Entering free running loop");

                long frameTimeMs = (int)Math.Round((1000.0f / fps));

                Debug.WriteLine("Running at target {0} FPS ~ frame every {1}ms", fps, frameTimeMs);

                long lastFrameMilliseconds = 0;

                var fpsTrackingTimer = ThreadPoolTimer.CreatePeriodicTimer((t) =>
                {
                    float effectiveFps = frameCount / (float)watch.Elapsed.TotalSeconds;
                    Debug.WriteLine(
                        string.Format(
                            "{0}s: FPS={1}, Frame-time={2}ms",
                            watch.Elapsed.TotalSeconds,
                            effectiveFps,
                            1000.0f / effectiveFps));
                }, TimeSpan.FromSeconds(1));

                isRunning = true;
                watch.Start();

                while (!shutdown)
                {
                    if (watch.ElapsedMilliseconds - lastFrameMilliseconds > frameTimeMs)
                    {
                        loop();
                        lastFrameMilliseconds = watch.ElapsedMilliseconds;
                        ++frameCount;
                    }
                }
                watch.Stop();
                fpsTrackingTimer.Cancel();

                cleanUp();

                Debug.WriteLine("Exiting free running loop");

            }).AsTask();
        }

        public void Shutdown()
        {
            Debug.WriteLine("Shuttingdown...");
            shutdown = true;
        }

        public bool IsRunning
        {
            get { return isRunning; }
        }

        class ScanState : ControllerState
        {
            public ScanState(ObjectTrackingController c)
            {
                controller = c;
            }

            public void Enter()
            {
            }

            public void Leave()
            {
            }

            public ControllerStateID Update()
            {
                controller.scannedBlocks = controller.pixyCam.GetBlocks(10);

                if (controller.scannedBlocks != null && controller.scannedBlocks.Count > 0)
                {
                    controller.OnBlocksReceived(new ObjectBlocksEventArgs(controller.scannedBlocks));
                    return ControllerStateID.Track;
                }
                else
                    return ControllerStateID.Scan;
            }

            ObjectTrackingController controller;
        }

        class TrackState : ControllerState
        {
            public TrackState(ObjectTrackingController c)
            {
                controller = c;
            }

            public void Enter()
            {
                // Set LED to blue
                // controller.pixyCam.SetLED(0, 0, 255);
            }

            public void Leave()
            {
            }

            public ControllerStateID Update()
            {
                controller.trackedBlock = controller.trackBlock(controller.scannedBlocks);

               if (controller.trackedBlock != null)
                    return ControllerStateID.Follow;
               else
                    return ControllerStateID.Scan;
            }

            ObjectTrackingController controller;
        }

        class FollowState : ControllerState
        {
            public FollowState(ObjectTrackingController c)
            {
                controller = c;
            }

            public void Enter()
            {
                controller.followBlock(controller.trackedBlock);
            }

            public void Leave()
            {
            }

            public ControllerStateID Update()
            {
                return ControllerStateID.Scan;
            }

            ObjectTrackingController controller;
        }

        private void loop()
        {
            var nextState = stateMachine[currentState].Update();
            if (nextState != currentState)
            {
                var oldState = currentState;
                stateMachine[oldState].Leave();

                currentState = nextState;
                stateMachine[currentState].Enter();

                Debug.WriteLine("@{0} Transition: {1}->{2}", frameCount, oldState, nextState);
            }

            ++frameCount;
        }

        private void cleanUp()
        {
            motorDriver.LeftMotorStop();
            motorDriver.RightMotorStop();

            isRunning = false;
        }

        // Track blocks via the Pixy pan/tilt mechanism
        private ObjectBlock trackBlock(List<ObjectBlock> blocks)
        {
            Debug.Assert(blocks != null && blocks.Count > 0);

            trackedBlock = null;
            long maxSize = 0;

            foreach (ObjectBlock block in blocks)
            {
                if (oldBlock == null || (block.Signature == oldBlock.Signature))
                {
                    long newSize = block.Height * block.Width;
                    if (newSize > maxSize)
                    {
                        trackedBlock = block;
                        maxSize = newSize;
                    }
                }
            }

            if (trackedBlock != null)
            {
                long panError = X_CENTER - trackedBlock.X;
                long tiltError = trackedBlock.Y - Y_CENTER;
                panLoop.Update(panError);
                tiltLoop.Update(tiltError);
                pixyCam.SetServos(panLoop.Position, tiltLoop.Position);

                oldBlock = trackedBlock;
            }

            return trackedBlock;
        }

        // Follow blocks via the Zumo robot drive
        // This code makes the robot base turn and move to follow the pan/tilt tracking of the head
        private void followBlock(ObjectBlock trackedBlock)
        {
            Debug.WriteLine(
                string.Format(
                    "Follow: {0}",
                    trackedBlock.ToString()));

            long followError = RCS_CENTER_POS - panLoop.Position;

            // Size is the area of the object
            // We keep a running average of the last 8
            size += trackedBlock.Width * trackedBlock.Height;
            size -= size >> 3;

            const int MinSpeed = -400;
            const int MaxSpeed = 400;

            // Forward speed decreases as we approach the object (size is larger)
            int forwardSpeed = Constrain(400 - ((int)size / 256), MinSpeed / 2, MaxSpeed);

            // Steering differential is proportional to the error times the forward speed
            long differential = (followError + (followError * forwardSpeed)) >> 8;

            // Adjust the left and right speeds by the steering differential
            int leftSpeed = Constrain((int)(forwardSpeed + differential), MinSpeed, MaxSpeed);
            int rightSpeed = Constrain((int)(forwardSpeed - differential), MinSpeed, MaxSpeed);

            float leftPower, rightPower;
            ZumoMotorDirection leftDir, rightDir;

            if (leftSpeed >= 0)
                leftDir = ZumoMotorDirection.Forward;
            else
                leftDir = ZumoMotorDirection.Backward;

            if (rightSpeed >= 0)
                rightDir = ZumoMotorDirection.Forward;
            else
                rightDir = ZumoMotorDirection.Backward;

            const float PowerScaleDown = 0.75f;
            leftPower = (Math.Abs(leftSpeed) / (float)MaxSpeed) * PowerScaleDown;
            rightPower = (Math.Abs(rightSpeed) / (float)MaxSpeed) * PowerScaleDown;

            Debug.WriteLine(
                string.Format("followError={0}, size={1}, differential={2}, leftPower={3}, rightPower={4}",
                    followError,
                    size,
                    differential,
                    leftPower,
                    rightPower));

            motorDriver.SetLeftMotorPower(leftDir, leftPower);
            motorDriver.SetRightMotorPower(rightDir, rightPower);
        }

        // Constrains number between lower and upper bounds (inclusive)
        public int Constrain(int num, int lower, int upper)
        {
            if (num <= lower)
            {
                return lower;
            }
            else if (num >= upper)
            {
                return upper;
            }

            return num;
        }

        public void Dispose()
        {
            if (pixyCam != null)
            {
                pixyCam.Close();
                pixyCam = null;
            }

            if (motorDriver != null)
            {
                motorDriver.Dispose();
                motorDriver = null;
            }
        }

        public event EventHandler<ObjectBlocksEventArgs> BlocksReceived;

        protected virtual void OnBlocksReceived(ObjectBlocksEventArgs e)
        {
            EventHandler<ObjectBlocksEventArgs> handler = BlocksReceived;
            if (handler != null)
            {
                handler(this, e);
            }
        }

        ZumoMotorShield motorDriver;
        PixyCam pixyCam;
        ServoLoop panLoop;
        ServoLoop tiltLoop;
        bool shutdown = false;
        bool isRunning = false;
        ObjectBlock oldBlock;
        long size = 400;
        long frameCount = 0;
        Stopwatch watch;
        ControllerStateID currentState;
        Dictionary<ControllerStateID, ControllerState> stateMachine;
        List<ObjectBlock> scannedBlocks;
        ObjectBlock trackedBlock;
    }
}
