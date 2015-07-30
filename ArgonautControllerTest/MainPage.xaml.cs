using Windows.UI.Xaml.Controls;
using System.Threading.Tasks;
using System.Diagnostics;
using ArgonautController.Actuators;
using ArgonautController;
using System.Threading;


// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace ArgonautControllerTest
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        public MainPage()
        {
            this.InitializeComponent();
            Loaded += MainPage_Loaded;

            Config = new ZumoMotorShieldConfig();
            Config.LeftMotorDirPin = 5;
            Config.RightMotorDirPin = 4;
            Config.LeftPwmChannel = 1;
            Config.RightPwmChannel = 0;
            Config.PwmDriverSlaveAddress = 0x40;
        }

        private async void MainPage_Loaded(object sender, object args)
        {
            //await MotorControlSmokeTest();
            await ObjectTrackingFreeFPSSmokeTest(60);
        }

        private async Task MotorControlSmokeTest()
        {
            using (ZumoMotorShield motorDriver = new ZumoMotorShield(Config))
            {
                await motorDriver.Init();

                motorDriver.SetLeftMotorPower(ZumoMotorDirection.Forward, 0.3f);
                await Task.Delay(2000);
                motorDriver.LeftMotorStop();

                await Task.Delay(500);

                motorDriver.SetLeftMotorPower(ZumoMotorDirection.Backward, 0.3f);
                await Task.Delay(2000);
                motorDriver.LeftMotorStop();

                await Task.Delay(500);

                motorDriver.SetLeftMotorPower(ZumoMotorDirection.Forward, 0.3f);
                await Task.Delay(2000);
                motorDriver.LeftMotorStop();

                await Task.Delay(500);

                motorDriver.SetLeftMotorPower(ZumoMotorDirection.Backward, 0.3f);
                await Task.Delay(2000);
                motorDriver.LeftMotorStop();

                await Task.Delay(500);

                bool flipDir = false;
                ZumoMotorDirection dirA = ZumoMotorDirection.Forward;
                ZumoMotorDirection dirB = ZumoMotorDirection.Backward;

                Debug.WriteLine("Testing Motor Power Control");

                for (int i = 20; i <= 100; i += 20)
                {
                    Debug.WriteLine("Motor Control Ticking");

                    if (!flipDir)
                    {
                        motorDriver.SetLeftMotorPower(dirA, (float)i / 100.0f);
                        motorDriver.SetRightMotorPower(dirB, (float)i / 100.0f);
                    }
                    else
                    {
                        motorDriver.SetLeftMotorPower(dirB, (float)i / 100.0f);
                        motorDriver.SetRightMotorPower(dirA, (float)i / 100.0f);
                    }

                    flipDir = !flipDir;

                    await Task.Delay(1500);
                }

                motorDriver.LeftMotorStop();
                motorDriver.RightMotorStop();
            }
        }

        private async Task ObjectTrackingFreeFPSSmokeTest(int seconds)
        {
            using (ObjectTrackingController controller = new ObjectTrackingController())
            {
                await controller.Init();
                var task = controller.RunAsync(50.0f);

                Debug.WriteLine(string.Format("Letting object tracking run for {0} seconds", seconds));
                await Task.Delay(seconds * 1000);

                controller.Shutdown();
                bool graceful = task.Wait(3000);
                Debug.WriteLineIf(graceful, "Shutdown successfully");
                Debug.WriteLineIf(!graceful, "Shutdown timedout");
            }
        }

        //private async Task ObjectTrackingLockedFPSSmokeTest(int seconds)
        //{
        //    using (ObjectTrackingController controller = new ObjectTrackingController())
        //    {
        //        await controller.Init();
        //        controller.RunPeriodicAsync(50.0f);

        //        Debug.WriteLine(string.Format("Letting object tracking run for {0} seconds", seconds));
        //        await Task.Delay(seconds * 1000);

        //        controller.Shutdown();
        //        await Task.Delay(3000);

        //        Debug.WriteLineIf(!controller.IsRunning, "Shutdown successfully");
        //        Debug.WriteLineIf(controller.IsRunning, "Shutdown timedout");
        //    }
        //}

        ZumoMotorShieldConfig Config;
    }
}
