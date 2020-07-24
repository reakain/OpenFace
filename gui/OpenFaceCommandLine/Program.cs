using System;

namespace OpenFaceCommandLine
{
    class Program
    {
        static void Main(string[] args)
        {
            CameraSelection cams = new CameraSelection();
            FaceAnalyser faceAnalyser = new FaceAnalyser();
            if (!cams.LoadCameras())
            {
                Console.WriteLine("No cameras connected.");
                return;
            }

            int cam_id = 0;
            bool do_analysis = false;

            if (args.Length < 1)
            {
                do_analysis = true;
            }
            else
            {
                for(int i = 0; i < args.Length; i++)
                {
                    switch(args[i])
                    {
                        case "-c":
                            if(int.TryParse(args[i+1],out int _num))
                            {
                                cam_id = _num;
                                do_analysis = true;
                            }
                            else
                            {
                                Console.WriteLine("No camera number enteres.");
                                return;
                            }
                            break;
                        case "list_cams":
                            cams.ListCameras();
                            break;
                        default:
                            break;
                    }
                }
            }
            if (do_analysis)
            {
                var cam = cams.SetCamera(cam_id);
                faceAnalyser.StartProcessing(cam);
            }
        }
    }
}
