using System;
using System.Collections.Generic;
using System.Text;

namespace OpenFaceCommandLine
{
    class CameraSelection
    {
        // id, width, height
        public bool no_cameras_found = false;
        public List<Tuple<int, String, List<Tuple<int, int>>, OpenCVWrappers.RawImage>> cams;


        public bool LoadCameras()
        {
            // Finding the cameras here
            if (cams == null)
            {
                String root = AppDomain.CurrentDomain.BaseDirectory;
                //cams = CameraInterop.Capture.GetCameras(root);
                cams = UtilitiesOF.SequenceReader.GetCameras(root);
            }
            return cams.Count > 0;
        }

        public void ListCameras()
        {
            if(cams == null)
            {
                LoadCameras();
            }

            if (cams.Count > 0)
            {
                int i = 0;
                foreach (var s in cams)
                {
                    Console.WriteLine(string.Format("{0}) {1}", i, s.Item2));
                }
            }
            else
            {
                Console.WriteLine("No cameras detected, please connect a webcam.");
            }
        }
        
        public Tuple<int, int, int> SetCamera(int cam_select)
        {
            int res = 0;
            var cam = cams[cam_select];
            for (res = 0; res < cam.Item3.Count; ++res)
            {
                if (cam.Item3[res].Item1 >= 640 && cam.Item3[res].Item2 >= 480)
                {
                    break;
                }
            }
            Console.WriteLine(string.Format("Camera is {0} with resolution {1}x{2}", cams[cam_select].Item2, cams[cam_select].Item3[res].Item1, cams[cam_select].Item3[res].Item2));
            return new Tuple<int, int, int>(cam_select, cams[cam_select].Item3[res].Item1, cams[cam_select].Item3[res].Item2);
        }
    }
}
