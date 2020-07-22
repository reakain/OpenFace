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
            return new Tuple<int, int, int>(cam_select, cams[cam_select].Item3[0].Item1, cams[cam_select].Item3[0].Item2);
        }
    }
}
