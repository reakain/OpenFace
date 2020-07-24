using System;
using System.Collections.Generic;
using System.Text;

// Internal libraries
using OpenCVWrappers;
using CppInterop.LandmarkDetector;
using FaceAnalyser_Interop;
using GazeAnalyser_Interop;
using FaceDetectorInterop;
using UtilitiesOF;
using ZeroMQ;
using System.Threading;
using System.Windows;

namespace OpenFaceCommandLine
{
    public class FaceAnalyser
    {
        Thread processing_thread;
        // Managing the running of the analysis system
        private volatile bool thread_running;
        private volatile bool thread_paused = false;
        // Allows for going forward in time step by step
        // Useful for visualising things
        private volatile int skip_frames = 0;

        FpsTracker processing_fps = new FpsTracker();

        // For tracking
        FaceDetector face_detector;
        FaceModelParameters face_model_params;
        CLNF landmark_detector;

        // For face analysis
        FaceAnalyserManaged face_analyser;
        GazeAnalyserManaged gaze_analyser;

        public bool RecordAligned { get; set; } = false; // Aligned face images
        public bool RecordHOG { get; set; } = false; // HOG features extracted from face images
        public bool Record2DLandmarks { get; set; } = true; // 2D locations of facial landmarks (in pixels)
        public bool Record3DLandmarks { get; set; } = true; // 3D locations of facial landmarks (in pixels)
        public bool RecordModelParameters { get; set; } = true; // Facial shape parameters (rigid and non-rigid geometry)
        public bool RecordPose { get; set; } = true; // Head pose (position and orientation)
        public bool RecordAUs { get; set; } = true; // Facial action units
        public bool RecordGaze { get; set; } = true; // Eye gaze
        public bool RecordTracked { get; set; } = true; // Recording tracked videos or images

        // Selecting which face detector will be used
        public bool DetectorHaar { get; set; } = false;
        public bool DetectorHOG { get; set; } = false;
        public bool DetectorCNN { get; set; } = true;

        // Selecting which landmark detector will be used
        public bool LandmarkDetectorCLM { get; set; } = false;
        public bool LandmarkDetectorCLNF { get; set; } = false;
        public bool LandmarkDetectorCECLM { get; set; } = true;

        // For AU prediction, if videos are long dynamic models should be used
        public bool DynamicAUModels { get; set; } = true;

        // Where the recording is done (by default in a record directory, from where the application executed)
        String record_root = "./processed";

        int image_output_size = 112;
        public bool MaskAligned { get; set; } = true; // Should the aligned images be masked

        // For broadcasting the results
        ZeroMQ.ZContext zero_mq_context;
        ZeroMQ.ZSocket zero_mq_socket;

        // Camera calibration parameters
        public float fx = -1, fy = -1, cx = -1, cy = -1;

        public FaceAnalyser()
        {
            String root = AppDomain.CurrentDomain.BaseDirectory;

            face_model_params = new FaceModelParameters(root, LandmarkDetectorCECLM, LandmarkDetectorCLNF, LandmarkDetectorCLM);
            // Initialize the face detector
            face_detector = new FaceDetector(face_model_params.GetHaarLocation(), face_model_params.GetMTCNNLocation());
            Console.WriteLine("Checking MTCNN Loaded...");
            // If MTCNN model not available, use HOG
            if (!face_detector.IsMTCNNLoaded())
            {
                DetectorCNN = false;
                DetectorHOG = true;
            }
            Console.WriteLine("Setting Face Model Parameters");
            face_model_params.SetFaceDetector(DetectorHaar, DetectorHOG, DetectorCNN);
            Console.WriteLine("Setting the CLNF landmark detector...");
            landmark_detector = new CLNF(face_model_params);
            Console.WriteLine("Setting the gaze analyser...");
            gaze_analyser = new GazeAnalyserManaged();

            // Create the ZeroMQ context for broadcasting the results
            zero_mq_context = ZContext.Create();
            zero_mq_socket = new ZSocket(zero_mq_context, ZSocketType.PUB);

            // Bind on localhost port 5000
            zero_mq_socket.Bind("tcp://127.0.0.1:5001");  
        }

        public void StartProcessing(Tuple<int, int, int> webcam)
        {
            SequenceReader reader = new SequenceReader(webcam.Item1, webcam.Item2, webcam.Item3, fx, fy, cx, cy);
            processing_thread = new Thread(() => ProcessSequence(reader));
            processing_thread.Name = "Webcam processing";
            processing_thread.Start();
        }

        // ----------------------------------------------------------
        // Actual work gets done here
        // The main function call for processing sequences
        private void ProcessSequence(SequenceReader reader)
        {
            Thread.CurrentThread.Priority = ThreadPriority.Highest;

            SetupFeatureExtractionMode();

            thread_running = true;

            // Reload the face landmark detector if needed
            ReloadLandmarkDetector();

            if (!landmark_detector.isLoaded())
            {
                DetectorNotFoundWarning();
                thread_running = false;
                return;
            }

            // Set the face detector
            face_model_params.SetFaceDetector(DetectorHaar, DetectorHOG, DetectorCNN);
            face_model_params.optimiseForVideo();

            // Setup the visualization
            Visualizer visualizer_of = new Visualizer(true || RecordTracked, true, true, false);

            // Initialize the face analyser
            face_analyser = new FaceAnalyserManaged(AppDomain.CurrentDomain.BaseDirectory, DynamicAUModels, image_output_size, MaskAligned);

            // Reset the tracker
            landmark_detector.Reset();

            // Loading an image file
            var frame = reader.GetNextImage();
            var gray_frame = reader.GetCurrentFrameGray();

            // Setup recording
            RecorderOpenFaceParameters rec_params = new RecorderOpenFaceParameters(true, reader.IsWebcam(),
                Record2DLandmarks, Record3DLandmarks, RecordModelParameters, RecordPose, RecordAUs,
                RecordGaze, RecordHOG, RecordTracked, RecordAligned, false,
                reader.GetFx(), reader.GetFy(), reader.GetCx(), reader.GetCy(), reader.GetFPS());

            RecorderOpenFace recorder = new RecorderOpenFace(reader.GetName(), rec_params, record_root);

            // For FPS tracking
            DateTime? startTime = DateTime.UtcNow;
            var lastFrameTime = DateTime.UtcNow;

            // Empty image would indicate that the stream is over
            while (!gray_frame.IsEmpty)
            {

                if (!thread_running)
                {
                    break;
                }

                double progress = reader.GetProgress();

                bool detection_succeeding = landmark_detector.DetectLandmarksInVideo(frame, face_model_params, gray_frame);

                // The face analysis step (for AUs and eye gaze)
                face_analyser.AddNextFrame(frame, landmark_detector.CalculateAllLandmarks(), detection_succeeding, false);

                gaze_analyser.AddNextFrame(landmark_detector, detection_succeeding, reader.GetFx(), reader.GetFy(), reader.GetCx(), reader.GetCy());

                // Only the final face will contain the details
                VisualizeFeatures(frame, visualizer_of, landmark_detector.CalculateAllLandmarks(), landmark_detector.GetVisibilities(), detection_succeeding, true, false, reader.GetFx(), reader.GetFy(), reader.GetCx(), reader.GetCy(), progress);

                // Record an observation
                RecordObservation(recorder, visualizer_of.GetVisImage(), 0, detection_succeeding, reader.GetFx(), reader.GetFy(), reader.GetCx(), reader.GetCy(), reader.GetTimestamp(), reader.GetFrameNumber());

                if (RecordTracked)
                {
                    recorder.WriteObservationTracked();
                }

                while (thread_running & thread_paused && skip_frames == 0)
                {
                    Thread.Sleep(10);
                }

                if (skip_frames > 0)
                    skip_frames--;

                frame = reader.GetNextImage();
                gray_frame = reader.GetCurrentFrameGray();

                lastFrameTime = DateTime.UtcNow;
                processing_fps.AddFrame();
            }

            // Finalize the recording and flush to disk
            recorder.Close();

            // Post-process the AU recordings
            if (RecordAUs)
            {
                face_analyser.PostProcessOutputFile(recorder.GetCSVFile());
            }

            // Close the open video/webcam
            reader.Close();

        }

        private void VisualizeFeatures(RawImage frame, Visualizer visualizer, List<Tuple<float, float>> landmarks, List<bool> visibilities, bool detection_succeeding,
            bool new_image, bool multi_face, float fx, float fy, float cx, float cy, double progress)
        {

            List<Tuple<Point, Point>> lines = null;
            List<Tuple<float, float>> eye_landmarks = null;
            List<Tuple<Point, Point>> gaze_lines = null;
            Tuple<float, float> gaze_angle = new Tuple<float, float>(0, 0);

            List<float> pose = new List<float>();
            landmark_detector.GetPose(pose, fx, fy, cx, cy);
            List<float> non_rigid_params = landmark_detector.GetNonRigidParams();

            double confidence = landmark_detector.GetConfidence();

            if (confidence < 0)
                confidence = 0;
            else if (confidence > 1)
                confidence = 1;

            double scale = landmark_detector.GetRigidParams()[0];

            // Helps with recording and showing the visualizations
            if (new_image)
            {
                visualizer.SetImage(frame, fx, fy, cx, cy);
            }
            visualizer.SetObservationHOG(face_analyser.GetLatestHOGFeature(), face_analyser.GetHOGRows(), face_analyser.GetHOGCols());
            visualizer.SetObservationLandmarks(landmarks, confidence, visibilities);
            visualizer.SetObservationPose(pose, confidence);
            visualizer.SetObservationGaze(gaze_analyser.GetGazeCamera().Item1, gaze_analyser.GetGazeCamera().Item2, landmark_detector.CalculateAllEyeLandmarks(), landmark_detector.CalculateAllEyeLandmarks3D(fx, fy, cx, cy), confidence);

            eye_landmarks = landmark_detector.CalculateVisibleEyeLandmarks();
            lines = landmark_detector.CalculateBox(fx, fy, cx, cy);

            gaze_lines = gaze_analyser.CalculateGazeLines(fx, fy, cx, cy);
            gaze_angle = gaze_analyser.GetGazeAngle();


        }


        // If the landmark detector model changed need to reload it
        private void ReloadLandmarkDetector()
        {
            bool reload = false;
            if (face_model_params.IsCECLM() && !LandmarkDetectorCECLM)
            {
                reload = true;
            }
            else if (face_model_params.IsCLNF() && !LandmarkDetectorCLNF)
            {
                reload = true;
            }
            else if (face_model_params.IsCLM() && !LandmarkDetectorCLM)
            {
                reload = true;
            }

            if (reload)
            {
                String root = AppDomain.CurrentDomain.BaseDirectory;

                face_model_params = new FaceModelParameters(root, LandmarkDetectorCECLM, LandmarkDetectorCLNF, LandmarkDetectorCLM);
                landmark_detector = new CLNF(face_model_params);
            }
        }

        private void DetectorNotFoundWarning()
        {
           Console.WriteLine(@"Could not open the landmark detector model file. For instructions of how to download them, see https://github.com/TadasBaltrusaitis/OpenFace/wiki/Model-download");
        }

        private void RecordObservation(RecorderOpenFace recorder, RawImage vis_image, int face_id, bool success, float fx, float fy, float cx, float cy, double timestamp, int frame_number)
        {

            recorder.SetObservationTimestamp(timestamp);

            double confidence = landmark_detector.GetConfidence();

            List<float> pose = new List<float>();
            landmark_detector.GetPose(pose, fx, fy, cx, cy);

            // Publish the pose information for other applications
            String str_head_pose = String.Format("{0}:{1:F2}, {2:F2}, {3:F2}, {4:F2}, {5:F2}, {6:F2}", "HeadPose", pose[0], pose[1], pose[2],
                pose[3] * 180 / Math.PI, pose[4] * 180 / Math.PI, pose[5] * 180 / Math.PI);
            zero_mq_socket.Send(new ZFrame(str_head_pose, Encoding.UTF8));

            recorder.SetObservationPose(pose);

            List<Tuple<float, float>> landmarks_2D = landmark_detector.CalculateAllLandmarks();
            List<Tuple<float, float, float>> landmarks_3D = landmark_detector.Calculate3DLandmarks(fx, fy, cx, cy);
            List<float> global_params = landmark_detector.GetRigidParams();
            List<float> local_params = landmark_detector.GetNonRigidParams();

            recorder.SetObservationLandmarks(landmarks_2D, landmarks_3D, global_params, local_params, confidence, success);

            var gaze = gaze_analyser.GetGazeCamera();
            var gaze_angle = gaze_analyser.GetGazeAngle();

            // Publish the gaze angle information for other applications
            String str_gaze = String.Format("{0}:{1:F2}, {2:F2}", "GazeAngle", gaze_angle.Item1 * (180.0 / Math.PI), gaze_angle.Item2 * (180.0 / Math.PI));
            zero_mq_socket.Send(new ZFrame(str_gaze, Encoding.UTF8));

            var landmarks_2d_eyes = landmark_detector.CalculateAllEyeLandmarks();
            var landmarks_3d_eyes = landmark_detector.CalculateAllEyeLandmarks3D(fx, fy, cx, cy);
            recorder.SetObservationGaze(gaze.Item1, gaze.Item2, gaze_angle, landmarks_2d_eyes, landmarks_3d_eyes);


            var au_regs = face_analyser.GetCurrentAUsReg();
            var au_classes = face_analyser.GetCurrentAUsClass();

            // Publish the au_reg information for other applications
            foreach (var key in au_regs.Keys)
            {
                if (au_regs.TryGetValue(key, out var key_val))
                {
                    var au_reg_string = string.Format("Regression_{0}:{1:F2}", key, key_val);
                    //Debug.WriteLine(au_reg_string);
                    zero_mq_socket.Send(new ZFrame(au_reg_string, Encoding.UTF8));
                }
            }
            // Publish the au_class information for other applications
            foreach (var key in au_classes.Keys)
            {
                if (au_classes.TryGetValue(key, out var key_val))
                {
                    var au_class_string = string.Format("Classification_{0}:{1}", key, key_val);
                    zero_mq_socket.Send(new ZFrame(au_class_string, Encoding.UTF8));
                }
            }

            recorder.SetObservationActionUnits(au_regs, au_classes);

            recorder.SetObservationFaceID(face_id);
            recorder.SetObservationFrameNumber(frame_number);

            recorder.SetObservationFaceAlign(face_analyser.GetLatestAlignedFace());

            var hog_feature = face_analyser.GetLatestHOGFeature();
            recorder.SetObservationHOG(success, hog_feature, face_analyser.GetHOGRows(), face_analyser.GetHOGCols(), face_analyser.GetHOGChannels());

            recorder.SetObservationVisualization(vis_image);

            recorder.WriteObservation();


        }

        private void StopTracking()
        {
            // First complete the running of the thread
            if (processing_thread != null)
            {
                // Tell the other thread to finish
                thread_running = false;
                processing_thread.Join();
            }
        }


        // ----------------------------------------------------------
        // Mode handling (image, video)
        // ----------------------------------------------------------

        // Disable GUI components that should not be active during processing
        private void SetupFeatureExtractionMode()
        {

        }


        // Stopping the tracking
        public void StopProcessing()
        {
            if (processing_thread != null)
            {
                // Stop capture and tracking
                thread_paused = false;
                thread_running = false;
                // Let the processing thread finish
                processing_thread.Join();
            }
        }

        public void PauseProcessing()
        {
            if (processing_thread != null)
            {
                // Stop capture and tracking                
                thread_paused = !thread_paused;
            }
        }

        public void setCameraParameters(int Fx, int Fy, int Cx, int Cy)
        {
            fx = Fx;
            fy = Fy;
            cx = Cx;
            cy = Cy;
        }

        public void setRecordingOutputDir(string directory)
        {
            record_root = directory;
        }
    }
}
