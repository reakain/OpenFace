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

namespace OpenFaceCommandLine
{
    public class FaceAnalyser
    {
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

            // If MTCNN model not available, use HOG
            if (!face_detector.IsMTCNNLoaded())
            {
                FaceDetCNN.IsEnabled = false;
                DetectorCNN = false;
                DetectorHOG = true;
            }
            face_model_params.SetFaceDetector(DetectorHaar, DetectorHOG, DetectorCNN);

            landmark_detector = new CLNF(face_model_params);

            gaze_analyser = new GazeAnalyserManaged();

            // Create the ZeroMQ context for broadcasting the results
            zero_mq_context = ZContext.Create();
            zero_mq_socket = new ZSocket(zero_mq_context, ZSocketType.PUB);

            // Bind on localhost port 5000
            zero_mq_socket.Bind("tcp://127.0.0.1:5000");
        }

        // ----------------------------------------------------------
        // Actual work gets done here

        // Wrapper for processing multiple sequences
        private void ProcessSequences(List<String> filenames)
        {
            for (int i = 0; i < filenames.Count; ++i)
            {
                SequenceReader reader = new SequenceReader(filenames[i], false, fx, fy, cx, cy);
                ProcessSequence(reader);

                // Before continuing to next video make sure the user did not stop the processing
                if (!thread_running)
                {
                    break;
                }
            }

        }

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
                EndMode();
                thread_running = false;
                return;
            }

            // Set the face detector
            face_model_params.SetFaceDetector(DetectorHaar, DetectorHOG, DetectorCNN);
            face_model_params.optimiseForVideo();

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
            DateTime? startTime = CurrentTime;
            var lastFrameTime = CurrentTime;

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

                lastFrameTime = CurrentTime;
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

            EndMode();

        }

        private void ProcessIndividualImages(ImageReader reader)
        {
            // Make sure the GUI is setup appropriately
            SetupFeatureExtractionMode();

            // Indicate we will start running the thread
            thread_running = true;

            // Reload the face landmark detector if needed
            ReloadLandmarkDetector();

            if (!landmark_detector.isLoaded())
            {
                DetectorNotFoundWarning();
                EndMode();
                thread_running = false;
                return;
            }

            // Setup the parameters optimized for working on individual images rather than sequences
            face_model_params.optimiseForImages();

            // Initialize the face detector if it has not been initialized yet
            if (face_detector == null)
            {
                face_detector = new FaceDetector(face_model_params.GetHaarLocation(), face_model_params.GetMTCNNLocation());
            }

            // Initialize the face analyser
            face_analyser = new FaceAnalyserManaged(AppDomain.CurrentDomain.BaseDirectory, false, image_output_size, MaskAligned);

            // Loading an image file
            var frame = reader.GetNextImage();
            var gray_frame = reader.GetCurrentFrameGray();

            // For FPS tracking
            DateTime? startTime = CurrentTime;
            var lastFrameTime = CurrentTime;

            // This will be false when the image is not available
            while (reader.isOpened())
            {
                if (!thread_running)
                {
                    break;
                }

                // Setup recording
                RecorderOpenFaceParameters rec_params = new RecorderOpenFaceParameters(false, false,
                    Record2DLandmarks, Record3DLandmarks, RecordModelParameters, RecordPose, RecordAUs,
                    RecordGaze, RecordHOG, RecordTracked, RecordAligned, true,
                    reader.GetFx(), reader.GetFy(), reader.GetCx(), reader.GetCy(), 0);

                RecorderOpenFace recorder = new RecorderOpenFace(reader.GetName(), rec_params, record_root);

                visualizer_of.SetImage(frame, reader.GetFx(), reader.GetFy(), reader.GetCx(), reader.GetCy());

                // Detect faces here and return bounding boxes
                List<Rect> face_detections = new List<Rect>();
                List<float> confidences = new List<float>();
                if (DetectorHOG)
                {
                    face_detector.DetectFacesHOG(face_detections, gray_frame, confidences);
                }
                else if (DetectorCNN)
                {
                    face_detector.DetectFacesMTCNN(face_detections, frame, confidences);
                }
                else if (DetectorHaar)
                {
                    face_detector.DetectFacesHaar(face_detections, gray_frame, confidences);
                }

                // For visualization
                double progress = reader.GetProgress();

                for (int i = 0; i < face_detections.Count; ++i)
                {
                    bool detection_succeeding = landmark_detector.DetectFaceLandmarksInImage(frame, face_detections[i], face_model_params, gray_frame);

                    var landmarks = landmark_detector.CalculateAllLandmarks();

                    // Predict action units
                    var au_preds = face_analyser.PredictStaticAUsAndComputeFeatures(frame, landmarks);

                    // Predic eye gaze
                    gaze_analyser.AddNextFrame(landmark_detector, detection_succeeding, reader.GetFx(), reader.GetFy(), reader.GetCx(), reader.GetCy());

                    // Record an observation
                    RecordObservation(recorder, visualizer_of.GetVisImage(), i, detection_succeeding, reader.GetFx(), reader.GetFy(), reader.GetCx(), reader.GetCy(), 0, 0);

                }

                recorder.SetObservationVisualization(visualizer_of.GetVisImage());

                frame = reader.GetNextImage();
                gray_frame = reader.GetCurrentFrameGray();

                // Write out the tracked image
                if (RecordTracked)
                {
                    recorder.WriteObservationTracked();
                }

                // Do not cary state accross images
                landmark_detector.Reset();
                face_analyser.Reset();
                recorder.Close();

                lastFrameTime = CurrentTime;
                processing_fps.AddFrame();

                // TODO how to report errors from the reader here? exceptions? logging? Problem for future versions?
            }

            EndMode();

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
            string messageBoxText = "Could not open the landmark detector model file. For instructions of how to download them, see https://github.com/TadasBaltrusaitis/OpenFace/wiki/Model-download";
            string caption = "Model file not found or corrupt";
            MessageBoxButton button = MessageBoxButton.OK;
            MessageBoxImage icon = MessageBoxImage.Warning;

            // Display message box
            System.Windows.MessageBox.Show(messageBoxText, caption, button, icon);

        }

        private void RecordObservation(RecorderOpenFace recorder, RawImage vis_image, int face_id, bool success, float fx, float fy, float cx, float cy, double timestamp, int frame_number)
        {

            recorder.SetObservationTimestamp(timestamp);

            double confidence = landmark_detector.GetConfidence();

            List<float> pose = new List<float>();
            landmark_detector.GetPose(pose, fx, fy, cx, cy);
            recorder.SetObservationPose(pose);

            List<Tuple<float, float>> landmarks_2D = landmark_detector.CalculateAllLandmarks();
            List<Tuple<float, float, float>> landmarks_3D = landmark_detector.Calculate3DLandmarks(fx, fy, cx, cy);
            List<float> global_params = landmark_detector.GetRigidParams();
            List<float> local_params = landmark_detector.GetNonRigidParams();

            recorder.SetObservationLandmarks(landmarks_2D, landmarks_3D, global_params, local_params, confidence, success);

            var gaze = gaze_analyser.GetGazeCamera();
            var gaze_angle = gaze_analyser.GetGazeAngle();

            var landmarks_2d_eyes = landmark_detector.CalculateAllEyeLandmarks();
            var landmarks_3d_eyes = landmark_detector.CalculateAllEyeLandmarks3D(fx, fy, cx, cy);
            recorder.SetObservationGaze(gaze.Item1, gaze.Item2, gaze_angle, landmarks_2d_eyes, landmarks_3d_eyes);

            var au_regs = face_analyser.GetCurrentAUsReg();
            var au_classes = face_analyser.GetCurrentAUsClass();

            foreach (var key in au_regs.Keys)
            {
                if (au_regs.TryGetValue(key, out var key_val))
                {
                    var au_reg_string = string.Format("Regression_{0}:{1:F2}", key, key_val);
                    Debug.WriteLine(au_reg_string);
                    zero_mq_socket.Send(new ZFrame(au_reg_string, Encoding.UTF8));
                }
            }

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

        // When the processing is done re-enable the components
        private void EndMode()
        {

        }

        // ----------------------------------------------------------
        // Opening Videos/Images
        // ----------------------------------------------------------

        // Some utilities for opening images/videos and directories

        private void openWebcamClick(object sender, RoutedEventArgs e)
        {
            StopTracking();

            // If camera selection has already been done, no need to re-populate the list as it is quite slow
            if (cam_sec == null)
            {
                cam_sec = new CameraSelection();
            }
            else
            {
                cam_sec = new CameraSelection(cam_sec.cams);
                cam_sec.Visibility = System.Windows.Visibility.Visible;
            }

            // Set the icon
            Uri iconUri = new Uri("logo1.ico", UriKind.RelativeOrAbsolute);
            cam_sec.Icon = BitmapFrame.Create(iconUri);

            if (!cam_sec.no_cameras_found)
                cam_sec.ShowDialog();

            if (cam_sec.camera_selected)
            {
                int cam_id = cam_sec.selected_camera.Item1;
                int width = cam_sec.selected_camera.Item2;
                int height = cam_sec.selected_camera.Item3;

                SequenceReader reader = new SequenceReader(cam_id, width, height, fx, fy, cx, cy);

                processing_thread = new Thread(() => ProcessSequence(reader));
                processing_thread.Name = "Webcam processing";
                processing_thread.Start();

            }
        }


        // --------------------------------------------------------
        // Button handling
        // --------------------------------------------------------

        // Cleanup stuff when closing the window
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (processing_thread != null)
            {
                // Stop capture and tracking
                thread_running = false;
                processing_thread.Join();
            }
        }

        // Stopping the tracking
        private void StopButton_Click(object sender, RoutedEventArgs e)
        {
            if (processing_thread != null)
            {
                // Stop capture and tracking
                thread_paused = false;
                thread_running = false;
                // Let the processing thread finish
                processing_thread.Join();

                // Clean up the interface
                EndMode();
            }
        }

        private void PauseButton_Click(object sender, RoutedEventArgs e)
        {
            if (processing_thread != null)
            {
                // Stop capture and tracking                
                thread_paused = !thread_paused;

                NextFrameButton.IsEnabled = thread_paused;
                NextFiveFramesButton.IsEnabled = thread_paused;

                if (thread_paused)
                {
                    PauseButton.Content = "Resume";
                }
                else
                {
                    PauseButton.Content = "Pause";
                }
            }
        }

        private void setCameraParameters_Click(object sender, RoutedEventArgs e)
        {
            CameraParametersEntry camera_params_entry_window = new CameraParametersEntry(fx, fy, cx, cy);
            camera_params_entry_window.Icon = this.Icon;

            camera_params_entry_window.WindowStartupLocation = WindowStartupLocation.CenterScreen;

            if (camera_params_entry_window.ShowDialog() == true)
            {
                fx = camera_params_entry_window.Fx;
                fy = camera_params_entry_window.Fy;
                cx = camera_params_entry_window.Cx;
                cy = camera_params_entry_window.Cy;
            }
        }
    }
}
