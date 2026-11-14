// SPDX-License-Identifier: MPL-2.0
// Upper-body pipeline modeled on Webcam Motion Receiver (behavior analyzed at function level and
// re-implemented; see WmcUpperBodyMath). Kalidokit output is computed only for diagnostics.
// Writes Spine/Chest/UpperChest, shoulders, arms and hands; never Hips, legs, neck or head.
using System;
using LiveVRStudio.Avatar;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using Mediapipe;
using Mediapipe.Tasks.Core;
using Mediapipe.Tasks.Vision.Core;
using Mediapipe.Tasks.Vision.HandLandmarker;
using Mediapipe.Tasks.Vision.PoseLandmarker;
using Unity.Collections;
using UniVRM10;
using Unity.Profiling;
using UnityEngine;
using N = System.Numerics;
namespace LiveVRStudio.TechnicalValidation
{
    [DefaultExecutionOrder(10500)]
    public sealed class MediaPipeUpperBodyInput : MonoBehaviour
    {
        Func<TrackingInputPart,bool> defaultWriteGate;
        public void SetDefaultWriteGate(Func<TrackingInputPart,bool> gate){defaultWriteGate=gate;RefreshWriteGates();}
        void RefreshWriteGates()
        {
            foreach(var channel in channels)
            {
                bool allow=true;try{allow=defaultWriteGate==null||defaultWriteGate(channel.Part);}catch(Exception){allow=false;}
                if(channel.Writable!=allow)pendingSolve=true;
                channel.SetWritable(allow);
            }
        }
        sealed class Channel
        {
            public Transform Bone; public Quaternion Rest, Current, Neutral, Target;
            public TrackingInputPart Part; public bool Writable=true;
            public Quaternion LocalRotation {get=>Bone.localRotation;set{if(Writable)Bone.localRotation=value;}}
            public Quaternion WorldRotation {get=>Bone.rotation;set{if(Writable)Bone.rotation=value;}}
            public void SetWritable(bool value){if(value&&!Writable&&Bone!=null)Current=Target=Bone.localRotation;Writable=value;}
            public Channel(Transform bone) { Bone=bone; Rest=Current=Neutral=Target=bone!=null?bone.localRotation:Quaternion.identity; }
            public void ToRest(){if(Bone!=null&&Writable)Bone.localRotation=Rest;}
            public void Follow(Quaternion goal,float share){if(Bone==null||!Writable)return;Current=Quaternion.Slerp(Current,goal,share);Bone.localRotation=Current;}
        }
        // One avatar side. Landmark ids follow the existing screen-side association (12 -> avatar Left).
        sealed class Arm
        {
            public readonly bool AvatarLeft;public readonly int Shoulder,Elbow,Wrist,Index,Pinky;
            public Channel Collar,Upper,Lower,Hand;public Transform IndexBone,LittleBone;
            // Thumb, index, middle, ring, little; proximal, intermediate, distal each.
            public readonly Channel[] Fingers=new Channel[15];
            public Quaternion CollarSmoothed;public bool CollarStarted,HandFound;public double HandAt=double.NegativeInfinity;
            // Extra arm smoothing stage (Issue #711): the last smoothed rotation of each arm
            // channel, and whether the newest sample solved that channel. A channel a sample
            // did not solve (unseen elbow/wrist, no hand) keeps its state untouched.
            public Quaternion UpperSmoothed,LowerSmoothed,HandSmoothed;public bool UpperStarted,LowerStarted,HandStarted;
            public bool UpperSolved,LowerSolved,HandSolved;
            // When the elbow (or wrist) left the camera's view; the arm is held, then eased back to the relaxed pose.
            public double UnseenAt=double.NaN,WristUnseenAt=double.NaN;public bool RelaxArm,RelaxForearm,HoldArm,HoldForearm;
            public readonly WmcPointFilter[] HandFilters=new WmcPointFilter[21];
            public readonly N.Vector3[] HandPoints=new N.Vector3[21],HandImage=new N.Vector3[21],HandWorld=new N.Vector3[21],HandBuilt=new N.Vector3[21];
            public HandLandmarker Task;public NativeArray<byte> Crop;
            public Arm(bool left)
            {
                AvatarLeft=left;Shoulder=left?12:11;Elbow=left?14:13;Wrist=left?16:15;Index=left?20:19;Pinky=left?18:17;
                for(int i=0;i<21;i++)HandFilters[i]=new WmcPointFilter(WmcUpperBodyMath.HandMinCutoff,WmcUpperBodyMath.HandBeta);
            }
            public void ResetSignal(){HandFound=false;HandAt=double.NegativeInfinity;CollarStarted=false;UpperStarted=LowerStarted=HandStarted=false;UnseenAt=WristUnseenAt=double.NaN;RelaxArm=RelaxForearm=HoldArm=HoldForearm=false;foreach(var f in HandFilters)f.Reset();}
        }
        // Detector output copied off the worker thread; everything Unity-facing happens in ApplyDetection.
        sealed class Detection
        {
            public bool Valid,HandsRan,HandTimed;public Exception Error,HandError;public int FailureStage;public double At,PoseMs,HandMs;public float Aspect;
            public readonly Vector3[] World=new Vector3[33];public readonly N.Vector3[] Image=new N.Vector3[33];
            public readonly float[] Visibility=new float[33],Presence=new float[6];
            public readonly bool[] HandOk=new bool[2];
            public readonly N.Vector3[][] HandImage={new N.Vector3[21],new N.Vector3[21]},HandWorld={new N.Vector3[21],new N.Vector3[21]};
        }
        SingleAvatarRuntime runtime;SingleAvatarMediaPipeLocalInput face;SingleAvatarPoseDriver poses;
        SingleAvatarVrmFaceDriver faceDriver;PoseLandmarker task;byte[] model,handModel;string modelDirectory;
        GameObject avatar;Vrm10Instance vrm;Transform hips,neck,leftLeg,rightLeg;
        Channel spine,chest,upperChest;Channel[] channels=Array.Empty<Channel>();
        readonly Arm left=new Arm(true),right=new Arm(false);readonly Arm[] arms;
        static readonly int[] JointIds={12,14,16,11,13,15};
        public const int HandCropSize=224;
        // Hold the last pose this long after the newest sample, then return to the relaxed pose.
        internal const double HoldSeconds=.25;
        public static double ObservationHoldSeconds => HoldSeconds;
        // How long the raw in-frame test for both hips must hold before the trunk
        // estimate follows it (see Ingest). Matches the face/joint holds (#1568/#1588).
        const double HipsModeHoldSeconds=.35;
        // An arm whose elbow or wrist is out of the camera's view keeps its last pose this long, then returns to the relaxed pose.
        // The detector still prints a guessed position for such a joint; following it makes the arms cross or move together.
        const double UnseenHoldSeconds=.5;
        // Two hand detections closer than this (fraction of the image width) are the same hand.
        const float SameHandDistance=.05f;
        // WMC keeps the last hand array until a new one arrives: a hand missed for a few frames is held, not
        // replaced by the rest wrist and fingers for one frame. Session diagnostics (#1594) showed most
        // wrist losses land in the 0.3-0.5s band, which a 0.3s window could not absorb; the window now
        // matches the arm's unseen hold.
        const double HandHoldSeconds=.5;
        // After the held-points window above, a hand whose dedicated detection is still missing keeps
        // its last solved pose this long instead of returning to the rest wrist and fingers (#1594).
        // The arm-side hold keys off the pose wrist, so it never covered a hand-only detection loss.
        const double HandPoseHoldSeconds=1.0;
        static readonly string[] FingerNames={"Thumb","Index","Middle","Ring","Little"},JointNames={"Proximal","Intermediate","Distal"};
        float fallbackOpening=20;
        readonly Vector3[] world=new Vector3[33];
        readonly N.Vector3[] filtered=new N.Vector3[33],built=new N.Vector3[33],imagePoints=new N.Vector3[33],worldPoints=new N.Vector3[33];
        readonly float[] visibility=new float[33];
        readonly WmcPointFilter[] joints=new WmcPointFilter[33];
        Quaternion bodySmoothed=Quaternion.identity,twistSmoothed=Quaternion.identity;
        BodyTiltSettings bodyTilt=BodyTiltSettings.Default;
        public void ConfigureBodyTilt(BodyTiltSettings value)
        {
            value.Validate();if(bodyTilt.Equals(value))return;
            bodyTilt=value;bodyStarted=false;pendingSolve=true; // No hand reference or collar reset.
        }
        // Arm smoothing strength in seconds (a time constant), set from the
        // tracking panel's slider; the preference itself is stored globally
        // by LiveVRUpperBodyRestore, not per model.
        float armSmoothingSeconds=ArmSmoothingPolicy.DefaultSeconds;
        double armSampleInterval; // Seconds between the last two samples; set in Ingest.
        public float ArmSmoothingSeconds=>armSmoothingSeconds;
        public void ConfigureArmSmoothing(float seconds)=>armSmoothingSeconds=ArmSmoothingPolicy.Clamp(seconds);
        // Arm dead zone in degrees (Issue #837): the largest change from
        // the last emitted rotation that is still ignored, set from the
        // tracking panel's slider; the preference itself is stored
        // globally by LiveVRUpperBodyRestore, not per model.
        float armDeadZoneDegrees=ArmDeadZonePolicy.DefaultDegrees;
        public float ArmDeadZoneDegrees=>armDeadZoneDegrees;
        public void ConfigureArmDeadZone(float degrees)=>armDeadZoneDegrees=ArmDeadZonePolicy.Clamp(degrees);
        // hipsInFrame: both hips are inside the image. Otherwise their depth is a guess (seated at a desk).
        bool bodyStarted,twistStarted,pendingSolve,leftValid,rightValid,torsoValid,hipsInFrame;
        bool hipsRawInFrame;double hipsRawSince=double.NegativeInfinity;
        double sampleAt=double.NegativeInfinity;float frameAspect;
        // Posture shown as the avatar's straight rest torso, in measurement space: the first posture after tracking
        // starts, replaced by CalibrateNeutral. It corrects only torso directions; arms keep the camera's view.
        BodyNeutral neutral;Quaternion correction=Quaternion.identity;
        // Read by the worker thread. handsUnavailable is permanent until tracking is requested again.
        volatile bool requested,handsUnavailable;
        // Detector-only capture for a body calibration run (Issue #1550):
        // samples flow while this is set, but Requested (the user's
        // tracking switch) is never touched by it.
        bool calibrationCapture;
        // Serializes detector use (worker or synchronous) against ResetTask closing the tasks.
        readonly object detectorGate=new object();
        Detection workerDetection;
        public bool Requested=>requested;
        public bool CalibrationCaptureActive=>calibrationCapture;
        public bool LeftFound=>leftValid;
        public bool RightFound=>rightValid;
        // Arm data for collab (#1497, #820): arm rotations count as tracking data only
        // while tracking is requested and at least one arm is actually detected.
        // All-or-none for the 8 bones is kept (the wire format has one arm flag):
        // the detected side is live and the other side follows the same
        // held/relaxed behaviour the sender's own avatar shows.
        public static bool ArmDataAvailable(bool requested,bool leftFound,bool rightFound)=>requested&&(leftFound||rightFound);
        // The send gate's time-aware form (#820): the found flags are
        // recomputed only when a sample arrives, so a sender whose samples
        // stop arriving (a stalled worker, a frozen camera) would keep them
        // set forever and go on sending arm rotations its own avatar no
        // longer follows. Arm data therefore also has to be fresh: the
        // newest pose sample may be no older than the found-hold window
        // (WmcFoundHold) the tracking path itself applies to those flags.
        public static bool ArmDataAvailable(bool requested,bool leftFound,bool rightFound,double lastSampleAt,double now)
            =>ArmDataAvailable(requested,leftFound,rightFound)&&!double.IsNegativeInfinity(lastSampleAt)&&now-lastSampleAt<=WmcFoundHold.DefaultHoldSeconds;
        public bool HasArmData=>ArmDataAvailable(requested,leftValid,rightValid);
        // Arm data as the collab send gate reads it at time 'now' (#820):
        // requested, an arm detected, and the newest sample still fresh.
        public bool HasArmDataAt(double now)=>ArmDataAvailable(requested,leftValid,rightValid,sampleAt,now);
        public bool TorsoFound=>torsoValid;
        public double LastPoseSampleAt=>sampleAt;
        public bool CalibrationReferenceValid
        {
            get { try { new BodyCalibrationReference{Set=true,Up=TrunkUp(),ShoulderLine=filtered[11]-filtered[12]}.Validate();return true; }catch(ArgumentException){return false;} }
        }
        HandCalibrationReference handReference=new HandCalibrationReference();
        BodyCalibrationReference savedBodyReference;
        bool calibrationRestored;
        readonly N.Quaternion[] handRaw=new N.Quaternion[HandCalibrationReference.ChannelCount],handRest=new N.Quaternion[HandCalibrationReference.ChannelCount];
        readonly bool[] handSupported=new bool[HandCalibrationReference.ChannelCount];
        double handSolvedAt=double.NegativeInfinity;
        public bool CanCalibrateHands=>!pendingSolve && HandCalibrationReference.CanCaptureAny(requested,left.HandFound,right.HandFound,Time.unscaledTimeAsDouble,left.HandAt,right.HandAt) &&
            HandCalibrationReference.CanCapture(true,true,true,Time.unscaledTimeAsDouble,handSolvedAt,handSolvedAt);
        // Why the hand calibration cannot start now (shown as the button's tooltip), or null.
        public string HandCalibrationBlockedReason
        {
            get
            {
                if(!requested)return "手・腕のトラッキングをONにしてください。";
                if(!HandCalibrationReference.CanCaptureAny(true,left.HandFound,right.HandFound,Time.unscaledTimeAsDouble,left.HandAt,right.HandAt))
                    return "手のひらをカメラに向け、手首までカメラの枠に入れてください。片手だけでも校正できます。";
                return CanCalibrateHands?null:"手の入力を更新しています。少し待ってください。";
            }
        }
        public bool TryCaptureHandReference(out HandCalibrationReference value)
        {
            value=null;if(!CanCalibrateHands)return false;
            try { value=HandCalibrationReference.CaptureHands(handReference,handRaw,handRest,handSupported,left.HandFound,right.HandFound);return true; }
            catch(ArgumentException){return false;}
        }
        public void RestoreCalibration(BodyCalibrationReference body,HandCalibrationReference hands)
        {
            body.Validate();if(hands==null)throw new ArgumentNullException(nameof(hands));
            if(!calibrationRestored||!savedBodyReference.Equals(body))
            { savedBodyReference=body;RestoreNeutral(new BodyNeutral{Set=body.Set,Up=body.Up,ShoulderLine=body.ShoulderLine}); }
            if(!calibrationRestored||!handReference.SameReference(hands))
            { handReference=hands.Clone();pendingSolve=true; }
            calibrationRestored=true;
        }
        static N.Quaternion Numeric(Quaternion q)=>new N.Quaternion(q.x,q.y,q.z,q.w);
        static Quaternion UnityRotation(N.Quaternion q)=>new Quaternion(q.X,q.Y,q.Z,q.W);
        void ApplyHandCalibration()
        {
            var valid=true;
            foreach(var arm in arms)
            {
                int offset=arm.AvatarLeft?0:HandCalibrationReference.ChannelsPerHand;
                for(int i=0;i<HandCalibrationReference.ChannelsPerHand;i++)
                {
                    var channel=i==0?arm.Hand:arm.Fingers[i-1];int index=offset+i;
                    handSupported[index]=channel.Bone!=null;
                    if(channel.Bone==null)continue;
                    handRaw[index]=Numeric(channel.LocalRotation);handRest[index]=Numeric(channel.Rest);
                    if(!arm.HandFound)continue;
                    try{channel.LocalRotation=UnityRotation(handReference.Apply(index,handRaw[index]));}
                    catch(ArgumentException){channel.LocalRotation=channel.Rest;valid=false;} // Reject malformed observations without changing the saved reference.
                }
            }
            handSolvedAt=valid?sampleAt:double.NegativeInfinity;
        }
        public struct BodyNeutral { public bool Set; public N.Vector3 Up, ShoulderLine; }
        public BodyNeutral Neutral=>neutral;
        // Calibration: the current filtered posture becomes upright and square to the camera. Shoulders are always
        // estimated (visibility forced), so any pose sample will do even with a shoulder at the frame edge.
        public bool CalibrateNeutral()
        {
            if(!(requested||calibrationCapture)||double.IsNegativeInfinity(sampleAt))return false;
            var F=filtered;
            neutral=new BodyNeutral{Set=true,Up=TrunkUp(),ShoulderLine=F[11]-F[12]};
            RestartSmoothing();return true;
        }
        public void RestoreNeutral(BodyNeutral value){neutral=value;RestartSmoothing();}
        // Turning tracking on re-runs the posture reference (Issue #1494): a
        // reference the user explicitly calibrated and saved is restored as
        // the reference; without one, the session reference is dropped so the
        // first valid samples capture a fresh neutral instead of reusing a
        // stale one from an earlier ON period. Before the saved settings were
        // ever restored there is nothing to decide yet, so the current state
        // is left alone (RestoreCalibration applies the saved value later).
        void RestartPostureReference()
        {
            if(!calibrationRestored)return;
            if(savedBodyReference.Set)RestoreNeutral(new BodyNeutral{Set=true,Up=savedBodyReference.Up,ShoulderLine=savedBodyReference.ShoulderLine});
            else { neutral=default; RestartSmoothing(); }
        }
        // Jump to the new reference instead of drifting there at the per-sample follow rate.
        void RestartSmoothing(){bodyStarted=twistStarted=false;foreach(var arm in arms){arm.CollarStarted=false;arm.UpperStarted=arm.LowerStarted=arm.HandStarted=false;}pendingSolve=true;}
        public bool[] JointFound { get; }=new bool[6]; // L shoulder/elbow/wrist, R shoulder/elbow/wrist (screen-side convention).
        // Found flags pass through a short temporal hold (see WmcFoundHold): the raw per-frame
        // verdict flips at frame rate for landmarks on the frame edge or the visibility threshold,
        // and that flapping reached the arm/torso drive as visible trembling (#1588).
        private readonly WmcFoundHold jointFoundHold=new WmcFoundHold(6);
        // The raw per-sample verdict, without the hold above. The arm
        // hold/relax drive (SolveArm's Track) consumes this one: with the
        // held flag an unseen elbow stayed "seen" for the hold window and
        // the arm followed the guessed (crossed) position instead of
        // holding its last pose (#1633). The held flag keeps serving the
        // torso validity, calibration gating and found events, whose
        // flapping the hold exists to suppress.
        private readonly bool[] rawJointFound=new bool[6];
        public float[] JointVisibility { get; }=new float[]{float.NaN,float.NaN,float.NaN,float.NaN,float.NaN,float.NaN};
        public float[] JointPresence { get; }=new float[]{float.NaN,float.NaN,float.NaN,float.NaN,float.NaN,float.NaN};
        // Diagnostics only: the older Kalidokit angles of the latest landmarks.
        public SolvedUpperBody RawPose=>KalidokitUpperBodySolver.Solve(world);
        // Arm frame for display, in image coordinates (0..1, y down): the detector's points and the filtered
        // points the avatar follows, for landmark ids 11-16, 23, 24. False before the first sample.
        public float FrameAspect=>frameAspect;
        public static readonly int[] FrameIds={0,2,5,7,8,9,10,11,12,13,14,15,16,23,24};
        public bool TryGetFrame(Vector2[] detected,Vector2[] used,bool[] seen)
        {
            if(double.IsNegativeInfinity(sampleAt)||frameAspect<=0)return false;
            for(int i=0;i<FrameIds.Length;i++)
            {
                int id=FrameIds[i];
                detected[i]=new Vector2(imagePoints[id].X,-imagePoints[id].Y/frameAspect);
                used[i]=new Vector2(filtered[id].X,-filtered[id].Y/frameAspect);
                seen[i]=visibility[id]>WmcUpperBodyMath.VisibleThreshold;
            }
            return true;
        }
        // Filtered hand landmarks (21) of one avatar side, same coordinates as TryGetFrame. False while the hand is not held.
        public bool TryGetHandFrame(bool avatarLeft,Vector2[] points)
        {
            var arm=avatarLeft?left:right;
            if(!arm.HandFound||frameAspect<=0)return false;
            for(int i=0;i<21;i++)points[i]=new Vector2(arm.HandPoints[i].X,-arm.HandPoints[i].Y/frameAspect);
            return true;
        }
        public Vector3 ObservedJoint(int index)=>index>=0&&index<world.Length?world[index]:Vector3.zero;
        public double LastMilliseconds { get; private set; }
        // Hand inference of the latest applied detection, timed separately
        // from LastMilliseconds (which is PoseMs only) (#1593). HandAnalysisCount
        // counts applied detections whose hand stage actually ran, so a mean
        // over an interval divides by the right number of samples.
        public double LastHandMilliseconds { get; private set; }
        public long HandAnalysisCount { get; private set; }
        public long AnalysisCount { get; private set; }
        public Exception LastError { get; private set; }
        public Exception LastHandError { get; private set; }
        public int FailureStage { get; private set; }
        public string Status { get; private set; }="手・腕のトラッキングはOFFです。";
        public string HandStatus { get; private set; }="";
        public static event Action<int> TrackingEvent;
        public MediaPipeUpperBodyInput()
        {
            arms=new[]{left,right};
            for(int i=0;i<joints.Length;i++)joints[i]=new WmcPointFilter(WmcUpperBodyMath.PoseMinCutoff,WmcUpperBodyMath.PoseBeta);
        }
        public void Bind(SingleAvatarRuntime target,SingleAvatarMediaPipeLocalInput source)
        {
            if (source == null) return;
            runtime=target;face=source;modelDirectory=Application.streamingAssetsPath;
            face.CameraTextureReady+=AnalyzeTexture;face.CameraImageOnWorker+=AnalyzeOnWorker;face.CameraWorkerApplied+=ApplyWorkerDetection;face.CameraSessionReset+=ResetFaceCameraSession;
            face.SecondaryAnalysisDemand=requested;
            runtime.AvatarLoaded+=BindAvatar;BindAvatar(runtime.DisplayedAvatar);
        }
        public bool SetRequested(bool value)
        {
            // Bindings normally follow AvatarLoaded. If a commit ever swapped the
            // displayed avatar without that notification, the retained reference
            // points at a destroyed predecessor and every request would be judged
            // against it; rebind to the avatar actually on stage first (the same
            // self-healing SingleAvatarPoseDriver.EnsureCurrentAvatar performs).
            if(value&&runtime!=null&&avatar!=runtime.DisplayedAvatar)BindAvatar(runtime.DisplayedAvatar);
            if(value && (vrm==null||vrm.Runtime.ControlRig==null)){Status="Humanoidモデルを読み込んでください。";HandStatus="";return false;}
            requested=value;if(!value)calibrationCapture=false;if(face!=null)face.SecondaryAnalysisDemand=value;if(value)RestartPostureReference();
            ResetTask();LastError=null;LastHandError=null;FailureStage=0;handsUnavailable=false;HandStatus="";
            if(value)foreach(var channel in channels)channel.Current=channel.LocalRotation;
            if(faceDriver!=null)faceDriver.SetExternalBodyOwner(value);
            if(poses==null)poses=FindAnyObjectByType<SingleAvatarPoseDriver>();
            if(poses!=null)SetFallbackOpening(poses.RelaxedOpeningDegrees);
            poses?.SetTrackingOwnership(value?AvatarPoseArms.Both:AvatarPoseArms.None);
            if(!value&&runtime!=null)runtime.SelectNormalBody();
            Status=value?"肩・肘・手首の入力を待っています。":"手・腕のトラッキングはOFFです。";
            TrackingEvent?.Invoke(value?28:29);return true;
        }
        // Starts or ends the detector-only capture a body calibration
        // uses while tracking is off (Issue #1550). Capture runs the
        // same detector pipeline into the same filters the calibration
        // reads, but it is not tracking: Requested stays off, so the
        // panel switch, the arm preview, the pose ownership and the
        // avatar drive all keep their off state, the status lines keep
        // saying tracking is off, and nothing is persisted. Starting
        // needs the same Humanoid rig SetRequested(true) needs.
        public bool SetCalibrationCapture(bool value)
        {
            if(value)
            {
                if(calibrationCapture)return true;
                if(vrm==null||vrm.Runtime==null||vrm.Runtime.ControlRig==null)return false;
                calibrationCapture=true;
                if(face!=null)face.SecondaryAnalysisDemand=true;
                ResetTask();
                return true;
            }
            if(!calibrationCapture)return true;
            calibrationCapture=false;
            if(!requested)
            {
                if(face!=null)face.SecondaryAnalysisDemand=false;
                ResetTask();
                Status="手・腕のトラッキングはOFFです。";HandStatus="";
            }
            return true;
        }
        void BindAvatar(GameObject next)
        {
            if(faceDriver!=null)faceDriver.SetExternalBodyOwner(false);
            neutral=default;handReference=new HandCalibrationReference();calibrationRestored=false;handSolvedAt=double.NegativeInfinity;
            avatar=next;vrm=next!=null?next.GetComponent<Vrm10Instance>():null;
            faceDriver=next!=null?next.GetComponent<SingleAvatarVrmFaceDriver>():null;
            ResetTask();channels=Array.Empty<Channel>();
            var rig=vrm!=null&&vrm.Runtime!=null?vrm.Runtime.ControlRig:null;
            if(rig==null)
            {
                // The new model cannot take upper-body tracking (#1498/#1511):
                // drop the request instead of leaving Requested set with no
                // effect behind it, and clear both status lines so the previous
                // model's "tracking" text cannot survive the switch.
                if(requested)SetRequested(false);
                Status=next==null?"手・腕のトラッキングはOFFです。":"Humanoidモデルを読み込んでください。";
                HandStatus="";
                return;
            }
            Transform T(HumanBodyBones id)=>rig?.GetBoneTransform(id);
            Channel C(HumanBodyBones id)=>new Channel(T(id)){Part=TrackingInputBoneWriter.Supports(TrackingInputPart.LeftHand,id)?TrackingInputPart.LeftHand:TrackingInputBoneWriter.Supports(TrackingInputPart.RightHand,id)?TrackingInputPart.RightHand:TrackingInputPart.UpperBody};
            spine=C(HumanBodyBones.Spine);chest=C(HumanBodyBones.Chest);upperChest=C(HumanBodyBones.UpperChest);
            hips=T(HumanBodyBones.Hips);neck=T(HumanBodyBones.Neck);leftLeg=T(HumanBodyBones.LeftUpperLeg);rightLeg=T(HumanBodyBones.RightUpperLeg);
            void BindArm(Arm arm,HumanBodyBones collar,HumanBodyBones upper,HumanBodyBones lower,HumanBodyBones hand,HumanBodyBones index,HumanBodyBones little)
            {
                arm.Collar=C(collar);arm.Upper=C(upper);arm.Lower=C(lower);arm.Hand=C(hand);arm.IndexBone=T(index);arm.LittleBone=T(little);
                for(int i=0;i<15;i++)arm.Fingers[i]=C((HumanBodyBones)Enum.Parse(typeof(HumanBodyBones),(arm.AvatarLeft?"Left":"Right")+FingerNames[i/3]+JointNames[i%3]));
            }
            BindArm(left,HumanBodyBones.LeftShoulder,HumanBodyBones.LeftUpperArm,HumanBodyBones.LeftLowerArm,HumanBodyBones.LeftHand,HumanBodyBones.LeftIndexProximal,HumanBodyBones.LeftLittleProximal);
            BindArm(right,HumanBodyBones.RightShoulder,HumanBodyBones.RightUpperArm,HumanBodyBones.RightLowerArm,HumanBodyBones.RightHand,HumanBodyBones.RightIndexProximal,HumanBodyBones.RightLittleProximal);
            // Only bones the model has; every per-frame loop runs over this array.
            var bound=new List<Channel>();
            var all=new List<Channel>{spine,chest,upperChest,left.Collar,left.Upper,left.Lower,left.Hand,right.Collar,right.Upper,right.Lower,right.Hand};
            all.AddRange(left.Fingers);all.AddRange(right.Fingers);
            foreach(var channel in all)
                if(channel.Bone!=null)bound.Add(channel);
            channels=bound.ToArray();RefreshWriteGates();
            Relaxed(left.Upper,left.Lower,true);Relaxed(right.Upper,right.Lower,false);
            if(requested)SetRequested(true);
            else { Status="手・腕のトラッキングはOFFです。"; HandStatus=""; }
        }
        public void SetFallbackOpening(float degrees)
        {
            fallbackOpening=degrees;
            if(vrm!=null){Relaxed(left.Upper,left.Lower,true);Relaxed(right.Upper,right.Lower,false);}
        }
        void Relaxed(Channel upper,Channel elbow,bool left)
        {
            if(upper?.Bone==null||elbow?.Bone==null)return;
            var down=-avatar.transform.up;var outward=avatar.transform.right*(left?-1:1);
            var desired=down*Mathf.Cos(fallbackOpening*Mathf.Deg2Rad)+outward*Mathf.Sin(fallbackOpening*Mathf.Deg2Rad);
            var rotation=Quaternion.FromToRotation(elbow.Bone.position-upper.Bone.position,desired)*upper.WorldRotation;
            upper.Neutral=upper.Bone.parent!=null?Quaternion.Inverse(upper.Bone.parent.rotation)*rotation:rotation;
        }
        // Detector use is serialized by detectorGate, so a task detached under
        // the gate is never in use. Closing can block on native teardown, so the
        // close itself runs on a pool thread instead of the caller (usually the
        // main thread); detaching under the gate is the only synchronous part.
        static void CloseDetachedTasks(PoseLandmarker pose,HandLandmarker[] hands)
        {
            var any=pose!=null;
            if(hands!=null)foreach(var hand in hands)any|=hand!=null;
            if(!any)return;
            ThreadPool.QueueUserWorkItem(_ =>
            {
                try {pose?.Close();} catch(Exception) { }
                if(hands!=null)foreach(var hand in hands){try {hand?.Close();} catch(Exception) { }}
            });
        }
        void ResetFaceCameraSession() { if(externalCameraOwner==null)ResetTask(); }
        // 信号リセットは校正入力の所有関係を解かない (P2-4): 所有者は
        // CameraCalibratedTrackingBridge が BindCalibratedInput で取り、
        // Disconnect 時の ReleaseCalibratedInput で手放す。ResetTask で
        // 外すと、同値の SetRequested(true) 再設定だけで三角測量が
        // 恒久的に止まる (dispatcher は生き続け再バインドしないため)。
        void ResetTask()
        {
            calibratedSample=false;detachedInferenceOwner=null;
            PoseLandmarker closingPose;var closingHands=new HandLandmarker[arms.Length];
            lock(detectorGate)
            {
                closingPose=task;task=null;
                for(var i=0;i<arms.Length;i++)
                {
                    closingHands[i]=arms[i].Task;arms[i].Task=null;
                    if(arms[i].Crop.IsCreated)arms[i].Crop.Dispose();
                }
            }
            CloseDetachedTasks(closingPose,closingHands);
            Interlocked.Exchange(ref workerDetection,null);
            foreach(var arm in arms)arm.ResetSignal();
            foreach(var filter in joints)filter.Reset();
            jointFoundHold.Reset();
            handSolvedAt=double.NegativeInfinity;
            sampleAt=double.NegativeInfinity;bodyStarted=twistStarted=pendingSolve=false;
            leftValid=rightValid=torsoValid=false;hipsInFrame=hipsRawInFrame=false;hipsRawSince=double.NegativeInfinity;
            for(int i=0;i<6;i++){JointFound[i]=false;JointVisibility[i]=JointPresence[i]=float.NaN;}
        }
        // Multi-camera capture owns image inference while explicitly active. The
        // face camera still feeds facial tracking, but cannot race body inference.
        volatile object externalCameraOwner;
        public bool SetExternalCameraOwner(object owner, bool enabled)
        {
            lock(detectorGate)
            {
                if(owner==null)return false;
                if(enabled){if(externalCameraOwner!=null&&!ReferenceEquals(externalCameraOwner,owner))return false;externalCameraOwner=owner;Interlocked.Exchange(ref workerDetection,null);return true;}
                if(!ReferenceEquals(externalCameraOwner,owner))return false;externalCameraOwner=null;Interlocked.Exchange(ref workerDetection,null);return true;
            }
        }
        public bool SubmitCameraInference(object owner,SyntheticFrame frame,double receivedAt)
        {
            if(!ReferenceEquals(externalCameraOwner,owner))return false;
            ClearCalibratedObservation();return SubmitSyntheticFrame(frame,receivedAt);
        }
        // Synchronous path (still images, Editor checks): detect and apply in one call.
        void AnalyzeTexture(Texture2D texture,long time,ImageProcessingOptions options,double receivedAt)
        {
            if(externalCameraOwner!=null||!requested||vrm==null||texture.format!=TextureFormat.RGBA32)return;
            if(modelDirectory==null)modelDirectory=Application.streamingAssetsPath;
            ApplyDetection(Detect(texture.GetRawTextureData<byte>(),texture.width,texture.height,time,options,receivedAt));
        }
        // Worker thread: detectors only. The newest detection replaces any unapplied one.
        void AnalyzeOnWorker(NativeArray<byte> rgba,int width,int height,long time,ImageProcessingOptions options,double receivedAt)
        {
            if(externalCameraOwner!=null||(!requested&&!calibrationCapture))return;
            // Per-part analysis rate (#1589): a throttled pose stage writes no detection
            // at all, so the previous pose state simply persists (ApplyWorkerDetection
            // consumes by exchange, so nothing stale is re-applied either).
            if(!TrackingAnalysisRateGate.ShouldRun(TrackingAnalysisStage.Body,receivedAt))return;
            lock(detectorGate) { if(externalCameraOwner!=null)return; Volatile.Write(ref workerDetection,Detect(rgba,width,height,time,options,receivedAt,true)); }
        }
        void ApplyWorkerDetection()
        {
            var detection=Interlocked.Exchange(ref workerDetection,null);
            if(detection!=null&&(requested||calibrationCapture)&&vrm!=null)ApplyDetection(detection);
        }
        // Thread-agnostic: no Unity object access, only MediaPipe and plain data. Timestamps are already
        // strictly increasing per camera session (SingleAvatarMediaPipeLocalInput), and a session reset closes the tasks.
        Detection Detect(NativeArray<byte> rgba,int width,int height,long time,ImageProcessingOptions options,double receivedAt,bool rateGated=false)
        {
            var d=new Detection{At=receivedAt,Aspect=width>0?(float)height/width:1f};
            var watch=System.Diagnostics.Stopwatch.StartNew();
            lock(detectorGate)
            {
                try
                {
                    if(task==null)
                    {
                        d.FailureStage=1;
                        if(model==null)model=File.ReadAllBytes(Path.Combine(modelDirectory,"pose_landmarker_full.bytes"));
                        d.FailureStage=2;
                        task=PoseLandmarker.CreateFromOptions(new PoseLandmarkerOptions(new BaseOptions(BaseOptions.Delegate.CPU,modelAssetBuffer:model),
                            runningMode:RunningMode.VIDEO,numPoses:1,minPoseDetectionConfidence:.5f,minPosePresenceConfidence:.5f,minTrackingConfidence:.5f));
                    }
                    d.FailureStage=3;
                    PoseLandmarkerResult result;
                    // SDK Packet.CreateImageAt consumes Image. The pixels stay owned by the caller.
                    using(var image=new Image(ImageFormat.Types.Format.Srgba,width,height,width*4,rgba))
                        result=task.DetectForVideo(image,time,options);
                    d.FailureStage=4;
                    d.Valid=result.poseWorldLandmarks!=null&&result.poseWorldLandmarks.Count>0&&result.poseWorldLandmarks[0].landmarks.Count>=33&&
                        result.poseLandmarks!=null&&result.poseLandmarks.Count>0&&result.poseLandmarks[0].landmarks.Count>=33;
                    if(d.Valid)
                    {
                        var points=result.poseWorldLandmarks[0].landmarks;var image=result.poseLandmarks[0].landmarks;
                        for(int i=0;i<33;i++)
                        {
                            var p=points[i];var q=image[i];d.World[i]=new Vector3(p.x,p.y,p.z);d.Image[i]=new N.Vector3(q.x,q.y,q.z);
                            d.Visibility[i]=p.visibility??q.visibility??1f;
                        }
                        for(int i=0;i<6;i++)d.Presence[i]=points[JointIds[i]].presence??float.NaN;
                    }
                    watch.Stop();d.PoseMs=watch.Elapsed.TotalMilliseconds;
                    // Hands need an upright image: the crop works in raw buffer coordinates.
                    if(d.Valid&&!handsUnavailable&&options.rotationDegrees==0)
                    {
                        // Split timing (#1593): the hand stage is timed on its own
                        // watch so the 1 Hz diagnostics can record PoseMs and the
                        // hand inference time separately. A throttled hand stage
                        // leaves HandTimed false and contributes no hand sample.
                        var handWatch=System.Diagnostics.Stopwatch.StartNew();var handRan=false;
                        try
                        {
                            if(handModel==null)handModel=File.ReadAllBytes(Path.Combine(modelDirectory,"hand_landmarker.bytes"));
                            var pose=result.poseLandmarks[0].landmarks;
                            // Per-part analysis rate (#1589): a throttled hand stage leaves
                            // HandOk false for this frame; the existing hold keeps the last hand.
                            if(!rateGated||TrackingAnalysisRateGate.ShouldRun(TrackingAnalysisStage.Hand,receivedAt))
                            { DetectHand(left,0,d,rgba,width,height,pose,time);DetectHand(right,1,d,rgba,width,height,pose,time);handRan=true; }
                            d.HandsRan=true;
                        }
                        catch(Exception error){d.HandError=error;}
                        handWatch.Stop();if(handRan){d.HandMs=handWatch.Elapsed.TotalMilliseconds;d.HandTimed=true;}
                    }
                }
                catch(Exception error){d.Error=error;}
            }
            return d;
        }
        // Main thread: copy the detection into tracking state, then filter and solve.
        void ApplyDetection(Detection d)
        {
            ClearCalibratedObservation();
            if(d.Error!=null)
            {
                LastError=d.Error;FailureStage=d.FailureStage;ResetTask();
                if(calibrationCapture&&!requested){calibrationCapture=false;if(face!=null)face.SecondaryAnalysisDemand=false;TrackingEvent?.Invoke(36);return;}
                requested=false;faceDriver?.SetExternalBodyOwner(false);poses?.SetTrackingOwnership(AvatarPoseArms.None);
                // Exclusive switch (Issue #1551): releasing ownership must also
                // bring the normal (mapping) body follow back, as SetRequested(false) does.
                if(runtime!=null)runtime.SelectNormalBody();
                Status="手・腕の解析を開始できません。顔の追跡は継続しています。";HandStatus="";TrackingEvent?.Invoke(36);return;
            }
            AnalysisCount++;LastMilliseconds=d.PoseMs;FailureStage=d.FailureStage;if(d.HandTimed){HandAnalysisCount++;LastHandMilliseconds=d.HandMs;}
            if(!d.Valid){Lost();if(requested){Status="肩・肘・手首をカメラに収めてください。";HandStatus="";}return;}
            DetectionPowerGate.NoteDetection(d.At);
            for(int i=0;i<33;i++)
            {
                world[i]=d.World[i];imagePoints[i]=WmcUpperBodyMath.FromImage(d.Image[i],d.Aspect);
                worldPoints[i]=WmcUpperBodyMath.FromWorld(new N.Vector3(d.World[i].x,d.World[i].y,d.World[i].z));visibility[i]=d.Visibility[i];
            }
            for(int i=0;i<6;i++)JointPresence[i]=d.Presence[i];
            var width=Ingest(d.Aspect,d.At);
            if(requested)Status=leftValid&&rightValid?"左右の肩・肘を追跡中です。":leftValid?"左上腕を追跡中です。右肘をカメラに収めてください。":rightValid?"右上腕を追跡中です。左肘をカメラに収めてください。":"肩・肘・手首をカメラに収めてください。";
            if(d.HandError!=null)
            {
                LastHandError=d.HandError;handsUnavailable=true;
                var failedHands=new HandLandmarker[arms.Length];
                lock(detectorGate)for(var i=0;i<arms.Length;i++){failedHands[i]=arms[i].Task;arms[i].Task=null;}
                CloseDetachedTasks(null,failedHands);
            }
            // One hand found in both crops (hands close together) belongs to the arm whose pose wrist is nearer.
            if(d.HandOk[0]&&d.HandOk[1])
            {
                var a=d.HandImage[0][0];var b=d.HandImage[1][0];
                if(new Vector2(a.X-b.X,a.Y-b.Y).magnitude<SameHandDistance)
                {
                    var wristA=d.Image[arms[0].Wrist];var wristB=d.Image[arms[1].Wrist];
                    var nearA=new Vector2(a.X-wristA.X,a.Y-wristA.Y).magnitude;var nearB=new Vector2(b.X-wristB.X,b.Y-wristB.Y).magnitude;
                    d.HandOk[nearA<=nearB?1:0]=false;
                }
            }
            for(int slot=0;slot<2;slot++)
            {
                var arm=arms[slot];
                arm.HandFound=d.At-arm.HandAt<=HandHoldSeconds;
                if(!d.HandOk[slot])continue;
                for(int i=0;i<21;i++){arm.HandImage[i]=WmcUpperBodyMath.FromImage(d.HandImage[slot][i],d.Aspect);arm.HandWorld[i]=WmcUpperBodyMath.FromWorld(d.HandWorld[slot][i]);}
                if(WmcUpperBodyMath.IsCollapsed(arm.HandWorld))continue;
                WmcUpperBodyMath.BuildHandPoints(arm.HandImage,arm.HandWorld,arm.HandBuilt);
                for(int i=0;i<21;i++)arm.HandPoints[i]=arm.HandFilters[i].Filter(arm.HandBuilt[i],d.At,width,1f);
                arm.HandFound=true;arm.HandAt=d.At;
            }
            if(requested)HandStatus=handsUnavailable?"手の解析を開始できません。腕と顔の追跡は継続しています。":!d.HandsRan?"手首の向きは回転したカメラ映像では使えません。":
                left.HandFound&&right.HandFound?"両手の向きを追跡中です。":left.HandFound||right.HandFound?"片手の向きを追跡中です。":"手のひらをカメラに向けると手首の向きを追跡します。";
        }
        void Lost()
        {
            bool oldLeft=leftValid,oldRight=rightValid,oldTorso=torsoValid;
            leftValid=rightValid=torsoValid=false;
            for(int i=0;i<6;i++){if(JointFound[i])TrackingEvent?.Invoke(40+i*2);JointFound[i]=false;rawJointFound[i]=false;JointVisibility[i]=JointPresence[i]=float.NaN;}
            jointFoundHold.Reset();
            if(oldLeft)TrackingEvent?.Invoke(30);if(oldRight)TrackingEvent?.Invoke(32);if(oldTorso)TrackingEvent?.Invoke(34);
        }
        // One synthetic inference result for the QA rig (#1573), in the
        // same shape Detect produces: pose world landmarks as Unity
        // vectors, image landmarks and hand landmarks as Numerics vectors.
        public struct SyntheticFrame
        {
            public bool Valid,HandsRan;
            public float Aspect;
            public Vector3[] World;         // 33 pose world landmarks.
            public N.Vector3[] Image;       // 33 normalized pose landmarks.
            public float[] Visibility;      // 33, one per landmark.
            public float[] Presence;        // 6, in JointIds order.
            public bool[] HandOk;           // 2: slot 0 = left arm, slot 1 = right arm.
            public N.Vector3[][] HandImage; // [slot][21]
            public N.Vector3[][] HandWorld; // [slot][21]
        }
        object detachedInferenceOwner;long detachedInferenceTime=-1;
        // Dedicated detector instances only. Never reuse the active single-camera
        // component: VIDEO trackers and their timestamps must remain per-camera.
        public bool BeginDetachedInference(object owner)
        {
            if(owner==null||runtime!=null||face!=null||avatar!=null||requested||calibrationCapture||detachedInferenceOwner!=null)return false;
            modelDirectory=Application.streamingAssetsPath;detachedInferenceOwner=owner;detachedInferenceTime=-1;return true;
        }
        public bool TryAnalyzeDetachedFrame(object owner,NativeArray<byte> rgba,int width,int height,long timestampMilliseconds,double receivedAt,out SyntheticFrame frame)
        {
            frame=default(SyntheticFrame);
            lock(detectorGate)
            {
                if(owner==null||!ReferenceEquals(owner,detachedInferenceOwner)||!ReferenceEquals(runtime,null)||!ReferenceEquals(face,null)||!ReferenceEquals(avatar,null)||requested||calibrationCapture||!rgba.IsCreated||width<16||height<16||(long)width*height>1920L*1080||rgba.Length!=(long)width*height*4||timestampMilliseconds<0||timestampMilliseconds<=detachedInferenceTime||double.IsNaN(receivedAt)||double.IsInfinity(receivedAt)||receivedAt<0)return false;
                detachedInferenceTime=timestampMilliseconds;
                var d=Detect(rgba,width,height,timestampMilliseconds,new ImageProcessingOptions(),receivedAt);
                if(d.Error!=null)return false;
                frame=new SyntheticFrame{Valid=d.Valid,HandsRan=d.HandsRan,Aspect=d.Aspect,World=d.World,Image=d.Image,Visibility=d.Visibility,Presence=d.Presence,HandOk=d.HandOk,HandImage=d.HandImage,HandWorld=d.HandWorld};
                return true;
            }
        }
        public void EndDetachedInference(object owner)
        {
            lock(detectorGate){if(!ReferenceEquals(owner,detachedInferenceOwner))return;detachedInferenceOwner=null;ResetTask();}
        }
        // QA synthetic entry (#1573): hands a synthetic inference result
        // to the same slot and apply step the worker thread uses, so the
        // frame passes through ApplyDetection — point filters, solve,
        // smoothing and the ownership switch — exactly like a camera
        // result. Unlike SubmitObservation it does not bypass
        // ApplyDetection. Returns false when the frame is malformed; a
        // well-formed frame submitted while tracking is off is dropped,
        // as a worker result would be.
        public bool SubmitSyntheticFrame(SyntheticFrame frame,double receivedAt)
        {
            if(frame.Valid&&(frame.World==null||frame.World.Length!=33||frame.Image==null||frame.Image.Length!=33||
               frame.Visibility==null||frame.Visibility.Length!=33||frame.Presence==null||frame.Presence.Length!=6))return false;
            if(!requested&&!calibrationCapture)return true;
            var d=new Detection{Valid=frame.Valid,HandsRan=frame.HandsRan,At=receivedAt,Aspect=frame.Aspect};
            if(frame.Valid)
            {
                for(int i=0;i<33;i++){d.World[i]=frame.World[i];d.Image[i]=frame.Image[i];d.Visibility[i]=frame.Visibility[i];}
                for(int i=0;i<6;i++)d.Presence[i]=frame.Presence[i];
                if(frame.HandOk!=null&&frame.HandImage!=null&&frame.HandWorld!=null)
                    for(int slot=0;slot<2;slot++)
                    {
                        if(!frame.HandOk[slot]||frame.HandImage[slot]==null||frame.HandWorld[slot]==null||
                           frame.HandImage[slot].Length!=21||frame.HandWorld[slot].Length!=21)continue;
                        d.HandOk[slot]=true;
                        for(int i=0;i<21;i++){d.HandImage[slot][i]=frame.HandImage[slot][i];d.HandWorld[slot][i]=frame.HandWorld[slot][i];}
                    }
            }
            Volatile.Write(ref workerDetection,d);
            ApplyWorkerDetection();
            return true;
        }
        public bool HasCalibratedSample => calibratedSample;
        bool calibratedSample;object calibratedOwner;readonly CalibratedObservationClock calibratedClock=new CalibratedObservationClock();
        public bool BindCalibratedInput(object owner,GameObject expectedModel)
        {
            if(owner==null||expectedModel!=avatar||vrm==null||!requested)return false;
            ClearCalibratedObservation();calibratedOwner=owner;return true;
        }
        // 所有者の明示的な手放し。校正ブリッジの切断時だけが呼ぶ (P2-4)。
        // 信号リセット (ResetTask) は所有者を外さない。所有者が一致しない
        // 呼び出しは何もしない。
        public void ReleaseCalibratedInput(object owner,GameObject expectedModel)
        {
            if(owner==null||!ReferenceEquals(owner,calibratedOwner)||expectedModel!=avatar)return;
            calibratedOwner=null;ClearCalibratedObservation();
        }
        // Integration hook for an already paired/calibrated frame. Coordinates are explicitly
        // in this solver's x-right/y-up/z-back convention, not raw camera or MediaPipe axes.
        // This does not open cameras, claim tracking ownership or enable tracking.
        public bool SubmitCalibratedObservation(object owner,GameObject expectedModel,N.Vector3[] solverPoints,N.Vector3[] primaryImage,float[] confidence,float aspect,double at,double now)
        {
            bool Valid(double v)=>!double.IsNaN(v)&&!double.IsInfinity(v);
            if(owner==null||!ReferenceEquals(owner,calibratedOwner)||!requested||vrm==null||expectedModel!=avatar)return false;
            if(solverPoints==null||solverPoints.Length!=33||primaryImage==null||primaryImage.Length!=33||confidence==null||confidence.Length!=33||!Valid(aspect)||aspect<=0||!Valid(at)||!Valid(now)||at<0||at>now||now-at>HoldSeconds||!calibratedClock.Acceptable(at))return false;
            for(int i=0;i<33;i++)if(!WmcUpperBodyMath.Finite(solverPoints[i])||!WmcUpperBodyMath.Finite(solverPoints[i].LengthSquared())||!WmcUpperBodyMath.Finite(primaryImage[i])||primaryImage[i].X<0||primaryImage[i].X>1||primaryImage[i].Y<0||primaryImage[i].Y>1||!WmcUpperBodyMath.Finite(WmcUpperBodyMath.FromImage(primaryImage[i],aspect))||!Valid(confidence[i])||confidence[i]<0||confidence[i]>1)return false;
            var measuredWidth=N.Vector3.Distance(solverPoints[11],solverPoints[12]);
            if(!WmcUpperBodyMath.Finite(measuredWidth)||measuredWidth<=0)return false;
            if(!calibratedSample){foreach(var filter in joints)filter.Reset();jointFoundHold.Reset();foreach(var arm in arms)arm.ResetSignal();}
            calibratedSample=true;calibratedClock.NoteAccepted(at);
            for(int i=0;i<33;i++){worldPoints[i]=solverPoints[i];imagePoints[i]=WmcUpperBodyMath.FromImage(primaryImage[i],aspect);visibility[i]=confidence[i];world[i]=new Vector3(solverPoints[i].X,-solverPoints[i].Y,-solverPoints[i].Z);}
            for(int i=0;i<6;i++)JointPresence[i]=confidence[JointIds[i]];
            AnalysisCount++;Ingest(aspect,at,true);return true;
        }
        public bool SubmitCalibratedHand(object owner,GameObject expectedModel,bool avatarLeft,N.Vector3[] points,double at,double now)
        {
            bool Valid(double v)=>!double.IsNaN(v)&&!double.IsInfinity(v);
            if(owner==null||!ReferenceEquals(owner,calibratedOwner)||expectedModel!=avatar||!requested||vrm==null||!calibratedSample||points==null||points.Length!=21||!Valid(at)||!Valid(now)||at!=sampleAt||now<at||now-at>HoldSeconds)return false;
            var arm=avatarLeft?left:right;if(at<=arm.HandAt)return false;
            for(int i=0;i<21;i++)if(!WmcUpperBodyMath.Finite(points[i])||!WmcUpperBodyMath.Finite(points[i].LengthSquared()))return false;
            if(WmcUpperBodyMath.IsCollapsed(points))return false;
            var width=N.Vector3.Distance(filtered[11],filtered[12]);if(!WmcUpperBodyMath.Finite(width)||width<=0)return false;
            for(int i=0;i<21;i++)arm.HandPoints[i]=arm.HandFilters[i].Filter(points[i],at,width,1f);
            arm.HandFound=true;arm.HandAt=at;pendingSolve=true;return true;
        }
        public void ClearCalibratedHand(object owner,GameObject expectedModel,bool avatarLeft)
        {
            if(!ReferenceEquals(owner,calibratedOwner)||expectedModel!=avatar||!calibratedSample)return;
            var arm=avatarLeft?left:right;arm.HandFound=false;arm.HandAt=double.NegativeInfinity;foreach(var filter in arm.HandFilters)filter.Reset();pendingSolve=true;
        }
        public void ClearCalibratedObservation(object owner,GameObject expectedModel){if(ReferenceEquals(owner,calibratedOwner)&&expectedModel==avatar)ClearCalibratedObservation();}
        void ClearCalibratedObservation()
        {
            calibratedClock.ClearCalibrated();
            if(!calibratedSample)return;calibratedSample=false;
            foreach(var filter in joints)filter.Reset();jointFoundHold.Reset();foreach(var arm in arms)arm.ResetSignal();
            sampleAt=double.NegativeInfinity;pendingSolve=false;leftValid=rightValid=torsoValid=false;
            handSolvedAt=double.NegativeInfinity;bodyStarted=twistStarted=false;
            for(int i=0;i<6;i++){JointFound[i]=false;JointVisibility[i]=JointPresence[i]=float.NaN;}
        }
        // Test and replay entry: MediaPipe world landmarks seen by a synthetic camera (image = 0.5 * world + 0.5,
        // aspect 1), every landmark visible. Uses the same path as the camera.
        public void SubmitObservation(Vector3[] landmarks,double receivedAt)
        {
            ClearCalibratedObservation();
            if(landmarks==null||landmarks.Length<33)throw new ArgumentException("33 pose world landmarks required");
            for(int i=0;i<33;i++)
            {
                var p=landmarks[i];world[i]=p;
                imagePoints[i]=WmcUpperBodyMath.FromImage(new N.Vector3(p.x*.5f+.5f,p.y*.5f+.5f,p.z*.5f),1f);
                worldPoints[i]=WmcUpperBodyMath.FromWorld(new N.Vector3(p.x,p.y,p.z));visibility[i]=1f;
            }
            AnalysisCount++;
            Ingest(1f,receivedAt);
        }
        // Builds pseudo-3D points, filters each once, and marks a new body solve. Returns the shoulder
        // width that scales filter speeds for this sample (hands use the same scale).
        float Ingest(float aspect,double receivedAt,bool calibrated=false)
        {
            bool oldLeft=leftValid,oldRight=rightValid,oldTorso=torsoValid;
            if(!calibrated)WmcUpperBodyMath.ForceTorsoVisible(visibility);
            frameAspect=aspect;
            var width=WmcUpperBodyMath.ShoulderWidth(imagePoints[11],imagePoints[12]);
            if(calibrated){Array.Copy(worldPoints,built,33);width=N.Vector3.Distance(built[11],built[12]);}
            else WmcUpperBodyMath.BuildPoints(imagePoints,worldPoints,visibility,built);
            for(int i=0;i<33;i++)filtered[i]=joints[i].Filter(built[i],receivedAt,width,Mathf.Clamp01(visibility[i]));
            for(int i=0;i<6;i++)
            {
                int id=JointIds[i];bool rawGood=WmcUpperBodyMath.InFrame(imagePoints[id],aspect)&&visibility[id]>WmcUpperBodyMath.VisibleThreshold;
                rawJointFound[i]=rawGood;
                bool good=jointFoundHold.Update(i,rawGood,receivedAt);
                JointVisibility[i]=visibility[id];
                if(good!=JointFound[i])TrackingEvent?.Invoke(40+i*2+(good?1:0));JointFound[i]=good;
            }
            // TrunkUp switches the trunk estimate on this flag (hip midpoint vs the
            // shoulder-line estimate, forward depth on/off). A hip hovering on the
            // frame edge flips the raw test at sample rate and every flip moves the
            // torso target, which reads as trembling while the arms -- which never
            // consume this flag -- stay smooth (#1588). The mode therefore follows
            // the raw test only after it has held for HipsModeHoldSeconds.
            bool rawHipsInFrame=WmcUpperBodyMath.InFrame(imagePoints[23],aspect)&&WmcUpperBodyMath.InFrame(imagePoints[24],aspect);
            if(rawHipsInFrame!=hipsRawInFrame){hipsRawInFrame=rawHipsInFrame;hipsRawSince=receivedAt;}
            if(rawHipsInFrame!=hipsInFrame&&receivedAt-hipsRawSince>=HipsModeHoldSeconds)hipsInFrame=rawHipsInFrame;
            leftValid=JointFound[0]&&JointFound[1];rightValid=JointFound[3]&&JointFound[4];torsoValid=JointFound[0]&&JointFound[3];
            armSampleInterval=double.IsNegativeInfinity(sampleAt)?0:receivedAt-sampleAt;
            sampleAt=receivedAt;pendingSolve=true;
            if(requested&&!neutral.Set&&torsoValid)CalibrateNeutral();
            if(oldLeft!=leftValid)TrackingEvent?.Invoke(leftValid?31:30);
            if(oldRight!=rightValid)TrackingEvent?.Invoke(rightValid?33:32);
            if(oldTorso!=torsoValid)TrackingEvent?.Invoke(torsoValid?35:34);
            return width;
        }
        // Worker-safe (called inside Detect under detectorGate). Needs a visible in-frame wrist,
        // the same rule as JointFound.
        void DetectHand(Arm arm,int slot,Detection d,NativeArray<byte> pixels,int width,int height,IList<Mediapipe.Tasks.Components.Containers.NormalizedLandmark> pose,long time)
        {
            var w=pose[arm.Wrist];
            if(!(d.Visibility[arm.Wrist]>WmcUpperBodyMath.VisibleThreshold&&w.x>0&&w.x<1&&w.y>0&&w.y<1))return;
            Vector2 P(int id)=>new Vector2(pose[id].x*width,pose[id].y*height);
            var wrist=P(arm.Wrist);var knuckles=(P(arm.Index)+P(arm.Pinky))*.5f;
            // Square crop around the hand, sized from the hand span and forearm, kept inside the frame.
            var side=Mathf.Max(2.6f*Vector2.Distance(wrist,knuckles),1.1f*Vector2.Distance(wrist,P(arm.Elbow)),48f);
            side=Mathf.Min(side,Mathf.Min(width,height));
            var center=Vector2.Lerp(wrist,knuckles,.6f);
            var x0=Mathf.Clamp(center.x-side*.5f,0,width-side);var y0=Mathf.Clamp(center.y-side*.5f,0,height-side);
            if(!arm.Crop.IsCreated)arm.Crop=new NativeArray<byte>(HandCropSize*HandCropSize*4,Allocator.Persistent);
            var scale=side/HandCropSize;
            for(int y=0;y<HandCropSize;y++)
            {
                int sy=Mathf.Min(height-1,(int)(y0+(y+.5f)*scale));
                for(int x=0;x<HandCropSize;x++)
                {
                    int sx=Mathf.Min(width-1,(int)(x0+(x+.5f)*scale));int from=(sy*width+sx)*4,to=(y*HandCropSize+x)*4;
                    arm.Crop[to]=pixels[from];arm.Crop[to+1]=pixels[from+1];arm.Crop[to+2]=pixels[from+2];arm.Crop[to+3]=pixels[from+3];
                }
            }
            if(arm.Task==null)arm.Task=HandLandmarker.CreateFromOptions(new HandLandmarkerOptions(new BaseOptions(BaseOptions.Delegate.CPU,modelAssetBuffer:handModel),
                runningMode:RunningMode.VIDEO,numHands:1,minHandDetectionConfidence:.5f,minHandPresenceConfidence:.5f,minTrackingConfidence:.5f));
            HandLandmarkerResult hand;
            using(var image=new Image(ImageFormat.Types.Format.Srgba,HandCropSize,HandCropSize,HandCropSize*4,arm.Crop))
                hand=arm.Task.DetectForVideo(image,time);
            if(hand.handWorldLandmarks==null||hand.handWorldLandmarks.Count==0||hand.handLandmarks==null||hand.handLandmarks.Count==0)return;
            var crop=hand.handLandmarks[0].landmarks;var points=hand.handWorldLandmarks[0].landmarks;
            if(crop.Count<21||points.Count<21)return;
            // Reject a hand found elsewhere in the crop (the other hand or a false detection).
            if(Vector2.Distance(new Vector2(x0+crop[0].x*side,y0+crop[0].y*side),wrist)>.6f*side)return;
            for(int i=0;i<21;i++)
            {
                d.HandImage[slot][i]=new N.Vector3((x0+crop[i].x*side)/width,(y0+crop[i].y*side)/height,0);
                d.HandWorld[slot][i]=new N.Vector3(points[i].x,points[i].y,points[i].z);
            }
            d.HandOk[slot]=true;
        }
        void LateUpdate()
        {
            if(!requested||vrm==null)return;
            ApplyPose(Time.unscaledTimeAsDouble,Time.unscaledDeltaTime);
        }
        // New samples are solved once; every frame each bone moves 30% (at 60 fps) toward its target.
        // #1703: Profiler marker naming the per-frame upper-body pose application.
        private static readonly ProfilerMarker ProfileMarker = new ProfilerMarker("LiveVR.Tracking.UpperBody");

        public void ApplyPose(double now,float deltaSeconds)
        {
            using var profileScope = ProfileMarker.Auto();
            if(vrm==null)return;
            RefreshWriteGates();
            bool fresh=now-sampleAt<=HoldSeconds;
            if(calibratedSample)foreach(var arm in arms)if(arm.HandFound&&now-arm.HandAt>HoldSeconds){arm.HandFound=false;pendingSolve=true;}
            if(fresh&&pendingSolve){Solve();pendingSolve=false;}
            var share=WmcUpperBodyMath.FrameFollow(WmcUpperBodyMath.BoneKeep,deltaSeconds);
            foreach(var channel in channels)channel.Follow(fresh?channel.Target:channel.Neutral,share);
        }
        Vector3 W(N.Vector3 v)=>avatar.transform.TransformDirection(new Vector3(v.X,v.Y,v.Z));
        // Torso directions relative to the neutral posture.
        Vector3 Torso(N.Vector3 v)=>correction*W(v);
        // Point a bone's child along a world direction, from the bone's rest rotation.
        static void Aim(Channel bone,Transform child,Vector3 direction){if(child!=null)Aim(bone,child.localPosition,direction);}
        // restDirection: the bone's segment in its own rest frame (the child's local position).
        static void Aim(Channel bone,Vector3 restDirection,Vector3 direction)
        {
            if(bone.Bone==null||restDirection.sqrMagnitude<1e-12f||direction.sqrMagnitude<1e-10f)return;
            bone.ToRest();
            bone.LocalRotation=bone.Rest*Quaternion.FromToRotation(restDirection,bone.Bone.InverseTransformDirection(direction));
        }
        static Quaternion Between(Vector3 from,Vector3 to,Vector3 axis)
        {
            from.Normalize();to.Normalize();var dot=Vector3.Dot(from,to);
            if(dot>.9999985f)return Quaternion.identity;
            if(dot<-.9999985f)return Quaternion.AngleAxis(Vector3.Angle(from,to),axis);
            return Quaternion.FromToRotation(from,to);
        }
        // Hip midpoint to shoulder midpoint. Out-of-frame hips have no usable depth (it jumped 8-36 cm between
        // samples in a seated camera log): forward/back lean stays at the neutral posture. Their image position is
        // a guess too: MediaPipe puts them under the shoulders, so the spine stays upright however far the user
        // bends sideways. Then the side lean comes from the shoulder line's roll, which a bend carries with it.
        N.Vector3 TrunkUp()
        {
            var F=filtered;var spine=(F[11]+F[12])*.5f-(F[23]+F[24])*.5f;
            if(!hipsInFrame)
            {
                // Perpendicular to the shoulder line in the image plane, on the upper side; the spine's length is kept.
                var line=F[11]-F[12];var across=new N.Vector2(line.X,line.Y).Length();
                if(across>1e-4f)
                {
                    var side=line.X<0?-1f:1f;var length=new N.Vector2(spine.X,spine.Y).Length();
                    spine.X=-line.Y/across*side*length;spine.Y=line.X/across*side*length;
                }
            }
            if(!hipsInFrame)spine.Z=neutral.Set&&Math.Abs(neutral.Up.Y)>1e-4f?spine.Y*neutral.Up.Z/neutral.Up.Y:0;
            return WmcUpperBodyMath.SuppressUprightDepth(spine);
        }
        // WMC (cjs) moves the hand to where the camera sees it, relative to the body, then reaches it with IK:
        // arm angles alone put the hand elsewhere when the avatar's arms and shoulders differ from the user's.
        // The seen wrist, relative to the shoulder midpoint and scaled by shoulder width, sets the target across
        // the image. Like WMC, depth is not taken from the detector (seated users' 3D estimate disagrees with the
        // image): only its sign is kept, and its size makes the shoulder-to-hand distance the arm's measured reach.
        // Two-bone IK keeps the elbow on its measured side.
        void ReachSeenHand(Arm arm,ref Vector3 upperDirection,ref Vector3 lowerDirection)
        {
            if(left.Upper.Bone==null||right.Upper.Bone==null)return;
            var F=filtered;
            var seenWidth=W(new N.Vector3(F[11].X-F[12].X,F[11].Y-F[12].Y,0)).magnitude;
            if(seenWidth<1e-4f)return;
            var shoulder=arm.Upper.Bone.position;
            float a=Vector3.Distance(shoulder,arm.Lower.Bone.position),b=Vector3.Distance(arm.Lower.Bone.position,arm.Hand.Bone.position);
            var avatarMid=(left.Upper.Bone.position+right.Upper.Bone.position)*.5f;
            var scale=Vector3.Distance(left.Upper.Bone.position,right.Upper.Bone.position)/seenWidth;
            var mid=(F[11]+F[12])*.5f;
            var across=W(new N.Vector3(F[arm.Wrist].X-mid.X,F[arm.Wrist].Y-mid.Y,0))*scale;
            var forward=avatar.transform.forward;
            var armHand=shoulder+upperDirection.normalized*a+lowerDirection.normalized*b;
            var planar=Vector3.ProjectOnPlane(avatarMid+Vector3.ProjectOnPlane(across,forward)-shoulder,forward);
            var armReach=Mathf.Clamp((armHand-shoulder).magnitude,Mathf.Abs(a-b)+1e-4f,(a+b)*.999f);
            var depth=planar.magnitude<armReach?Mathf.Sqrt(armReach*armReach-planar.sqrMagnitude):0f;
            var toTarget=planar+forward*(Vector3.Dot(armHand-shoulder,forward)<0?-depth:depth);
            var reach=Mathf.Clamp(toTarget.magnitude,Mathf.Abs(a-b)+1e-4f,(a+b)*.999f);
            if(toTarget.sqrMagnitude<1e-8f)return;
            var axis=toTarget.normalized;
            // Elbow on the measured side of the shoulder-target line.
            var bend=Vector3.ProjectOnPlane(upperDirection.normalized*a,axis);
            if(bend.sqrMagnitude<1e-8f)bend=Vector3.ProjectOnPlane(-avatar.transform.up,axis);
            if(bend.sqrMagnitude<1e-8f)return;
            var along=(a*a-b*b+reach*reach)/(2*reach);var height=Mathf.Sqrt(Mathf.Max(0,a*a-along*along));
            var elbow=shoulder+axis*along+bend.normalized*height;
            upperDirection=elbow-shoulder;lowerDirection=shoulder+axis*reach-elbow;
        }
        // Solves on the bones from their rest pose, keeps the result as targets, restores the shown pose.
        void Solve()
        {
            foreach(var channel in channels)channel.ToRest();
            // Time-normalized body follow (Issue #1551): the share follows the
            // actual sample interval, not a fixed per-sample fraction.
            var bodyFollowShare=WmcUpperBodyMath.BodyFollowShare(armSampleInterval);
            var F=filtered;var frame=avatar.transform;
            var trunk=neck!=null&&hips!=null?neck.position-hips.position:frame.up;
            // The neutral posture maps onto the rest trunk and rest shoulder line.
            correction=Quaternion.identity;
            if(neutral.Set)
            {
                var lean=Quaternion.FromToRotation(W(neutral.Up),trunk);
                var restLine=left.Upper.Bone!=null&&right.Upper.Bone!=null?right.Upper.Bone.position-left.Upper.Bone.position:frame.right;
                var line=Vector3.ProjectOnPlane(lean*W(neutral.ShoulderLine),trunk);restLine=Vector3.ProjectOnPlane(restLine,trunk);
                correction=line.sqrMagnitude>1e-10f&&restLine.sqrMagnitude>1e-10f?Between(line,restLine,trunk)*lean:lean;
            }
            // Trunk lean (applied at the spine; this app never writes Hips) and hip-line yaw.
            var up=Torso(TrunkUp()).normalized;
            // Skip only the identity policy (scale 1, unlocked, full range): the default scale is 2
            // since #1078, so equalling the default is no longer a no-op.
            if(!(bodyTilt.LateralScale==1f && !bodyTilt.PitchLocked && bodyTilt.ForwardDegrees==90f && bodyTilt.BackwardDegrees==90f))
            {
                var baseLocal=frame.InverseTransformDirection(trunk);var upLocal=frame.InverseTransformDirection(up);
                var adjusted=BodyTiltPolicy.Trunk(new N.Vector3(baseLocal.x,baseLocal.y,baseLocal.z),new N.Vector3(upLocal.x,upLocal.y,upLocal.z),bodyTilt);
                up=frame.TransformDirection(new Vector3(adjusted.X,adjusted.Y,adjusted.Z)).normalized;
            }
            var body=Quaternion.FromToRotation(trunk,up);
            if(leftLeg!=null&&rightLeg!=null)
            {
                var avatarHips=Vector3.ProjectOnPlane(body*(rightLeg.position-leftLeg.position),up);
                var measuredHips=Vector3.ProjectOnPlane(Torso(F[23]-F[24]),up);
                if(avatarHips.sqrMagnitude>1e-10f&&measuredHips.sqrMagnitude>1e-10f)body=Between(avatarHips,measuredHips,up)*body;
            }
            bodySmoothed=bodyStarted?Quaternion.Slerp(bodySmoothed,body,bodyFollowShare):body;bodyStarted=true;
            if(spine.Bone!=null)spine.WorldRotation=bodySmoothed*spine.WorldRotation;
            // Shoulder-line twist, spread over spine, chest and upper chest.
            if(left.Upper.Bone!=null&&right.Upper.Bone!=null)
            {
                var avatarLine=Vector3.ProjectOnPlane(right.Upper.Bone.position-left.Upper.Bone.position,up);
                var measuredLine=Vector3.ProjectOnPlane(Torso(F[11]-F[12]),up);
                if(avatarLine.sqrMagnitude>1e-10f&&measuredLine.sqrMagnitude>1e-10f)
                {
                    var twist=Between(avatarLine,measuredLine,up);
                    twistSmoothed=twistStarted?Quaternion.Slerp(twistSmoothed,twist,bodyFollowShare):twist;twistStarted=true;
                    twistSmoothed.ToAngleAxis(out var angle,out var axis);
                    if(angle>180)angle-=360;
                    var shares=WmcUpperBodyMath.TwistShares(upperChest.Bone!=null);
                    if(spine.Bone!=null)spine.WorldRotation=Quaternion.AngleAxis(angle*shares.spine,axis)*spine.WorldRotation;
                    if(chest.Bone!=null)chest.WorldRotation=Quaternion.AngleAxis(angle*shares.chest,axis)*chest.WorldRotation;
                    if(upperChest.Bone!=null)upperChest.WorldRotation=Quaternion.AngleAxis(angle*shares.upperChest,axis)*upperChest.WorldRotation;
                }
            }
            // Shoulders follow the measured shoulder line (raise and forward), smoothed per sample.
            var shoulderLine=Torso(F[11]-F[12]).normalized;
            foreach(var arm in arms)
            {
                if(arm.Collar.Bone==null)continue;
                Aim(arm.Collar,arm.Upper.Bone,arm.AvatarLeft?-shoulderLine:shoulderLine);
                var local=arm.Collar.LocalRotation;
                arm.CollarSmoothed=arm.CollarStarted?Quaternion.Slerp(arm.CollarSmoothed,local,bodyFollowShare):local;arm.CollarStarted=true;
                arm.Collar.LocalRotation=arm.CollarSmoothed;
            }
            foreach(var arm in arms){SolveArm(arm);SmoothArm(arm);}
            ApplyHandCalibration();
            foreach(var channel in channels){channel.Target=channel.LocalRotation;channel.LocalRotation=channel.Current;}
            // Unseen parts go back to the relaxed pose; until then Target equals Current, which holds the last pose.
            foreach(var arm in arms)
            {
                var parts=new List<Channel>{arm.Lower,arm.Hand};parts.AddRange(arm.Fingers);
                if(arm.HoldArm&&arm.Upper.Bone!=null)arm.Upper.Target=arm.Upper.Current;
                if(arm.RelaxArm&&arm.Upper.Bone!=null)arm.Upper.Target=arm.Upper.Neutral;
                bool holdHand=!arm.HandFound&&sampleAt-arm.HandAt<=HandPoseHoldSeconds;
                foreach(var part in parts)
                {
                    if(part==null||part.Bone==null)continue;
                    if(arm.HoldArm||arm.HoldForearm||(holdHand&&part!=arm.Lower))part.Target=part.Current;
                    if(arm.RelaxArm||arm.RelaxForearm)part.Target=part.Neutral;
                }
            }
        }
        // The arm channels' extra smoothing stage (Issue #711): ease each channel this
        // sample actually solved toward its kept smoothed rotation, by the share the
        // user's time constant gives for this sample interval. Channels a sample did
        // not solve keep their state, so a reappearing arm resumes from its last
        // smoothed pose instead of having drifted toward the rest pose meanwhile.
        void SmoothArm(Arm arm)
        {
            var share=ArmSmoothingPolicy.FollowShare(armSmoothingSeconds,armSampleInterval);
            if(arm.UpperSolved&&arm.Upper.Bone!=null)arm.Upper.LocalRotation=EmitChannel(ref arm.UpperSmoothed,ref arm.UpperStarted,arm.Upper.LocalRotation,share);
            if(arm.LowerSolved&&arm.Lower.Bone!=null)arm.Lower.LocalRotation=EmitChannel(ref arm.LowerSmoothed,ref arm.LowerStarted,arm.Lower.LocalRotation,share);
            if(arm.HandSolved&&arm.Hand.Bone!=null)arm.Hand.LocalRotation=EmitChannel(ref arm.HandSmoothed,ref arm.HandStarted,arm.Hand.LocalRotation,share);
        }
        // One channel's emitted rotation: ease the kept state toward the
        // solved target by the smoothing share, then pass the candidate
        // through the dead zone (Issue #837). The kept state is the
        // rotation emitted last, so a candidate inside the zone leaves it
        // untouched (the previous output stands), and a candidate outside
        // the zone moves it by the excess over the threshold only, which
        // keeps the pose continuous at the boundary.
        Quaternion EmitChannel(ref Quaternion state,ref bool started,Quaternion target,float share)
        {
            if(!started){state=target;started=true;return target;}
            var candidate=share>=1f?target:Quaternion.Slerp(state,target,share);
            if(armDeadZoneDegrees<=0f){state=candidate;return candidate;}
            var follow=ArmDeadZonePolicy.FollowShare(Quaternion.Angle(state,candidate),armDeadZoneDegrees);
            if(follow<=0f)return state;
            state=follow>=1f?candidate:Quaternion.Slerp(state,candidate,follow);
            return state;
        }
        // seen: start/stop the unseen clock; relax becomes true once the joint stayed unseen for UnseenHoldSeconds.
        void Track(ref double unseenAt,bool seen,out bool relax)
        {
            if(seen){unseenAt=double.NaN;relax=false;return;}
            if(double.IsNaN(unseenAt))unseenAt=sampleAt;
            relax=sampleAt-unseenAt>UnseenHoldSeconds;
        }
        void SolveArm(Arm arm)
        {
            arm.UpperSolved=arm.LowerSolved=arm.HandSolved=false;
            if(arm.Upper.Bone==null||arm.Lower.Bone==null||arm.Hand.Bone==null)return;
            var F=filtered;
            // A joint outside the camera's view has only a guessed position. Seated users keep their arms below the frame, so
            // that guess is followed (the forearm hangs where the arm is). The guess is refused only when it crosses over to the
            // other side of the body, which is how a hidden arm ends up copying the other one. A refused arm is held, then eased
            // back to the relaxed pose.
            var midline=(imagePoints[11].X+imagePoints[12].X)*.5f;var margin=.15f*WmcUpperBodyMath.ShoulderWidth(imagePoints[11],imagePoints[12]);
            bool Crossed(int id)=>arm.AvatarLeft?imagePoints[id].X>midline+margin:imagePoints[id].X<midline-margin;
            bool elbowSeen=(arm.AvatarLeft?rawJointFound[1]:rawJointFound[4])||!Crossed(arm.Elbow);
            bool wristSeen=elbowSeen&&((arm.AvatarLeft?rawJointFound[2]:rawJointFound[5])||!Crossed(arm.Wrist));
            Track(ref arm.UnseenAt,elbowSeen,out arm.RelaxArm);
            Track(ref arm.WristUnseenAt,wristSeen,out arm.RelaxForearm);
            arm.HoldArm=!elbowSeen&&!arm.RelaxArm;arm.HoldForearm=!wristSeen&&!arm.RelaxForearm;
            if(!elbowSeen)return;
            // Track the filtered pose joints even without a dedicated hand detection. The relaxed
            // opening belongs only to the loss/rest pose; it must not push observed arms outward.
            var upperDirection=W(F[arm.Elbow]-F[arm.Shoulder]);
            var lowerDirection=W(F[arm.Wrist]-F[arm.Elbow]);
            // Reuse image-plane IK for low wrists as well: raw 3D depth alone is unreliable while seated.
            if(WmcUpperBodyMath.InFrame(imagePoints[arm.Wrist],frameAspect))ReachSeenHand(arm,ref upperDirection,ref lowerDirection);
            Aim(arm.Upper,arm.Lower.Bone,upperDirection);arm.UpperSolved=true;
            if(!wristSeen)return;
            Aim(arm.Lower,arm.Hand.Bone,lowerDirection);arm.LowerSolved=true;
            if(!arm.HandFound)return;
            if(arm.IndexBone==null||arm.LittleBone==null)return;
            var H=arm.HandPoints;
            // Forearm roll: avatar index->little line vs the measured one, around the forearm axis.
            var axis=arm.Hand.Bone.localPosition;
            var avatarAcross=Vector3.ProjectOnPlane(arm.Lower.Bone.InverseTransformDirection(arm.IndexBone.position-arm.LittleBone.position),axis);
            var measuredAcross=Vector3.ProjectOnPlane(arm.Lower.Bone.InverseTransformDirection(W(H[5]-H[17])),axis);
            if(avatarAcross.sqrMagnitude>1e-10f&&measuredAcross.sqrMagnitude>1e-10f)
            {
                Between(avatarAcross,measuredAcross,axis).ToAngleAxis(out var degrees,out var rollAxis);
                if(Vector3.Dot(rollAxis,axis)<0)degrees=-degrees;
                var split=WmcUpperBodyMath.SplitTwist(degrees,arm.AvatarLeft);
                arm.Upper.LocalRotation*=Quaternion.AngleAxis(split.upper,arm.Lower.Bone.localPosition);
                Aim(arm.Lower,arm.Hand.Bone,lowerDirection);
                arm.Lower.LocalRotation*=Quaternion.AngleAxis(split.lower,axis);
            }
            // Wrist: match the palm normal, then the index direction within the palm.
            var toIndex=W(H[5]-H[0]);
            var toPinky=W(H[17]-H[0]);
            var normal=Vector3.Cross(arm.Hand.Bone.InverseTransformDirection(toIndex),arm.Hand.Bone.InverseTransformDirection(toPinky));
            var restNormal=Vector3.Cross(arm.IndexBone.localPosition,arm.LittleBone.localPosition);
            if(normal.sqrMagnitude<1e-12f||restNormal.sqrMagnitude<1e-12f)return;
            normal.Normalize();
            arm.Hand.LocalRotation=arm.Hand.Rest*Quaternion.FromToRotation(restNormal.normalized,normal);
            var restIndex=Vector3.ProjectOnPlane(arm.IndexBone.localPosition,normal);
            var index=Vector3.ProjectOnPlane(arm.Hand.Bone.InverseTransformDirection(toIndex),normal);
            if(restIndex.sqrMagnitude>1e-10f&&index.sqrMagnitude>1e-10f)arm.Hand.LocalRotation*=Quaternion.FromToRotation(restIndex,index);
            arm.HandSolved=true;
            SolveFingers(arm);
        }
        // WMC (cjs): each finger joint, base to tip, points its segment along the measured hand landmarks
        // (thumb 1-4, index 5-8, middle 9-12, ring 13-16, little 17-20), in the frame its parent already took.
        // The distal segment continues the intermediate one. Without a hand the fingers keep their rest pose.
        void SolveFingers(Arm arm)
        {
            var H=arm.HandPoints;
            for(int finger=0;finger<5;finger++)
                for(int joint=0;joint<3;joint++)
                {
                    var bone=arm.Fingers[finger*3+joint];if(bone.Bone==null)continue;
                    var rest=joint<2&&arm.Fingers[finger*3+joint+1].Bone!=null?arm.Fingers[finger*3+joint+1].Bone.localPosition:bone.Bone.localPosition;
                    int tip=finger*4+joint+2;
                    Aim(bone,rest,W(H[tip]-H[tip-1]));
                }
        }
        void OnDestroy()
        {
            if(face!=null){face.CameraTextureReady-=AnalyzeTexture;face.CameraImageOnWorker-=AnalyzeOnWorker;face.CameraWorkerApplied-=ApplyWorkerDetection;face.CameraSessionReset-=ResetFaceCameraSession;}if(runtime!=null)runtime.AvatarLoaded-=BindAvatar;
            ResetTask();faceDriver?.SetExternalBodyOwner(false);poses?.SetTrackingOwnership(AvatarPoseArms.None);
        }
    }
}
