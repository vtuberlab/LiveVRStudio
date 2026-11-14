// Pure math for the upper-body pipeline, ported from the behavior of Webcam Motion Receiver
// (function-level analysis of its IL2CPP build; re-implemented, no original code copied).
// No Unity types, so it is testable in plain .NET.
using System;
using System.Collections.Generic;
using System.Numerics;

namespace LiveVRStudio.TechnicalValidation
{
    // Speed-adaptive low-pass for points. Differs from the textbook One Euro filter in four ways:
    // speed comes from the filtered previous value, speed is measured on x/y only and divided by a
    // body-size scale, both smoothing factors are multiplied by a per-sample weight, and a sample
    // without newer time is returned unchanged without touching the state.
    public sealed class WmcPointFilter
    {
        public readonly float MinCutoff, Beta, DerivativeCutoff;
        private Vector3 previous, derivative;
        private double previousTime;
        public bool Started { get; private set; }
        public Vector3 Value => previous;

        public WmcPointFilter(float minCutoff, float beta, float derivativeCutoff = 1f)
        { MinCutoff = minCutoff; Beta = beta; DerivativeCutoff = derivativeCutoff; }

        public Vector3 Filter(Vector3 sample, double time, float speedScale, float weight, bool planarSpeed = true)
        {
            if (!WmcUpperBodyMath.Finite(sample) || double.IsNaN(time) || double.IsInfinity(time)) return Started ? previous : sample;
            if (!Started) { previous = sample; derivative = Vector3.Zero; previousTime = time; Started = true; return sample; }
            var dt = (float)(time - previousTime);
            if (!(dt > 0)) return sample;
            var aD = Alpha(dt, DerivativeCutoff) * weight;
            derivative = (sample - previous) / dt * aD + derivative * (1 - aD);
            var speed = planarSpeed ? new Vector2(derivative.X, derivative.Y).Length() : derivative.Length();
            var cutoff = speed / Math.Max(speedScale, .05f) * Beta + MinCutoff;
            var a = Alpha(dt, cutoff) * weight;
            previous = previous * (1 - a) + sample * a;
            previousTime = time;
            return previous;
        }

        public void Reset() { previous = derivative = Vector3.Zero; previousTime = 0; Started = false; }
        private static float Alpha(float dt, float cutoff) => dt / (1f / (cutoff * 2f * (float)Math.PI) + dt);
    }

    // Temporal hold for per-joint detection validity. A joint that was
    // found stays found until it has been continuously missing for the
    // hold window; without this, a landmark hovering on the frame edge
    // or the visibility threshold flips the found flag at frame rate,
    // and every consumer of the flag (arm hold/relax decisions, torso
    // validity, calibration gating) sees the flapping as target jumps.
    // The window matches the face path's detection-loss hold (#1568).
    public sealed class WmcFoundHold
    {
        public const double DefaultHoldSeconds = .35;
        private readonly double[] lastFoundAt;
        private readonly double holdSeconds;

        public WmcFoundHold(int count, double holdSeconds = DefaultHoldSeconds)
        {
            if (count <= 0) throw new ArgumentOutOfRangeException(nameof(count));
            if (!(holdSeconds >= 0)) throw new ArgumentOutOfRangeException(nameof(holdSeconds));
            lastFoundAt = new double[count];
            for (int i = 0; i < count; i++) lastFoundAt[i] = double.NegativeInfinity;
            this.holdSeconds = holdSeconds;
        }

        public bool Update(int index, bool rawFound, double at)
        {
            if (rawFound) { lastFoundAt[index] = at; return true; }
            return at - lastFoundAt[index] <= holdSeconds;
        }

        public void Reset()
        {
            for (int i = 0; i < lastFoundAt.Length; i++) lastFoundAt[i] = double.NegativeInfinity;
        }
    }

    public static class WmcUpperBodyMath
    {
        // Tuning observed in the receiver.
        public const float PoseMinCutoff = .07f, PoseBeta = 1.2f, HandMinCutoff = .07f, HandBeta = 5f;
        public const float BodyFollow = .08f;      // Per-sample slerp toward the newest body rotation.
        // BodyFollow is calibrated at this sample rate. BodyFollowShare converts
        // it to the share for an actual sample interval, so the wall-clock follow
        // speed stays put when the analysis rate drops (Issue #1551): two
        // half-interval steps land exactly where one full step lands.
        public const double BodyFollowReferenceRate = 30d;
        public static float BodyFollowShare(double intervalSeconds)
        {
            if (double.IsNaN(intervalSeconds) || double.IsInfinity(intervalSeconds) || intervalSeconds <= 0d) return BodyFollow;
            var share = 1d - Math.Pow(1d - BodyFollow, intervalSeconds * BodyFollowReferenceRate);
            return share >= 1d ? 1f : (float)share;
        }
        public const float BoneKeep = .7f;         // Per-frame share of the previous bone rotation.
        public const float SpineShare = .7f, ChestShare = .2f, UpperChestShare = .1f;
        public const float TwistDeadZoneDegrees = 20f, ForearmTwistLimitDegrees = 100f;
        public const float VisibleThreshold = .5f;
        // Centroid landmarks: face and arms (0..12) and hips (23, 24).
        public static readonly int[] CentroidIds = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 23, 24 };

        public static bool Finite(Vector3 v) => Finite(v.X) && Finite(v.Y) && Finite(v.Z);
        public static bool Finite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);

        // Image landmark (normalized x, y down, z away) to the working frame (x, y up scaled by the
        // image aspect height/width, z toward camera). World landmarks use the same axes in meters.
        public static Vector3 FromImage(Vector3 p, float aspect) => new Vector3(p.X, -p.Y * aspect, -p.Z);
        public static Vector3 FromWorld(Vector3 p) => new Vector3(p.X, -p.Y, -p.Z);

        // Shoulders and hips always count as visible; the receiver forces them to 1.
        public static void ForceTorsoVisible(float[] visibility)
        { visibility[11] = visibility[12] = visibility[23] = visibility[24] = 1f; }

        // Body-size scale for filter speeds: shoulder distance on the image plane.
        public static float ShoulderWidth(Vector3 image11, Vector3 image12)
            => new Vector2(image11.X - image12.X, image11.Y - image12.Y).Length();

        // Pseudo-3D points: x/y from the image, depth from world landmarks scaled into image units by
        // the visibility-weighted ratio of image to world spread around both centroids.
        public static void BuildPoints(IReadOnlyList<Vector3> image, IReadOnlyList<Vector3> world, IReadOnlyList<float> visibility, Vector3[] output)
        {
            double weights = 0; var c1 = Vector3.Zero; var c2 = Vector3.Zero;
            foreach (var i in CentroidIds) { var w = visibility[i]; weights += w; c1 += image[i] * w; c2 += world[i] * w; }
            if (weights < 1e-6) { for (int i = 0; i < output.Length; i++) output[i] = image[i]; return; }
            c1 /= (float)weights; c2 /= (float)weights;
            double ratio = 0;
            foreach (var i in CentroidIds)
            {
                var a = new Vector2(image[i].X - c1.X, image[i].Y - c1.Y).Length();
                var b = new Vector2(world[i].X - c2.X, world[i].Y - c2.Y).Length();
                if (b > .001f) ratio += a / b * visibility[i];
            }
            var scale = (float)(ratio / weights);
            for (int i = 0; i < output.Length; i++)
                output[i] = new Vector3(image[i].X, image[i].Y, (world[i].Z - c2.Z) * scale + c1.Z);
        }

        // Hand points in the same units: every landmark counts equally.
        public static void BuildHandPoints(IReadOnlyList<Vector3> image, IReadOnlyList<Vector3> world, Vector3[] output)
        {
            var c1 = Vector3.Zero; var c2 = Vector3.Zero; var n = image.Count;
            for (int i = 0; i < n; i++) { c1 += image[i]; c2 += world[i]; }
            c1 /= n; c2 /= n;
            double ratio = 0; int used = 0;
            for (int i = 0; i < n; i++)
            {
                var b = new Vector2(world[i].X - c2.X, world[i].Y - c2.Y).Length();
                if (b > .001f) { ratio += new Vector2(image[i].X - c1.X, image[i].Y - c1.Y).Length() / b; used++; }
            }
            var scale = used > 0 ? (float)(ratio / used) : 0f;
            for (int i = 0; i < n; i++) output[i] = new Vector3(image[i].X, image[i].Y, (world[i].Z - c2.Z) * scale + c1.Z);
        }

        // In-frame test of a working-frame point (y is negative inside the image).
        public static bool InFrame(Vector3 p, float aspect) => p.X > 0 && p.X < 1 && p.Y > -aspect && p.Y < 0;

        // Near-vertical spines lose their noisy forward/back component: the unit spine direction's z is
        // scaled by a logistic weight that is ~0 above y=0.9 and ~1 below it (WMC normalizes first).
        public static Vector3 SuppressUprightDepth(Vector3 spine)
        {
            var up = Vector3.Normalize(spine);
            var e = Math.Exp((up.Y - .9) * -50.0);
            var factor = double.IsInfinity(e) ? 1.0 : e / (e + 1.0);
            return new Vector3(up.X, up.Y, (float)(up.Z * factor));
        }

        // Spine/chest/upper-chest twist shares. Without an upper chest the chest keeps its share.
        public static (float spine, float chest, float upperChest) TwistShares(bool hasUpperChest)
            => hasUpperChest ? (SpineShare, ChestShare, UpperChestShare) : (1f - ChestShare, ChestShare, 0f);

        // Forearm roll in degrees, wrapped per side, split between forearm and upper arm: the first
        // 20 degrees are ignored, the forearm takes up to 100 more, the upper arm takes the rest.
        public static (float upper, float lower) SplitTwist(float degrees, bool avatarLeft)
        {
            if (!Finite(degrees)) return (0, 0);
            float high = avatarLeft ? 210f : 150f, low = avatarLeft ? -150f : -210f;
            if (degrees > high) degrees -= 360f; else if (degrees < low) degrees += 360f;
            var sign = degrees > 0 ? 1f : -1f;
            var magnitude = Math.Abs(degrees);
            if (!(magnitude > TwistDeadZoneDegrees)) return (0, 0);
            var lower = Math.Max(0f, Math.Min(ForearmTwistLimitDegrees, magnitude - TwistDeadZoneDegrees));
            var upper = Math.Max(0f, magnitude - TwistDeadZoneDegrees - lower);
            return (upper * sign, lower * sign);
        }

        // Per-frame share of the new target for a frame of the given length (reference 60 fps).
        public static float FrameFollow(float keepPerFrame, float deltaSeconds)
            => deltaSeconds > 0 && Finite(deltaSeconds) ? 1f - (float)Math.Pow(keepPerFrame, deltaSeconds * 60f) : 0f;

        // A plausible hand spreads in at least two directions; reject results squeezed toward a line.
        public static bool IsCollapsed(IReadOnlyList<Vector3> points)
        {
            var mean = Vector3.Zero;
            foreach (var p in points) mean += p;
            mean /= points.Count;
            double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
            foreach (var p in points)
            {
                var d = p - mean;
                xx += d.X * d.X; xy += d.X * d.Y; xz += d.X * d.Z; yy += d.Y * d.Y; yz += d.Y * d.Z; zz += d.Z * d.Z;
            }
            var n = points.Count;
            var e = SymmetricEigenvalues(xx / n, xy / n, xz / n, yy / n, yz / n, zz / n);
            if (!(e[0] > 1e-4)) return true;
            return e[1] < .02 * e[0];
        }

        // Eigenvalues of a symmetric 3x3 matrix, descending (cyclic Jacobi).
        public static double[] SymmetricEigenvalues(double xx, double xy, double xz, double yy, double yz, double zz)
        {
            var a = new double[3, 3] { { xx, xy, xz }, { xy, yy, yz }, { xz, yz, zz } };
            for (int sweep = 0; sweep < 32; sweep++)
            {
                if (a[0, 1] * a[0, 1] + a[0, 2] * a[0, 2] + a[1, 2] * a[1, 2] < 1e-24) break;
                for (int p = 0; p < 2; p++)
                    for (int q = p + 1; q < 3; q++)
                    {
                        if (Math.Abs(a[p, q]) < 1e-30) continue;
                        var theta = (a[q, q] - a[p, p]) / (2 * a[p, q]);
                        var t = theta == 0 ? 1 : Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1));
                        var c = 1 / Math.Sqrt(t * t + 1); var s = t * c;
                        for (int k = 0; k < 3; k++) { var akp = a[k, p]; var akq = a[k, q]; a[k, p] = c * akp - s * akq; a[k, q] = s * akp + c * akq; }
                        for (int k = 0; k < 3; k++) { var apk = a[p, k]; var aqk = a[q, k]; a[p, k] = c * apk - s * aqk; a[q, k] = s * apk + c * aqk; }
                    }
            }
            var values = new[] { a[0, 0], a[1, 1], a[2, 2] };
            Array.Sort(values); Array.Reverse(values);
            return values;
        }
    }
}
