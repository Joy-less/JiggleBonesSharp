namespace Godot;

[Tool, GlobalClass, Icon("res://addons/jigglebones/icon.svg")]
public partial class JiggleBone : Node3D {
    [Export] public bool Enabled { get; set; } = true;
    [Export(PropertyHint.NodePathValidTypes, "Skeleton3D")] public NodePath SkeletonPath { get; set; } = "..";
    [Export] public string? BoneName { get; set; } = null;
    [Export(PropertyHint.Range, "0.1,100,0.1")] public float Stiffness { get; set; } = 1;
    [Export(PropertyHint.Range, "0,100,0.1")] public float Damping { get; set; } = 0;
    [Export] public bool UseGravity { get; set; } = false;
    [Export] public Vector3 Gravity { get; set; } = new(0, -9.81f, 0);
    [Export] public Vector3 MaxDegrees { get; set; } = new(360, 360, 360);
    [Export] public Vector3 MinDegrees { get; set; } = new(-360, -360, -360);
    [Export] public JiggleBoneAxis ForwardAxis { get; set; } = JiggleBoneAxis.Z_Minus;

    private Vector3 PreviousPosition;

    public override void _Ready() {
        TopLevel = true; // Ignore parent transformation
        PreviousPosition = GlobalPosition;
    }
    public override void _PhysicsProcess(double Delta) {
        if (!Enabled) {
            return;
        }

        if (BoneName is null || GetNodeOrNull<Skeleton3D>(SkeletonPath) is not Skeleton3D Skeleton) {
            return;
        }
        int BoneId = Skeleton.FindBone(BoneName);

        Transform3D BoneTransformObject = Skeleton.GetBoneGlobalPose(BoneId);
        Transform3D BoneTransformWorld = Skeleton.GlobalTransform * BoneTransformObject;

        Transform3D BoneTransformRestObject = Skeleton.GetBoneGlobalRest(BoneId);
        Transform3D BoneTransformRestWorld = Skeleton.GlobalTransform * BoneTransformRestObject;

        //======= Integrate velocity (Verlet integration) =======\\

        // If not using gravity, apply force in the direction of the bone (so it always wants to point "forward")
        Vector3 Gravity = this.Gravity;
        if (!UseGravity) {
            Gravity = (BoneTransformRestWorld.Basis * ForwardAxis.ToVector()).Normalized() * 9.81f;
        }
        Vector3 Velocity = (GlobalTransform.Origin - PreviousPosition) / (float)Delta;

        Gravity *= Stiffness;
        Velocity += Gravity;
        Velocity -= Velocity * Damping * (float)Delta;

        PreviousPosition = GlobalTransform.Origin;
        GlobalPosition += Velocity * (float)Delta;

        //======= Solve distance constraint =======\\

        Vector3 GoalPosition = Skeleton.ToGlobal(Skeleton.GetBoneGlobalPose(BoneId).Origin);
        GlobalPosition = GoalPosition + (GlobalPosition - GoalPosition).Normalized();

        //======= Rotate the bone to point to this object =======\\

        Vector3 BoneForwardLocal = ForwardAxis.ToVector();
        Vector3 DiffVectorLocal = (BoneTransformWorld.AffineInverse() * GlobalPosition).Normalized();

        // The axis+angle to rotate on, in local-to-bone space
        Vector3 BoneRotateAxis = BoneForwardLocal.Cross(DiffVectorLocal);
        float BoneRotateAngle = Mathf.Acos(BoneForwardLocal.Dot(DiffVectorLocal));

        // Min/max rotation degrees constraint
        Vector3 RotatedAxisContribution = (BoneRotateAxis * BoneRotateAngle).Clamp(MinDegrees.DegToRad(), MaxDegrees.DegToRad());
        BoneRotateAxis = RotatedAxisContribution.Normalized();
        BoneRotateAngle = RotatedAxisContribution.Length();

        // Already aligned, no need to rotate
        if (BoneRotateAxis.IsZeroApprox()) {
            return;
        }

        // Bring the axis to object space, WITHOUT position (so only the BASIS is used) since vectors shouldn't be translated
        Vector3 BoneRotateAxisObject = (BoneTransformObject.Basis * BoneRotateAxis).Normalized();
        Transform3D BoneNewTransformObject = new(BoneTransformObject.Basis.Rotated(BoneRotateAxisObject, BoneRotateAngle), BoneTransformObject.Origin);

#pragma warning disable CS0618
        Skeleton.SetBoneGlobalPoseOverride(BoneId, BoneNewTransformObject, amount: 0.5f, persistent: true);
#pragma warning restore

        // Orient this object to the jigglebone
        GlobalBasis = (Skeleton.GlobalTransform * Skeleton.GetBoneGlobalPose(BoneId)).Basis;
    }
}

public enum JiggleBoneAxis {
    X_Plus,
    Y_Plus,
    Z_Plus,
    X_Minus,
    Y_Minus,
    Z_Minus,
}

public static class JiggleBoneExtensions {
    public static Vector3 ToVector(this JiggleBoneAxis Axis) {
        return Axis switch {
            JiggleBoneAxis.X_Plus => Vector3.Right,
            JiggleBoneAxis.Y_Plus => Vector3.Up,
            JiggleBoneAxis.Z_Plus => Vector3.Back,
            JiggleBoneAxis.X_Minus => Vector3.Left,
            JiggleBoneAxis.Y_Minus => Vector3.Down,
            JiggleBoneAxis.Z_Minus or _ => Vector3.Forward,
        };
    }
    public static Vector3 DegToRad(this Vector3 Degrees) {
        return new Vector3(Mathf.DegToRad(Degrees.X), Mathf.DegToRad(Degrees.Y), Mathf.DegToRad(Degrees.Z));
    }
    public static Vector3 RadToDeg(this Vector3 Radians) {
        return new Vector3(Mathf.RadToDeg(Radians.X), Mathf.RadToDeg(Radians.Y), Mathf.RadToDeg(Radians.Z));
    }
}