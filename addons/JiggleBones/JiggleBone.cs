namespace Godot;

[Tool, GlobalClass, Icon("res://addons/jigglebones/icon.svg")]
public partial class JiggleBone : SkeletonModifier3D {
    [Export] public string? BoneName { get; set; } = null;
    [Export(PropertyHint.Range, "0,10")] public float Stiffness { get; set; } = 1;
    [Export(PropertyHint.Range, "0,10")] public float Damping { get; set; } = 0;
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
    public override void _ProcessModification() {
        // Get bone ID from name
        if (BoneName is null || GetSkeleton() is not Skeleton3D Skeleton) {
            return;
        }
        int BoneId = Skeleton.FindBone(BoneName);

        // Get seconds since last frame
        double Delta = Skeleton.ModifierCallbackModeProcess switch {
            Skeleton3D.ModifierCallbackModeProcessEnum.Physics => GetPhysicsProcessDeltaTime(),
            Skeleton3D.ModifierCallbackModeProcessEnum.Idle => GetProcessDeltaTime(),
            _ => throw new NotImplementedException()
        };

        // Get bone pose transform
        Transform3D BoneTransformObject = Skeleton.GetBoneGlobalPose(BoneId);
        Transform3D BoneTransformWorld = Skeleton.GlobalTransform * BoneTransformObject;

        // Get bone rest transform
        Transform3D BoneTransformRestObject = Skeleton.GetBoneGlobalRest(BoneId);
        Transform3D BoneTransformRestWorld = Skeleton.GlobalTransform * BoneTransformRestObject;

        //======= Integrate velocity (Verlet integration) =======\\

        // If not using gravity, apply force in the direction of the bone (so it always wants to point "forward")
        Vector3 Gravity = UseGravity
            ? this.Gravity
            : (BoneTransformRestWorld.Basis * ForwardAxis.ToVector()).Normalized() * 9.81f;
        Vector3 Velocity = (GlobalPosition - PreviousPosition) / (float)Delta;

        Gravity *= Stiffness;
        Velocity += Gravity;
        Velocity -= Velocity * Damping * (float)Delta;

        PreviousPosition = GlobalPosition;
        GlobalPosition += Velocity * (float)Delta;

        //======= Solve distance constraint =======\\

        Vector3 GoalPosition = BoneTransformWorld.Origin;
        GlobalPosition = GoalPosition + (GlobalPosition - GoalPosition).Normalized();

        //======= Rotate the bone to point to this object =======\\

        Vector3 BoneForwardLocal = ForwardAxis.ToVector();
        Vector3 DiffVectorLocal = (BoneTransformWorld.AffineInverse() * GlobalPosition).Normalized();

        // The axis+angle to rotate on, in local-to-bone space
        Vector3 BoneRotateAxis = BoneForwardLocal.Cross(DiffVectorLocal);

        // Min/max rotation degrees constraint
        BoneRotateAxis = BoneRotateAxis.Clamp(MinDegrees.DegToRad(), MaxDegrees.DegToRad());
        float BoneRotateAngle = BoneRotateAxis.Length();
        BoneRotateAxis = BoneRotateAxis.Normalized();

        // Already aligned, no need to rotate
        if (Mathf.IsZeroApprox(BoneRotateAngle)) {
            return;
        }

        // Bring the axis to object space, WITHOUT position (so only the BASIS is used) since vectors shouldn't be translated
        Vector3 BoneRotateAxisObject = (BoneTransformObject.Basis * BoneRotateAxis).Normalized();
        Transform3D BoneNewTransformObject = new(BoneTransformObject.Basis.Rotated(BoneRotateAxisObject, BoneRotateAngle), BoneTransformObject.Origin);

        Skeleton.SetBoneGlobalPose(BoneId, BoneNewTransformObject);

        // Orient this object to the jigglebone
        GlobalBasis = (Skeleton.GlobalTransform * Skeleton.GetBoneGlobalPose(BoneId)).Basis;
    }
    public override void _ValidateProperty(Collections.Dictionary Property) {
        if (((string)Property["name"]) == PropertyName.BoneName.ToString()) {
            if (GetSkeleton() is Skeleton3D Skeleton) {
                Property["hint"] = (long)PropertyHint.Enum;
                Property["hint_string"] = Skeleton.GetConcatenatedBoneNames();
            }
        }
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