[Tool, GlobalClass, Icon("res://addons/jigglebones/icon.svg")]
public partial class JiggleBone : Node3D {
    [Export] public bool Enabled { get; set; } = true;
    [Export(PropertyHint.NodePathValidTypes, "Skeleton3D")] public NodePath SkeletonPath { get; set; } = "..";
    [Export] public string? BoneName { get; set; } = null;

    [Export(PropertyHint.Range, "0.1,100,0.1")] public float Stiffness { get; set; } = 1;
    [Export(PropertyHint.Range, "0,100,0.1")] public float Damping { get; set; } = 0;
    [Export] public bool UseGravity { get; set; } = false;
    [Export] public Vector3 Gravity { get; set; } = new(0, -9.81f, 0);
    [Export] public Axis ForwardAxis { get; set; } = Axis.Z_Minus;
    [Export] public CollisionShape3D? CollisionSphere { get; set; } = null;

    private Vector3 PreviousPosition;

    public override void _Ready() {
        TopLevel = true; // Ignore parent transformation
        PreviousPosition = GlobalPosition;
    }
    public override void _PhysicsProcess(double Delta) {
        if (!Enabled || BoneName is null || GetNodeOrNull<Skeleton3D>(SkeletonPath) is not Skeleton3D Skeleton) {
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
            Gravity = (BoneTransformRestWorld.Basis * GetBoneForwardLocal()).Normalized() * 9.81f;
        }
        Vector3 Velocity = (GlobalTransform.Origin - PreviousPosition) / (float)Delta;

        Gravity *= Stiffness;
        Velocity += Gravity;
        Velocity -= Velocity * Damping * (float)Delta;

        PreviousPosition = GlobalTransform.Origin;
        GlobalPosition += Velocity * (float)Delta;

        //======= Solve distance constraint =======\\

        Vector3 GoalPosition = Skeleton.ToGlobal(BoneTransformObject.Origin);
        GlobalPosition = GoalPosition + (GlobalPosition - GoalPosition).Normalized();

        // If bone is inside the collision sphere, push it out
        if (IsInstanceValid(CollisionSphere)) {
            Vector3 TestVector = GlobalPosition - CollisionSphere.GlobalPosition;
            float Distance = TestVector.Length() - ((SphereShape3D)CollisionSphere.Shape).Radius;
            if (Distance < 0) {
                GlobalPosition -= TestVector.Normalized() * Distance;
            }
        }

        //======= Rotate the bone to point to this object =======\\

        Vector3 DiffVectorLocal = (BoneTransformWorld.AffineInverse() * GlobalPosition).Normalized();
        Vector3 BoneForwardLocal = GetBoneForwardLocal();

        // The axis+angle to rotate on, in local-to-bone space
        Vector3 BoneRotateAxis = BoneForwardLocal.Cross(DiffVectorLocal);
        float BoneRotateAngle = MathF.Acos(BoneForwardLocal.Dot(DiffVectorLocal));

        // Already aligned, no need to rotate
        if (BoneRotateAxis.IsZeroApprox()) {
            return;
        }

        BoneRotateAxis = BoneRotateAxis.Normalized();

        // Bring the axis to object space, WITHOUT position (so only the BASIS is used) since vectors shouldn't be translated
        Vector3 BoneRotateAxisObject = (BoneTransformObject.Basis * BoneRotateAxis).Normalized();
        Transform3D BoneNewTransformObject = new(BoneTransformObject.Basis.Rotated(BoneRotateAxisObject, BoneRotateAngle), BoneTransformObject.Origin);

#pragma warning disable CS0618
        Skeleton.SetBoneGlobalPoseOverride(BoneId, BoneNewTransformObject, amount: 0.5f, persistent: true);
#pragma warning restore

        // Orient this object to the jigglebone
        GlobalBasis = (Skeleton.GlobalTransform * Skeleton.GetBoneGlobalPose(BoneId)).Basis;
    }

    private Vector3 GetBoneForwardLocal() {
        return ForwardAxis switch {
            Axis.X_Plus => Vector3.Right,
            Axis.Y_Plus => Vector3.Up,
            Axis.Z_Plus => Vector3.Back,
            Axis.X_Minus => Vector3.Left,
            Axis.Y_Minus => Vector3.Down,
            Axis.Z_Minus or _ => Vector3.Forward,
        };
    }

    public enum Axis {
        X_Plus,
        Y_Plus,
        Z_Plus,
        X_Minus,
        Y_Minus,
        Z_Minus,
    }
}