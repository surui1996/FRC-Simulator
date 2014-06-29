using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Simulator.Animation3D;
using Microsoft.Xna.Framework.Input;

namespace Simulator.PhysicalModeling
{
    class Robot : DynamicObject
    {
        //m.k.s., degrees

        public float GyroAngle { get; set; }
        public float EncoderRight { get; set; }
        public float EncoderLeft { get; set; }

        public float RightOutput { get; set; }
        public float LeftOutput { get; set; }

        public float Orientation { get; set; }
        public float CameraOrientation { get; set; }

        public float AngularVelocity { get; set; }

        private Vector3 cornerFrontLeft;
        private Vector3 cornerFrontRight;
        private Vector3 cornerRearLeft;
        private Vector3 cornerRearRight;

        private float mapHeight;

        private const float MAXIMUM_VELOCITY = 3f; //m/s
        private const float CHASSIS_WIDTH = 0.5f; //m
        private const float CHASSIS_LENGTH = CHASSIS_WIDTH * 2; //m
        private const float WHEEL_RADIUS = 0.15f; //m

        private WheeledBox wheeledBox;

        private static Vector3 CameraRelativePosition = Vector3.Zero;
        private static Vector3 CameraStartingOrientation = new Vector3(0, 0, 1);

        public Robot(Vector3 position3D, float mapMetersToPixel, float mapHeight,
            Texture2D body, Texture2D wheelSide, Texture2D wheelCircumference,
            Texture2D robot2D)
        {
            this.initialPositionOn3D = position3D;
            this.Position = position3D / FieldConstants.PIXELS_IN_ONE_METER;
            this.mapMetersToPixel = mapMetersToPixel;
            this.mapHeight = mapHeight;

            this.textureOnMap = robot2D;

            GyroAngle = 0;
            EncoderRight = 0;
            EncoderLeft = 0;

            RightOutput = 0;
            LeftOutput = 0;

            Orientation = MathHelper.ToRadians(0);
            CameraOrientation = MathHelper.ToRadians(0);
            CameraRelativePosition = Vector3.UnitY * FieldConstants.HEIGHT_ABOVE_CARPET / 3 * FieldConstants.C;

            float c = FieldConstants.PIXELS_IN_ONE_METER;
            wheeledBox = new WheeledBox(body, wheelSide, wheelCircumference, CHASSIS_LENGTH * c,
                CHASSIS_WIDTH * c, WHEEL_RADIUS * c);
            
        }

        public void Update(float dt) //dt = timeSinceLastUpdate
        {
            float vL = LeftOutput * MAXIMUM_VELOCITY;
            float vR = RightOutput * MAXIMUM_VELOCITY;

            float velocity = (vR + vL) / 2;
            float angularVelocity = (vR - vL) / CHASSIS_WIDTH;
            angularVelocity /= 6f; //because of friction
            
            Velocity = Vector3.Transform(Vector3.UnitZ * velocity, Matrix.CreateRotationY(Orientation));
            AngularVelocity = angularVelocity;
          
            cornerFrontLeft = Position + Vector3.Transform(new Vector3(CHASSIS_WIDTH / 2, 0, CHASSIS_LENGTH / 2),
                Matrix.CreateRotationY(Orientation));
            cornerRearLeft = Position + Vector3.Transform(new Vector3(CHASSIS_WIDTH / 2, 0, -CHASSIS_LENGTH / 2),
                Matrix.CreateRotationY(Orientation));
            cornerFrontRight = Position + Vector3.Transform(new Vector3(-CHASSIS_WIDTH / 2, 0, CHASSIS_LENGTH / 2),
                Matrix.CreateRotationY(Orientation));
            cornerRearRight = Position + Vector3.Transform(new Vector3(-CHASSIS_WIDTH / 2, 0, -CHASSIS_LENGTH / 2),
                Matrix.CreateRotationY(Orientation));

            GyroAngle += MathHelper.ToDegrees(angularVelocity * dt);
            EncoderLeft += vL * dt;
            EncoderRight += vR * dt;

            for (float t = 0; t < dt; t += 0.001f)
                Field.UpdateWallRobotInteraction(0.001f, this);

            Position += Velocity * dt;
            Orientation += AngularVelocity * dt;
        }

        public Matrix GetCameraView()
        {
            Vector3 cameraPosition = Position * FieldConstants.PIXELS_IN_ONE_METER + CameraRelativePosition;

            Matrix rotationMatrix = Matrix.CreateRotationX(CameraOrientation)
                * Matrix.CreateRotationY(Orientation);

            // Create a vector pointing the direction the camera is facing.
            Vector3 cameraDirection = Vector3.Transform(CameraStartingOrientation, rotationMatrix);

            // Calculate the position the camera is looking at.
            Vector3 cameraLookat = cameraPosition + cameraDirection;

            // Set up the view matrix and projection matrix.
            return Matrix.CreateLookAt(cameraPosition, cameraLookat, Vector3.Up);
        }

        public BoundingBox GetBoundingBox()
        {
            List<Vector3> corners = GetCorners();
            float minX = corners[0].X, minZ = corners[0].Z;
            float maxX = minX, maxZ = minZ;
            for (int i = 1; i < 4; i++)
            {
                if (corners[i].X < minX)
                    minX = corners[i].X;
                if (corners[i].Z < minZ)
                    minZ = corners[i].Z;

                if (corners[i].X > maxX)
                    maxX = corners[i].X;
                if (corners[i].Z > maxZ)
                    maxZ = corners[i].Z;
            }

            Vector3 min = new Vector3(minX, WHEEL_RADIUS * 0.5f, minZ) * FieldConstants.PIXELS_IN_ONE_METER;
            Vector3 max = new Vector3(maxX, WHEEL_RADIUS * 1.5f, maxZ) * FieldConstants.PIXELS_IN_ONE_METER;

            return new BoundingBox(min, max);
        }

        public BoundingSphere GetBoundingSphere()
        {
            Vector3 centerPosition = Position * FieldConstants.PIXELS_IN_ONE_METER;

            return new BoundingSphere(centerPosition + Vector3.UnitY * WHEEL_RADIUS * FieldConstants.PIXELS_IN_ONE_METER,
                CHASSIS_LENGTH * FieldConstants.PIXELS_IN_ONE_METER / 2f);
        }

        public List<Vector3> GetCorners()
        {
            List<Vector3> l = new List<Vector3>();
            l.Add(cornerFrontLeft);
            l.Add(cornerFrontRight);
            l.Add(cornerRearLeft);
            l.Add(cornerRearRight);
            return l;
        }

        public override void Reset()
        {
            GyroAngle = 0;
            EncoderRight = 0;
            EncoderLeft = 0;

            RightOutput = 0;
            LeftOutput = 0;

            Position = initialPositionOn3D / FieldConstants.PIXELS_IN_ONE_METER;

            Orientation = MathHelper.ToRadians(0);
            CameraOrientation = MathHelper.ToRadians(0);
            CameraRelativePosition = Vector3.UnitY * FieldConstants.HEIGHT_ABOVE_CARPET / 3 * FieldConstants.C;
        }

        public void ResetGyro()
        {
            GyroAngle = 0;
        }

        public void ResetEncoders()
        {
            EncoderLeft = 0;
            EncoderRight = 0;
        }

        private float Limit(float value)
        {
            if (value > 1f) value = 1f;
            else if (value < -1f) value = -1f;
            return value;
        }

        public void ArcadeDrive(float forward, float curve)
        {
            float leftMotorOutput;
            float rightMotorOutput;

            forward = Limit(forward);
            curve = -Limit(curve);


            if (forward > 0.0)
            {
                if (curve > 0.0)
                {
                    leftMotorOutput = forward - curve;
                    rightMotorOutput = Math.Max(forward, curve);
                }
                else
                {
                    leftMotorOutput = Math.Max(forward, -curve);
                    rightMotorOutput = forward + curve;
                }
            }
            else
            {
                if (curve > 0.0)
                {
                    leftMotorOutput = -Math.Max(-forward, curve);
                    rightMotorOutput = forward + curve;
                }
                else
                {
                    leftMotorOutput = forward - curve;
                    rightMotorOutput = -Math.Max(-forward, -curve);
                }
            }
            SetOutputs(leftMotorOutput, rightMotorOutput);

        }

        public void TankDrive(float left, float right)
        {
            SetOutputs(Limit(left), Limit(right));
        }

        public void KeyboardDrive(KeyboardState state)
        {
            if (GamePad.GetState(PlayerIndex.One).ThumbSticks.Left.Y != 0
                || GamePad.GetState(PlayerIndex.One).ThumbSticks.Right.Y != 0)
                return;

            float move = 0, rotate = 0;

            if (state.IsKeyDown(Keys.Left))
                rotate = -1;
            else if (state.IsKeyDown(Keys.Right))
                rotate = 1;
            else
                rotate = 0;

            if (state.IsKeyDown(Keys.Up))
                move = 1;
            else if (state.IsKeyDown(Keys.Down))
                move = -1;
            else
                move = 0;

            ArcadeDrive(move, rotate);
        }

        private void SetOutputs(float left, float right)
        {
            LeftOutput = left;
            RightOutput = right;
        }

        //draw must be with map viewport
        public override void DrawOnMap(SpriteBatch spriteBatch)
        {
            Vector3 mapVector = Position * mapMetersToPixel;
            spriteBatch.Draw(textureOnMap, new Vector2(mapVector.Z, spriteBatch.GraphicsDevice.Viewport.Height - mapVector.X),
                null, Color.White, -this.Orientation, new Vector2(50, 25), (mapMetersToPixel) / 100,
                SpriteEffects.None, 0);
        }

        public override void Draw(GraphicsDevice device, BasicEffect effect, BasicEffect lighting)
        {
            Matrix oldWorld = effect.World;
            Matrix oldLighting = lighting.World;

            effect.World = Matrix.CreateTranslation(Position * FieldConstants.PIXELS_IN_ONE_METER) * effect.World;
            lighting.World = Matrix.CreateTranslation(Position * FieldConstants.PIXELS_IN_ONE_METER) * lighting.World;
            
            wheeledBox.Draw(device, effect, lighting, Orientation);

            effect.World = oldWorld;
            lighting.World = oldLighting;
        }
    }
}
