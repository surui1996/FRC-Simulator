﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Content;
using Simulator.PhysicalModeling;

namespace Simulator.Animation3D
{
    class Field
    {
        Box wallRed, wallBlue, carpet3, boundryLeft, boundryRight;
        Box lowGoal, truss, trussVertical;
        static List<Wall> walls;
        static BoundingBox trussBox;

        public Field(ContentManager content)
        {
            wallRed = new Box(content.Load<Texture2D>("visionRed"), FieldConstants.WIDTH, FieldConstants.HEIGHT_ABOVE_CARPET, 0.1f,
               new Vector3(0, 0, FieldConstants.HEIGHT), FieldConstants.C);

            wallBlue = new Box(content.Load<Texture2D>("wallBlue"), FieldConstants.WIDTH, FieldConstants.HEIGHT_ABOVE_CARPET, 0.1f,
                Vector3.Zero, FieldConstants.C);

            boundryRight = new Box(content.Load<Texture2D>("boundryGrey"), FieldConstants.WIDTH / 50,
                FieldConstants.HEIGHT_ABOVE_CARPET / 7, FieldConstants.HEIGHT,
                -Vector3.UnitX * FieldConstants.WIDTH / 50, FieldConstants.C);

            boundryLeft = new Box(content.Load<Texture2D>("boundryGrey"), FieldConstants.WIDTH / 50,
               FieldConstants.HEIGHT_ABOVE_CARPET / 7, FieldConstants.HEIGHT,
               Vector3.UnitX * FieldConstants.WIDTH, FieldConstants.C);

            carpet3 = new Box(content.Load<Texture2D>("carpet3D"), FieldConstants.WIDTH, 0.01f / FieldConstants.C, FieldConstants.HEIGHT,
                Vector3.Zero, FieldConstants.C);

            float a = FieldConstants.LOWGOAL_WIDTH;


            lowGoal = new Box(content.Load<Texture2D>("goalFrame"), a, a, a,
               Vector3.Zero, FieldConstants.C);

            truss = new Box(content.Load<Texture2D>("truss"), FieldConstants.WIDTH * 1.5f, FieldConstants.TRUSS_SQUARE_EDGE, FieldConstants.TRUSS_SQUARE_EDGE,
               new Vector3(-0.25f * FieldConstants.WIDTH, FieldConstants.TRUSS_HEIGHT_ABOVE_CARPET, FieldConstants.TRUSS_Z), FieldConstants.C);

            trussBox = new BoundingBox(new Vector3(0, FieldConstants.TRUSS_HEIGHT_ABOVE_CARPET, FieldConstants.TRUSS_Z) * FieldConstants.C,
                new Vector3(FieldConstants.WIDTH, FieldConstants.TRUSS_HEIGHT_ABOVE_CARPET + FieldConstants.TRUSS_SQUARE_EDGE, FieldConstants.TRUSS_Z + FieldConstants.TRUSS_SQUARE_EDGE) * FieldConstants.C);

            trussVertical = new Box(content.Load<Texture2D>("trussVertical"), FieldConstants.TRUSS_SQUARE_EDGE, FieldConstants.TRUSS_HEIGHT_ABOVE_CARPET, FieldConstants.TRUSS_SQUARE_EDGE,
              new Vector3(-0.25f * FieldConstants.WIDTH, 0, FieldConstants.HEIGHT / 2 - 0.5f), FieldConstants.C);


            walls = new List<Wall>();
            walls.Add(new Wall(Axis.X, Direction.NegativeDirection, 0));
            walls.Add(new Wall(Axis.X, Direction.PositiveDirection, FieldConstants.WIDTH_IN_METERS));
            walls.Add(new Wall(Axis.Z, Direction.NegativeDirection, 0));
            walls.Add(new Wall(Axis.Z, Direction.PositiveDirection, FieldConstants.HEIGHT_IN_METERS));
        }

        public void Draw(GraphicsDevice graphicsDevice, BasicEffect effect)
        {
            wallRed.Draw(graphicsDevice, effect);
            wallBlue.Draw(graphicsDevice, effect);
            boundryRight.Draw(graphicsDevice, effect);
            boundryLeft.Draw(graphicsDevice, effect);
            carpet3.Draw(graphicsDevice, effect);

            Matrix oldWorld = effect.World;
            lowGoal.Draw(graphicsDevice, effect);
            effect.World = Matrix.CreateTranslation(Vector3.UnitZ * ((FieldConstants.HEIGHT - FieldConstants.LOWGOAL_WIDTH ) * FieldConstants.C - 1f)) * effect.World;
            lowGoal.Draw(graphicsDevice, effect);
            effect.World = oldWorld;

            effect.World = Matrix.CreateTranslation(Vector3.UnitX * (FieldConstants.WIDTH - FieldConstants.LOWGOAL_WIDTH) * FieldConstants.C) * effect.World;
            lowGoal.Draw(graphicsDevice, effect);
            effect.World = Matrix.CreateTranslation(Vector3.UnitZ * ((FieldConstants.HEIGHT - FieldConstants.LOWGOAL_WIDTH)* FieldConstants.C - 1f)) * effect.World;
            lowGoal.Draw(graphicsDevice, effect);
            effect.World = oldWorld;

            truss.Draw(graphicsDevice, effect);
            trussVertical.Draw(graphicsDevice, effect);

            oldWorld = effect.World;
            effect.World = Matrix.CreateTranslation(Vector3.UnitX * (FieldConstants.WIDTH * 1.5f - FieldConstants.TRUSS_SQUARE_EDGE) * FieldConstants.C) * effect.World;
            trussVertical.Draw(graphicsDevice, effect);
            effect.World = oldWorld;
        }

        public static void UpdateWallRobotInteraction(float dt, Robot r)
        {
            foreach (Wall wall in walls)
            {
                wall.Interact(dt, r);
            }
        }

        static bool[] lastInteraction = new bool[2];
        public static void UpdateTrussBallInteraction(float dt, GameBall ball)
        {
            bool intersectionY = false, intersectionZ = false;
            BoundingSphere s = ball.GetBoundingSphere();
            float c = FieldConstants.C;

            if (s.Intersects(trussBox))
            {
                if ((s.Center.Y < FieldConstants.TRUSS_HEIGHT_ABOVE_CARPET * c
                    && s.Center.Y + s.Radius > FieldConstants.TRUSS_HEIGHT_ABOVE_CARPET * c)
                    || (s.Center.Y > (FieldConstants.TRUSS_HEIGHT_ABOVE_CARPET + FieldConstants.TRUSS_SQUARE_EDGE) * c
                    && s.Center.Y - s.Radius < (FieldConstants.TRUSS_HEIGHT_ABOVE_CARPET + FieldConstants.TRUSS_SQUARE_EDGE) * c))
                    intersectionY = true;

                if ((s.Center.Z < FieldConstants.TRUSS_Z * c
                    && s.Center.Z + s.Radius > FieldConstants.TRUSS_Z * c)
                    || (s.Center.Z > (FieldConstants.TRUSS_Z + FieldConstants.TRUSS_SQUARE_EDGE) * c
                    && s.Center.Z - s.Radius < (FieldConstants.TRUSS_Z + FieldConstants.TRUSS_SQUARE_EDGE) * c))
                    intersectionZ = true;

                if (lastInteraction[0] == intersectionY && lastInteraction[1] == intersectionZ)
                    return;

                if (intersectionZ && intersectionY)
                    ball.Velocity -= 1.8f * ball.Velocity;
                else if (intersectionY)
                    ball.Velocity -= new Vector3(ball.Velocity.X / 10,
                        1.7f * ball.Velocity.Y, ball.Velocity.Z / 10);
                else if (intersectionZ)
                    ball.Velocity -= new Vector3(ball.Velocity.X / 10,
                        ball.Velocity.Y / 10, 1.8f * ball.Velocity.Z);  
            }

            lastInteraction[0] = intersectionY; lastInteraction[1] = intersectionZ;
        }

        static bool lastUpdateScored = false;
        public static bool IsTrussScored(GameBall ball)
        {
            bool score = false;
            BoundingSphere s = ball.GetBoundingSphere();
            float c = FieldConstants.C;
            if (s.Center.Y > FieldConstants.TRUSS_HEIGHT_ABOVE_CARPET * c
                && s.Center.Z > (FieldConstants.TRUSS_Z + (FieldConstants.TRUSS_SQUARE_EDGE * 0.2f)) * c
                && s.Center.Z < (FieldConstants.TRUSS_Z + (FieldConstants.TRUSS_SQUARE_EDGE * 0.8f)) * c)
                score = true;

            if (score && !lastUpdateScored)
            {
                lastUpdateScored = true;
                return true;
            }
            else if (!score && lastUpdateScored)
                lastUpdateScored = false;
            
            return false;
        }
    }
}
