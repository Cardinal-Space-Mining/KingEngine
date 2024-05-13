// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.geometry;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.geometry.proto.Pose3dProto;
import edu.wpi.first.math.geometry.struct.Pose3dStruct;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

/** Represents a 3D pose containing translational and rotational elements. */
@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public class Pose3d implements Interpolatable<Pose3d>, ProtobufSerializable, StructSerializable {
  private final Translation3d m_translation;
  private final Rotation3d m_rotation;

  /** Constructs a pose at the origin facing toward the positive X axis. */
  public Pose3d() {
    m_translation = new Translation3d();
    m_rotation = new Rotation3d();
  }

  /**
   * Constructs a pose with the specified translation and rotation.
   *
   * @param translation The translational component of the pose.
   * @param rotation The rotational component of the pose.
   */
  @JsonCreator
  public Pose3d(
      @JsonProperty(required = true, value = "translation") Translation3d translation,
      @JsonProperty(required = true, value = "rotation") Rotation3d rotation) {
    m_translation = translation;
    m_rotation = rotation;
  }

  /**
   * Constructs a pose with x, y, and z translations instead of a separate Translation3d.
   *
   * @param x The x component of the translational component of the pose.
   * @param y The y component of the translational component of the pose.
   * @param z The z component of the translational component of the pose.
   * @param rotation The rotational component of the pose.
   */
  public Pose3d(double x, double y, double z, Rotation3d rotation) {
    m_translation = new Translation3d(x, y, z);
    m_rotation = rotation;
  }

  /**
   * Constructs a 3D pose from a 2D pose in the X-Y plane.
   *
   * @param pose The 2D pose.
   */
  public Pose3d(Pose2d pose) {
    m_translation = new Translation3d(pose.getX(), pose.getY(), 0.0);
    m_rotation = new Rotation3d(0.0, 0.0, pose.getRotation().getRadians());
  }

  /**
   * Transforms the pose by the given transformation and returns the new transformed pose. The
   * transform is applied relative to the pose's frame. Note that this differs from {@link
   * Pose3d#rotateBy(Rotation3d)}, which is applied relative to the global frame and around the
   * origin.
   *
   * @param other The transform to transform the pose by.
   * @return The transformed pose.
   */
  public Pose3d plus(Transform3d other) {
    return transformBy(other);
  }

  /**
   * Returns the Transform3d that maps the one pose to another.
   *
   * @param other The initial pose of the transformation.
   * @return The transform that maps the other pose to the current pose.
   */
  public Transform3d minus(Pose3d other) {
    final var pose = this.relativeTo(other);
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Returns the translation component of the transformation.
   *
   * @return The translational component of the pose.
   */
  @JsonProperty
  public Translation3d getTranslation() {
    return m_translation;
  }

  /**
   * Returns the X component of the pose's translation.
   *
   * @return The x component of the pose's translation.
   */
  public double getX() {
    return m_translation.getX();
  }

  /**
   * Returns the Y component of the pose's translation.
   *
   * @return The y component of the pose's translation.
   */
  public double getY() {
    return m_translation.getY();
  }

  /**
   * Returns the Z component of the pose's translation.
   *
   * @return The z component of the pose's translation.
   */
  public double getZ() {
    return m_translation.getZ();
  }

  /**
   * Returns the rotational component of the transformation.
   *
   * @return The rotational component of the pose.
   */
  @JsonProperty
  public Rotation3d getRotation() {
    return m_rotation;
  }

  /**
   * Multiplies the current pose by a scalar.
   *
   * @param scalar The scalar.
   * @return The new scaled Pose3d.
   */
  public Pose3d times(double scalar) {
    return new Pose3d(m_translation.times(scalar), m_rotation.times(scalar));
  }

  /**
   * Divides the current pose by a scalar.
   *
   * @param scalar The scalar.
   * @return The new scaled Pose3d.
   */
  public Pose3d div(double scalar) {
    return times(1.0 / scalar);
  }

  /**
   * Rotates the pose around the origin and returns the new pose.
   *
   * @param other The rotation to transform the pose by, which is applied extrinsically (from the
   *     global frame).
   * @return The rotated pose.
   */
  public Pose3d rotateBy(Rotation3d other) {
    return new Pose3d(m_translation.rotateBy(other), m_rotation.rotateBy(other));
  }

  /**
   * Transforms the pose by the given transformation and returns the new transformed pose. The
   * transform is applied relative to the pose's frame. Note that this differs from {@link
   * Pose3d#rotateBy(Rotation3d)}, which is applied relative to the global frame and around the
   * origin.
   *
   * @param other The transform to transform the pose by.
   * @return The transformed pose.
   */
  public Pose3d transformBy(Transform3d other) {
    return new Pose3d(
        m_translation.plus(other.getTranslation().rotateBy(m_rotation)),
        other.getRotation().plus(m_rotation));
  }

  /**
   * Returns the current pose relative to the given pose.
   *
   * <p>This function can often be used for trajectory tracking or pose stabilization algorithms to
   * get the error between the reference and the current pose.
   *
   * @param other The pose that is the origin of the new coordinate frame that the current pose will
   *     be converted into.
   * @return The current pose relative to the new origin pose.
   */
  public Pose3d relativeTo(Pose3d other) {
    var transform = new Transform3d(other, this);
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Obtain a new Pose3d from a (constant curvature) velocity.
   *
   * <p>The twist is a change in pose in the robot's coordinate frame since the previous pose
   * update. When the user runs exp() on the previous known field-relative pose with the argument
   * being the twist, the user will receive the new field-relative pose.
   *
   * <p>"Exp" represents the pose exponential, which is solving a differential equation moving the
   * pose forward in time.
   *
   * @param twist The change in pose in the robot's coordinate frame since the previous pose update.
   *     For example, if a non-holonomic robot moves forward 0.01 meters and changes angle by 0.5
   *     degrees since the previous pose update, the twist would be Twist3d(0.01, 0.0, 0.0, new new
   *     Rotation3d(0.0, 0.0, Units.degreesToRadians(0.5))).
   * @return The new pose of the robot.
   */
  public Pose3d exp(Twist3d twist) {
    var quaternion = this.getRotation().getQuaternion();
    double[] resultArray =
        WPIMathJNI.expPose3d(
            this.getX(),
            this.getY(),
            this.getZ(),
            quaternion.getW(),
            quaternion.getX(),
            quaternion.getY(),
            quaternion.getZ(),
            twist.dx,
            twist.dy,
            twist.dz,
            twist.rx,
            twist.ry,
            twist.rz);
    return new Pose3d(
        resultArray[0],
        resultArray[1],
        resultArray[2],
        new Rotation3d(
            new Quaternion(resultArray[3], resultArray[4], resultArray[5], resultArray[6])));
  }

  /**
   * Returns a Twist3d that maps this pose to the end pose. If c is the output of {@code a.Log(b)},
   * then {@code a.Exp(c)} would yield b.
   *
   * @param end The end pose for the transformation.
   * @return The twist that maps this to end.
   */
  public Twist3d log(Pose3d end) {
    var thisQuaternion = this.getRotation().getQuaternion();
    var endQuaternion = end.getRotation().getQuaternion();
    double[] resultArray =
        WPIMathJNI.logPose3d(
            this.getX(),
            this.getY(),
            this.getZ(),
            thisQuaternion.getW(),
            thisQuaternion.getX(),
            thisQuaternion.getY(),
            thisQuaternion.getZ(),
            end.getX(),
            end.getY(),
            end.getZ(),
            endQuaternion.getW(),
            endQuaternion.getX(),
            endQuaternion.getY(),
            endQuaternion.getZ());
    return new Twist3d(
        resultArray[0],
        resultArray[1],
        resultArray[2],
        resultArray[3],
        resultArray[4],
        resultArray[5]);
  }

  /**
   * Returns a Pose2d representing this Pose3d projected into the X-Y plane.
   *
   * @return A Pose2d representing this Pose3d projected into the X-Y plane.
   */
  public Pose2d toPose2d() {
    return new Pose2d(m_translation.toTranslation2d(), m_rotation.toRotation2d());
  }

  @Override
  public String toString() {
    return String.format("Pose3d(%s, %s)", m_translation, m_rotation);
  }

  /**
   * Checks equality between this Pose3d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Pose3d) {
      return ((Pose3d) obj).m_translation.equals(m_translation)
          && ((Pose3d) obj).m_rotation.equals(m_rotation);
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(m_translation, m_rotation);
  }

  @Override
  public Pose3d interpolate(Pose3d endValue, double t) {
    if (t < 0) {
      return this;
    } else if (t >= 1) {
      return endValue;
    } else {
      var twist = this.log(endValue);
      var scaledTwist =
          new Twist3d(
              twist.dx * t, twist.dy * t, twist.dz * t, twist.rx * t, twist.ry * t, twist.rz * t);
      return this.exp(scaledTwist);
    }
  }

  /** Pose3d protobuf for serialization. */
  public static final Pose3dProto proto = new Pose3dProto();

  /** Pose3d struct for serialization. */
  public static final Pose3dStruct struct = new Pose3dStruct();
}
