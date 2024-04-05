package frc.robot.utilities;

public class FiducialTarget {
  public Results Results;

  public Results getResults() {
    return Results;
  }

  public void setResults(Results results) {
    Results = results;
  }

  public class Results {}

  // @JsonProperty("botpose")
  // public BotPose botPose;

  // public BotPose getBotPose() {
  //   return botPose;
  // }

  // public void setBotPose(BotPose botPose) {
  //   this.botPose = botPose;
  // }

  // public class BotPose {
  //   @JsonProperty("tl")
  //   public double targetLatency;

  //   @JsonProperty("cl")
  //   public double captureLatency;

  //   @JsonProperty("ts")
  //   public double timestampMilliseconds;

  //   @JsonProperty("v")
  //   public double seeingAnything;

  //   @JsonProperty("botpose")
  //   public double[] botpose;

  //   @JsonProperty("botpose_wpired")
  //   public double[] botposeRed;

  //   @JsonProperty("botpose_wpiblue")
  //   public double[] botposeBlue;

  //   @JsonProperty("botpose_orb")
  //   public double[] botposeOrb;

  //   @JsonProperty("botpose_wpired_orb")
  //   public double[] botposeRedOrb;

  //   @JsonProperty("botpose_wpiblue_orb")
  //   public double[] botposeBlueOrb;

  //   @JsonProperty("botpose_avgarea")
  //   public double averageArea;

  //   @JsonProperty("avgdist")
  //   public double maxDistance;

  //   @JsonProperty("botpose_span")
  //   public double what;

  //   @JsonProperty("botpose_tagcount")
  //   public double numTags;

  //   public double getTargetLatency() {
  //     return targetLatency;
  //   }

  //   public void setTargetLatency(double targetLatency) {
  //     this.targetLatency = targetLatency;
  //   }

  //   public double getCaptureLatency() {
  //     return captureLatency;
  //   }

  //   public void setCaptureLatency(double captureLatency) {
  //     this.captureLatency = captureLatency;
  //   }

  //   public double getTimestampMilliseconds() {
  //     return timestampMilliseconds;
  //   }

  //   public void setTimestampMilliseconds(double timestampMilliseconds) {
  //     this.timestampMilliseconds = timestampMilliseconds;
  //   }

  //   public double getSeeingAnything() {
  //     return seeingAnything;
  //   }

  //   public void setSeeingAnything(double seeingAnything) {
  //     this.seeingAnything = seeingAnything;
  //   }

  //   public double[] getBotpose() {
  //     return botpose;
  //   }

  //   public void setBotpose(double[] botpose) {
  //     this.botpose = botpose;
  //   }

  //   public double[] getBotposeRed() {
  //     return botposeRed;
  //   }

  //   public void setBotposeRed(double[] botposeRed) {
  //     this.botposeRed = botposeRed;
  //   }

  //   public double[] getBotposeBlue() {
  //     return botposeBlue;
  //   }

  //   public void setBotposeBlue(double[] botposeBlue) {
  //     this.botposeBlue = botposeBlue;
  //   }

  //   public double[] getBotposeOrb() {
  //     return botposeOrb;
  //   }

  //   public void setBotposeOrb(double[] botposeOrb) {
  //     this.botposeOrb = botposeOrb;
  //   }

  //   public double[] getBotposeRedOrb() {
  //     return botposeRedOrb;
  //   }

  //   public void setBotposeRedOrb(double[] botposeRedOrb) {
  //     this.botposeRedOrb = botposeRedOrb;
  //   }

  //   public double[] getBotposeBlueOrb() {
  //     return botposeBlueOrb;
  //   }

  //   public void setBotposeBlueOrb(double[] botposeBlueOrb) {
  //     this.botposeBlueOrb = botposeBlueOrb;
  //   }

  //   public double getAverageArea() {
  //     return averageArea;
  //   }

  //   public void setAverageArea(double averageArea) {
  //     this.averageArea = averageArea;
  //   }

  //   public double getMaxDistance() {
  //     return maxDistance;
  //   }

  //   public void setMaxDistance(double maxDistance) {
  //     this.maxDistance = maxDistance;
  //   }

  //   public double getWhat() {
  //     return what;
  //   }

  //   public void setWhat(double what) {
  //     this.what = what;
  //   }

  //   public double getNumTags() {
  //     return numTags;
  //   }

  //   public void setNumTags(double numTags) {
  //     this.numTags = numTags;
  //   }

  //   private static Pose3d toPose3D(double[] inData) {
  //     if (inData.length < 6) {
  //       System.err.println("Bad LL 3D Pose Data!");
  //       return new Pose3d();
  //     }
  //     return new Pose3d(
  //         new Translation3d(inData[0], inData[1], inData[2]),
  //         new Rotation3d(
  //             Units.degreesToRadians(inData[3]),
  //             Units.degreesToRadians(inData[4]),
  //             Units.degreesToRadians(inData[5])));
  //   }
  // }

  // private static Pose2d toPose2D(double[] inData) {
  //   if (inData.length < 6) {
  //     System.err.println("Bad LL 2D Pose Data!");
  //     return new Pose2d();
  //   }
  //   Translation2d tran2d = new Translation2d(inData[0], inData[1]);
  //   Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
  //   return new Pose2d(tran2d, r2d);
  // }
}
